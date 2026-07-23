from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from time import perf_counter
from typing import Iterable

import numpy as np
from scipy.spatial import cKDTree
from skimage.draw import line as raster_line

from graph_construction.graph_memory import (
    EdgeKey,
    GraphState,
    InternalEdge,
    normalize_edge_key,
)
from graph_construction.grid_types import ClassifiedGrid


@dataclass
class EdgeBuildStats:
    """记录最近一帧局部 pair 生成, 安全检查和增量结果"""

    local_node_count: int = 0
    candidate_pair_count: int = 0
    clearance_check_count: int = 0
    added_edge_count: int = 0
    removed_edge_count: int = 0
    kept_edge_count: int = 0
    pair_generation_seconds: float = 0.0
    validation_seconds: float = 0.0


@dataclass
class EdgeDelta:
    """描述持久图需要执行的最小边变化"""

    edges_to_add: list[InternalEdge]
    edge_keys_to_remove: set[EdgeKey]


class EdgeBuilder:
    """按论文规则重算当前局部图中的全部半径 pair"""

    def __init__(self, edge_radius: float) -> None:
        self.edge_radius = float(edge_radius)
        self.last_stats = EdgeBuildStats()

    def build_delta(
        self,
        graph: GraphState,
        grid: ClassifiedGrid,
        sdf_obstacle: np.ndarray,
        sdf_unknown: np.ndarray,
        min_clearance: float,
        node_ids: Iterable[int],
    ) -> EdgeDelta:
        """构建当前窗口的边增量

        半径内每个无向 pair 只检查一次, 新边要求整条走廊已知且安全
        已确认边遇到 unknown 时保留, 遇到可见障碍或超出半径时删除
        """
        nodes = []
        node_grid_indices = []
        for node_id in sorted(node_ids):
            node = graph.nodes.get(node_id)
            if node is None:
                continue
            grid_index = grid.world_to_grid(
                node.position[0],
                node.position[1],
            )
            if grid_index is None:
                continue
            nodes.append(node)
            node_grid_indices.append(grid_index)
        self.last_stats = EdgeBuildStats(local_node_count=len(nodes))

        pair_started = perf_counter()
        pair_indices = _query_pair_indices(nodes, self.edge_radius)
        self.last_stats.pair_generation_seconds = perf_counter() - pair_started
        self.last_stats.candidate_pair_count = len(pair_indices)

        local_node_ids = {node.node_id for node in nodes}
        existing_local_keys = {
            edge_key
            for edge_key in graph.edge_keys_for_nodes(local_node_ids)
            if edge_key[0] in local_node_ids and edge_key[1] in local_node_ids
        }
        candidate_keys = {
            normalize_edge_key(
                nodes[int(pair[0])].node_id,
                nodes[int(pair[1])].node_id,
            )
            for pair in pair_indices
        }

        # 这部分旧边的两个端点都可见, 但距离已经超过论文连接半径
        edge_keys_to_remove = existing_local_keys - candidate_keys
        edges_to_add: list[InternalEdge] = []
        kept_edge_count = 0
        corridor_states = _corridor_state_grid(
            grid,
            sdf_obstacle,
            sdf_unknown,
            min_clearance,
        )

        validation_started = perf_counter()
        for pair in pair_indices:
            node_index_a = int(pair[0])
            node_index_b = int(pair[1])
            node_a = nodes[node_index_a]
            node_b = nodes[node_index_b]
            edge_key = normalize_edge_key(node_a.node_id, node_b.node_id)
            edge_exists = edge_key in graph.edges
            self.last_stats.clearance_check_count += 1
            state = _edge_corridor_state(
                node_grid_indices[node_index_a],
                node_grid_indices[node_index_b],
                corridor_states,
            )

            if state == "free":
                if edge_exists:
                    kept_edge_count += 1
                    continue
                edges_to_add.append(
                    InternalEdge(
                        from_id=edge_key[0],
                        to_id=edge_key[1],
                        cost=hypot(
                            node_a.position[0] - node_b.position[0],
                            node_a.position[1] - node_b.position[1],
                        ),
                    )
                )
                continue

            if not edge_exists:
                continue
            if state == "obstacle":
                edge_keys_to_remove.add(edge_key)
            else:
                kept_edge_count += 1

        self.last_stats.validation_seconds = (
            perf_counter() - validation_started
        )
        self.last_stats.added_edge_count = len(edges_to_add)
        self.last_stats.removed_edge_count = len(edge_keys_to_remove)
        self.last_stats.kept_edge_count = kept_edge_count
        return EdgeDelta(
            edges_to_add=edges_to_add,
            edge_keys_to_remove=edge_keys_to_remove,
        )


def _query_pair_indices(nodes, edge_radius: float) -> np.ndarray:
    """使用 SciPy 编译空间索引一次返回全部无向半径 pair"""
    if len(nodes) < 2:
        return np.empty((0, 2), dtype=np.int64)
    positions = np.asarray(
        [node.position[:2] for node in nodes],
        dtype=np.float64,
    )
    return cKDTree(positions).query_pairs(
        float(edge_radius),
        output_type="ndarray",
    )


def _corridor_state_grid(
    grid: ClassifiedGrid,
    sdf_obstacle: np.ndarray,
    sdf_unknown: np.ndarray,
    min_clearance: float,
) -> np.ndarray:
    """预先合并分类和净空结果, 减少逐 pair NumPy 调用"""
    states = np.zeros(grid.free.shape, dtype=np.uint8)
    states[grid.unknown] = 1
    if min_clearance > 0.0:
        states[sdf_unknown < min_clearance] = 1
        obstacle_clearance_failed = (
            ~grid.unknown
            & (sdf_obstacle < min_clearance)
        )
        states[obstacle_clearance_failed] = 2
    states[grid.obstacle] = 2
    return states


def _edge_corridor_state(
    start: tuple[int, int],
    end: tuple[int, int],
    corridor_states: np.ndarray,
) -> str:
    """将 pair 统一分类为 free, unknown 或 obstacle

    可见障碍优先否决历史边, unknown 只阻止新边
    栅格状态已经在 pair 循环外合并, 每条线只执行一次数组查询
    """
    # skimage.draw.line 在 Cython 中生成线段索引, 避免逐 cell Python generator
    index_y, index_x = raster_line(
        start[1],
        start[0],
        end[1],
        end[0],
    )
    state = int(np.max(corridor_states[index_y, index_x]))
    if state == 2:
        return "obstacle"
    if state == 1:
        return "unknown"
    return "free"
