from __future__ import annotations

from math import hypot
from typing import Dict, Iterable, List, Tuple

import numpy as np
from scipy.spatial import cKDTree

from graph_construction.graph_memory import GraphState, InternalEdge
from graph_construction.grid_types import ClassifiedGrid


class EdgeBuilder:
    """为邻近图节点构建无碰撞加权边

    第一版使用空间近邻加 Bresenham 直线检测
    这能快速形成可被 graphnav_planner 消费的稀疏图
    后续接真实 elevation map 后, 可以在这里加入坡度, 粗糙度, footprint inflation 等代价
    """

    def __init__(
        self,
        edge_radius: float,
        max_neighbors_per_node: int = 4,
        current_node_max_neighbors: int = 12,
    ) -> None:
        self.edge_radius = edge_radius
        self.max_neighbors_per_node = max_neighbors_per_node
        self.current_node_max_neighbors = current_node_max_neighbors

    def build_edges(
        self,
        graph: GraphState,
        grid: ClassifiedGrid,
        sdf_obstacle: np.ndarray | None = None,
        sdf_unknown: np.ndarray | None = None,
        min_clearance: float = 0.0,
        node_ids: Iterable[int] | None = None,
        focus_node_ids: Iterable[int] | None = None,
    ) -> List[InternalEdge]:
        """重建当前图的无向边集合

        论文和常见 sparse roadmap 都只连接局部近邻, 避免半径内近似全连接
        当前节点保留更多边, 便于机器人附近路径选择更灵活
        """
        # 新边只在当前地图窗口生成, 窗口外边由历史边合并逻辑维护
        candidate_ids = (
            graph.nodes.keys()
            if node_ids is None
            else sorted(node_ids)
        )
        nodes = []
        for node_id in candidate_ids:
            node = graph.nodes.get(node_id)
            if node is None:
                continue
            if grid.world_to_grid(node.position[0], node.position[1]) is not None:
                nodes.append(node)
        focus_ids = (
            None
            if focus_node_ids is None
            else set(focus_node_ids)
        )
        selected_edges: Dict[Tuple[int, int], InternalEdge] = {}
        candidate_edges: Dict[int, List[Tuple[float, int, InternalEdge]]] = {
            node.node_id: []
            for node in nodes
        }

        if len(nodes) >= 2:
            positions = np.asarray(
                [(node.position[0], node.position[1]) for node in nodes],
                dtype=np.float64,
            )
            spatial_index = cKDTree(positions)
            nearby_pairs = sorted(spatial_index.query_pairs(self.edge_radius))
        else:
            nearby_pairs = []

        for index_a, index_b in nearby_pairs:
            node_a = nodes[index_a]
            node_b = nodes[index_b]
            if (
                focus_ids is not None
                and node_a.node_id not in focus_ids
                and node_b.node_id not in focus_ids
            ):
                continue
            dx = node_a.position[0] - node_b.position[0]
            dy = node_a.position[1] - node_b.position[1]
            distance = hypot(dx, dy)

            # 只有整条走廊都远离 obstacle 和 unknown, 才允许 planner 使用这条边
            if not _edge_has_clearance(
                grid,
                (node_a.position[0], node_a.position[1]),
                (node_b.position[0], node_b.position[1]),
                sdf_obstacle,
                sdf_unknown,
                min_clearance,
            ):
                continue

            edge = InternalEdge(
                from_id=node_a.node_id,
                to_id=node_b.node_id,
                cost=distance,
            )
            candidate_edges[node_a.node_id].append((distance, node_b.node_id, edge))
            candidate_edges[node_b.node_id].append((distance, node_a.node_id, edge))

        output_nodes = (
            nodes
            if focus_ids is None
            else [node for node in nodes if node.node_id in focus_ids]
        )
        for node in output_nodes:
            max_neighbors = self._neighbor_limit(graph, node.node_id)
            sorted_candidates = sorted(
                candidate_edges[node.node_id],
                key=lambda item: item[0],
            )
            for _, _, edge in sorted_candidates[:max_neighbors]:
                key = _edge_key(edge.from_id, edge.to_id)
                selected_edges[key] = edge

        return list(selected_edges.values())

    def merge_historical_edges(
        self,
        graph: GraphState,
        current_edges: Iterable[InternalEdge],
        grid: ClassifiedGrid,
        sdf_obstacle: np.ndarray | None = None,
        min_clearance: float = 0.0,
        historical_edge_keys: Iterable[Tuple[int, int]] | None = None,
    ) -> List[InternalEdge]:
        """保留未被当前可见障碍证伪的历史边

        新边仍必须穿过已知 free 区域, 历史边只用当前可见段做安全否决
        这样机器人转向导致局部图变 unknown 时, 不会把已经走通过的边误删
        """
        selected_edges: Dict[Tuple[int, int], InternalEdge] = {}
        for edge in current_edges:
            selected_edges[_edge_key(edge.from_id, edge.to_id)] = edge

        if historical_edge_keys is None:
            historical_edges = graph.edges.values()
        else:
            historical_edges = (
                graph.edges[edge_key]
                for edge_key in historical_edge_keys
                if edge_key in graph.edges
            )
        for edge in historical_edges:
            key = _edge_key(edge.from_id, edge.to_id)
            if key in selected_edges:
                continue
            node_a = graph.nodes.get(edge.from_id)
            node_b = graph.nodes.get(edge.to_id)
            if node_a is None or node_b is None:
                continue
            dx = node_a.position[0] - node_b.position[0]
            dy = node_a.position[1] - node_b.position[1]
            if hypot(dx, dy) > self.edge_radius:
                continue
            if not _historical_edge_has_no_local_contradiction(
                grid,
                (node_a.position[0], node_a.position[1]),
                (node_b.position[0], node_b.position[1]),
                sdf_obstacle,
                min_clearance,
            ):
                continue
            selected_edges[key] = InternalEdge(
                from_id=key[0],
                to_id=key[1],
                cost=edge.cost,
            )

        return list(selected_edges.values())

    def build_robot_anchor_edges(
        self,
        graph: GraphState,
        anchor_id: int,
        grid: ClassifiedGrid,
        sdf_obstacle: np.ndarray | None = None,
        min_clearance: float = 0.0,
        edge_radius: float | None = None,
        max_edges: int | None = None,
    ) -> List[InternalEdge]:
        """为机器人锚点连接当前可见段未碰障碍的近邻节点"""
        anchor = graph.nodes.get(anchor_id)
        if anchor is None:
            return []

        radius = (
            self.edge_radius
            if edge_radius is None or edge_radius <= 0.0
            else float(edge_radius)
        )
        limit = max(
            1,
            int(
                max_edges
                if max_edges is not None
                else self.current_node_max_neighbors
            ),
        )
        candidates: List[Tuple[float, InternalEdge]] = []
        nearby_node_ids = graph.node_ids_within(anchor.position, radius)
        for node_id in nearby_node_ids:
            node = graph.nodes.get(node_id)
            if node is None:
                continue
            if node.node_id == anchor_id or node.is_robot_anchor:
                continue
            dx = anchor.position[0] - node.position[0]
            dy = anchor.position[1] - node.position[1]
            distance = hypot(dx, dy)
            if distance > radius:
                continue
            if not _historical_edge_has_no_local_contradiction(
                grid,
                (anchor.position[0], anchor.position[1]),
                (node.position[0], node.position[1]),
                sdf_obstacle,
                min_clearance,
            ):
                continue
            candidates.append(
                (
                    distance,
                    InternalEdge(from_id=anchor_id, to_id=node.node_id, cost=distance),
                )
            )

        return [edge for _, edge in sorted(candidates, key=lambda item: item[0])[:limit]]

    def _neighbor_limit(self, graph: GraphState, node_id: int) -> int:
        """当前节点允许更多近邻边, 其他节点保持稀疏"""
        if graph.current_node_id == node_id:
            return max(1, int(self.current_node_max_neighbors))
        return max(1, int(self.max_neighbors_per_node))


def _edge_key(from_id: int, to_id: int) -> Tuple[int, int]:
    """生成稳定无向边键"""
    return (from_id, to_id) if from_id <= to_id else (to_id, from_id)


def _edge_has_clearance(
    grid: ClassifiedGrid,
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
    sdf_obstacle: np.ndarray | None,
    sdf_unknown: np.ndarray | None,
    min_clearance: float,
) -> bool:
    """检查 edge 中心线和走廊 clearance, 避免贴墙切角"""
    line_cells = list(grid.world_line_cells(start_xy, end_xy))
    if not line_cells:
        return False
    cell_array = np.asarray(line_cells, dtype=np.int64)
    index_x = cell_array[:, 0]
    index_y = cell_array[:, 1]
    if np.any(grid.obstacle[index_y, index_x] | grid.unknown[index_y, index_x]):
        return False
    if min_clearance <= 0.0 or sdf_obstacle is None or sdf_unknown is None:
        return True
    clearance = np.minimum(
        sdf_obstacle[index_y, index_x],
        sdf_unknown[index_y, index_x],
    )
    return bool(np.all(clearance >= min_clearance))


def _historical_edge_has_no_local_contradiction(
    grid: ClassifiedGrid,
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
    sdf_obstacle: np.ndarray | None,
    min_clearance: float,
) -> bool:
    """只用当前可见障碍否决历史边, unknown 不删除历史通路"""
    line_cells = list(grid.world_line_cells_clipped(start_xy, end_xy))
    if not line_cells:
        return True
    cell_array = np.asarray(line_cells, dtype=np.int64)
    index_x = cell_array[:, 0]
    index_y = cell_array[:, 1]
    if np.any(grid.obstacle[index_y, index_x]):
        return False
    if min_clearance <= 0.0 or sdf_obstacle is None:
        return True
    known = ~grid.unknown[index_y, index_x]
    return bool(np.all(sdf_obstacle[index_y[known], index_x[known]] >= min_clearance))
