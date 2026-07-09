from __future__ import annotations

from math import hypot
from typing import Dict, List, Tuple

import numpy as np

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
    ) -> List[InternalEdge]:
        """重建当前图的无向边集合

        论文和常见 sparse roadmap 都只连接局部近邻, 避免半径内近似全连接
        当前节点保留更多边, 便于机器人附近路径选择更灵活
        """
        nodes = list(graph.nodes.values())
        selected_edges: Dict[Tuple[int, int], InternalEdge] = {}
        candidate_edges: Dict[int, List[Tuple[float, int, InternalEdge]]] = {
            node.node_id: []
            for node in nodes
        }

        for index, node_a in enumerate(nodes):
            for node_b in nodes[index + 1 :]:
                dx = node_a.position[0] - node_b.position[0]
                dy = node_a.position[1] - node_b.position[1]
                distance = hypot(dx, dy)
                if distance > self.edge_radius:
                    continue

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

                edge = InternalEdge(from_id=node_a.node_id, to_id=node_b.node_id, cost=distance)
                candidate_edges[node_a.node_id].append((distance, node_b.node_id, edge))
                candidate_edges[node_b.node_id].append((distance, node_a.node_id, edge))

        for node in nodes:
            max_neighbors = self._neighbor_limit(graph, node.node_id)
            for _, _, edge in sorted(candidate_edges[node.node_id], key=lambda item: item[0])[:max_neighbors]:
                key = _edge_key(edge.from_id, edge.to_id)
                selected_edges[key] = edge

        return list(selected_edges.values())

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
    for ix, iy in line_cells:
        if grid.is_obstacle_index(ix, iy) or grid.is_unknown_index(ix, iy):
            return False
        if min_clearance <= 0.0 or sdf_obstacle is None or sdf_unknown is None:
            continue
        clearance = min(float(sdf_obstacle[iy, ix]), float(sdf_unknown[iy, ix]))
        if clearance < min_clearance:
            return False
    return True
