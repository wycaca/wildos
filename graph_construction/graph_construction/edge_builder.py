from __future__ import annotations

from math import hypot
from typing import List

from graph_construction.graph_memory import GraphState, InternalEdge
from graph_construction.grid_adapter import ClassifiedGrid


class EdgeBuilder:
    """为邻近图节点构建无碰撞加权边

    第一版使用空间近邻加 Bresenham 直线检测
    这能快速形成可被 graphnav_planner 消费的稀疏图
    后续接真实 elevation map 后, 可以在这里加入坡度, 粗糙度, footprint inflation 等代价
    """

    def __init__(self, edge_radius: float) -> None:
        self.edge_radius = edge_radius

    def build_edges(self, graph: GraphState, grid: ClassifiedGrid) -> List[InternalEdge]:
        """重建当前图的无向边集合"""
        nodes = list(graph.nodes.values())
        edges: List[InternalEdge] = []

        for index, node_a in enumerate(nodes):
            for node_b in nodes[index + 1 :]:
                dx = node_a.position[0] - node_b.position[0]
                dy = node_a.position[1] - node_b.position[1]
                distance = hypot(dx, dy)
                if distance > self.edge_radius:
                    continue

                # 只有整条连线都在 free 区域中, 才允许 planner 使用这条边
                if not grid.is_world_collision_free(
                    (node_a.position[0], node_a.position[1]),
                    (node_b.position[0], node_b.position[1]),
                ):
                    continue
                edges.append(InternalEdge(from_id=node_a.node_id, to_id=node_b.node_id, cost=distance))

        return edges
