from __future__ import annotations

from math import hypot
from typing import Dict, List

from graph_construction.graph_memory import GraphState, Point3


class DeadendRecovery:
    """记录已消失 frontier, 避免重复进入死路

    当一个节点曾经是 frontier, 但连续若干次更新后不再有 frontier_points
    说明这片未知边界大概率已经被探索或被证明不可继续前进
    这些点会被加入 removed_frontiers, 后续 frontier 分配会在附近做 suppression
    """

    def __init__(self, observation_count: int, suppression_radius: float) -> None:
        self.observation_count = observation_count
        self.suppression_radius = suppression_radius
        self._missing_counts: Dict[int, int] = {}
        self._last_frontier_points: Dict[int, List[Point3]] = {}

    def update(self, graph: GraphState) -> None:
        """更新 frontier 生命周期记录"""
        active_node_ids = set(graph.nodes.keys())

        for node in graph.nodes.values():
            if node.is_frontier:
                self._missing_counts[node.node_id] = 0
                self._last_frontier_points[node.node_id] = list(node.frontier_points)
                continue

            if node.node_id not in self._last_frontier_points:
                continue

            self._missing_counts[node.node_id] = self._missing_counts.get(node.node_id, 0) + 1
            if self._missing_counts[node.node_id] >= self.observation_count:
                graph.removed_frontiers.extend(self._last_frontier_points[node.node_id])
                self._last_frontier_points.pop(node.node_id, None)
                self._missing_counts.pop(node.node_id, None)

        # 如果节点被删除, 它之前关联的 frontier 也需要进入 removed memory
        for node_id in list(self._last_frontier_points.keys()):
            if node_id in active_node_ids:
                continue
            graph.removed_frontiers.extend(self._last_frontier_points[node_id])
            self._last_frontier_points.pop(node_id, None)
            self._missing_counts.pop(node_id, None)

        self._compact_removed_frontiers(graph)

    def _compact_removed_frontiers(self, graph: GraphState) -> None:
        """合并距离很近的 removed frontier, 防止列表无限增长"""
        compacted = []
        for point in graph.removed_frontiers:
            duplicate = False
            for kept in compacted:
                if hypot(kept[0] - point[0], kept[1] - point[1]) <= self.suppression_radius:
                    duplicate = True
                    break
            if not duplicate:
                compacted.append(point)
        graph.removed_frontiers = compacted[-200:]
