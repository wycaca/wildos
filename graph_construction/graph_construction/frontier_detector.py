from __future__ import annotations

from math import ceil, hypot
from typing import List, Sequence, Tuple

from graph_construction.graph_memory import GraphState, Point3
from graph_construction.grid_adapter import ClassifiedGrid


GridIndex = Tuple[int, int]


class FrontierDetector:
    """检测 free 和 unknown 的边界, 并分配给图节点

    论文中的 frontier node 不是每个 frontier cell 都单独成为节点
    本模块先找出密集 frontier cells, 再把它们挂到最近的 collision-free graph node 上
    这样输出的图更稀疏, 也更适合 WildOS 对 frontier node 做视觉评分
    """

    def __init__(
        self,
        frontier_assign_radius: float,
        frontier_min_points: int,
        frontier_min_span: float,
        frontier_border_margin: float,
        removed_frontier_suppression_radius: float,
    ) -> None:
        self.frontier_assign_radius = frontier_assign_radius
        self.frontier_min_points = frontier_min_points
        self.frontier_min_span = frontier_min_span
        self.frontier_border_margin = frontier_border_margin
        self.removed_frontier_suppression_radius = removed_frontier_suppression_radius

    def detect_frontier_cells(self, grid: ClassifiedGrid) -> List[GridIndex]:
        """扫描所有 free cell, 找出和 unknown 相邻的边界 cell"""
        frontier_cells: List[GridIndex] = []
        for iy in range(grid.height):
            for ix in range(grid.width):
                if not grid.is_free_index(ix, iy):
                    continue
                if self._near_grid_border(grid, ix, iy):
                    continue
                if self._touches_unknown(grid, ix, iy):
                    frontier_cells.append((ix, iy))
        return frontier_cells

    def assign_frontiers(self, graph: GraphState, grid: ClassifiedGrid, frontier_cells: Sequence[GridIndex]) -> None:
        """将 frontier cells 聚合到附近的 graph node 上

        每次重新分配前先清空旧 frontier_points
        这样已被探索过的 frontier 会自然消失, deadend_recovery 会记录这些消失的点
        """
        for node in graph.nodes.values():
            node.frontier_points.clear()
            node.is_frontier = False

        for ix, iy in frontier_cells:
            frontier_point = grid.grid_to_world(ix, iy)
            if self._near_removed_frontier(graph, frontier_point):
                continue
            owner = graph.nearest_node(frontier_point, max_distance=self.frontier_assign_radius)
            if owner is None:
                continue
            if self._inside_explored_area(owner.position, frontier_point, owner.explored_radius, grid.resolution):
                continue
            if not grid.is_world_collision_free(
                (owner.position[0], owner.position[1]),
                (frontier_point[0], frontier_point[1]),
            ):
                continue
            owner.frontier_points.append(frontier_point)

        # frontier_min_points 和 frontier_min_span 共同过滤孤立噪声边界
        for node in graph.nodes.values():
            node.is_frontier = (
                len(node.frontier_points) >= self.frontier_min_points
                and self._frontier_span(node.frontier_points) >= self.frontier_min_span
            )
            if not node.is_frontier:
                node.frontier_points.clear()

    def _near_grid_border(self, grid: ClassifiedGrid, ix: int, iy: int) -> bool:
        """过滤局部滑窗外边界, 避免把地图边缘当作 frontier"""
        margin_cells = max(0, int(ceil(self.frontier_border_margin / grid.resolution)))
        if margin_cells <= 0:
            return False
        return (
            ix < margin_cells
            or iy < margin_cells
            or ix >= grid.width - margin_cells
            or iy >= grid.height - margin_cells
        )

    def _touches_unknown(self, grid: ClassifiedGrid, ix: int, iy: int) -> bool:
        """检查一个 free cell 的 8 邻域是否包含 unknown"""
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if grid.is_unknown_index(ix + dx, iy + dy):
                    return True
        return False

    def _near_removed_frontier(self, graph: GraphState, frontier_point: Point3) -> bool:
        """抑制最近刚消失的 frontier, 降低死路附近反复探索的概率"""
        if self.removed_frontier_suppression_radius <= 0.0:
            return False
        for removed in graph.removed_frontiers:
            if hypot(removed[0] - frontier_point[0], removed[1] - frontier_point[1]) <= self.removed_frontier_suppression_radius:
                return True
        return False

    def _frontier_span(self, points: Sequence[Point3]) -> float:
        """估计 frontier points 的空间跨度, 用于过滤短小噪声"""
        if not points:
            return 0.0
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        return hypot(max(xs) - min(xs), max(ys) - min(ys))

    def _inside_explored_area(
        self,
        node_position: Point3,
        frontier_point: Point3,
        explored_radius: float,
        resolution: float,
    ) -> bool:
        """判断 frontier point 是否已经落入节点的已探索区域"""
        if explored_radius <= 0.0:
            return False
        distance = hypot(node_position[0] - frontier_point[0], node_position[1] - frontier_point[1])
        return distance < max(0.0, explored_radius - resolution)
