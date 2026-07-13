from __future__ import annotations

from dataclasses import dataclass
from math import ceil, hypot
from typing import Dict, Iterable, List, Sequence, Tuple

from graph_construction.graph_memory import GraphState, InternalNode, Point3
from graph_construction.grid_types import ClassifiedGrid


GridIndex = Tuple[int, int]


@dataclass
class FrontierAssignmentStats:
    """记录 frontier 分配前后的数量, 便于定位候选过密问题"""

    raw_cell_count: int = 0
    candidate_cell_count: int = 0
    assigned_point_count: int = 0


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
        frontier_candidate_spacing: float = 0.0,
    ) -> None:
        self.frontier_assign_radius = frontier_assign_radius
        self.frontier_min_points = frontier_min_points
        self.frontier_min_span = frontier_min_span
        self.frontier_border_margin = frontier_border_margin
        self.frontier_candidate_spacing = frontier_candidate_spacing

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

    def assign_frontiers(
        self,
        graph: GraphState,
        grid: ClassifiedGrid,
        frontier_cells: Sequence[GridIndex],
    ) -> FrontierAssignmentStats:
        """增量验证历史 frontier, 再把新边界分配给稳定 owner

        滚动局部地图只能否定当前可见范围内的历史 frontier
        窗口外 frontier 必须保留, 否则机器人移动一个局部地图宽度后会丢失探索记忆
        当前可见的历史点需要重新满足 free/unknown 边界和 owner 可达条件
        新边界使用世界坐标键去重, 已保留的 owner 不会被每帧最近邻结果替换
        """
        preserved_owner_ids, assigned_frontier_keys = self._validate_historical_frontiers(
            graph,
            grid,
        )

        candidate_cells = self._select_frontier_candidates(grid, frontier_cells)
        assignable_nodes = [
            node for node in graph.nodes.values()
            if not node.is_robot_anchor
        ]
        node_index = _NodeSpatialIndex(assignable_nodes, self.frontier_assign_radius)
        assigned_point_count = 0

        for ix, iy in candidate_cells:
            frontier_point = grid.grid_to_world(ix, iy)
            frontier_key = self._frontier_key(frontier_point, grid.resolution)
            if frontier_key in assigned_frontier_keys:
                continue
            owner_candidates = node_index.candidate_ids(frontier_point)
            owner = graph.nearest_node(
                frontier_point,
                max_distance=self.frontier_assign_radius,
                candidates=owner_candidates,
            )
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
            assigned_frontier_keys.add(frontier_key)
            assigned_point_count += 1

        # 历史 owner 已经通过往帧观测确认, 不因当前窗口只剩少量可见点而失忆
        # 新 owner 仍使用数量和跨度过滤当前帧产生的孤立噪声
        for node in graph.nodes.values():
            if node.is_robot_anchor:
                node.is_frontier = False
                node.frontier_points.clear()
                continue
            is_supported = (
                len(node.frontier_points) >= self.frontier_min_points
                and self._frontier_span(node.frontier_points) >= self.frontier_min_span
            )
            node.is_frontier = bool(node.frontier_points) and (
                node.node_id in preserved_owner_ids or is_supported
            )
            if not node.is_frontier:
                node.frontier_points.clear()

        return FrontierAssignmentStats(
            raw_cell_count=len(frontier_cells),
            candidate_cell_count=len(candidate_cells),
            assigned_point_count=assigned_point_count,
        )

    def _validate_historical_frontiers(
        self,
        graph: GraphState,
        grid: ClassifiedGrid,
    ) -> Tuple[set[int], set[Tuple[int, int]]]:
        """保留窗口外历史点, 并重新验证当前可见历史点

        frontier point 在本实现中位于 free cell, 且其 8 邻域必须接触 unknown
        可见点若已成为普通 known free、unknown、obstacle 或无法从 owner 安全到达则删除
        世界坐标键同时消除不同 owner 上重复保存的同一物理边界
        """
        preserved_owner_ids: set[int] = set()
        assigned_frontier_keys: set[Tuple[int, int]] = set()

        for node in graph.nodes.values():
            if node.is_robot_anchor:
                node.frontier_points.clear()
                node.is_frontier = False
                continue

            preserved_points: List[Point3] = []
            for point in node.frontier_points:
                frontier_key = self._frontier_key(point, grid.resolution)
                if frontier_key in assigned_frontier_keys:
                    continue

                grid_index = grid.world_to_grid(point[0], point[1])
                if grid_index is None:
                    preserved_points.append(point)
                    assigned_frontier_keys.add(frontier_key)
                    continue

                ix, iy = grid_index
                if not grid.is_free_index(ix, iy):
                    continue
                if self._near_grid_border(grid, ix, iy):
                    continue
                if not self._touches_unknown(grid, ix, iy):
                    continue
                if not grid.is_world_collision_free(
                    (node.position[0], node.position[1]),
                    (point[0], point[1]),
                ):
                    continue

                preserved_points.append(point)
                assigned_frontier_keys.add(frontier_key)

            node.frontier_points = preserved_points
            node.is_frontier = bool(preserved_points)
            if preserved_points:
                preserved_owner_ids.add(node.node_id)

        return preserved_owner_ids, assigned_frontier_keys

    @staticmethod
    def _frontier_key(point: Point3, resolution: float) -> Tuple[int, int]:
        """将 world frontier 量化为稳定键, 避免滚动窗口重复分配"""
        key_resolution = max(float(resolution), 1e-6)
        return (
            int(round(point[0] / key_resolution)),
            int(round(point[1] / key_resolution)),
        )

    def _select_frontier_candidates(
        self,
        grid: ClassifiedGrid,
        frontier_cells: Sequence[GridIndex],
    ) -> List[GridIndex]:
        """按米制间距抽取代表 cell, 减少重复 owner 和 collision 检查"""
        spacing = float(self.frontier_candidate_spacing)
        if spacing <= grid.resolution:
            return list(frontier_cells)

        bucket_cells = max(1, int(round(spacing / grid.resolution)))
        selected: Dict[GridIndex, GridIndex] = {}
        for ix, iy in frontier_cells:
            key = (ix // bucket_cells, iy // bucket_cells)
            if key not in selected:
                selected[key] = (ix, iy)
        return list(selected.values())

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


class _NodeSpatialIndex:
    """按 assign radius 建立临时节点桶, 避免每个 frontier 扫描全图节点"""

    def __init__(self, nodes: Iterable[InternalNode], bucket_size: float) -> None:
        self.bucket_size = max(0.01, float(bucket_size))
        self.buckets: Dict[Tuple[int, int], List[int]] = {}
        for node in nodes:
            key = self._key(node.position)
            self.buckets.setdefault(key, []).append(node.node_id)

    def candidate_ids(self, position: Point3) -> List[int]:
        """返回可能落在 assign radius 内的节点 id"""
        center_x, center_y = self._key(position)
        candidates: List[int] = []
        for by in range(center_y - 1, center_y + 2):
            for bx in range(center_x - 1, center_x + 2):
                candidates.extend(self.buckets.get((bx, by), ()))
        return candidates

    def _key(self, position: Point3) -> Tuple[int, int]:
        return (
            int(position[0] // self.bucket_size),
            int(position[1] // self.bucket_size),
        )
