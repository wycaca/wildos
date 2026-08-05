from __future__ import annotations

from math import ceil, hypot, isfinite
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from scipy import ndimage

from graph_construction.graph_memory import GraphState, Point3
from graph_construction.grid_types import ClassifiedGrid


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
        frontier_candidate_spacing: float = 0.0,
        frontier_visited_corridor_radius: float = 0.0,
    ) -> None:
        self.frontier_assign_radius = frontier_assign_radius
        self.frontier_min_points = frontier_min_points
        self.frontier_min_span = frontier_min_span
        self.frontier_border_margin = frontier_border_margin
        self.frontier_candidate_spacing = frontier_candidate_spacing
        self.frontier_visited_corridor_radius = max(
            0.0,
            float(frontier_visited_corridor_radius),
        )
        self._active_owner_ids: set[int] = set()
        self._index_initialized = False
        self._max_explored_radius = 0.0
        self._visited_corridor = _VisitedTrajectoryIndex(
            (),
            self.frontier_visited_corridor_radius,
        )
        self._latest_frontier_mask: np.ndarray | None = None
        self._latest_frontier_grid_id: int | None = None
        self.last_candidate_count = 0

    @property
    def active_owner_count(self) -> int:
        """返回当前活动 Frontier owner 数量"""
        return len(self._active_owner_ids)

    def detect_frontier_cells(
        self,
        grid: ClassifiedGrid,
        candidate_region: np.ndarray | None = None,
    ) -> List[GridIndex]:
        """更新完整 Frontier mask, 只返回变化区域内的新候选"""
        touches_unknown = ndimage.binary_dilation(
            grid.unknown,
            structure=np.ones((3, 3), dtype=bool),
            border_value=0,
        )
        frontier = grid.free & touches_unknown
        margin_cells = max(
            0,
            int(ceil(self.frontier_border_margin / grid.resolution)),
        )
        if margin_cells > 0:
            frontier[:margin_cells, :] = False
            frontier[-margin_cells:, :] = False
            frontier[:, :margin_cells] = False
            frontier[:, -margin_cells:] = False
        self._latest_frontier_mask = frontier
        self._latest_frontier_grid_id = id(grid)
        candidates = frontier
        if candidate_region is not None:
            if candidate_region.shape != frontier.shape:
                raise ValueError("candidate_region shape must match frontier mask")
            candidates = frontier & candidate_region
        return [
            (int(ix), int(iy))
            for iy, ix in np.argwhere(candidates)
        ]

    def assign_frontiers(
        self,
        graph: GraphState,
        grid: ClassifiedGrid,
        frontier_cells: Sequence[GridIndex],
        local_node_ids: Iterable[int] | None = None,
    ) -> None:
        """验证当前可见 Frontier, 再把新边界分配给稳定 owner

        Frontier 只表达当前地图的 free/unknown 边界, 窗口外分支由 planner 单独记忆
        当前可见的历史点需要重新满足边界语义、探索覆盖和 owner 可达条件
        新边界使用世界坐标键去重, 已保留的 owner 不会被每帧最近邻结果替换
        """
        self._refresh_active_index(graph, local_node_ids)
        explored_areas = _ExploredAreaIndex(
            graph,
            grid.resolution,
            self._max_explored_radius,
        )
        self._visited_corridor.update(graph.trajectory_points)
        preserved_owner_ids, assigned_frontier_keys = self._validate_historical_frontiers(
            graph,
            grid,
            explored_areas,
            self._visited_corridor,
        )

        candidate_cells = self._select_frontier_candidates(grid, frontier_cells)
        self.last_candidate_count = len(candidate_cells)
        updated_owner_ids: set[int] = set()

        for ix, iy in candidate_cells:
            frontier_point = grid.grid_to_world(ix, iy)
            frontier_key = self._frontier_key(frontier_point, grid.resolution)
            if frontier_key in assigned_frontier_keys:
                continue
            owner_candidates = graph.node_ids_within(
                frontier_point,
                self.frontier_assign_radius,
            )
            owner = graph.nearest_node(
                frontier_point,
                candidates=owner_candidates,
            )
            if owner is None:
                continue
            if explored_areas.contains(frontier_point):
                continue
            if self._visited_corridor.contains(frontier_point):
                continue
            if not grid.is_world_collision_free(
                (owner.position[0], owner.position[1]),
                (frontier_point[0], frontier_point[1]),
            ):
                continue
            owner.frontier_points.append(frontier_point)
            updated_owner_ids.add(owner.node_id)
            assigned_frontier_keys.add(frontier_key)

        # 历史 owner 已经通过往帧观测确认, 不因当前窗口只剩少量可见点而失忆
        # 新 owner 仍使用数量和跨度过滤当前帧产生的孤立噪声
        candidate_owner_ids = preserved_owner_ids | updated_owner_ids
        next_active_owner_ids: set[int] = set()
        for node_id in candidate_owner_ids:
            node = graph.nodes.get(node_id)
            if node is None:
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
            else:
                next_active_owner_ids.add(node.node_id)
        self._active_owner_ids = next_active_owner_ids

    def _validate_historical_frontiers(
        self,
        graph: GraphState,
        grid: ClassifiedGrid,
        explored_areas: _ExploredAreaIndex,
        visited_corridor: _VisitedTrajectoryIndex,
    ) -> Tuple[set[int], set[Tuple[int, int]]]:
        """只保留当前地图仍能验证的历史 Frontier

        frontier point 在本实现中位于 free cell, 且其 8 邻域必须接触 unknown
        地图外点转交 planner 的 deferred branch 记忆, 不再作为活动 Frontier 发布
        可见点若已被其他节点探索、失去边界语义或无法从 owner 安全到达则删除
        世界坐标键同时消除不同 owner 上重复保存的同一物理边界
        """
        preserved_owner_ids: set[int] = set()
        assigned_frontier_keys: set[Tuple[int, int]] = set()

        for node_id in tuple(self._active_owner_ids):
            node = graph.nodes.get(node_id)
            if node is None:
                continue

            preserved_points: List[Point3] = []
            for point in node.frontier_points:
                frontier_key = self._frontier_key(point, grid.resolution)
                if frontier_key in assigned_frontier_keys:
                    continue

                grid_index = grid.world_to_grid(point[0], point[1])
                if grid_index is None:
                    continue

                ix, iy = grid_index
                if not self._is_frontier_index(grid, ix, iy):
                    continue
                if explored_areas.contains(point, excluded_node_id=node.node_id):
                    continue
                if visited_corridor.contains(point):
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

    def _refresh_active_index(
        self,
        graph: GraphState,
        local_node_ids: Iterable[int] | None,
    ) -> None:
        """首次扫描全图, 后续只吸收局部节点的 Frontier 和探索半径变化"""
        if not self._index_initialized:
            candidate_nodes = graph.nodes.values()
            self._index_initialized = True
        else:
            candidate_ids = graph.nodes.keys() if local_node_ids is None else local_node_ids
            candidate_nodes = (
                graph.nodes[node_id]
                for node_id in candidate_ids
                if node_id in graph.nodes
            )

        self._active_owner_ids.intersection_update(graph.nodes)
        for node in candidate_nodes:
            if node.is_frontier or node.frontier_points:
                self._active_owner_ids.add(node.node_id)
            if isfinite(node.explored_radius):
                self._max_explored_radius = max(
                    self._max_explored_radius,
                    node.explored_radius,
                )

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
        cells = np.asarray(frontier_cells, dtype=np.int64)
        if cells.size == 0:
            return []
        buckets = cells // bucket_cells
        _, first_indices = np.unique(buckets, axis=0, return_index=True)
        return [
            (int(cells[index, 0]), int(cells[index, 1]))
            for index in np.sort(first_indices)
        ]

    def _is_frontier_index(self, grid: ClassifiedGrid, ix: int, iy: int) -> bool:
        """优先复用当前帧向量化 Frontier mask"""
        if (
            self._latest_frontier_grid_id == id(grid)
            and self._latest_frontier_mask is not None
        ):
            return grid.in_bounds(ix, iy) and bool(
                self._latest_frontier_mask[iy, ix]
            )
        return (
            grid.is_free_index(ix, iy)
            and not self._near_grid_border(grid, ix, iy)
            and self._touches_unknown(grid, ix, iy)
        )

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


class _ExploredAreaIndex:
    """查询 Frontier 是否已被持久图节点的探索半径覆盖"""

    def __init__(
        self,
        graph: GraphState,
        resolution: float,
        max_radius: float,
    ) -> None:
        self.graph = graph
        self.resolution = max(0.0, float(resolution))
        self.max_radius = max(self.resolution, float(max_radius))

    def contains(self, point: Point3, excluded_node_id: int | None = None) -> bool:
        """只扫描附近节点, owner 可排除以保留自身仍有效的边界"""
        for node_id in self.graph.node_ids_within(point, self.max_radius):
            if node_id == excluded_node_id:
                continue
            node = self.graph.nodes.get(node_id)
            if node is None or not isfinite(node.explored_radius):
                continue
            radius = max(0.0, node.explored_radius - self.resolution)
            if radius <= 0.0:
                continue
            if node.distance_xy(point) < radius:
                return True
        return False


class _VisitedTrajectoryIndex:
    """查询 Frontier 是否落入机器人已走过的轨迹走廊"""

    def __init__(self, points: Iterable[Point3], radius: float) -> None:
        self.radius = max(0.0, float(radius))
        self.bucket_size = max(self.radius, 0.01)
        self.buckets: Dict[Tuple[int, int], List[Point3]] = {}
        self._point_count = 0
        self.update(list(points))

    def update(self, points: Sequence[Point3]) -> None:
        """只把新轨迹点追加到持久桶索引"""
        if self.radius <= 0.0:
            self._point_count = len(points)
            return
        if len(points) < self._point_count:
            self.buckets.clear()
            self._point_count = 0
        for point in points[self._point_count:]:
            self.buckets.setdefault(self._key(point), []).append(point)
        self._point_count = len(points)

    def contains(self, point: Point3) -> bool:
        """只检查相邻轨迹桶, 避免随轨迹增长线性扫描"""
        if self.radius <= 0.0:
            return False
        center_x, center_y = self._key(point)
        for by in range(center_y - 1, center_y + 2):
            for bx in range(center_x - 1, center_x + 2):
                for visited in self.buckets.get((bx, by), ()):
                    if hypot(visited[0] - point[0], visited[1] - point[1]) <= self.radius:
                        return True
        return False

    def _key(self, point: Point3) -> Tuple[int, int]:
        return (
            int(point[0] // self.bucket_size),
            int(point[1] // self.bucket_size),
        )
