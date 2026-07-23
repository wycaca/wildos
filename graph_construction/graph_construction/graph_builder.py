from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot
import math
from time import perf_counter
from typing import Tuple

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from graph_construction.edge_builder import EdgeBuilder
from graph_construction.frontier_detector import FrontierDetector
from graph_construction.graph_memory import GraphState
from graph_construction.grid_types import ClassifiedGrid, distance_to_mask


@dataclass
class GraphBuilderConfig:
    """图构建运行参数

    这些参数对应论文中的局部地图半径, 节点采样间距, free radius, edge radius 等概念
    """

    # 过滤局部高程尖峰和帘状面噪声
    grid_map_max_surface_step: float = 0.35

    # 用周边可靠地面修补机器人脚下的雷达盲区
    robot_blind_zone_radius: float = 4.0
    robot_blind_zone_elevation_search_radius: float = 6.0
    robot_ground_height_offset: float = 0.22
    robot_ground_elevation_tolerance: float = 0.5
    robot_blind_zone_initial_only: bool = True

    sample_stride: int = 8
    min_node_separation: float = 1.0
    max_free_radius: float = 4.0
    min_obstacle_clearance: float = 0.5
    edge_radius: float = 8.0
    max_edge_neighbors: int = 6
    current_node_max_edge_neighbors: int = 10
    max_edge_candidates_per_node: int = 24
    frontier_assign_radius: float = 5.0
    frontier_min_points: int = 4
    frontier_min_span: float = 0.6
    frontier_border_margin: float = 0.8
    frontier_candidate_spacing: float = 0.0
    frontier_visited_corridor_radius: float = 0.65


@dataclass
class GraphUpdateResult:
    """返回给 ROS 适配层的纯图更新结果"""

    graph: GraphState
    classified_grid: ClassifiedGrid
    stats: GraphUpdateStats


@dataclass
class GraphUpdateStats:
    """记录单次局部更新工作量和阶段耗时"""

    local_node_count: int = 0
    total_node_count: int = 0
    affected_edge_count: int = 0
    total_edge_count: int = 0
    dirty_cell_count: int = 0
    newly_free_cell_count: int = 0
    newly_obstacle_cell_count: int = 0
    edge_rebuild_node_count: int = 0
    newly_free_rebuild_node_count: int = 0
    obstacle_affected_edge_count: int = 0
    edge_candidate_pair_count: int = 0
    edge_clearance_check_count: int = 0
    historical_edge_check_count: int = 0
    frontier_candidate_count: int = 0
    active_frontier_owner_count: int = 0
    stage_seconds: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class _GridStateSnapshot:
    """保存上一帧分类栅格及其世界坐标信息"""

    state: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    center_x: float | None
    center_y: float | None
    length_x: float | None
    length_y: float | None
    yaw: float
    convention: bool


@dataclass(frozen=True)
class _GridChanges:
    """区分地图变化类型, 避免用同一策略重建全部附近边"""

    dirty: np.ndarray
    newly_free: np.ndarray
    newly_obstacle: np.ndarray


@dataclass(frozen=True)
class _BlindZoneRepair:
    """记录本帧人工初始化的 free cell"""

    cells: np.ndarray
    complete: bool


class SparseGraphBuilder:
    """从局部栅格增量构建稀疏 NavigationGraph

    主流程保持和论文 Algorithm 1 一致
    先更新已有节点, 再采样新节点, 再检测 frontier, 最后重建边和 current node
    输出的 NavigationGraph 会被 WildOS scoring 继续加工为 scored_nav_graph
    """

    def __init__(self, config: GraphBuilderConfig) -> None:
        self.config = config
        self.graph = GraphState()
        self.frontier_detector = FrontierDetector(
            frontier_assign_radius=config.frontier_assign_radius,
            frontier_min_points=config.frontier_min_points,
            frontier_min_span=config.frontier_min_span,
            frontier_border_margin=config.frontier_border_margin,
            frontier_candidate_spacing=config.frontier_candidate_spacing,
            frontier_visited_corridor_radius=config.frontier_visited_corridor_radius,
        )
        self.edge_builder = EdgeBuilder(
            edge_radius=config.edge_radius,
            max_neighbors_per_node=config.max_edge_neighbors,
            current_node_max_neighbors=config.current_node_max_edge_neighbors,
            max_candidates_per_node=config.max_edge_candidates_per_node,
        )
        self._previous_grid_state: _GridStateSnapshot | None = None
        self._blind_zone_initialization_complete = False

    def update(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        stamp_seconds: float,
    ) -> GraphUpdateResult:
        """根据已解码 grid 和机器人位置更新稀疏图"""
        return self._update_classified_grid(grid, robot_position, stamp_seconds)

    def _sanitize_grid_surface(self, grid: ClassifiedGrid) -> np.ndarray:
        """清洗孤立高程尖峰, 避免 graph 采到帘状面噪声"""
        protected_unknown = np.zeros((grid.height, grid.width), dtype=bool)
        if grid.elevation is None:
            return protected_unknown

        max_step = self.config.grid_map_max_surface_step
        if max_step <= 0.0:
            return protected_unknown

        elevation = np.asarray(grid.elevation)
        height, width = elevation.shape
        neighbors = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.full((height, width), np.inf, dtype=elevation.dtype)
                src_y, dst_y = _shift_slices(height, dy)
                src_x, dst_x = _shift_slices(width, dx)
                values = elevation[src_y, src_x]
                shifted[dst_y, dst_x] = np.where(np.isfinite(values), values, np.inf)
                neighbors.append(shifted)

        neighbor_values = np.stack(neighbors)
        neighbor_values.sort(axis=0)
        finite_count = np.sum(np.isfinite(neighbor_values), axis=0)
        median_index = np.minimum(finite_count // 2, len(neighbors) - 1)
        upper_median = np.take_along_axis(
            neighbor_values,
            median_index[np.newaxis, ...],
            axis=0,
        )[0]
        curtain = (
            np.isfinite(elevation)
            & (finite_count >= 3)
            & (np.abs(elevation - upper_median) > max_step)
        )
        grid.elevation[curtain] = math.nan
        grid.unknown[curtain] = True
        grid.free[curtain] = False
        grid.obstacle[curtain] = False
        return curtain

    def _update_classified_grid(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        stamp_seconds: float,
    ) -> GraphUpdateResult:
        stage_seconds: dict[str, float] = {}
        previous_current_id = self.graph.current_node_id
        previous_current_position = None
        if previous_current_id in self.graph.nodes:
            previous_current_position = self.graph.nodes[previous_current_id].position
        first_new_node_id = self.graph.next_node_id

        stage_started = perf_counter()
        protected_unknown = self._sanitize_grid_surface(grid)
        blind_zone_repair = self._apply_initial_blind_zone_repair(
            grid,
            robot_position,
            protected_unknown,
        )

        robot_ground_position, robot_ground_projected = (
            grid.project_to_elevation_with_status(robot_position)
        )
        if robot_ground_projected:
            self.graph.append_trajectory_point(robot_ground_position, 0.25)
        reachable_free = self._reachable_free_mask(grid, robot_ground_position)
        stage_seconds["preprocess"] = perf_counter() - stage_started

        stage_started = perf_counter()
        grid_changes = self._grid_change_masks(
            grid,
            ignored_new_free=blind_zone_repair.cells,
        )
        dirty_cells = grid_changes.dirty
        newly_free_cells = grid_changes.newly_free
        stage_seconds["dirty"] = perf_counter() - stage_started

        stage_started = perf_counter()
        # 距离场用于节点 clearance 和 frontier 生命周期判断
        sdf_obstacle = distance_to_mask(grid.obstacle, grid.resolution)
        # rolling GridMap 外部必须视为 unknown, 否则全 known 局部图会产生无限探索半径
        sdf_unknown = distance_to_mask(
            grid.unknown,
            grid.resolution,
            include_grid_exterior=True,
        )
        stage_seconds["distance"] = perf_counter() - stage_started

        # 后续阶段共享同一局部查询结果, 避免重复扫描历史节点
        stage_started = perf_counter()
        local_node_ids = self._node_ids_in_grid(grid)
        topology_dirty_node_ids = self._update_existing_nodes(
            grid,
            sdf_obstacle,
            sdf_unknown,
            stamp_seconds,
            local_node_ids,
        )
        stage_seconds["nodes"] = perf_counter() - stage_started

        # 在当前观测到的 free 区域补充稀疏节点
        stage_started = perf_counter()
        new_node_ids = self._sample_new_nodes(
            grid,
            sdf_obstacle,
            sdf_unknown,
            stamp_seconds,
            reachable_free,
        )
        stage_seconds["sampling"] = perf_counter() - stage_started

        stage_started = perf_counter()
        fallback_node_id = self._update_current_node(
            grid,
            robot_position,
            robot_ground_position,
            robot_ground_projected,
            stamp_seconds,
            sdf_obstacle,
            sdf_unknown,
        )
        if fallback_node_id is not None:
            new_node_ids.add(fallback_node_id)
        stage_seconds["current"] = perf_counter() - stage_started

        # frontier cell 需要绑定到附近可用图节点
        stage_started = perf_counter()
        frontier_refresh_region = ndimage.binary_dilation(
            dirty_cells,
            structure=np.ones((3, 3), dtype=bool),
            border_value=0,
        )
        if new_node_ids and not np.all(frontier_refresh_region):
            frontier_refresh_region |= self._node_frontier_refresh_region(
                grid,
                new_node_ids,
            )
        frontier_cells = self.frontier_detector.detect_frontier_cells(
            grid,
            candidate_region=frontier_refresh_region,
        )
        local_node_ids = self._node_ids_in_grid(grid)
        self.frontier_detector.assign_frontiers(
            self.graph,
            grid,
            frontier_cells,
            local_node_ids=local_node_ids,
        )
        stage_seconds["frontier"] = perf_counter() - stage_started

        # 新 free 局部重连旧节点, 新障碍只复查实际经过附近的历史边
        stage_started = perf_counter()
        local_node_ids = self._node_ids_in_grid(grid)
        edge_rebuild_node_ids = set(topology_dirty_node_ids)
        edge_rebuild_node_ids.update(
            node_id
            for node_id in range(first_new_node_id, self.graph.next_node_id)
            if node_id in self.graph.nodes
        )
        edge_rebuild_node_ids.update(
            self._current_node_edge_neighborhood(
                previous_current_id,
                previous_current_position,
            )
        )
        newly_free_rebuild_node_ids = self._node_ids_near_newly_free(
            grid,
            newly_free_cells,
            first_new_node_id,
            local_node_ids,
        )
        edge_rebuild_node_ids.update(newly_free_rebuild_node_ids)
        obstacle_affected_edge_keys = self._edge_keys_near_obstacles(
            grid,
            grid_changes.newly_obstacle,
        )
        affected_edge_keys = self.graph.edge_keys_for_nodes(
            edge_rebuild_node_ids
        )
        affected_edge_keys.update(obstacle_affected_edge_keys)
        if edge_rebuild_node_ids:
            next_edges = self.edge_builder.build_edges(
                self.graph,
                grid,
                sdf_obstacle,
                sdf_unknown,
                self.config.min_obstacle_clearance,
                node_ids=local_node_ids,
                focus_node_ids=edge_rebuild_node_ids,
            )
        else:
            self.edge_builder.reset_stats()
            next_edges = []
        next_edges = self.edge_builder.merge_historical_edges(
            self.graph,
            next_edges,
            grid,
            sdf_obstacle,
            self.config.min_obstacle_clearance,
            historical_edge_keys=affected_edge_keys,
        )
        self.graph.replace_edges(affected_edge_keys, next_edges)
        stage_seconds["edges"] = perf_counter() - stage_started
        self._previous_grid_state = _snapshot_grid_state(grid)

        return GraphUpdateResult(
            graph=self.graph,
            classified_grid=grid,
            stats=GraphUpdateStats(
                local_node_count=len(local_node_ids),
                total_node_count=len(self.graph.nodes),
                affected_edge_count=len(affected_edge_keys),
                total_edge_count=len(self.graph.edges),
                dirty_cell_count=int(np.count_nonzero(dirty_cells)),
                newly_free_cell_count=int(np.count_nonzero(newly_free_cells)),
                newly_obstacle_cell_count=int(
                    np.count_nonzero(grid_changes.newly_obstacle)
                ),
                edge_rebuild_node_count=len(edge_rebuild_node_ids),
                newly_free_rebuild_node_count=len(
                    newly_free_rebuild_node_ids
                ),
                obstacle_affected_edge_count=len(obstacle_affected_edge_keys),
                edge_candidate_pair_count=(
                    self.edge_builder.last_stats.candidate_pair_count
                ),
                edge_clearance_check_count=(
                    self.edge_builder.last_stats.clearance_check_count
                ),
                historical_edge_check_count=(
                    self.edge_builder.last_stats.historical_check_count
                ),
                frontier_candidate_count=(
                    self.frontier_detector.last_candidate_count
                ),
                active_frontier_owner_count=(
                    self.frontier_detector.active_owner_count
                ),
                stage_seconds=stage_seconds,
            ),
        )

    def _grid_change_masks(
        self,
        grid: ClassifiedGrid,
        ignored_new_free: np.ndarray | None = None,
    ) -> _GridChanges:
        """返回全部变化、新 free 和新 obstacle 三类 cell

        初始化人工填充仍属于地图变化, 但不能触发传感器新 free 的旧边重建
        """
        current_state = _grid_state_codes(grid)
        snapshot = self._previous_grid_state
        if snapshot is None:
            dirty = np.ones((grid.height, grid.width), dtype=bool)
            newly_free = current_state == 1
            if ignored_new_free is not None:
                newly_free &= ~ignored_new_free
            return _GridChanges(
                dirty=dirty,
                newly_free=newly_free,
                newly_obstacle=current_state == 2,
            )
        if (
            snapshot.frame_id != grid.frame_id
            or snapshot.convention != grid.grid_map_convention
            or not math.isclose(snapshot.resolution, grid.resolution, abs_tol=1e-6)
        ):
            dirty = np.ones((grid.height, grid.width), dtype=bool)
            newly_free = current_state == 1
            if ignored_new_free is not None:
                newly_free &= ~ignored_new_free
            return _GridChanges(
                dirty=dirty,
                newly_free=newly_free,
                newly_obstacle=current_state == 2,
            )

        world_x, world_y = _grid_world_coordinates(grid)
        previous_x, previous_y, valid = _world_to_snapshot_indices(
            snapshot,
            world_x,
            world_y,
        )
        dirty = ~valid
        newly_free = ~valid & (current_state == 1)
        newly_obstacle = ~valid & (current_state == 2)
        if np.any(valid):
            previous_state = snapshot.state[previous_y[valid], previous_x[valid]]
            dirty[valid] = current_state[valid] != previous_state
            newly_free[valid] = (
                (current_state[valid] == 1)
                & (previous_state != 1)
            )
            newly_obstacle[valid] = (
                (current_state[valid] == 2)
                & (previous_state != 2)
            )
        if ignored_new_free is not None:
            newly_free &= ~ignored_new_free
        return _GridChanges(dirty, newly_free, newly_obstacle)

    def _apply_initial_blind_zone_repair(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        protected_unknown: np.ndarray,
    ) -> _BlindZoneRepair:
        """只在初始化阶段填充机器人脚下盲区"""
        empty = np.zeros((grid.height, grid.width), dtype=bool)
        if (
            self.config.robot_blind_zone_initial_only
            and self._blind_zone_initialization_complete
        ):
            if grid.stats is None:
                grid.stats = {}
            grid.stats["robot_blind_zone_filled"] = 0
            grid.stats["robot_blind_zone_artificial_free"] = 0
            grid.stats["robot_blind_zone_status"] = "initial_only_complete"
            return _BlindZoneRepair(empty, True)

        repaired_cells = self._repair_robot_blind_zone(
            grid,
            robot_position,
            protected_unknown,
        )
        status = str(grid.stats.get("robot_blind_zone_status", "unknown"))
        complete = status in {
            "center_obstacle",
            "disabled",
            "known_ground",
            "repaired",
        }
        if self.config.robot_blind_zone_initial_only and complete:
            self._blind_zone_initialization_complete = True
        return _BlindZoneRepair(repaired_cells, complete)

    def _node_ids_near_newly_free(
        self,
        grid: ClassifiedGrid,
        newly_free: np.ndarray,
        first_new_node_id: int,
        local_node_ids: set[int],
    ) -> set[int]:
        """局部查找可能因 unknown 变 free 而新增边的旧节点

        一条边受新增 free cell 影响时, 至少一个端点靠近该 cell
        先用持久空间索引缩小范围, 再用 KDTree 精确过滤
        """
        if first_new_node_id <= 0 or not np.any(newly_free):
            return set()

        world_x, world_y = _grid_world_coordinates(grid)
        points = np.column_stack(
            (world_x[newly_free], world_y[newly_free])
        )
        influence_radius = sum(
            (
                0.5 * self.config.edge_radius,
                self.config.min_obstacle_clearance,
                math.sqrt(2.0) * grid.resolution,
            )
        )
        candidate_ids = self.graph.node_ids_in_bounds(
            float(np.min(points[:, 0])) - influence_radius,
            float(np.max(points[:, 0])) + influence_radius,
            float(np.min(points[:, 1])) - influence_radius,
            float(np.max(points[:, 1])) + influence_radius,
        )
        point_tree = cKDTree(points)
        affected_node_ids = set()
        for node_id in candidate_ids:
            if node_id >= first_new_node_id or node_id not in local_node_ids:
                continue
            node = self.graph.nodes.get(node_id)
            if node is None:
                continue
            distance, _ = point_tree.query(
                node.position[:2],
                distance_upper_bound=influence_radius,
            )
            if math.isfinite(float(distance)):
                affected_node_ids.add(node_id)
        return affected_node_ids

    def _edge_keys_near_obstacles(
        self,
        grid: ClassifiedGrid,
        newly_obstacle: np.ndarray,
    ) -> set[tuple[int, int]]:
        """用持久边空间索引找出新障碍附近可能冲突的历史边"""
        if not np.any(newly_obstacle):
            return set()
        world_x, world_y = _grid_world_coordinates(grid)
        points = np.column_stack(
            (world_x[newly_obstacle], world_y[newly_obstacle])
        )
        query_radius = self.config.min_obstacle_clearance + math.sqrt(2.0) * grid.resolution
        candidate_keys = self.graph.edge_keys_near_points(points, query_radius)
        return {
            edge_key
            for edge_key in candidate_keys
            if _edge_is_near_points(
                self.graph,
                edge_key,
                points,
                query_radius,
            )
        }

    def _current_node_edge_neighborhood(
        self,
        previous_node_id: int | None,
        previous_position: Tuple[float, float, float] | None,
    ) -> set[int]:
        """在 current node 切换时刷新新旧节点的局部连边"""
        current_node_id = self.graph.current_node_id
        current_node = self.graph.nodes.get(current_node_id)
        current_position = current_node.position if current_node is not None else None
        position_changed = (
            previous_position is None
            or current_position is None
            or hypot(
                previous_position[0] - current_position[0],
                previous_position[1] - current_position[1],
            ) > 1e-6
        )
        if previous_node_id == current_node_id and not position_changed:
            return set()

        node_ids = set()
        if previous_node_id in self.graph.nodes:
            node_ids.add(previous_node_id)
        if current_node_id in self.graph.nodes:
            node_ids.add(current_node_id)
        return node_ids

    def _repair_robot_blind_zone(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        protected_unknown: np.ndarray | None = None,
    ) -> np.ndarray:
        """只修补机器人可物理占用的脚下 unknown 区域

        高程取自盲区边缘最近的可靠地面中位数, 障碍物和本帧突变地形不会被清除
        """
        if grid.stats is None:
            grid.stats = {}
        repaired_cells = np.zeros((grid.height, grid.width), dtype=bool)
        grid.stats["robot_blind_zone_filled"] = 0
        grid.stats["robot_blind_zone_artificial_free"] = 0
        if grid.elevation is None or self.config.robot_blind_zone_radius <= 0.0:
            grid.stats["robot_blind_zone_status"] = "disabled"
            return repaired_cells

        center = grid.world_to_grid(robot_position[0], robot_position[1])
        if center is None:
            grid.stats["robot_blind_zone_status"] = "outside_grid"
            return repaired_cells
        if grid.is_obstacle_index(center[0], center[1]):
            grid.stats["robot_blind_zone_status"] = "center_obstacle"
            return repaired_cells

        resolution = max(grid.resolution, 1e-6)
        search_radius = max(
            self.config.robot_blind_zone_radius,
            self.config.robot_blind_zone_elevation_search_radius,
        )
        search_cells = max(1, int(math.ceil(search_radius / resolution)))
        center_x, center_y = center
        min_x = max(0, center_x - search_cells)
        max_x = min(grid.width, center_x + search_cells + 1)
        min_y = max(0, center_y - search_cells)
        max_y = min(grid.height, center_y + search_cells + 1)
        offset_y, offset_x = np.ogrid[
            min_y - center_y:max_y - center_y,
            min_x - center_x:max_x - center_x,
        ]
        distances = np.hypot(offset_x * resolution, offset_y * resolution)
        local_elevation = grid.elevation[min_y:max_y, min_x:max_x]
        sample_mask = (
            (distances <= search_radius)
            & ~grid.obstacle[min_y:max_y, min_x:max_x]
            & np.isfinite(local_elevation)
        )
        sample_distances = distances[sample_mask]
        sample_elevations = local_elevation[sample_mask]
        order = np.argsort(sample_distances)
        sample_distances = sample_distances[order]
        sample_elevations = sample_elevations[order]
        expected_ground = robot_position[2] - self.config.robot_ground_height_offset
        plausible = (
            np.abs(sample_elevations + grid.z_offset - expected_ground)
            <= self.config.robot_ground_elevation_tolerance
        )
        ground_source = "map"
        if np.any(plausible):
            sample_distances = sample_distances[plausible]
            sample_elevations = sample_elevations[plausible]
        else:
            sample_distances = np.asarray([0.0])
            sample_elevations = np.asarray([expected_ground - grid.z_offset])
            ground_source = "odom"

        nearest_distance = float(sample_distances[0])
        grid.stats["robot_blind_zone_nearest_ground"] = round(nearest_distance, 3)
        grid.stats["robot_blind_zone_ground_source"] = ground_source
        sample_band = nearest_distance + max(0.4, 2.0 * resolution)
        ground_samples = sample_elevations[sample_distances <= sample_band][:32]
        ground_elevation = float(np.median(ground_samples))

        repair_cells = max(
            1,
            int(math.ceil(self.config.robot_blind_zone_radius / resolution)),
        )
        repair_min_x = max(0, center_x - repair_cells)
        repair_max_x = min(grid.width, center_x + repair_cells + 1)
        repair_min_y = max(0, center_y - repair_cells)
        repair_max_y = min(grid.height, center_y + repair_cells + 1)
        repair_offset_y, repair_offset_x = np.ogrid[
            repair_min_y - center_y:repair_max_y - center_y,
            repair_min_x - center_x:repair_max_x - center_x,
        ]
        repair_region = np.hypot(
            repair_offset_x * resolution,
            repair_offset_y * resolution,
        ) <= self.config.robot_blind_zone_radius
        region_unknown = grid.unknown[
            repair_min_y:repair_max_y,
            repair_min_x:repair_max_x,
        ]
        region_obstacle = grid.obstacle[
            repair_min_y:repair_max_y,
            repair_min_x:repair_max_x,
        ]
        repair_mask = repair_region & region_unknown & ~region_obstacle
        if protected_unknown is not None:
            repair_mask &= ~protected_unknown[
                repair_min_y:repair_max_y,
                repair_min_x:repair_max_x,
            ]
        repaired = int(np.count_nonzero(repair_mask))
        repaired_cells[
            repair_min_y:repair_max_y,
            repair_min_x:repair_max_x,
        ] = repair_mask
        region_unknown[repair_mask] = False
        grid.free[
            repair_min_y:repair_max_y,
            repair_min_x:repair_max_x,
        ][repair_mask] = True
        grid.elevation[
            repair_min_y:repair_max_y,
            repair_min_x:repair_max_x,
        ][repair_mask] = ground_elevation

        if repaired > 0:
            grid.stats["robot_blind_zone_filled"] = repaired
            grid.stats["robot_blind_zone_artificial_free"] = repaired
            grid.stats["robot_blind_zone_status"] = "repaired"
            grid.stats["free"] = int(np.count_nonzero(grid.free))
            grid.stats["unknown"] = int(np.count_nonzero(grid.unknown))
        else:
            grid.stats["robot_blind_zone_status"] = "known_ground"
        return repaired_cells

    def _update_existing_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
        local_node_ids: set[int],
    ) -> set[int]:
        """用可靠局部观测刷新历史节点, unknown 和窗口外区域不否定记忆"""
        topology_dirty_node_ids = set()
        for node_id in tuple(local_node_ids):
            node = self.graph.nodes.get(node_id)
            if node is None:
                continue
            # 旧进程可能已保存非有限半径, 该值不能表达全局探索覆盖
            if not math.isfinite(node.explored_radius) or node.explored_radius < 0.0:
                node.explored_radius = 0.0
            grid_index = grid.world_to_grid(node.position[0], node.position[1])
            if grid_index is None:
                continue

            ix, iy = grid_index
            if grid.is_obstacle_index(ix, iy):
                for edge_key in self.graph.adjacency.get(node_id, ()):
                    topology_dirty_node_ids.update(edge_key)
                self.graph.remove_node(node_id)
                topology_dirty_node_ids.discard(node_id)
                continue
            if not grid.is_free_index(ix, iy):
                continue

            free_radius = min(
                float(sdf_obstacle[iy, ix]),
                float(sdf_unknown[iy, ix]),
                self.config.max_free_radius,
            )

            # min_obstacle_clearance 是新节点和新边的部署安全阈值
            # 历史节点只在自由圆完全消失时删除, 避免局部地图噪声擦除已走过路线
            if free_radius <= 0.0:
                for edge_key in self.graph.adjacency.get(node_id, ()):
                    topology_dirty_node_ids.update(edge_key)
                self.graph.remove_node(node_id)
                topology_dirty_node_ids.discard(node_id)
                continue

            node.free_radius = free_radius
            node.explored_radius = max(node.explored_radius, float(sdf_unknown[iy, ix]))
            surface_z = grid.elevation_at_world(node.position[0], node.position[1])
            if surface_z is not None:
                self.graph.move_node(
                    node_id,
                    (node.position[0], node.position[1], surface_z),
                )
            node.last_seen_time = stamp_seconds
        return topology_dirty_node_ids

    def _sample_new_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
        reachable_free: np.ndarray,
    ) -> set[int]:
        """在当前可达 free 区域补充分层世界网格节点

        最细网格由 sample stride 决定, free radius 越大则选择越粗的嵌套层级
        所有层级共享世界坐标锚点, rolling GridMap 移动时不会改变节点排列
        每帧重查固定局部采样点, 避免已知 free 区域后来可达时漏建节点
        """
        new_node_ids: set[int] = set()
        stride = max(1, int(self.config.sample_stride))
        base_spacing = max(grid.resolution, stride * grid.resolution)
        lattice_offset = grid.resolution * 0.5
        min_key_x, max_key_x, min_key_y, max_key_y = _world_lattice_bounds(
            grid,
            base_spacing,
            lattice_offset,
        )
        for lattice_y in range(min_key_y, max_key_y + 1):
            for lattice_x in range(min_key_x, max_key_x + 1):
                world_x = lattice_x * base_spacing + lattice_offset
                world_y = lattice_y * base_spacing + lattice_offset
                grid_index = grid.world_to_grid(world_x, world_y)
                if grid_index is None:
                    continue
                ix, iy = grid_index
                if not grid.is_free_index(ix, iy):
                    continue
                if not reachable_free[iy, ix]:
                    continue
                if float(sdf_obstacle[iy, ix]) < self.config.min_obstacle_clearance:
                    continue

                free_radius = min(
                    float(sdf_obstacle[iy, ix]),
                    float(sdf_unknown[iy, ix]),
                    self.config.max_free_radius,
                )
                lattice_multiple = _adaptive_lattice_multiple(
                    free_radius,
                    base_spacing,
                )
                if lattice_x % lattice_multiple != 0 or lattice_y % lattice_multiple != 0:
                    continue

                position = (world_x, world_y, grid.elevation_at_index(ix, iy))
                if self.graph.nearest_node(
                    position,
                    max_distance=self.config.min_node_separation,
                ) is not None:
                    continue

                node = self.graph.create_node(position=position, stamp_seconds=stamp_seconds)
                node.free_radius = free_radius
                node.explored_radius = float(sdf_unknown[iy, ix])
                new_node_ids.add(node.node_id)
        return new_node_ids

    def _node_frontier_refresh_region(
        self,
        grid: ClassifiedGrid,
        node_ids: set[int],
    ) -> np.ndarray:
        """刷新新节点分配半径内的 Frontier 候选"""
        refresh_region = np.zeros((grid.height, grid.width), dtype=bool)
        radius = max(0.0, float(self.config.frontier_assign_radius))
        radius_cells = max(
            0,
            int(math.ceil(radius / max(grid.resolution, 1e-6))),
        )
        for node_id in node_ids:
            node = self.graph.nodes.get(node_id)
            if node is None:
                continue
            center = grid.world_to_grid(node.position[0], node.position[1])
            if center is None:
                continue
            center_x, center_y = center
            min_x = max(0, center_x - radius_cells)
            max_x = min(grid.width, center_x + radius_cells + 1)
            min_y = max(0, center_y - radius_cells)
            max_y = min(grid.height, center_y + radius_cells + 1)
            offset_y, offset_x = np.ogrid[
                min_y - center_y:max_y - center_y,
                min_x - center_x:max_x - center_x,
            ]
            refresh_region[min_y:max_y, min_x:max_x] |= (
                np.hypot(offset_x, offset_y) * grid.resolution <= radius
            )
        return refresh_region

    def _update_current_node(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        robot_ground_position: Tuple[float, float, float],
        robot_ground_projected: bool,
        stamp_seconds: float,
        sdf_obstacle: np.ndarray,
        sdf_unknown: np.ndarray,
    ) -> int | None:
        """选择安全普通节点作为规划起点, 必要时创建 free 区兜底节点"""
        best_node = self._nearest_collision_free_node(grid, robot_ground_position)
        fallback_node_id = None
        if best_node is None:
            best_node = self._ensure_robot_fallback_node(
                grid,
                robot_ground_position,
                stamp_seconds,
                sdf_obstacle,
                sdf_unknown,
            )
            if best_node is not None:
                fallback_node_id = best_node.node_id
        self.graph.current_node_id = best_node.node_id if best_node is not None else None
        self.graph.update_robot_position(
            robot_position,
            robot_ground_position if robot_ground_projected else None,
        )
        return fallback_node_id

    def _ensure_robot_fallback_node(
        self,
        grid: ClassifiedGrid,
        position: Tuple[float, float, float],
        stamp_seconds: float,
        sdf_obstacle: np.ndarray,
        sdf_unknown: np.ndarray,
    ):
        """没有安全普通节点时, 只在已确认 free 的机器人位置创建普通节点"""
        grid_index = grid.world_to_grid(position[0], position[1])
        if grid_index is None:
            return None
        ix, iy = grid_index
        if not grid.is_free_index(ix, iy):
            return None
        clearance = min(
            float(sdf_obstacle[iy, ix]),
            float(sdf_unknown[iy, ix]),
            self.config.max_free_radius,
        )
        if clearance < self.config.min_obstacle_clearance:
            return None
        node = self.graph.create_node(
            position=position,
            stamp_seconds=stamp_seconds,
        )
        node.free_radius = clearance
        node.explored_radius = float(sdf_unknown[iy, ix])
        return node

    def _reachable_free_mask(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
    ) -> np.ndarray:
        """计算机器人脚下及盲区外围可安全采样的 free 分量

        外围分量只用于生成各自内部节点和边, 不允许跨 unknown 互连
        """
        reachable = np.zeros((grid.height, grid.width), dtype=bool)
        start = grid.world_to_grid(robot_position[0], robot_position[1])
        if start is None or grid.is_obstacle_index(start[0], start[1]):
            return reachable
        if not grid.is_free_index(start[0], start[1]):
            max_radius_cells = max(
                int(math.ceil(self.config.edge_radius / max(grid.resolution, 1e-6))),
                1,
            )
            start = self._nearest_free_neighbor(
                grid,
                start,
                max_radius_cells=max_radius_cells,
            )
        if start is None:
            return reachable

        labels, _ = ndimage.label(
            grid.free,
            structure=np.asarray(
                (
                    (0, 1, 0),
                    (1, 1, 1),
                    (0, 1, 0),
                ),
                dtype=np.uint8,
            ),
        )
        start_label = int(labels[start[1], start[0]])
        if start_label <= 0:
            return reachable
        reachable = labels == start_label

        # 同时采样附近所有无明确障碍隔断的外围 free 分量
        max_radius_cells = max(
            int(math.ceil(self.config.edge_radius / max(grid.resolution, 1e-6))),
            1,
        )
        nearby_labels = self._nearby_free_component_labels(
            grid,
            labels,
            start,
            max_radius_cells=max_radius_cells,
            excluded_label=start_label,
        )
        for outer_label in nearby_labels:
            reachable |= labels == outer_label
        return reachable

    def _nearby_free_component_labels(
        self,
        grid: ClassifiedGrid,
        labels: np.ndarray,
        start: tuple[int, int],
        max_radius_cells: int,
        excluded_label: int,
    ) -> set[int]:
        """返回搜索半径内无明确 obstacle 隔断的所有 free 分量

        先按连通分量聚合候选 cell, 再为每个分量寻找一条无 obstacle 视线
        unknown 只允许触发分量采样, 不会在分量之间生成边
        """
        start_x, start_y = start
        radius_squared = max_radius_cells * max_radius_cells
        component_candidates: dict[int, list[tuple[int, int, int]]] = {}
        for offset_y in range(-max_radius_cells, max_radius_cells + 1):
            for offset_x in range(-max_radius_cells, max_radius_cells + 1):
                distance_squared = offset_x * offset_x + offset_y * offset_y
                if distance_squared > radius_squared:
                    continue
                next_x = start_x + offset_x
                next_y = start_y + offset_y
                if not grid.in_bounds(next_x, next_y):
                    continue
                component_label = int(labels[next_y, next_x])
                if component_label <= 0 or component_label == excluded_label:
                    continue
                component_candidates.setdefault(component_label, []).append(
                    (distance_squared, next_x, next_y)
                )

        selected_labels: set[int] = set()
        start_world = grid.grid_to_world(start_x, start_y)[:2]
        for component_label, candidates in component_candidates.items():
            for _, next_x, next_y in sorted(candidates):
                line_cells = grid.world_line_cells_clipped(
                    start_world,
                    grid.grid_to_world(next_x, next_y)[:2],
                )
                if all(
                    not grid.is_obstacle_index(cell_x, cell_y)
                    for cell_x, cell_y in line_cells
                ):
                    selected_labels.add(component_label)
                    break
        return selected_labels

    def _nearest_free_neighbor(
        self,
        grid: ClassifiedGrid,
        start: tuple[int, int],
        max_radius_cells: int,
        excluded: np.ndarray | None = None,
    ) -> tuple[int, int] | None:
        """在 anchor 可连接范围内选择无明确障碍隔断的最近 free cell"""
        start_x, start_y = start
        candidates = []
        for offset_y in range(-max_radius_cells, max_radius_cells + 1):
            for offset_x in range(-max_radius_cells, max_radius_cells + 1):
                next_x = start_x + offset_x
                next_y = start_y + offset_y
                if not grid.is_free_index(next_x, next_y):
                    continue
                if excluded is not None and excluded[next_y, next_x]:
                    continue
                distance_sq = offset_x * offset_x + offset_y * offset_y
                candidates.append((distance_sq, next_x, next_y))
        for _, nearest_x, nearest_y in sorted(candidates):
            line_cells = grid.world_line_cells_clipped(
                grid.grid_to_world(start_x, start_y)[:2],
                grid.grid_to_world(nearest_x, nearest_y)[:2],
            )
            if all(
                not grid.is_obstacle_index(cell_x, cell_y)
                for cell_x, cell_y in line_cells
            ):
                return nearest_x, nearest_y
        return None

    def _nearest_collision_free_node(
        self,
        grid: ClassifiedGrid,
        position: Tuple[float, float, float],
    ):
        """优先选择和机器人之间直线无碰撞的最近节点"""
        candidate_ids = self.graph.node_ids_within(position, self.config.edge_radius)
        candidates = sorted(
            (
                self.graph.nodes[node_id]
                for node_id in candidate_ids
                if node_id in self.graph.nodes
                and grid.world_to_grid(
                    self.graph.nodes[node_id].position[0],
                    self.graph.nodes[node_id].position[1],
                )
                is not None
            ),
            key=lambda node: hypot(node.position[0] - position[0], node.position[1] - position[1]),
        )
        for node in candidates:
            if grid.is_world_collision_free(
                (position[0], position[1]),
                (node.position[0], node.position[1]),
            ):
                return node
        return None

    def _node_ids_in_grid(self, grid: ClassifiedGrid) -> set[int]:
        """粗筛 GridMap 包围盒后精确返回窗口内节点"""
        candidate_ids = self._node_ids_near_grid(grid, margin=0.0)
        return {
            node_id
            for node_id in candidate_ids
            if node_id in self.graph.nodes
            and grid.world_to_grid(
                self.graph.nodes[node_id].position[0],
                self.graph.nodes[node_id].position[1],
            )
            is not None
        }

    def _node_ids_near_grid(
        self,
        grid: ClassifiedGrid,
        margin: float,
    ) -> set[int]:
        """查询 GridMap 世界包围盒及安全边距内节点"""
        corners = [
            grid.grid_to_world(ix, iy)
            for ix in (0, grid.width - 1)
            for iy in (0, grid.height - 1)
        ]
        safe_margin = max(0.0, float(margin)) + grid.resolution
        return self.graph.node_ids_in_bounds(
            min(point[0] for point in corners) - safe_margin,
            max(point[0] for point in corners) + safe_margin,
            min(point[1] for point in corners) - safe_margin,
            max(point[1] for point in corners) + safe_margin,
        )


def _edge_is_near_points(
    graph: GraphState,
    edge_key: tuple[int, int],
    points: np.ndarray,
    radius: float,
) -> bool:
    """精确检查边中心线是否进入任一变化 cell 的安全影响范围"""
    edge = graph.edges.get(edge_key)
    if edge is None or points.size == 0:
        return False
    node_a = graph.nodes.get(edge.from_id)
    node_b = graph.nodes.get(edge.to_id)
    if node_a is None or node_b is None:
        return False
    start_x, start_y = node_a.position[:2]
    delta_x = node_b.position[0] - start_x
    delta_y = node_b.position[1] - start_y
    length_squared = delta_x * delta_x + delta_y * delta_y
    radius_squared = max(0.0, float(radius)) ** 2
    for point_x, point_y in points:
        relative_x = float(point_x) - start_x
        relative_y = float(point_y) - start_y
        if length_squared <= 1e-12:
            distance_squared = relative_x * relative_x + relative_y * relative_y
        else:
            ratio = min(
                max(
                    (relative_x * delta_x + relative_y * delta_y) / length_squared,
                    0.0,
                ),
                1.0,
            )
            nearest_x = start_x + ratio * delta_x
            nearest_y = start_y + ratio * delta_y
            distance_x = float(point_x) - nearest_x
            distance_y = float(point_y) - nearest_y
            distance_squared = distance_x * distance_x + distance_y * distance_y
        if distance_squared <= radius_squared:
            return True
    return False


def _snapshot_grid_state(grid: ClassifiedGrid) -> _GridStateSnapshot:
    """复制增量更新需要的轻量分类快照"""
    return _GridStateSnapshot(
        state=_grid_state_codes(grid),
        resolution=float(grid.resolution),
        origin_x=float(grid.origin_x),
        origin_y=float(grid.origin_y),
        frame_id=grid.frame_id,
        center_x=grid.grid_map_center_x,
        center_y=grid.grid_map_center_y,
        length_x=grid.grid_map_length_x,
        length_y=grid.grid_map_length_y,
        yaw=float(grid.grid_map_yaw),
        convention=bool(grid.grid_map_convention),
    )


def _grid_state_codes(grid: ClassifiedGrid) -> np.ndarray:
    """将分类数组压缩为可直接比较的单字节状态"""
    state = np.zeros((grid.height, grid.width), dtype=np.uint8)
    state[grid.free] = 1
    state[grid.obstacle] = 2
    return state


def _grid_world_coordinates(grid: ClassifiedGrid) -> tuple[np.ndarray, np.ndarray]:
    """一次计算当前栅格所有 cell 的世界坐标"""
    index_y, index_x = np.indices((grid.height, grid.width), dtype=float)
    if not grid.grid_map_convention:
        return (
            grid.origin_x + (index_x + 0.5) * grid.resolution,
            grid.origin_y + (index_y + 0.5) * grid.resolution,
        )

    length_x = grid.grid_map_length_x or grid.height * grid.resolution
    length_y = grid.grid_map_length_y or grid.width * grid.resolution
    local_x = length_x * 0.5 - (index_y + 0.5) * grid.resolution
    local_y = length_y * 0.5 - (index_x + 0.5) * grid.resolution
    center_x = grid.grid_map_center_x if grid.grid_map_center_x is not None else grid.origin_x
    center_y = grid.grid_map_center_y if grid.grid_map_center_y is not None else grid.origin_y
    yaw_cos = math.cos(grid.grid_map_yaw)
    yaw_sin = math.sin(grid.grid_map_yaw)
    return (
        center_x + yaw_cos * local_x - yaw_sin * local_y,
        center_y + yaw_sin * local_x + yaw_cos * local_y,
    )


def _world_to_snapshot_indices(
    snapshot: _GridStateSnapshot,
    world_x: np.ndarray,
    world_y: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """将世界坐标批量映射到上一帧 cell"""
    if snapshot.convention:
        center_x = snapshot.center_x if snapshot.center_x is not None else snapshot.origin_x
        center_y = snapshot.center_y if snapshot.center_y is not None else snapshot.origin_y
        dx = world_x - center_x
        dy = world_y - center_y
        yaw_cos = math.cos(snapshot.yaw)
        yaw_sin = math.sin(snapshot.yaw)
        local_x = yaw_cos * dx + yaw_sin * dy
        local_y = -yaw_sin * dx + yaw_cos * dy
        length_x = snapshot.length_x or snapshot.state.shape[0] * snapshot.resolution
        length_y = snapshot.length_y or snapshot.state.shape[1] * snapshot.resolution
        index_y = np.floor((length_x * 0.5 - local_x) / snapshot.resolution)
        index_x = np.floor((length_y * 0.5 - local_y) / snapshot.resolution)
    else:
        index_x = np.floor((world_x - snapshot.origin_x) / snapshot.resolution)
        index_y = np.floor((world_y - snapshot.origin_y) / snapshot.resolution)

    index_x = index_x.astype(np.int64)
    index_y = index_y.astype(np.int64)
    valid = (
        (index_x >= 0)
        & (index_y >= 0)
        & (index_x < snapshot.state.shape[1])
        & (index_y < snapshot.state.shape[0])
    )
    return index_x, index_y, valid


def _shift_slices(size: int, offset: int) -> tuple[slice, slice]:
    """返回数组平移使用的源和目标切片"""
    if offset < 0:
        return slice(-offset, size), slice(0, size + offset)
    if offset > 0:
        return slice(0, size - offset), slice(offset, size)
    return slice(0, size), slice(0, size)


def _adaptive_lattice_multiple(free_radius: float, base_spacing: float) -> int:
    """选择不超过局部自由半径的二次幂网格倍数"""
    safe_base_spacing = max(float(base_spacing), 1e-6)
    target_spacing = max(safe_base_spacing, float(free_radius))
    multiple = 1
    while safe_base_spacing * multiple * 2 <= target_spacing:
        multiple *= 2
    return multiple


def _world_lattice_bounds(
    grid: ClassifiedGrid,
    spacing: float,
    offset: float,
) -> Tuple[int, int, int, int]:
    """计算覆盖当前 GridMap 的世界坐标网格键范围"""
    corner_points = [
        grid.grid_to_world(ix, iy)
        for ix in (0, grid.width - 1)
        for iy in (0, grid.height - 1)
    ]
    margin = grid.resolution
    min_x = min(point[0] for point in corner_points) - margin
    max_x = max(point[0] for point in corner_points) + margin
    min_y = min(point[1] for point in corner_points) - margin
    max_y = max(point[1] for point in corner_points) + margin
    return (
        int(math.ceil((min_x - offset) / spacing)),
        int(math.floor((max_x - offset) / spacing)),
        int(math.ceil((min_y - offset) / spacing)),
        int(math.floor((max_y - offset) / spacing)),
    )
