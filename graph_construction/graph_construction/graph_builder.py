from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot
import math
from time import perf_counter
from typing import Tuple

import numpy as np
from scipy import ndimage

from graph_construction.edge_builder import EdgeBuilder
from graph_construction.frontier_detector import FrontierDetector
from graph_construction.graph_memory import EdgeKey, GraphState, normalize_edge_key
from graph_construction.grid_types import ClassifiedGrid, distance_to_mask


_FOUR_CONNECTED_STRUCTURE = np.asarray(
    (
        (0, 1, 0),
        (1, 1, 1),
        (0, 1, 0),
    ),
    dtype=np.uint8,
)
_NODE_SAMPLING_RANDOM_SEED = 7


@dataclass
class GraphBuilderConfig:
    """图构建运行参数

    这些参数对应论文中的随机采样, free radius, edge radius 等概念
    """

    # 过滤局部高程尖峰和帘状面噪声
    grid_map_max_surface_step: float = 0.35

    # 启动安全先验的种子半径和最大连通盲区搜索半径
    robot_blind_zone_radius: float = 0.8
    robot_blind_zone_elevation_search_radius: float = 2.0
    robot_ground_height_offset: float = 0.90
    robot_ground_elevation_tolerance: float = 0.5

    node_sample_count: int = 1000
    max_free_radius: float = 4.0
    min_obstacle_clearance: float = 0.5
    edge_radius: float = 8.0
    frontier_assign_radius: float = 5.0
    frontier_min_points: int = 4
    frontier_min_span: float = 0.6
    frontier_border_margin: float = 0.8
    frontier_candidate_spacing: float = 0.0
    frontier_visited_corridor_radius: float = 0.65

    def __post_init__(self) -> None:
        """验证直接影响节点密度和局部 pair 数的核心参数"""
        if self.node_sample_count < 0:
            raise ValueError("node_sample_count must not be negative")
        if self.max_free_radius <= 0.0:
            raise ValueError("max_free_radius must be greater than 0")
        if self.min_obstacle_clearance < 0.0:
            raise ValueError("min_obstacle_clearance must not be negative")
        if self.edge_radius <= 0.0:
            raise ValueError("edge_radius must be greater than 0")


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
    total_edge_count: int = 0
    dirty_cell_count: int = 0
    newly_free_cell_count: int = 0
    newly_obstacle_cell_count: int = 0
    local_pair_count: int = 0
    edge_clearance_check_count: int = 0
    edge_add_count: int = 0
    edge_remove_count: int = 0
    edge_keep_count: int = 0
    edge_update_skipped: bool = False
    edge_update_mode: str = "full"
    compacted_node_count: int = 0
    graph_component_count: int = 0
    current_component_node_count: int = 0
    current_component_local_node_count: int = 0
    blind_zone_status: str = ""
    blind_zone_filled_count: int = 0
    blind_zone_connected: bool = False
    blind_zone_search_radius: float = 0.0
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
    """区分地图变化类型, 用于 Frontier 刷新和诊断"""

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
    先更新已有节点, 再采样新节点, 再检测 frontier, 最后更新局部半径图
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
        self.edge_builder = EdgeBuilder(edge_radius=config.edge_radius)
        self._random = np.random.default_rng(_NODE_SAMPLING_RANDOM_SEED)
        self._previous_grid_state: _GridStateSnapshot | None = None
        self._previous_local_node_ids: set[int] = set()
        self._blind_zone_initialization_complete = False
        self._blind_zone_anchor_position: tuple[float, float, float] | None = None
        self._blind_zone_prior: dict[
            tuple[float, float],
            tuple[float, float, float],
        ] = {}

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
        local_node_positions = {
            node_id: self.graph.nodes[node_id].position[:2]
            for node_id in local_node_ids
            if node_id in self.graph.nodes
        }
        self._update_existing_nodes(
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

        # 地图开放后旧节点的 free radius 会扩大, 需要删除被覆盖的历史密集节点
        stage_started = perf_counter()
        compaction_candidate_ids: set[int] = set()
        if (
            self._previous_grid_state is not None
            and np.any(newly_free_cells)
        ):
            distance_to_new_free = distance_to_mask(
                newly_free_cells,
                grid.resolution,
            )
            for node_id in local_node_ids:
                node = self.graph.nodes.get(node_id)
                if node is None:
                    continue
                grid_index = grid.world_to_grid(
                    node.position[0],
                    node.position[1],
                )
                if grid_index is None:
                    continue
                ix, iy = grid_index
                if distance_to_new_free[iy, ix] <= self.config.max_free_radius:
                    compaction_candidate_ids.add(node_id)
        compacted_node_ids = self._compact_redundant_nodes(
            grid,
            local_node_ids,
            compaction_candidate_ids,
        )
        new_node_ids.difference_update(compacted_node_ids)
        stage_seconds["compaction"] = perf_counter() - stage_started

        # 稳定 pair 直接复用, 地图变化只重算受影响 pair
        stage_started = perf_counter()
        local_node_ids = self._node_ids_in_grid(grid)
        positions_changed = any(
            self.graph.nodes[node_id].position[:2]
            != local_node_positions[node_id]
            for node_id in local_node_ids & set(local_node_positions)
        )
        edge_update_mode = "full"
        edge_delta = None
        pair_generation_seconds = 0.0
        if self._previous_grid_state is None or positions_changed:
            edge_delta = self.edge_builder.build_delta(
                self.graph,
                grid,
                sdf_obstacle,
                sdf_unknown,
                self.config.min_obstacle_clearance,
                node_ids=local_node_ids,
            )
        else:
            pair_started = perf_counter()
            incident_node_ids = (
                new_node_ids
                | (local_node_ids - self._previous_local_node_ids)
            )
            affected_pair_keys = self._incremental_edge_pair_keys(
                grid,
                local_node_ids,
                newly_free_cells,
                grid_changes.newly_obstacle,
                incident_node_ids,
            )
            pair_generation_seconds = perf_counter() - pair_started
            if affected_pair_keys:
                edge_update_mode = "incremental"
                edge_delta = self.edge_builder.build_pair_delta(
                    self.graph,
                    grid,
                    sdf_obstacle,
                    sdf_unknown,
                    self.config.min_obstacle_clearance,
                    node_ids=local_node_ids,
                    pair_keys=affected_pair_keys,
                )
                self.edge_builder.last_stats.pair_generation_seconds = (
                    pair_generation_seconds
                )
            else:
                edge_update_mode = "reused"
                self.edge_builder.reuse_existing(
                    self.graph,
                    local_node_ids,
                )
        edge_update_skipped = edge_update_mode == "reused"
        if (
            edge_update_skipped
            and not np.any(dirty_cells)
            and local_node_ids == set(local_node_positions)
            and all(
                self.graph.nodes[node_id].position[:2]
                == local_node_positions[node_id]
                for node_id in local_node_ids
            )
        ):
            edge_update_mode = "stable_reused"
        stage_seconds["edge_pairs"] = (
            self.edge_builder.last_stats.pair_generation_seconds
        )
        stage_seconds["edge_validation"] = (
            self.edge_builder.last_stats.validation_seconds
        )
        delta_started = perf_counter()
        if edge_delta is not None:
            self.graph.apply_edge_delta(
                edge_delta.edge_keys_to_remove,
                edge_delta.edges_to_add,
            )
        stage_seconds["edge_delta"] = perf_counter() - delta_started
        stage_seconds["edges"] = perf_counter() - stage_started

        graph_component_count, current_component = self._graph_components()
        current_component_local_count = len(current_component & local_node_ids)
        grid.stats["navigation_graph_component_count"] = graph_component_count
        grid.stats["navigation_graph_current_component_nodes"] = len(
            current_component
        )
        grid.stats["navigation_graph_local_connected"] = (
            bool(self.graph.current_node_id is not None)
            and current_component_local_count == len(local_node_ids)
        )
        self._previous_grid_state = _snapshot_grid_state(grid)
        self._previous_local_node_ids = set(local_node_ids)

        return GraphUpdateResult(
            graph=self.graph,
            classified_grid=grid,
            stats=GraphUpdateStats(
                local_node_count=len(local_node_ids),
                total_node_count=len(self.graph.nodes),
                total_edge_count=len(self.graph.edges),
                dirty_cell_count=int(np.count_nonzero(dirty_cells)),
                newly_free_cell_count=int(np.count_nonzero(newly_free_cells)),
                newly_obstacle_cell_count=int(
                    np.count_nonzero(grid_changes.newly_obstacle)
                ),
                local_pair_count=(
                    self.edge_builder.last_stats.candidate_pair_count
                ),
                edge_clearance_check_count=(
                    self.edge_builder.last_stats.clearance_check_count
                ),
                edge_add_count=self.edge_builder.last_stats.added_edge_count,
                edge_remove_count=self.edge_builder.last_stats.removed_edge_count,
                edge_keep_count=self.edge_builder.last_stats.kept_edge_count,
                edge_update_skipped=edge_update_skipped,
                edge_update_mode=edge_update_mode,
                compacted_node_count=len(compacted_node_ids),
                graph_component_count=graph_component_count,
                current_component_node_count=len(current_component),
                current_component_local_node_count=current_component_local_count,
                blind_zone_status=str(
                    grid.stats.get("robot_blind_zone_status", "")
                ),
                blind_zone_filled_count=int(
                    grid.stats.get("robot_blind_zone_filled", 0)
                ),
                blind_zone_connected=bool(
                    grid.stats.get("robot_blind_zone_connected", False)
                ),
                blind_zone_search_radius=float(
                    grid.stats.get("robot_blind_zone_search_radius", 0.0)
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
        """初始化时扩展脚下盲区, 后续帧恢复固定世界区域先验"""
        if self._blind_zone_anchor_position is None:
            self._blind_zone_anchor_position = robot_position
        restored_cells = self._restore_initial_blind_zone_prior(
            grid,
            protected_unknown,
        )
        if self._blind_zone_initialization_complete:
            if grid.stats is None:
                grid.stats = {}
            restored_count = int(np.count_nonzero(restored_cells))
            connected = self._initial_prior_reaches_observed_free(
                grid,
                restored_cells,
            )
            grid.stats["robot_blind_zone_filled"] = restored_count
            grid.stats["robot_blind_zone_artificial_free"] = restored_count
            grid.stats["robot_blind_zone_search_radius"] = round(
                max(
                    self.config.robot_blind_zone_radius,
                    self.config.robot_blind_zone_elevation_search_radius,
                ),
                3,
            )
            grid.stats["robot_blind_zone_status"] = (
                "initial_prior_restored"
                if connected
                else "initial_prior_disconnected"
            )
            grid.stats["robot_blind_zone_connected"] = connected
            return _BlindZoneRepair(restored_cells, connected)

        repair = self._repair_robot_blind_zone(
            grid,
            robot_position,
            protected_unknown,
        )
        combined_cells = repair.cells | restored_cells
        self._remember_initial_blind_zone_prior(grid, repair.cells)
        if repair.complete:
            self._blind_zone_initialization_complete = True
        return _BlindZoneRepair(combined_cells, repair.complete)

    def _initial_prior_reaches_observed_free(
        self,
        grid: ClassifiedGrid,
        restored_cells: np.ndarray,
    ) -> bool:
        """确认固定启动先验当前仍连接至少一个非人工 free cell"""
        if self._blind_zone_anchor_position is None:
            return False
        center = grid.world_to_grid(
            self._blind_zone_anchor_position[0],
            self._blind_zone_anchor_position[1],
        )
        if center is None or not grid.is_free_index(center[0], center[1]):
            return False
        observed_free = grid.free & ~restored_cells
        return _free_component_reaches_target(
            grid.free,
            center,
            observed_free,
        )

    def _remember_initial_blind_zone_prior(
        self,
        grid: ClassifiedGrid,
        repaired_cells: np.ndarray,
    ) -> None:
        """以世界坐标保存人工 free, 防止下一帧原始 unknown 擦除启动通道"""
        if grid.elevation is None:
            return
        for iy, ix in np.argwhere(repaired_cells):
            world_x, world_y, _ = grid.grid_to_world(int(ix), int(iy))
            world_z = float(grid.elevation[iy, ix]) + grid.z_offset
            key = (round(world_x, 3), round(world_y, 3))
            self._blind_zone_prior[key] = (world_x, world_y, world_z)

    def _restore_initial_blind_zone_prior(
        self,
        grid: ClassifiedGrid,
        protected_unknown: np.ndarray,
    ) -> np.ndarray:
        """只恢复仍为 unknown 的启动 cell, 真实 free 和 obstacle 保持优先"""
        restored = np.zeros((grid.height, grid.width), dtype=bool)
        if grid.elevation is None or not self._blind_zone_prior:
            return restored
        for world_x, world_y, world_z in self._blind_zone_prior.values():
            grid_index = grid.world_to_grid(world_x, world_y)
            if grid_index is None:
                continue
            ix, iy = grid_index
            if (
                not grid.unknown[iy, ix]
                or grid.obstacle[iy, ix]
                or protected_unknown[iy, ix]
            ):
                continue
            grid.unknown[iy, ix] = False
            grid.free[iy, ix] = True
            grid.elevation[iy, ix] = world_z - grid.z_offset
            restored[iy, ix] = True
        restored_count = int(np.count_nonzero(restored))
        if restored_count and grid.stats is not None:
            grid.stats["free"] = int(np.count_nonzero(grid.free))
            grid.stats["unknown"] = int(np.count_nonzero(grid.unknown))
        return restored

    def _repair_robot_blind_zone(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        protected_unknown: np.ndarray | None = None,
    ) -> _BlindZoneRepair:
        """填充机器人脚下全部连通盲区, 并确认已连接外部已知 free

        固定半径只定义盲区种子范围, 与种子相连的 unknown 会继续填充到
        地面搜索半径, 避免人工 free 圆停在真实点云边界内形成孤岛
        只有修补后的机器人分量接触原始外部 free 时才结束启动修补

        高程取自盲区边缘最近的可靠地面中位数, 障碍物和本帧突变地形不会被清除
        """
        if grid.stats is None:
            grid.stats = {}
        repaired_cells = np.zeros((grid.height, grid.width), dtype=bool)
        grid.stats["robot_blind_zone_filled"] = 0
        grid.stats["robot_blind_zone_artificial_free"] = 0
        if grid.elevation is None or self.config.robot_blind_zone_radius <= 0.0:
            grid.stats["robot_blind_zone_status"] = "disabled"
            grid.stats["robot_blind_zone_connected"] = False
            return _BlindZoneRepair(repaired_cells, True)

        center = grid.world_to_grid(robot_position[0], robot_position[1])
        if center is None:
            grid.stats["robot_blind_zone_status"] = "outside_grid"
            grid.stats["robot_blind_zone_connected"] = False
            return _BlindZoneRepair(repaired_cells, False)
        if grid.is_obstacle_index(center[0], center[1]):
            grid.stats["robot_blind_zone_status"] = "center_obstacle"
            grid.stats["robot_blind_zone_connected"] = False
            return _BlindZoneRepair(repaired_cells, True)

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
            & grid.free[min_y:max_y, min_x:max_x]
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

        region_unknown = grid.unknown[min_y:max_y, min_x:max_x]
        region_obstacle = grid.obstacle[min_y:max_y, min_x:max_x]
        original_free = grid.free.copy()
        local_original_free = original_free[min_y:max_y, min_x:max_x]
        seed_margin = self.config.min_obstacle_clearance + math.sqrt(2.0) * resolution
        seed_radius = min(
            search_radius,
            self.config.robot_blind_zone_radius + seed_margin,
        )
        blind_candidates = (
            (distances <= search_radius)
            & region_unknown
            & ~region_obstacle
        )
        if protected_unknown is not None:
            blind_candidates &= ~protected_unknown[
                min_y:max_y,
                min_x:max_x,
            ]

        # 填充所有接触机器人种子范围的 unknown 分量, 而不是只画固定半径圆
        blind_labels, _ = ndimage.label(
            blind_candidates,
            structure=_FOUR_CONNECTED_STRUCTURE,
        )
        local_center = (center_x - min_x, center_y - min_y)
        original_free_labels, _ = ndimage.label(
            local_original_free,
            structure=_FOUR_CONNECTED_STRUCTURE,
        )
        center_free_label = int(
            original_free_labels[local_center[1], local_center[0]]
        )
        center_original_component = (
            original_free_labels == center_free_label
            if center_free_label > 0
            else np.zeros(local_original_free.shape, dtype=bool)
        )
        center_component_boundary = np.zeros(local_original_free.shape, dtype=bool)
        center_component_boundary[[0, -1], :] = True
        center_component_boundary[:, [0, -1]] = True
        search_limit = (
            center_component_boundary
            | (distances >= search_radius - math.sqrt(2.0) * resolution)
        )
        # 只把延伸到搜索边界的原始 free 当作外围地面, 避免误接局部噪声小岛
        boundary_free_labels = np.unique(
            original_free_labels[
                search_limit
                & (original_free_labels > 0)
            ]
        )
        center_component_reaches_limit = (
            center_free_label > 0
            and center_free_label in boundary_free_labels
        )
        grid.stats["robot_blind_zone_center_reaches_search_limit"] = (
            center_component_reaches_limit
        )
        grid.stats["robot_blind_zone_boundary_free_components"] = int(
            len(boundary_free_labels)
        )

        # 大于固定种子圆的已有脚下 free 岛也必须从自身边界继续填充盲区
        touches_center_component = ndimage.binary_dilation(
            center_original_component,
            structure=_FOUR_CONNECTED_STRUCTURE,
            border_value=0,
        )
        seed_labels = np.unique(
            blind_labels[
                (
                    (distances <= seed_radius)
                    | touches_center_component
                )
                & (blind_labels > 0)
            ]
        )
        repair_mask = (
            np.isin(blind_labels, seed_labels)
            if len(seed_labels)
            else np.zeros(blind_candidates.shape, dtype=bool)
        )
        repaired = int(np.count_nonzero(repair_mask))
        repaired_cells[
            min_y:max_y,
            min_x:max_x,
        ] = repair_mask
        region_unknown[repair_mask] = False
        grid.free[
            min_y:max_y,
            min_x:max_x,
        ][repair_mask] = True
        grid.elevation[
            min_y:max_y,
            min_x:max_x,
        ][repair_mask] = ground_elevation

        # 修补完成条件使用真实 free 连通性, 不能再用 repaired cell 数量代替
        external_original_free = np.zeros((grid.height, grid.width), dtype=bool)
        external_original_free[min_y:max_y, min_x:max_x] = (
            np.isin(original_free_labels, boundary_free_labels)
            & ~center_original_component
        )
        connected = (
            center_component_reaches_limit
            or _free_component_reaches_target(
                grid.free,
                center,
                external_original_free,
            )
        )
        grid.stats["robot_blind_zone_connected"] = connected
        grid.stats["robot_blind_zone_seed_radius"] = round(seed_radius, 3)
        grid.stats["robot_blind_zone_search_radius"] = round(search_radius, 3)

        if repaired > 0:
            grid.stats["robot_blind_zone_filled"] = repaired
            grid.stats["robot_blind_zone_artificial_free"] = repaired
            grid.stats["free"] = int(np.count_nonzero(grid.free))
            grid.stats["unknown"] = int(np.count_nonzero(grid.unknown))
            grid.stats["robot_blind_zone_status"] = (
                "repaired_connected"
                if connected
                else "repaired_waiting_boundary"
            )
        else:
            grid.stats["robot_blind_zone_status"] = (
                "known_ground"
                if connected
                else "known_ground_isolated"
            )
        return _BlindZoneRepair(repaired_cells, connected)

    def _update_existing_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
        local_node_ids: set[int],
    ) -> None:
        """用可靠局部观测刷新历史节点, unknown 和窗口外区域不否定记忆"""
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
                self.graph.remove_node(node_id)
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
                self.graph.remove_node(node_id)
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

    def _sample_new_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
        reachable_free: np.ndarray,
    ) -> set[int]:
        """按论文 Algorithm 3 在可达 free 区域随机采样稀疏节点

        候选落入任一已有节点的 free radius 时拒绝
        本帧新节点立即进入持久空间索引, 因此也参与后续覆盖判断
        """
        new_node_ids: set[int] = set()
        sample_count = max(0, int(self.config.node_sample_count))
        if sample_count == 0:
            return new_node_ids

        clearance = np.minimum(sdf_obstacle, sdf_unknown)
        candidate_mask = (
            reachable_free
            & grid.free
            & (clearance > self.config.min_obstacle_clearance)
        )
        candidate_flat_indices = np.flatnonzero(candidate_mask)
        if not len(candidate_flat_indices):
            return new_node_ids

        sampled_offsets = self._random.integers(
            0,
            len(candidate_flat_indices),
            size=sample_count,
        )
        sampled_flat_indices = candidate_flat_indices[sampled_offsets]
        accepted_cells: set[int] = set()
        for flat_index in sampled_flat_indices:
            cell_key = int(flat_index)
            if cell_key in accepted_cells:
                continue
            iy, ix = np.unravel_index(cell_key, candidate_mask.shape)
            world_x, world_y, world_z = grid.grid_to_world(int(ix), int(iy))
            position = (world_x, world_y, world_z)

            # max_free_radius 是所有覆盖圆上限, 可安全缩小空间桶查询范围
            nearby_node_ids = self.graph.node_ids_within(
                position,
                self.config.max_free_radius,
            )
            if any(
                self.graph.nodes[node_id].distance_xy(position)
                <= self.graph.nodes[node_id].free_radius
                for node_id in nearby_node_ids
                if node_id in self.graph.nodes
            ):
                continue

            free_radius = min(
                float(clearance[iy, ix]),
                self.config.max_free_radius,
            )
            node = self.graph.create_node(
                position=position,
                stamp_seconds=stamp_seconds,
            )
            node.free_radius = free_radius
            node.explored_radius = float(sdf_unknown[iy, ix])
            accepted_cells.add(cell_key)
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
        """只返回机器人脚下真实连通的 free 分量"""
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
        return labels == start_label

    def _compact_redundant_nodes(
        self,
        grid: ClassifiedGrid,
        local_node_ids: set[int],
        candidate_node_ids: set[int],
    ) -> set[int]:
        """删除被更大自由圆覆盖的普通节点, 修复 unknown 开放后的历史过密"""
        if not candidate_node_ids or len(local_node_ids) < 2:
            return set()
        protected_ids = {
            node_id
            for node_id in local_node_ids
            if node_id == self.graph.current_node_id
            or (
                node_id in self.graph.nodes
                and self.graph.nodes[node_id].is_frontier
            )
        }
        ordered_nodes = sorted(
            (
                self.graph.nodes[node_id]
                for node_id in local_node_ids
                if node_id in self.graph.nodes
            ),
            key=lambda node: (
                node.node_id not in protected_ids,
                -node.free_radius,
                node.node_id,
            ),
        )
        kept_nodes = []
        removed_node_ids: set[int] = set()
        for node in ordered_nodes:
            if node.node_id in protected_ids:
                kept_nodes.append(node)
                continue
            if node.node_id not in candidate_node_ids:
                kept_nodes.append(node)
                continue
            neighbor_ids = {
                (
                    edge_key[1]
                    if edge_key[0] == node.node_id
                    else edge_key[0]
                )
                for edge_key in self.graph.adjacency.get(node.node_id, ())
            }
            covered = False
            for keeper in kept_nodes:
                if (
                    keeper.distance_xy(node.position) > keeper.free_radius
                    or not grid.is_world_collision_free(
                        keeper.position[:2],
                        node.position[:2],
                    )
                ):
                    continue
                # 每个旧邻居必须已能绕过被删节点, 防止压缩掉唯一桥接点
                if all(
                    neighbor_id == keeper.node_id
                    or normalize_edge_key(
                        keeper.node_id,
                        neighbor_id,
                    )
                    in self.graph.edges
                    for neighbor_id in neighbor_ids
                ):
                    covered = True
                    break
            if covered:
                removed_node_ids.add(node.node_id)
                continue
            kept_nodes.append(node)
        for node_id in removed_node_ids:
            self.graph.remove_node(node_id)
        return removed_node_ids

    def _graph_components(self) -> tuple[int, set[int]]:
        """统计完整图分量并返回 current node 所在分量"""
        unvisited = set(self.graph.nodes)
        components: list[set[int]] = []
        while unvisited:
            start_node_id = min(unvisited)
            component = {start_node_id}
            pending = [start_node_id]
            unvisited.remove(start_node_id)
            while pending:
                node_id = pending.pop()
                for edge_key in self.graph.adjacency.get(node_id, ()):
                    other_id = (
                        edge_key[1]
                        if edge_key[0] == node_id
                        else edge_key[0]
                    )
                    if other_id not in unvisited:
                        continue
                    unvisited.remove(other_id)
                    component.add(other_id)
                    pending.append(other_id)
            components.append(component)
        current_component = next(
            (
                component
                for component in components
                if self.graph.current_node_id in component
            ),
            set(),
        )
        return len(components), current_component

    def _incremental_edge_pair_keys(
        self,
        grid: ClassifiedGrid,
        local_node_ids: set[int],
        newly_free_cells: np.ndarray,
        newly_obstacle_cells: np.ndarray,
        incident_node_ids: set[int],
    ) -> set[EdgeKey]:
        """将新增节点和分类变化精确转换为需要复查的半径 pair

        长度不超过 R 的线段经过一个变化 cell 时, 至少一个端点距该 cell
        不超过 R/2, 加上净空和 cell 对角线即可保守覆盖全部候选
        """
        pair_keys = self._incident_pair_keys(
            local_node_ids,
            incident_node_ids,
        )
        clearance_radius = (
            self.config.min_obstacle_clearance
            + math.sqrt(2.0) * grid.resolution
        )
        obstacle_points = self._mask_world_points(
            grid,
            newly_obstacle_cells,
        )
        if obstacle_points:
            pair_keys.update(
                edge_key
                for edge_key in self.graph.edge_keys_near_points(
                    obstacle_points,
                    radius=clearance_radius,
                )
                if edge_key[0] in local_node_ids
                and edge_key[1] in local_node_ids
            )

        new_free_points = self._mask_world_points(
            grid,
            newly_free_cells,
        )
        if not new_free_points:
            return pair_keys
        near_endpoint_ids: set[int] = set()
        endpoint_radius = 0.5 * self.config.edge_radius + clearance_radius
        for world_x, world_y in new_free_points:
            near_endpoint_ids.update(
                self.graph.node_ids_within(
                    (world_x, world_y, 0.0),
                    endpoint_radius,
                )
                & local_node_ids
            )
        pair_keys.update(
            self._incident_pair_keys(
                local_node_ids,
                near_endpoint_ids,
            )
        )
        return pair_keys

    def _incident_pair_keys(
        self,
        local_node_ids: set[int],
        endpoint_node_ids: set[int],
    ) -> set[EdgeKey]:
        """返回指定端点与当前局部半径邻居组成的全部无向 pair"""
        pair_keys: set[EdgeKey] = set()
        for node_id in endpoint_node_ids & local_node_ids:
            node = self.graph.nodes.get(node_id)
            if node is None:
                continue
            for other_id in (
                self.graph.node_ids_within(
                    node.position,
                    self.config.edge_radius,
                )
                & local_node_ids
            ):
                if node_id == other_id:
                    continue
                pair_keys.add(normalize_edge_key(node_id, other_id))
        return pair_keys

    @staticmethod
    def _mask_world_points(
        grid: ClassifiedGrid,
        mask: np.ndarray,
    ) -> list[tuple[float, float]]:
        """把变化 cell 中心转换为边空间索引使用的世界坐标"""
        return [
            grid.grid_to_world(int(ix), int(iy))[:2]
            for iy, ix in np.argwhere(mask)
        ]

    def _nearest_free_neighbor(
        self,
        grid: ClassifiedGrid,
        start: tuple[int, int],
        max_radius_cells: int,
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


def _free_component_reaches_target(
    free: np.ndarray,
    start: tuple[int, int],
    target: np.ndarray,
) -> bool:
    """检查机器人 free 分量是否接触启动范围外的原始 free"""
    start_x, start_y = start
    if (
        start_x < 0
        or start_y < 0
        or start_x >= free.shape[1]
        or start_y >= free.shape[0]
        or not free[start_y, start_x]
        or not np.any(target)
    ):
        return False
    labels, _ = ndimage.label(
        free,
        structure=_FOUR_CONNECTED_STRUCTURE,
    )
    start_label = int(labels[start_y, start_x])
    return start_label > 0 and bool(np.any(target & (labels == start_label)))


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
