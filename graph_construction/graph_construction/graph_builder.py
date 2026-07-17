from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import hypot
import math
from typing import Tuple

import numpy as np

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
    robot_blind_zone_radius: float = 0.8
    robot_blind_zone_elevation_search_radius: float = 6.0
    robot_ground_height_offset: float = 0.22
    robot_ground_elevation_tolerance: float = 0.5

    sample_stride: int = 8
    min_node_separation: float = 1.0
    max_free_radius: float = 4.0
    min_obstacle_clearance: float = 0.5
    edge_radius: float = 3.0
    max_edge_neighbors: int = 6
    current_node_max_edge_neighbors: int = 10
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
        )
        self.robot_anchor_node_id: int | None = None

    def update(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        stamp_seconds: float,
    ) -> GraphUpdateResult:
        """根据已解码 grid 和机器人位置更新稀疏图"""
        return self._update_classified_grid(grid, robot_position, stamp_seconds)

    def _sanitize_grid_surface(self, grid: ClassifiedGrid) -> None:
        """清洗孤立高程尖峰, 避免 graph 采到帘状面噪声"""
        if grid.elevation is None:
            return

        max_step = self.config.grid_map_max_surface_step
        if max_step <= 0.0:
            return

        h, w = grid.height, grid.width
        curtain_indices = []

        for iy in range(h):
            for ix in range(w):
                val = float(grid.elevation[iy, ix])
                if not math.isfinite(val):
                    continue
                neighbor_values = []
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = ix + dx, iy + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            n_val = float(grid.elevation[ny, nx])
                            if math.isfinite(n_val):
                                neighbor_values.append(n_val)
                if len(neighbor_values) < 3:
                    continue
                neighbor_values.sort()
                median = neighbor_values[len(neighbor_values) // 2]
                if abs(val - median) > max_step:
                    curtain_indices.append((ix, iy))

        for ix, iy in curtain_indices:
            grid.elevation[iy, ix] = math.nan
            grid.unknown[iy, ix] = True
            grid.free[iy, ix] = False
            grid.obstacle[iy, ix] = False

    def _update_classified_grid(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        stamp_seconds: float,
    ) -> GraphUpdateResult:
        self._sanitize_grid_surface(grid)
        self._repair_robot_blind_zone(grid, robot_position)

        robot_ground_position, robot_ground_projected = (
            grid.project_to_elevation_with_status(robot_position)
        )
        if robot_ground_projected:
            self.graph.append_trajectory_point(robot_ground_position, 0.25)
        reachable_free = self._reachable_free_mask(grid, robot_ground_position)
        # 距离场用于节点 clearance 和 frontier 生命周期判断
        sdf_obstacle = distance_to_mask(grid.obstacle, grid.resolution)
        # rolling GridMap 外部必须视为 unknown, 否则全 known 局部图会产生无限探索半径
        sdf_unknown = distance_to_mask(
            grid.unknown,
            grid.resolution,
            include_grid_exterior=True,
        )
        # 先刷新旧节点, 再采样和建边
        self._update_existing_nodes(grid, sdf_obstacle, sdf_unknown, stamp_seconds)

        # 在当前观测到的 free 区域补充稀疏节点
        self._sample_new_nodes(
            grid,
            sdf_obstacle,
            sdf_unknown,
            stamp_seconds,
            reachable_free,
        )
        # frontier cell 需要绑定到附近可用图节点
        frontier_cells = self.frontier_detector.detect_frontier_cells(grid)
        self.frontier_detector.assign_frontiers(
            self.graph,
            grid,
            frontier_cells,
        )

        self._update_current_node(
            grid,
            robot_position,
            robot_ground_position,
            robot_ground_projected,
            stamp_seconds,
        )
        # current node 确定后重建边, 便于优先保留机器人附近连接
        next_edges = self.edge_builder.build_edges(
            self.graph,
            grid,
            sdf_obstacle,
            sdf_unknown,
            self.config.min_obstacle_clearance,
        )
        # 持久边是主线语义, 当前窗口只能用可见障碍证伪历史边
        next_edges = self.edge_builder.merge_historical_edges(
            self.graph,
            next_edges,
            grid,
            sdf_obstacle,
            self.config.min_obstacle_clearance,
        )
        if (
            self.robot_anchor_node_id is not None
            and self.graph.current_node_id == self.robot_anchor_node_id
        ):
            next_edges.extend(
                self.edge_builder.build_robot_anchor_edges(
                    self.graph,
                    self.robot_anchor_node_id,
                    grid,
                    sdf_obstacle,
                    self.config.min_obstacle_clearance,
                    self.config.edge_radius,
                    self.config.current_node_max_edge_neighbors,
                )
            )
        self.graph.set_edges(next_edges)

        return GraphUpdateResult(
            graph=self.graph,
            classified_grid=grid,
        )

    def _repair_robot_blind_zone(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
    ) -> int:
        """只修补机器人可物理占用的脚下 unknown 区域

        高程取自盲区边缘最近的可靠地面中位数, 明确障碍物不会被清除
        """
        if grid.stats is None:
            grid.stats = {}
        grid.stats["robot_blind_zone_filled"] = 0
        if grid.elevation is None or self.config.robot_blind_zone_radius <= 0.0:
            grid.stats["robot_blind_zone_status"] = "disabled"
            return 0

        center = grid.world_to_grid(robot_position[0], robot_position[1])
        if center is None:
            grid.stats["robot_blind_zone_status"] = "outside_grid"
            return 0
        if grid.is_obstacle_index(center[0], center[1]):
            grid.stats["robot_blind_zone_status"] = "center_obstacle"
            return 0

        resolution = max(grid.resolution, 1e-6)
        search_radius = max(
            self.config.robot_blind_zone_radius,
            self.config.robot_blind_zone_elevation_search_radius,
        )
        search_cells = max(1, int(math.ceil(search_radius / resolution)))
        samples = []
        center_x, center_y = center
        for offset_y in range(-search_cells, search_cells + 1):
            for offset_x in range(-search_cells, search_cells + 1):
                ix = center_x + offset_x
                iy = center_y + offset_y
                distance = hypot(offset_x * resolution, offset_y * resolution)
                if distance > search_radius or not (
                    0 <= ix < grid.width and 0 <= iy < grid.height
                ):
                    continue
                if grid.is_obstacle_index(ix, iy):
                    continue
                elevation = float(grid.elevation[iy, ix])
                if math.isfinite(elevation):
                    samples.append((distance, elevation))
        samples.sort(key=lambda item: item[0])
        expected_ground = robot_position[2] - self.config.robot_ground_height_offset
        plausible_samples = [
            sample
            for sample in samples
            if abs(sample[1] + grid.z_offset - expected_ground)
            <= self.config.robot_ground_elevation_tolerance
        ]
        ground_source = "map"
        if plausible_samples:
            samples = plausible_samples
        else:
            samples = [(0.0, expected_ground - grid.z_offset)]
            ground_source = "odom"

        nearest_distance = samples[0][0]
        grid.stats["robot_blind_zone_nearest_ground"] = round(nearest_distance, 3)
        grid.stats["robot_blind_zone_ground_source"] = ground_source
        sample_band = nearest_distance + max(0.4, 2.0 * resolution)
        ground_samples = [
            elevation
            for distance, elevation in samples
            if distance <= sample_band
        ][:32]
        ground_elevation = float(np.median(ground_samples))

        repair_cells = max(
            1,
            int(math.ceil(self.config.robot_blind_zone_radius / resolution)),
        )
        repaired = 0
        for offset_y in range(-repair_cells, repair_cells + 1):
            for offset_x in range(-repair_cells, repair_cells + 1):
                if (
                    hypot(offset_x * resolution, offset_y * resolution)
                    > self.config.robot_blind_zone_radius
                ):
                    continue
                ix = center_x + offset_x
                iy = center_y + offset_y
                if not (0 <= ix < grid.width and 0 <= iy < grid.height):
                    continue
                if grid.is_obstacle_index(ix, iy) or not grid.is_unknown_index(ix, iy):
                    continue
                grid.unknown[iy, ix] = False
                grid.free[iy, ix] = True
                grid.elevation[iy, ix] = ground_elevation
                repaired += 1

        if repaired > 0:
            grid.stats["robot_blind_zone_filled"] = repaired
            grid.stats["robot_blind_zone_status"] = "repaired"
            grid.stats["free"] = int(np.count_nonzero(grid.free))
            grid.stats["unknown"] = int(np.count_nonzero(grid.unknown))
        else:
            grid.stats["robot_blind_zone_status"] = "known_ground"
        return repaired

    def _update_existing_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
    ) -> None:
        """用可靠局部观测刷新历史节点, unknown 和窗口外区域不否定记忆"""
        for node_id, node in list(self.graph.nodes.items()):
            # 旧进程可能已保存非有限半径, 该值不能表达全局探索覆盖
            if not math.isfinite(node.explored_radius) or node.explored_radius < 0.0:
                node.explored_radius = 0.0
            if node.is_robot_anchor:
                continue
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
                node.position = (node.position[0], node.position[1], surface_z)
            node.last_seen_time = stamp_seconds

    def _sample_new_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
        reachable_free: np.ndarray,
    ) -> None:
        """在机器人当前可达 free 分量中按分层世界网格补充节点

        最细网格由 sample stride 决定, free radius 越大则选择越粗的嵌套层级
        所有层级共享世界坐标锚点, rolling GridMap 移动时不会改变节点排列
        """
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

    def _update_current_node(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        robot_ground_position: Tuple[float, float, float],
        robot_ground_projected: bool,
        stamp_seconds: float,
    ) -> None:
        """用跟随机器人的 anchor 固定路径起点语义"""
        best_node = self._ensure_robot_anchor_node(
            grid,
            robot_ground_position,
            stamp_seconds,
        )
        if best_node is None:
            best_node = self._nearest_collision_free_node(grid, robot_ground_position)
        if best_node is None:
            best_node = self.graph.nearest_node(robot_ground_position)
        self.graph.current_node_id = best_node.node_id if best_node is not None else None
        self.graph.update_robot_position(
            robot_position,
            robot_ground_position if robot_ground_projected else None,
        )

    def _reachable_free_mask(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
    ) -> np.ndarray:
        """计算机器人脚下及盲区外围的可达 free 分量

        只允许跨越无明确障碍的 unknown 盲区选择最近外围分量
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

        queue = deque()

        def flood_fill(seed: tuple[int, int]) -> None:
            """从单个 seed 扩展完整 free 分量"""
            queue.append(seed)
            reachable[seed[1], seed[0]] = True
            while queue:
                current_x, current_y = queue.popleft()
                for offset_x, offset_y in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    next_x = current_x + offset_x
                    next_y = current_y + offset_y
                    if (
                        not grid.is_free_index(next_x, next_y)
                        or reachable[next_y, next_x]
                    ):
                        continue
                    reachable[next_y, next_x] = True
                    queue.append((next_x, next_y))

        queue.clear()
        flood_fill(start)

        # 修补后的脚下小岛仍需跨 unknown 盲区引导采样外围 free 分量
        max_radius_cells = max(
            int(math.ceil(self.config.edge_radius / max(grid.resolution, 1e-6))),
            1,
        )
        outer_start = self._nearest_free_neighbor(
            grid,
            start,
            max_radius_cells=max_radius_cells,
            excluded=reachable,
        )
        if outer_start is not None:
            flood_fill(outer_start)
        return reachable

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

    def _ensure_robot_anchor_node(
        self,
        grid: ClassifiedGrid,
        position: Tuple[float, float, float],
        stamp_seconds: float,
    ):
        """脚下局部 unknown 时创建机器人锚点, 让起点接回安全近邻图"""
        grid_index = grid.world_to_grid(position[0], position[1])
        if grid_index is None:
            return None
        ix, iy = grid_index
        if grid.is_obstacle_index(ix, iy):
            return None

        anchor = None
        if self.robot_anchor_node_id is not None:
            anchor = self.graph.nodes.get(self.robot_anchor_node_id)
        if anchor is not None and anchor.distance_xy(position) >= self.config.min_node_separation:
            # 固化旧 anchor 为路线 breadcrumb, 新 anchor 继续跟随机器人
            anchor.is_robot_anchor = False
            self.robot_anchor_node_id = None
            anchor = None
        if anchor is None:
            anchor = self.graph.create_node(
                position=position,
                stamp_seconds=stamp_seconds,
                is_robot_anchor=True,
            )
            self.robot_anchor_node_id = anchor.node_id
        else:
            anchor.position = position
            anchor.last_seen_time = stamp_seconds
            anchor.is_robot_anchor = True

        anchor.free_radius = max(anchor.free_radius, self.config.min_obstacle_clearance)
        anchor.explored_radius = max(anchor.explored_radius, grid.resolution)
        anchor.frontier_points.clear()
        anchor.is_frontier = False
        return anchor

    def _nearest_collision_free_node(
        self,
        grid: ClassifiedGrid,
        position: Tuple[float, float, float],
    ):
        """优先选择和机器人之间直线无碰撞的最近节点"""
        candidates = sorted(
            self.graph.nodes.values(),
            key=lambda node: hypot(node.position[0] - position[0], node.position[1] - position[1]),
        )
        for node in candidates:
            if grid.is_world_collision_free(
                (position[0], position[1]),
                (node.position[0], node.position[1]),
            ):
                return node
        return None


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
