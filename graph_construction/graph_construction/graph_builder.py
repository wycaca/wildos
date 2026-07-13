from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import hypot
import math
from time import perf_counter
from typing import Dict, Tuple

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


@dataclass
class GraphUpdateDiagnostics:
    """单次图更新的纯算法诊断数据"""

    stage_timings_ms: Dict[str, float] = field(default_factory=dict)
    node_count: int = 0
    edge_count: int = 0
    frontier_node_count: int = 0
    frontier_cell_count: int = 0
    frontier_candidate_count: int = 0
    connected_components: int = 0
    current_component_size: int = 0
    degree_min: int = 0
    degree_max: int = 0
    degree_avg: float = 0.0
    current_node_id: int | None = None
    current_node_status: str = "missing"


@dataclass
class GraphUpdateResult:
    """返回给 ROS 适配层的纯图更新结果"""

    graph: GraphState
    classified_grid: ClassifiedGrid
    frontier_cell_count: int
    diagnostics: GraphUpdateDiagnostics = field(default_factory=GraphUpdateDiagnostics)


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
        stage_timings_ms: Dict[str, float] = {}
        total_start = perf_counter()

        stage_start = perf_counter()
        self._sanitize_grid_surface(grid)

        robot_ground_position, robot_ground_projected = (
            grid.project_to_elevation_with_status(robot_position)
        )
        reachable_free = self._reachable_free_mask(grid, robot_ground_position)
        stage_timings_ms["prepare_grid"] = _elapsed_ms(stage_start)

        # 距离场用于节点 clearance 和 frontier 生命周期判断
        stage_start = perf_counter()
        sdf_obstacle = distance_to_mask(grid.obstacle, grid.resolution)
        sdf_unknown = distance_to_mask(grid.unknown, grid.resolution)
        stage_timings_ms["distance_fields"] = _elapsed_ms(stage_start)

        # 先刷新旧节点, 再采样和建边
        stage_start = perf_counter()
        self._update_existing_nodes(grid, sdf_obstacle, sdf_unknown, stamp_seconds)
        stage_timings_ms["update_nodes"] = _elapsed_ms(stage_start)

        # 在当前观测到的 free 区域补充稀疏节点
        stage_start = perf_counter()
        self._sample_new_nodes(
            grid,
            sdf_obstacle,
            sdf_unknown,
            stamp_seconds,
            reachable_free,
        )
        stage_timings_ms["sample_nodes"] = _elapsed_ms(stage_start)

        # frontier cell 需要绑定到附近可用图节点
        stage_start = perf_counter()
        frontier_cells = self.frontier_detector.detect_frontier_cells(grid)
        frontier_assignment = self.frontier_detector.assign_frontiers(
            self.graph,
            grid,
            frontier_cells,
        )
        stage_timings_ms["update_frontiers"] = _elapsed_ms(stage_start)

        stage_start = perf_counter()
        current_node_status = self._update_current_node(
            grid,
            robot_position,
            robot_ground_position,
            robot_ground_projected,
            stamp_seconds,
        )
        stage_timings_ms["current_node"] = _elapsed_ms(stage_start)

        # current node 确定后重建边, 便于优先保留机器人附近连接
        stage_start = perf_counter()
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
        stage_timings_ms["build_edges"] = _elapsed_ms(stage_start)
        stage_timings_ms["total"] = _elapsed_ms(total_start)
        diagnostics = _build_graph_update_diagnostics(
            self.graph,
            stage_timings_ms,
            frontier_cell_count=len(frontier_cells),
            frontier_candidate_count=frontier_assignment.candidate_cell_count,
            current_node_status=current_node_status,
        )

        return GraphUpdateResult(
            graph=self.graph,
            classified_grid=grid,
            frontier_cell_count=len(frontier_cells),
            diagnostics=diagnostics,
        )

    def _update_existing_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
    ) -> None:
        """用可靠局部观测刷新历史节点, unknown 和窗口外区域不否定记忆"""
        for node_id, node in list(self.graph.nodes.items()):
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
        """在机器人当前可达的 free 分量中按固定 stride 采样新节点

        可达分量约束替代旧的机器人高度差过滤, 避免长坡历史被误删
        同时防止在墙体另一侧或不连通高层表面生成规划节点
        """
        stride = max(1, int(self.config.sample_stride))
        for iy in range(0, grid.height, stride):
            for ix in range(0, grid.width, stride):
                if not grid.is_free_index(ix, iy):
                    continue
                if not reachable_free[iy, ix]:
                    continue
                if float(sdf_obstacle[iy, ix]) < self.config.min_obstacle_clearance:
                    continue

                position = grid.grid_to_world(ix, iy)

                if self.graph.nearest_node(
                    position,
                    max_distance=self.config.min_node_separation,
                ) is not None:
                    continue

                node = self.graph.create_node(position=position, stamp_seconds=stamp_seconds)
                node.free_radius = min(
                    float(sdf_obstacle[iy, ix]),
                    float(sdf_unknown[iy, ix]),
                    self.config.max_free_radius,
                )
                node.explored_radius = float(sdf_unknown[iy, ix])

    def _update_current_node(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
        robot_ground_position: Tuple[float, float, float],
        robot_ground_projected: bool,
        stamp_seconds: float,
    ) -> str:
        """把机器人当前位置映射到 NavigationGraph.current_node_idx"""
        best_node = self._nearest_collision_free_node(grid, robot_ground_position)
        current_node_status = "reachable" if best_node is not None else "missing"
        if best_node is None:
            best_node = self._ensure_robot_anchor_node(
                grid,
                robot_ground_position,
                stamp_seconds,
            )
            if best_node is not None:
                current_node_status = "robot_anchor"
        if best_node is None:
            best_node = self.graph.nearest_node(robot_ground_position)
            if best_node is not None:
                current_node_status = "geometry_fallback"
        self.graph.current_node_id = best_node.node_id if best_node is not None else None
        self.graph.update_robot_position(
            robot_position,
            robot_ground_position if robot_ground_projected else None,
        )
        if robot_ground_projected:
            self.graph.append_trajectory_point(
                robot_ground_position,
                0.25,
            )
        return current_node_status

    def _reachable_free_mask(
        self,
        grid: ClassifiedGrid,
        robot_position: Tuple[float, float, float],
    ) -> np.ndarray:
        """计算机器人脚下 free cell 的四连通分量

        脚下 cell 为 unknown 或 obstacle 时返回空 mask, 本帧依靠持久图和 anchor
        不从其他不连通 free 区域采样新节点
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

        queue = deque([start])
        reachable[start[1], start[0]] = True
        while queue:
            current_x, current_y = queue.popleft()
            for offset_x, offset_y in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                next_x = current_x + offset_x
                next_y = current_y + offset_y
                if not grid.is_free_index(next_x, next_y) or reachable[next_y, next_x]:
                    continue
                reachable[next_y, next_x] = True
                queue.append((next_x, next_y))
        return reachable

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

GraphBuilder = SparseGraphBuilder


def _elapsed_ms(start_time: float) -> float:
    """计算阶段耗时, 单位毫秒"""
    return (perf_counter() - start_time) * 1000.0


def _build_graph_update_diagnostics(
    graph: GraphState,
    stage_timings_ms: Dict[str, float],
    frontier_cell_count: int,
    frontier_candidate_count: int,
    current_node_status: str,
) -> GraphUpdateDiagnostics:
    """汇总图规模, 连通性和度数统计, 供 ROS 层打印日志"""
    node_ids = set(graph.nodes.keys())
    degrees = {node_id: 0 for node_id in node_ids}
    adjacency = {node_id: set() for node_id in node_ids}

    for edge in graph.edges.values():
        if edge.from_id not in node_ids or edge.to_id not in node_ids:
            continue
        degrees[edge.from_id] += 1
        degrees[edge.to_id] += 1
        adjacency[edge.from_id].add(edge.to_id)
        adjacency[edge.to_id].add(edge.from_id)

    component_sizes, current_component_size = _component_sizes(adjacency, graph.current_node_id)
    degree_values = list(degrees.values())
    degree_min = min(degree_values) if degree_values else 0
    degree_max = max(degree_values) if degree_values else 0
    degree_avg = sum(degree_values) / len(degree_values) if degree_values else 0.0

    return GraphUpdateDiagnostics(
        stage_timings_ms=dict(stage_timings_ms),
        node_count=len(graph.nodes),
        edge_count=len(graph.edges),
        frontier_node_count=sum(1 for node in graph.nodes.values() if node.is_frontier),
        frontier_cell_count=frontier_cell_count,
        frontier_candidate_count=frontier_candidate_count,
        connected_components=len(component_sizes),
        current_component_size=current_component_size,
        degree_min=degree_min,
        degree_max=degree_max,
        degree_avg=degree_avg,
        current_node_id=graph.current_node_id,
        current_node_status=current_node_status,
    )


def _component_sizes(
    adjacency: Dict[int, set[int]],
    current_node_id: int | None,
) -> tuple[list[int], int]:
    """计算无向图连通分量数量和 current node 所在分量大小"""
    visited = set()
    sizes = []
    current_component_size = 0

    for node_id in adjacency:
        if node_id in visited:
            continue
        stack = [node_id]
        visited.add(node_id)
        component = []
        while stack:
            current = stack.pop()
            component.append(current)
            for neighbor in adjacency[current]:
                if neighbor in visited:
                    continue
                visited.add(neighbor)
                stack.append(neighbor)

        component_size = len(component)
        sizes.append(component_size)
        if current_node_id in component:
            current_component_size = component_size

    return sizes, current_component_size
