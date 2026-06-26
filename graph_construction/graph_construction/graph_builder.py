from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Tuple

from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header

from graph_construction.deadend_recovery import DeadendRecovery
from graph_construction.edge_builder import EdgeBuilder
from graph_construction.frontier_detector import FrontierDetector
from graph_construction.graph_memory import GraphState
from graph_construction.grid_adapter import (
    ClassifiedGrid,
    classify_grid_map,
    classify_occupancy_grid,
    distance_to_mask,
)
from graph_construction.msg_utils import graph_to_msg


@dataclass
class GraphBuilderConfig:
    """图构建运行参数

    这些参数对应论文中的局部地图半径, 节点采样间距, free radius, edge radius 等概念
    第一版参数以可调试和保守为主, 不追求完全复现论文未开源实现
    """

    global_frame: str = "spot1/odom"
    trav_class: str = "default"
    free_threshold: int = 20
    obstacle_threshold: int = 65
    grid_map_traversability_layer: str = "traversability"
    grid_map_elevation_layer: str = "elevation"
    grid_map_free_threshold: float = 0.2
    grid_map_obstacle_threshold: float = 0.05
    grid_map_normalize_traversability: bool = True
    grid_map_normalize_low_quantile: float = 0.05
    grid_map_normalize_high_quantile: float = 0.95
    grid_map_z_offset: float = 0.08
    grid_map_enable_postprocess: bool = True
    grid_map_min_free_component_cells: int = 25
    grid_map_fill_hole_max_cells: int = 90
    grid_map_fill_hole_min_free_neighbor_ratio: float = 0.65
    grid_map_majority_fill_iterations: int = 1
    grid_map_majority_fill_min_neighbors: int = 6
    grid_map_max_node_odom_z_delta: float = 1.5
    grid_map_transpose: bool = False
    grid_map_flip_x: bool = False
    grid_map_flip_y: bool = False
    sample_stride: int = 8
    min_node_separation: float = 1.0
    max_free_radius: float = 4.0
    min_obstacle_clearance: float = 0.5
    edge_radius: float = 8.0
    max_edge_neighbors: int = 4
    current_node_max_edge_neighbors: int = 12
    frontier_assign_radius: float = 5.0
    frontier_min_points: int = 4
    frontier_min_span: float = 0.6
    frontier_border_margin: float = 0.8
    deadend_observation_count: int = 3
    removed_frontier_suppression_radius: float = 1.0
    trajectory_min_separation: float = 0.25


class GraphBuilder:
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
            removed_frontier_suppression_radius=config.removed_frontier_suppression_radius,
        )
        self.edge_builder = EdgeBuilder(
            edge_radius=config.edge_radius,
            max_neighbors_per_node=config.max_edge_neighbors,
            current_node_max_neighbors=config.current_node_max_edge_neighbors,
        )
        self.deadend_recovery = DeadendRecovery(
            observation_count=config.deadend_observation_count,
            suppression_radius=config.removed_frontier_suppression_radius,
        )

    def update(self, grid_msg: OccupancyGrid, odom_msg: Odometry):
        """兼容旧调用, 使用 OccupancyGrid 更新 NavigationGraph"""
        return self.update_occupancy_grid(grid_msg, odom_msg)

    def update_occupancy_grid(self, grid_msg: OccupancyGrid, odom_msg: Odometry):
        """处理一帧 OccupancyGrid 和 odom, 返回 NavigationGraph 消息, header 和分类 grid"""
        grid = classify_occupancy_grid(
            grid_msg,
            free_threshold=self.config.free_threshold,
            obstacle_threshold=self.config.obstacle_threshold,
        )
        return self._update_classified_grid(grid, grid_msg.header, odom_msg)

    def update_grid_map(self, grid_msg: GridMap, odom_msg: Odometry):
        """处理一帧 GridMap 和 odom, 返回 NavigationGraph 消息, header 和分类 grid"""
        grid = classify_grid_map(
            grid_msg,
            traversability_layer=self.config.grid_map_traversability_layer,
            elevation_layer=self.config.grid_map_elevation_layer,
            free_threshold=self.config.grid_map_free_threshold,
            obstacle_threshold=self.config.grid_map_obstacle_threshold,
            normalize_traversability=self.config.grid_map_normalize_traversability,
            normalize_low_quantile=self.config.grid_map_normalize_low_quantile,
            normalize_high_quantile=self.config.grid_map_normalize_high_quantile,
            z_offset=self.config.grid_map_z_offset,
            enable_postprocess=self.config.grid_map_enable_postprocess,
            min_free_component_cells=self.config.grid_map_min_free_component_cells,
            fill_hole_max_cells=self.config.grid_map_fill_hole_max_cells,
            fill_hole_min_free_neighbor_ratio=self.config.grid_map_fill_hole_min_free_neighbor_ratio,
            majority_fill_iterations=self.config.grid_map_majority_fill_iterations,
            majority_fill_min_neighbors=self.config.grid_map_majority_fill_min_neighbors,
            transpose=self.config.grid_map_transpose,
            flip_x=self.config.grid_map_flip_x,
            flip_y=self.config.grid_map_flip_y,
        )
        return self._update_classified_grid(grid, grid_msg.header, odom_msg)

    def _update_classified_grid(self, grid: ClassifiedGrid, grid_header: Header, odom_msg: Odometry):
        """Run source-agnostic graph update on free, obstacle, unknown masks"""
        robot_position = _robot_position(odom_msg)
        robot_ground_position = grid.project_to_elevation(robot_position)

        # obstacle 距离场用于估计节点安全半径
        # unknown 距离场用于估计已探索半径和 frontier 生命周期
        sdf_obstacle = distance_to_mask(grid.obstacle, grid.resolution)
        sdf_unknown = distance_to_mask(grid.unknown, grid.resolution)
        stamp_seconds = _stamp_to_seconds(grid_header)

        # 先处理旧节点, 保证已经变成 obstacle 的节点不会继续参与采样和建边
        self._update_existing_nodes(grid, sdf_obstacle, sdf_unknown, stamp_seconds, robot_ground_position)

        # 再在 free 区域补充节点, 让稀疏图持续覆盖当前局部地图
        self._sample_new_nodes(grid, sdf_obstacle, sdf_unknown, stamp_seconds, robot_ground_position)

        # frontier cell 是 free 和 unknown 的边界, 后续会被聚合到附近 graph node 上
        frontier_cells = self.frontier_detector.detect_frontier_cells(grid)
        self.frontier_detector.assign_frontiers(self.graph, grid, frontier_cells)
        self.deadend_recovery.update(self.graph)

        self._update_current_node(grid, robot_position, robot_ground_position)

        # 边需要在 current node 更新后重建, 机器人附近允许保留更多局部连接
        self.graph.set_edges(self.edge_builder.build_edges(self.graph, grid))

        header = Header()
        header.stamp = grid_header.stamp
        header.frame_id = self.config.global_frame or grid.frame_id
        return graph_to_msg(self.graph, header, self.config.trav_class), header, grid

    def _update_existing_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
        robot_ground_position: Tuple[float, float, float],
    ) -> None:
        """根据最新局部地图刷新已有节点的半径和有效性"""
        for node_id, node in list(self.graph.nodes.items()):
            grid_index = grid.world_to_grid(node.position[0], node.position[1])
            if grid_index is None:
                continue

            ix, iy = grid_index
            if not grid.is_free_index(ix, iy):
                self.graph.remove_node(node_id)
                continue
            if not self._node_height_is_near_robot(grid, node.position, robot_ground_position):
                self.graph.remove_node(node_id)
                continue

            free_radius = min(
                float(sdf_obstacle[iy, ix]),
                float(sdf_unknown[iy, ix]),
                self.config.max_free_radius,
            )

            # clearance 过小的节点不应继续作为路径图节点, 否则 planner 会走到障碍附近
            if free_radius < self.config.min_obstacle_clearance:
                self.graph.remove_node(node_id)
                continue

            node.free_radius = free_radius
            node.explored_radius = max(node.explored_radius, float(sdf_unknown[iy, ix]))
            node.last_seen_time = stamp_seconds

    def _sample_new_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
        robot_ground_position: Tuple[float, float, float],
    ) -> None:
        """在 free 区域按固定 stride 采样新节点

        使用规则采样是为了第一版输出稳定, 便于和 RViz 结果对齐调试
        如果后续需要更接近论文, 可以替换为随机采样 N_samples
        """
        stride = max(1, int(self.config.sample_stride))
        for iy in range(0, grid.height, stride):
            for ix in range(0, grid.width, stride):
                if not grid.is_free_index(ix, iy):
                    continue
                if float(sdf_obstacle[iy, ix]) < self.config.min_obstacle_clearance:
                    continue

                position = grid.grid_to_world(ix, iy)
                if not self._node_height_is_near_robot(grid, position, robot_ground_position):
                    continue
                if self.graph.nearest_node(position, max_distance=self.config.min_node_separation) is not None:
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
    ) -> None:
        """把机器人当前位置映射到 NavigationGraph.current_node_idx"""
        best_node = self._nearest_collision_free_node(grid, robot_ground_position)
        if best_node is None:
            best_node = self.graph.nearest_node(robot_ground_position)
        self.graph.current_node_id = best_node.node_id if best_node is not None else None
        self.graph.update_robot_position(robot_position, robot_ground_position)
        self.graph.append_trajectory_point(robot_ground_position, self.config.trajectory_min_separation)

    def _node_height_is_near_robot(
        self,
        grid: ClassifiedGrid,
        node_position: Tuple[float, float, float],
        robot_position: Tuple[float, float, float] | None = None,
    ) -> bool:
        """过滤明显不在机器人局部地面高度附近的 GridMap cell"""
        if grid.elevation is None or self.config.grid_map_max_node_odom_z_delta <= 0.0:
            return True
        reference = robot_position if robot_position is not None else self.graph.latest_robot_position
        if reference is None:
            return True
        return abs(float(node_position[2]) - float(reference[2])) <= self.config.grid_map_max_node_odom_z_delta

    def _nearest_collision_free_node(self, grid: ClassifiedGrid, position: Tuple[float, float, float]):
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


def _stamp_to_seconds(header: Header) -> float:
    """将 ROS 时间戳转换为秒, 便于内部状态记录"""
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def _robot_position(odom_msg: Odometry) -> Tuple[float, float, float]:
    """从 odom 中提取机器人当前位置"""
    return (
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z,
    )
