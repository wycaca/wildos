from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Tuple

from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header

from graph_construction.deadend_recovery import DeadendRecovery
from graph_construction.edge_builder import EdgeBuilder
from graph_construction.frontier_detector import FrontierDetector
from graph_construction.graph_memory import GraphState
from graph_construction.grid_adapter import (
    ClassifiedGrid,
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
    sample_stride: int = 8
    min_node_separation: float = 1.0
    max_free_radius: float = 4.0
    min_obstacle_clearance: float = 0.5
    edge_radius: float = 8.0
    frontier_assign_radius: float = 5.0
    frontier_min_points: int = 2
    deadend_observation_count: int = 3
    removed_frontier_suppression_radius: float = 1.0


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
            removed_frontier_suppression_radius=config.removed_frontier_suppression_radius,
        )
        self.edge_builder = EdgeBuilder(edge_radius=config.edge_radius)
        self.deadend_recovery = DeadendRecovery(
            observation_count=config.deadend_observation_count,
            suppression_radius=config.removed_frontier_suppression_radius,
        )

    def update(self, grid_msg: OccupancyGrid, odom_msg: Odometry):
        """处理一帧 grid 和 odom, 返回 NavigationGraph 消息和可视化 header"""
        grid = classify_occupancy_grid(
            grid_msg,
            free_threshold=self.config.free_threshold,
            obstacle_threshold=self.config.obstacle_threshold,
        )

        # obstacle 距离场用于估计节点安全半径
        # unknown 距离场用于估计已探索半径和 frontier 生命周期
        sdf_obstacle = distance_to_mask(grid.obstacle, grid.resolution)
        sdf_unknown = distance_to_mask(grid.unknown, grid.resolution)
        stamp_seconds = _stamp_to_seconds(grid_msg.header)

        # 先处理旧节点, 保证已经变成 obstacle 的节点不会继续参与采样和建边
        self._update_existing_nodes(grid, sdf_obstacle, sdf_unknown, stamp_seconds)

        # 再在 free 区域补充节点, 让稀疏图持续覆盖当前局部地图
        self._sample_new_nodes(grid, sdf_obstacle, sdf_unknown, stamp_seconds)

        # frontier cell 是 free 和 unknown 的边界, 后续会被聚合到附近 graph node 上
        frontier_cells = self.frontier_detector.detect_frontier_cells(grid)
        self.frontier_detector.assign_frontiers(self.graph, grid, frontier_cells)
        self.deadend_recovery.update(self.graph)

        # 边需要在 frontier 更新后重建, 因为节点删除或新增会改变连通性
        self.graph.set_edges(self.edge_builder.build_edges(self.graph, grid))
        self._update_current_node(grid, odom_msg)

        header = Header()
        header.stamp = grid_msg.header.stamp
        header.frame_id = self.config.global_frame or grid.frame_id
        return graph_to_msg(self.graph, header, self.config.trav_class), header

    def _update_existing_nodes(
        self,
        grid: ClassifiedGrid,
        sdf_obstacle,
        sdf_unknown,
        stamp_seconds: float,
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
                if self.graph.nearest_node(position, max_distance=self.config.min_node_separation) is not None:
                    continue

                node = self.graph.create_node(position=position, stamp_seconds=stamp_seconds)
                node.free_radius = min(
                    float(sdf_obstacle[iy, ix]),
                    float(sdf_unknown[iy, ix]),
                    self.config.max_free_radius,
                )
                node.explored_radius = float(sdf_unknown[iy, ix])

    def _update_current_node(self, grid: ClassifiedGrid, odom_msg: Odometry) -> None:
        """把机器人当前位置映射到 NavigationGraph.current_node_idx"""
        robot_position = (
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
        )
        best_node = self._nearest_collision_free_node(grid, robot_position)
        if best_node is None:
            best_node = self.graph.nearest_node(robot_position)
        self.graph.current_node_id = best_node.node_id if best_node is not None else None

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
