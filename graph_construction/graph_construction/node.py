from __future__ import annotations

import argparse
from dataclasses import fields
from pathlib import Path
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
from grid_map_msgs.msg import GridMap
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from graph_construction.graph_builder import GraphBuilder, GraphBuilderConfig
from graph_construction.viz import GraphVisualizer


DEFAULT_CONFIG: Dict[str, Any] = {
    "global_frame": "map",
    "odom_topic": "/unity/odom",
    "grid_input_type": "occupancy_grid",
    "grid_topic": "/spot1/traversability_grid",
    "grid_map_topic": "/elevation_mapping_node/elevation_map_raw",
    "nav_graph_topic": "/spot1/nav_graph",
    "viz_topic": "/spot1/graph_construction_viz",
    "trav_class": "default",
    "publish_rate_hz": 2.0,
    "free_threshold": 20,
    "obstacle_threshold": 65,
    "grid_map_traversability_layer": "traversability",
    "grid_map_elevation_layer": "elevation",
    "grid_map_free_threshold": 0.2,
    "grid_map_obstacle_threshold": 0.05,
    "grid_map_normalize_traversability": True,
    "grid_map_normalize_low_quantile": 0.05,
    "grid_map_normalize_high_quantile": 0.95,
    "grid_map_z_offset": 0.08,
    "grid_map_enable_postprocess": True,
    "grid_map_min_free_component_cells": 25,
    "grid_map_fill_hole_max_cells": 90,
    "grid_map_fill_hole_min_free_neighbor_ratio": 0.65,
    "grid_map_majority_fill_iterations": 1,
    "grid_map_majority_fill_min_neighbors": 6,
    "grid_map_max_node_odom_z_delta": 1.5,
    "grid_map_transpose": False,
    "grid_map_flip_x": False,
    "grid_map_flip_y": False,
    "sample_stride": 8,
    "min_node_separation": 1.0,
    "max_free_radius": 4.0,
    "min_obstacle_clearance": 0.5,
    "edge_radius": 8.0,
    "max_edge_neighbors": 4,
    "current_node_max_edge_neighbors": 12,
    "frontier_assign_radius": 5.0,
    "frontier_min_points": 4,
    "frontier_min_span": 0.6,
    "frontier_border_margin": 0.8,
    "deadend_observation_count": 3,
    "removed_frontier_suppression_radius": 1.0,
    "trajectory_min_separation": 0.25,
}


class GraphConstructionNode(Node):
    """发布稀疏 NavigationGraph 的 ROS2 节点

    节点订阅 odom 和几何地图输入
    每次 timer 触发时, 使用最新两类消息更新内部图并发布 NavigationGraph
    视觉评分和路径规划由 WildOS 已有节点继续处理
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        super().__init__("graph_construction")
        self.config = {**DEFAULT_CONFIG, **config}
        self.builder = GraphBuilder(_builder_config(self.config))
        self.visualizer = GraphVisualizer()

        self.latest_grid = None
        self.latest_odom = None
        self.grid_input_type = str(self.config["grid_input_type"])
        self._logged_first_grid = False
        self._logged_first_odom = False
        self._logged_first_publish = False

        self.nav_graph_pub = self.create_publisher(
            NavigationGraph,
            self.config["nav_graph_topic"],
            10,
        )
        self.viz_pub = self.create_publisher(MarkerArray, self.config["viz_topic"], 10)

        if self.grid_input_type == "grid_map":
            self.create_subscription(
                GridMap,
                self.config["grid_map_topic"],
                self._on_grid_map,
                10,
            )
            input_topic = self.config["grid_map_topic"]
        else:
            self.create_subscription(
                OccupancyGrid,
                self.config["grid_topic"],
                self._on_grid,
                10,
            )
            input_topic = self.config["grid_topic"]

        self.create_subscription(
            Odometry,
            self.config["odom_topic"],
            self._on_odom,
            10,
        )

        publish_rate = max(0.1, float(self.config["publish_rate_hz"]))
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"Graph construction started, input_type={self.grid_input_type}, grid={input_topic}, "
            f"odom={self.config['odom_topic']}"
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        """缓存最新 grid, 避免在 subscriber 回调里做重计算"""
        self.latest_grid = msg
        if not self._logged_first_grid:
            self.get_logger().info(
                f"Received first grid, frame={msg.header.frame_id}, size={msg.info.width}x{msg.info.height}"
            )
            self._logged_first_grid = True

    def _on_grid_map(self, msg: GridMap) -> None:
        """缓存最新 GridMap, 用 elevation 和 traversability layer 构建图"""
        self.latest_grid = msg
        if not self._logged_first_grid:
            self.get_logger().info(
                f"Received first GridMap, frame={msg.header.frame_id}, layers={list(msg.layers)}"
            )
            self._logged_first_grid = True

    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新 odom, current_node_idx 计算依赖它"""
        self.latest_odom = msg
        if not self._logged_first_odom:
            position = msg.pose.pose.position
            self.get_logger().info(
                f"Received first odom, frame={msg.header.frame_id}, "
                f"position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
            )
            self._logged_first_odom = True

    def _on_timer(self) -> None:
        """周期性构建并发布导航图

        这里不做 message_filters 同步, 第一版优先保证简单可跑
        如果后续发现 grid 和 odom 时间差影响较大, 再换成近似时间同步
        """
        if self.latest_grid is None or self.latest_odom is None:
            return

        try:
            if self.grid_input_type == "grid_map":
                nav_graph, header, classified_grid = self.builder.update_grid_map(self.latest_grid, self.latest_odom)
            else:
                nav_graph, header, classified_grid = self.builder.update_occupancy_grid(
                    self.latest_grid,
                    self.latest_odom,
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to update navigation graph: {exc}")
            return

        self.nav_graph_pub.publish(nav_graph)
        self.viz_pub.publish(self.visualizer.build_markers(self.builder.graph, header, classified_grid))
        if not self._logged_first_publish:
            frontier_count = sum(
                1
                for node in nav_graph.nodes
                if node.trav_properties and node.trav_properties[0].is_frontier
            )
            grid_stats = _format_grid_stats(classified_grid.stats)
            robot_stats = _format_robot_stats(
                self.builder.graph.latest_robot_odom_position,
                self.builder.graph.latest_robot_position,
            )
            self.get_logger().info(
                f"Published first graph, nodes={len(nav_graph.nodes)}, edges={len(nav_graph.edges)}, "
                f"frontier={frontier_count}{grid_stats}{robot_stats}"
            )
            self._logged_first_publish = True


def _builder_config(config: Dict[str, Any]) -> GraphBuilderConfig:
    """从完整 ROS 配置中提取 GraphBuilder 需要的字段"""
    allowed = {field.name for field in fields(GraphBuilderConfig)}
    values = {
        key: value
        for key, value in config.items()
        if key in allowed
    }
    return GraphBuilderConfig(**values)


def _format_grid_stats(stats: Any) -> str:
    """Format optional grid classification stats for first-publish diagnostics"""
    if not stats:
        return ""
    return (
        f", valid={stats.get('valid', 0)}, raw_free={stats.get('raw_free', 0)}, "
        f"raw_obstacle={stats.get('raw_obstacle', 0)}, raw_unknown={stats.get('raw_unknown', 0)}, "
        f"free={stats.get('free', 0)}, obstacle={stats.get('obstacle', 0)}, unknown={stats.get('unknown', 0)}"
    )


def _format_robot_stats(odom_position: Any, ground_position: Any) -> str:
    """Format robot odom and GridMap-projected positions for diagnostics"""
    if odom_position is None or ground_position is None:
        return ""
    return (
        f", robot_odom=({odom_position[0]:.3f}, {odom_position[1]:.3f}, {odom_position[2]:.3f}), "
        f"robot_ground=({ground_position[0]:.3f}, {ground_position[1]:.3f}, {ground_position[2]:.3f})"
    )


def _load_config(config_name: str) -> Dict[str, Any]:
    """加载安装目录或绝对路径中的 yaml 配置"""
    config_path = Path(config_name)
    if not config_path.is_absolute():
        share_dir = Path(get_package_share_directory("graph_construction"))
        config_path = share_dir / "configs" / config_name

    if not config_path.exists():
        return {}

    try:
        import yaml
    except ImportError:
        return {}

    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}
    return loaded


def main(args=None) -> None:
    """ROS2 console script 入口"""
    rclpy.init(args=args)
    custom_args = rclpy.utilities.remove_ros_args(args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="graph_construction.yaml")
    parsed = parser.parse_args(custom_args[1:] if custom_args else [])

    node = GraphConstructionNode(_load_config(parsed.config))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
