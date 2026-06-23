from __future__ import annotations

import argparse
from dataclasses import fields
from pathlib import Path
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from graph_construction.graph_builder import GraphBuilder, GraphBuilderConfig
from graph_construction.viz import GraphVisualizer


DEFAULT_CONFIG: Dict[str, Any] = {
    "global_frame": "spot1/odom",
    "odom_topic": "/spot1/odom",
    "grid_topic": "/spot1/traversability_grid",
    "nav_graph_topic": "/spot1/nav_graph",
    "viz_topic": "/spot1/graph_construction_viz",
    "trav_class": "default",
    "publish_rate_hz": 2.0,
    "free_threshold": 20,
    "obstacle_threshold": 65,
    "sample_stride": 8,
    "min_node_separation": 1.0,
    "max_free_radius": 4.0,
    "min_obstacle_clearance": 0.5,
    "edge_radius": 8.0,
    "frontier_assign_radius": 5.0,
    "frontier_min_points": 2,
    "deadend_observation_count": 3,
    "removed_frontier_suppression_radius": 1.0,
}


class GraphConstructionNode(Node):
    """发布稀疏 NavigationGraph 的 ROS2 节点

    节点只订阅 odom 和 traversability grid
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

        self.nav_graph_pub = self.create_publisher(
            NavigationGraph,
            self.config["nav_graph_topic"],
            10,
        )
        self.viz_pub = self.create_publisher(MarkerArray, self.config["viz_topic"], 10)

        self.create_subscription(
            OccupancyGrid,
            self.config["grid_topic"],
            self._on_grid,
            10,
        )
        self.create_subscription(
            Odometry,
            self.config["odom_topic"],
            self._on_odom,
            10,
        )

        publish_rate = max(0.1, float(self.config["publish_rate_hz"]))
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"Graph construction started, grid={self.config['grid_topic']}, odom={self.config['odom_topic']}"
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        """缓存最新 grid, 避免在 subscriber 回调里做重计算"""
        self.latest_grid = msg

    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新 odom, current_node_idx 计算依赖它"""
        self.latest_odom = msg

    def _on_timer(self) -> None:
        """周期性构建并发布导航图

        这里不做 message_filters 同步, 第一版优先保证简单可跑
        如果后续发现 grid 和 odom 时间差影响较大, 再换成近似时间同步
        """
        if self.latest_grid is None or self.latest_odom is None:
            return

        try:
            nav_graph, header = self.builder.update(self.latest_grid, self.latest_odom)
        except Exception as exc:
            self.get_logger().error(f"Failed to update navigation graph: {exc}")
            return

        self.nav_graph_pub.publish(nav_graph)
        self.viz_pub.publish(self.visualizer.build_markers(self.builder.graph, header))


def _builder_config(config: Dict[str, Any]) -> GraphBuilderConfig:
    """从完整 ROS 配置中提取 GraphBuilder 需要的字段"""
    allowed = {field.name for field in fields(GraphBuilderConfig)}
    values = {
        key: value
        for key, value in config.items()
        if key in allowed
    }
    return GraphBuilderConfig(**values)


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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
