from __future__ import annotations

import argparse
from dataclasses import fields
from pathlib import Path
from typing import Any, Dict, Mapping

from ament_index_python.packages import get_package_share_directory
from grid_map_msgs.msg import GridMap
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from graph_construction.graph_builder import GraphBuilderConfig, SparseGraphBuilder
from graph_construction.grid_adapter import classify_grid_map
from graph_construction.msg_utils import graph_to_msg
from graph_construction.viz import GraphVisualizer


DEFAULT_CONFIG: Dict[str, Any] = {
    "global_frame": "map",
    "odom_topic": "/odom",
    "grid_map_topic": "/elevation_mapping_node/elevation_map_raw",
    "nav_graph_topic": "/spot1/nav_graph",
    "viz_topic": "/spot1/graph_construction_viz",
    "viz_show_radius_markers": False,
    "publish_rate_hz": 2.0,
    "grid_map_traversability_layer": "traversability",
    "grid_map_elevation_layer": "elevation",
    "grid_map_free_threshold": 0.2,
    "grid_map_obstacle_threshold": 0.05,
    "grid_map_normalize_traversability": True,
    "grid_map_normalize_low_quantile": 0.05,
    "grid_map_normalize_high_quantile": 0.95,
    "grid_map_z_offset": 0.08,
    "grid_map_min_free_component_cells": 25,
    "grid_map_fill_hole_max_cells": 90,
    "grid_map_fill_hole_min_free_neighbor_ratio": 0.65,
    "grid_map_fill_elevation_radius_cells": 5,
    "grid_map_majority_fill_iterations": 1,
    "grid_map_majority_fill_min_neighbors": 6,
}

TRAVERSABILITY_CLASS = "default"


class GraphConstructionNode(Node):
    """发布稀疏 NavigationGraph 的 ROS2 节点

    节点订阅 odom 和几何地图输入
    每次 timer 触发时, 使用最新两类消息更新内部图并发布 NavigationGraph
    视觉评分和路径规划由 WildOS 已有节点继续处理
    """

    def __init__(self, config: Dict[str, Any]) -> None:
        super().__init__("graph_construction")
        self.config = _resolve_config(config)
        self.builder = SparseGraphBuilder(_builder_config(self.config))
        self.visualizer = GraphVisualizer(
            show_radius_markers=bool(
                self.config.get("viz_show_radius_markers", False)
            )
        )

        self.latest_grid = None
        self.latest_odom = None
        self._logged_first_grid = False
        self._logged_first_odom = False
        self._logged_first_publish = False

        self.nav_graph_pub = self.create_publisher(
            NavigationGraph,
            self.config["nav_graph_topic"],
            10,
        )
        self.viz_pub = self.create_publisher(MarkerArray, self.config["viz_topic"], 10)

        self.create_subscription(
            GridMap,
            self.config["grid_map_topic"],
            self._on_grid_map,
            10,
        )

        self.create_subscription(
            Odometry,
            self.config["odom_topic"],
            self._on_odom,
            10,
        )

        publish_rate = float(self.config["publish_rate_hz"])
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"Graph construction 已启动, grid_map={self.config['grid_map_topic']}, "
            f"odom={self.config['odom_topic']}"
        )

    def _on_grid_map(self, msg: GridMap) -> None:
        """缓存最新 GridMap, 用 elevation 和 traversability layer 构建图"""
        self.latest_grid = msg
        if not self._logged_first_grid:
            self.get_logger().info(
                f"收到第一帧 GridMap, frame={msg.header.frame_id}, layers={list(msg.layers)}"
            )
            self._logged_first_grid = True

    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新 odom, current_node_idx 计算依赖它"""
        self.latest_odom = msg
        if not self._logged_first_odom:
            position = msg.pose.pose.position
            self.get_logger().info(
                f"收到第一帧 odom, frame={msg.header.frame_id}, "
                f"position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
            )
            self._logged_first_odom = True

    def _on_timer(self) -> None:
        """周期性构建并发布导航图

        节点使用最新 grid 和 odom 快照, 不要求时间戳严格同步
        地图解码或图更新失败时保留历史图, 当前周期不发布部分结果
        graph message 和 marker 使用同一份更新结果, 避免可视化与规划图不一致
        """
        if self.latest_grid is None or self.latest_odom is None:
            return

        try:
            classified_grid = self._classify_latest_grid()
            update_result = self.builder.update(
                classified_grid,
                _robot_position(self.latest_odom),
                _stamp_to_seconds(self.latest_grid.header),
            )
        except Exception as exc:
            self.get_logger().error(f"更新导航图失败: {exc}")
            return

        header = self._graph_header(self.latest_grid.header, classified_grid.frame_id)
        nav_graph = graph_to_msg(update_result.graph, header, TRAVERSABILITY_CLASS)

        self.nav_graph_pub.publish(nav_graph)

        self.viz_pub.publish(
            self.visualizer.build_markers(
                update_result.graph,
                header,
            )
        )

        if not self._logged_first_publish:
            frontier_count = sum(
                1
                for node in nav_graph.nodes
                if node.trav_properties and node.trav_properties[0].is_frontier
            )
            grid_stats = _format_grid_stats(update_result.classified_grid.stats)
            robot_stats = _format_robot_stats(
                self.builder.graph.latest_robot_odom_position,
                self.builder.graph.latest_robot_position,
                self.builder.graph.latest_robot_ground_projected,
            )
            self.get_logger().info(
                f"已发布第一帧导航图, nodes={len(nav_graph.nodes)}, edges={len(nav_graph.edges)}, "
                f"frontier={frontier_count}{grid_stats}{robot_stats}"
            )
            self._logged_first_publish = True

    def _classify_latest_grid(self):
        """按配置的输入后端把 ROS 地图解码为统一 ClassifiedGrid

        GridMap 的 rolling buffer 已由 grid_adapter 解包, 这里固定启用经过验证的
        拓扑后处理和标准轴约定, 避免重新开放会破坏坐标一致性的历史调试开关
        """
        return classify_grid_map(
            self.latest_grid,
            traversability_layer=self.config["grid_map_traversability_layer"],
            elevation_layer=self.config["grid_map_elevation_layer"],
            free_threshold=self.config["grid_map_free_threshold"],
            obstacle_threshold=self.config["grid_map_obstacle_threshold"],
            normalize_traversability=self.config["grid_map_normalize_traversability"],
            normalize_low_quantile=self.config["grid_map_normalize_low_quantile"],
            normalize_high_quantile=self.config["grid_map_normalize_high_quantile"],
            z_offset=self.config["grid_map_z_offset"],
            min_free_component_cells=self.config["grid_map_min_free_component_cells"],
            fill_hole_max_cells=self.config["grid_map_fill_hole_max_cells"],
            fill_hole_min_free_neighbor_ratio=self.config["grid_map_fill_hole_min_free_neighbor_ratio"],
            majority_fill_iterations=self.config["grid_map_majority_fill_iterations"],
            majority_fill_min_neighbors=self.config["grid_map_majority_fill_min_neighbors"],
            fill_elevation_radius_cells=self.config["grid_map_fill_elevation_radius_cells"],
        )

    def _graph_header(self, input_header, fallback_frame: str):
        """构造 ROS 适配层负责的输出 graph header"""
        header = Header()
        header.stamp = input_header.stamp
        header.frame_id = self.config["global_frame"] or fallback_frame
        return header


def _resolve_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    """合并稳定默认值并验证 Graph Construction 配置契约

    ROS 适配默认值来自 DEFAULT_CONFIG, 纯算法默认值来自 GraphBuilderConfig
    未知字段直接报错, 避免拼写错误或已删除参数被静默忽略
    """
    allowed_keys = set(DEFAULT_CONFIG)
    allowed_keys.update(field.name for field in fields(GraphBuilderConfig))
    unknown_keys = sorted(set(config) - allowed_keys)
    if unknown_keys:
        raise ValueError(f"Unknown graph construction config keys: {', '.join(unknown_keys)}")

    resolved = {**DEFAULT_CONFIG, **config}
    if float(resolved["publish_rate_hz"]) <= 0.0:
        raise ValueError("publish_rate_hz must be greater than 0")
    if float(resolved["grid_map_obstacle_threshold"]) > float(resolved["grid_map_free_threshold"]):
        raise ValueError("grid_map_obstacle_threshold must not exceed grid_map_free_threshold")

    low_quantile = float(resolved["grid_map_normalize_low_quantile"])
    high_quantile = float(resolved["grid_map_normalize_high_quantile"])
    if not 0.0 <= low_quantile < high_quantile <= 1.0:
        raise ValueError("GridMap normalization quantiles must satisfy 0 <= low < high <= 1")
    return resolved


def _builder_config(config: Dict[str, Any]) -> GraphBuilderConfig:
    """只把纯算法字段传给 GraphBuilderConfig"""
    allowed = {field.name for field in fields(GraphBuilderConfig)}
    values = {
        key: value
        for key, value in config.items()
        if key in allowed
    }
    return GraphBuilderConfig(**values)


def _format_grid_stats(stats: Any) -> str:
    """格式化首帧发布诊断用的 grid 分类统计"""
    if not stats:
        return ""
    return (
        f", 有效={stats.get('valid', 0)}, 原始free={stats.get('raw_free', 0)}, "
        f"原始obstacle={stats.get('raw_obstacle', 0)}, 原始unknown={stats.get('raw_unknown', 0)}, "
        f"free={stats.get('free', 0)}, obstacle={stats.get('obstacle', 0)}, "
        f"unknown={stats.get('unknown', 0)}, "
        f"脚下盲区修补={stats.get('robot_blind_zone_filled', 0)}, "
        f"盲区状态={stats.get('robot_blind_zone_status', 'unknown')}, "
        f"地面来源={stats.get('robot_blind_zone_ground_source', 'none')}, "
        f"最近地面={stats.get('robot_blind_zone_nearest_ground', 'none')}m"
    )


def _format_robot_stats(odom_position: Any, ground_position: Any, ground_projected: bool) -> str:
    """格式化机器人 odom 和 GridMap 投影位置诊断信息"""
    if odom_position is None:
        return ""
    if ground_position is None:
        return (
            f", 机器人odom=({odom_position[0]:.3f}, {odom_position[1]:.3f}, {odom_position[2]:.3f}), "
            "机器人地面投影=缺失"
        )
    return (
        f", 机器人odom=({odom_position[0]:.3f}, {odom_position[1]:.3f}, {odom_position[2]:.3f}), "
        f"机器人地面=({ground_position[0]:.3f}, {ground_position[1]:.3f}, {ground_position[2]:.3f}), "
        f"机器人地面投影={'正常' if ground_projected else '缺失'}"
    )


def _stamp_to_seconds(header: Any) -> float:
    """将 ROS header stamp 转成纯算法层使用的秒"""
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def _robot_position(odom_msg: Odometry):
    """从 Odometry 提取纯算法层使用的机器人 XYZ"""
    return (
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z,
    )


def _load_config(config_name: str, overrides: list[str] | None = None) -> Dict[str, Any]:
    """加载 YAML 配置并应用 launch 覆盖

    相对路径从已安装 package share 的 configs 目录解析
    缺失文件, 缺失 YAML 依赖和非 mapping 顶层结构都视为启动错误
    """
    config_path = Path(config_name)
    if not config_path.is_absolute():
        share_dir = Path(get_package_share_directory("graph_construction"))
        config_path = share_dir / "configs" / config_name

    if not config_path.exists():
        raise FileNotFoundError(f"Graph construction config not found: {config_path}")

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load graph construction config") from exc

    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Graph construction config must be a mapping: {config_path}")
    return _resolve_config(_apply_config_overrides(loaded, overrides or []))


def _apply_config_overrides(config: Dict[str, Any], overrides: list[str]) -> Dict[str, Any]:
    """解析 launch 传入的 key=value 标量覆盖项"""
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to parse graph construction overrides") from exc

    for item in overrides:
        if "=" not in item:
            raise ValueError(f"Invalid config override '{item}', expected key=value")
        key, raw_value = item.split("=", 1)
        config[key] = yaml.safe_load(raw_value)
    return config


def main(args=None) -> None:
    """ROS2 控制台脚本入口"""
    rclpy.init(args=args)
    custom_args = rclpy.utilities.remove_ros_args(args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="graph_construction.yaml")
    parser.add_argument("--config-override", action="append", default=[])
    parsed = parser.parse_args(custom_args[1:] if custom_args else [])

    node = GraphConstructionNode(_load_config(parsed.config, parsed.config_override))
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
