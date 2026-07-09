from __future__ import annotations

import argparse
from dataclasses import fields
from pathlib import Path
from time import perf_counter
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
from grid_map_msgs.msg import GridMap
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from graph_construction.graph_builder import GraphBuilder, GraphBuilderConfig
from graph_construction.grid_adapter import classify_grid_map, classify_occupancy_grid
from graph_construction.msg_utils import graph_to_msg
from graph_construction.viz import GraphVisualizer


DEFAULT_CONFIG: Dict[str, Any] = {
    "global_frame": "map",
    "odom_topic": "/odom",
    "grid_input_type": "occupancy_grid",
    "grid_topic": "/spot1/traversability_grid",
    "grid_map_topic": "/elevation_mapping_node/elevation_map_raw",
    "nav_graph_topic": "/spot1/nav_graph",
    "viz_topic": "/spot1/graph_construction_viz",
    "trav_class": "default",
    "publish_rate_hz": 2.0,
    "diagnostics_log_period_sec": 5.0,
    "slow_update_warning_ms": 200.0,
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
    "prune_disconnected_nodes": False,
    "frontier_assign_radius": 5.0,
    "frontier_min_points": 4,
    "frontier_min_span": 0.6,
    "frontier_border_margin": 0.8,
    "frontier_candidate_spacing": 0.0,
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
        self._last_diagnostics_log_time = perf_counter()

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
            f"Graph construction 已启动, input_type={self.grid_input_type}, grid={input_topic}, "
            f"odom={self.config['odom_topic']}"
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        """缓存最新 grid, 避免在 subscriber 回调里做重计算"""
        self.latest_grid = msg
        if not self._logged_first_grid:
            self.get_logger().info(
                f"收到第一帧 grid, frame={msg.header.frame_id}, size={msg.info.width}x{msg.info.height}"
            )
            self._logged_first_grid = True

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

        这里不做 message_filters 同步, 第一版优先保证简单可跑
        如果后续发现 grid 和 odom 时间差影响较大, 再换成近似时间同步
        """
        if self.latest_grid is None or self.latest_odom is None:
            return

        update_start = perf_counter()
        node_stage_timings_ms: Dict[str, float] = {}
        try:
            stage_start = perf_counter()
            classified_grid = self._classify_latest_grid()
            node_stage_timings_ms["classify_grid"] = _elapsed_ms(stage_start)

            stage_start = perf_counter()
            update_result = self.builder.update(
                classified_grid,
                _robot_position(self.latest_odom),
                _stamp_to_seconds(self.latest_grid.header),
            )
            node_stage_timings_ms["builder_update"] = _elapsed_ms(stage_start)
        except Exception as exc:
            self.get_logger().error(f"更新导航图失败: {exc}")
            return

        stage_start = perf_counter()
        header = self._graph_header(self.latest_grid.header, classified_grid.frame_id)
        nav_graph = graph_to_msg(update_result.graph, header, self.config["trav_class"])
        node_stage_timings_ms["convert_graph_msg"] = _elapsed_ms(stage_start)

        stage_start = perf_counter()
        self.nav_graph_pub.publish(nav_graph)
        node_stage_timings_ms["publish_graph"] = _elapsed_ms(stage_start)

        stage_start = perf_counter()
        self.viz_pub.publish(
            self.visualizer.build_markers(
                update_result.graph,
                header,
                update_result.classified_grid,
            )
        )
        node_stage_timings_ms["publish_viz"] = _elapsed_ms(stage_start)
        node_stage_timings_ms["total"] = _elapsed_ms(update_start)

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
        # self._maybe_log_diagnostics(update_result, node_stage_timings_ms)

    def _classify_latest_grid(self):
        """把缓存的 ROS 地图消息解码为通用 grid"""
        if self.grid_input_type == "grid_map":
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
                enable_postprocess=self.config["grid_map_enable_postprocess"],
                min_free_component_cells=self.config["grid_map_min_free_component_cells"],
                fill_hole_max_cells=self.config["grid_map_fill_hole_max_cells"],
                fill_hole_min_free_neighbor_ratio=self.config["grid_map_fill_hole_min_free_neighbor_ratio"],
                majority_fill_iterations=self.config["grid_map_majority_fill_iterations"],
                majority_fill_min_neighbors=self.config["grid_map_majority_fill_min_neighbors"],
                transpose=self.config["grid_map_transpose"],
                flip_x=self.config["grid_map_flip_x"],
                flip_y=self.config["grid_map_flip_y"],
            )

        return classify_occupancy_grid(
            self.latest_grid,
            free_threshold=self.config["free_threshold"],
            obstacle_threshold=self.config["obstacle_threshold"],
        )

    def _graph_header(self, input_header, fallback_frame: str):
        """构造 ROS 适配层负责的输出 graph header"""
        header = Header()
        header.stamp = input_header.stamp
        header.frame_id = self.config["global_frame"] or fallback_frame
        return header

    def _maybe_log_diagnostics(self, update_result, node_stage_timings_ms: Dict[str, float]) -> None:
        """按周期打印诊断日志, 慢帧单独升为 warning"""
        now = perf_counter()
        log_period = float(self.config["diagnostics_log_period_sec"])
        slow_update_warning_ms = float(self.config["slow_update_warning_ms"])
        total_ms = node_stage_timings_ms.get("total", 0.0)
        should_warn = slow_update_warning_ms > 0.0 and total_ms >= slow_update_warning_ms
        should_log = log_period > 0.0 and now - self._last_diagnostics_log_time >= log_period

        if not should_warn and not should_log:
            return

        if should_log:
            self._last_diagnostics_log_time = now

        message = _format_update_diagnostics(update_result, node_stage_timings_ms)
        if should_warn:
            self.get_logger().warn(f"导航图更新较慢, {message}")
        else:
            self.get_logger().info(f"导航图更新诊断, {message}")


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
    """格式化首帧发布诊断用的 grid 分类统计"""
    if not stats:
        return ""
    return (
        f", 有效={stats.get('valid', 0)}, 原始free={stats.get('raw_free', 0)}, "
        f"原始obstacle={stats.get('raw_obstacle', 0)}, 原始unknown={stats.get('raw_unknown', 0)}, "
        f"free={stats.get('free', 0)}, obstacle={stats.get('obstacle', 0)}, unknown={stats.get('unknown', 0)}"
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


def _format_update_diagnostics(update_result: Any, node_stage_timings_ms: Dict[str, float]) -> str:
    """格式化周期性 graph update 诊断日志"""
    diagnostics = update_result.diagnostics
    current_node_status = _format_current_node_status(diagnostics.current_node_status)
    return (
        f"节点数={diagnostics.node_count}, 边数={diagnostics.edge_count}, "
        f"frontier节点={diagnostics.frontier_node_count}, frontier栅格={diagnostics.frontier_cell_count}, "
        f"frontier候选={diagnostics.frontier_candidate_count}, "
        f"连通分量={diagnostics.connected_components}, 当前分量大小={diagnostics.current_component_size}, "
        f"当前节点={diagnostics.current_node_id}, 当前节点状态={current_node_status}, "
        f"节点度数=min/{diagnostics.degree_min},avg/{diagnostics.degree_avg:.2f},max/{diagnostics.degree_max}"
        f"{_format_grid_stats(update_result.classified_grid.stats)}, "
        f"阶段耗时={_format_stage_timings(update_result.diagnostics.stage_timings_ms, node_stage_timings_ms)}"
    )


def _format_current_node_status(status: str) -> str:
    """把内部 current node 状态映射为中文日志"""
    labels = {
        "reachable": "安全可达",
        "geometry_fallback": "几何回退",
        "missing": "缺失",
    }
    return labels.get(status, status)


def _format_stage_timings(
    builder_stage_timings_ms: Dict[str, float],
    node_stage_timings_ms: Dict[str, float],
) -> str:
    """按固定顺序输出 builder 和 ROS node 两层耗时"""
    entries: list[tuple[str, float]] = []
    node_order = [
        "classify_grid",
        "builder_update",
        "convert_graph_msg",
        "publish_graph",
        "publish_viz",
        "total",
    ]
    builder_order = [
        "prepare_grid",
        "distance_fields",
        "update_nodes",
        "sample_nodes",
        "update_frontiers",
        "current_node",
        "build_edges",
        "total",
    ]

    for name in node_order:
        if name in node_stage_timings_ms:
            entries.append((f"node.{name}", node_stage_timings_ms[name]))
    for name in builder_order:
        if name in builder_stage_timings_ms:
            entries.append((f"builder.{name}", builder_stage_timings_ms[name]))

    seen_names = {name for name, _ in entries}
    for name, value in node_stage_timings_ms.items():
        entry_name = f"node.{name}"
        if entry_name not in seen_names:
            entries.append((entry_name, value))
            seen_names.add(entry_name)
    for name, value in builder_stage_timings_ms.items():
        entry_name = f"builder.{name}"
        if entry_name not in seen_names:
            entries.append((entry_name, value))

    return ", ".join(f"{name}={value:.1f}ms" for name, value in entries)


def _elapsed_ms(start_time: float) -> float:
    """计算阶段耗时, 单位毫秒"""
    return (perf_counter() - start_time) * 1000.0


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
    return _apply_config_overrides(loaded, overrides or [])


def _apply_config_overrides(config: Dict[str, Any], overrides: list[str]) -> Dict[str, Any]:
    """应用 launch 传入的 key=value 覆盖项"""
    try:
        import yaml
    except ImportError:
        yaml = None

    for item in overrides:
        if "=" not in item:
            raise ValueError(f"Invalid config override '{item}', expected key=value")
        key, raw_value = item.split("=", 1)
        config[key] = yaml.safe_load(raw_value) if yaml is not None else raw_value
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
