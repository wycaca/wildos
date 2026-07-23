from __future__ import annotations

import argparse
from dataclasses import fields
import math
from pathlib import Path
import time
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
from graph_construction.msg_utils import GraphMessageCache, graph_to_msg
from graph_construction.performance_stats import EventRate, TimingWindow
from graph_construction.viz import GraphVisualizer


DEFAULT_CONFIG: Dict[str, Any] = {
    "global_frame": "map",
    "odom_topic": "/odom",
    "grid_map_topic": "/elevation_mapping_node/elevation_map_raw",
    "nav_graph_topic": "/spot1/nav_graph",
    "viz_topic": "/spot1/graph_construction_viz",
    "viz_show_radius_markers": False,
    "viz_publish_rate_hz": 1.0,
    "publish_rate_hz": 2.0,
    "max_grid_odom_time_delta_sec": 0.5,
    "diagnostics_log_period_sec": 30.0,
    "slow_cycle_warning_ms": 250.0,
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


class InputFreshnessGate:
    """跟踪地图和里程计时间戳, 拒绝旧数据和不匹配快照"""

    def __init__(self) -> None:
        self.grid_stamp_ns: int | None = None
        self.odom_stamp_ns: int | None = None

    def accept_grid(self, stamp_ns: int) -> bool:
        """仅接收时间严格前进的地图"""
        if self.grid_stamp_ns is not None and stamp_ns <= self.grid_stamp_ns:
            return False
        self.grid_stamp_ns = stamp_ns
        return True

    def accept_odom(self, stamp_ns: int) -> bool:
        """仅接收时间严格前进的里程计"""
        if self.odom_stamp_ns is not None and stamp_ns <= self.odom_stamp_ns:
            return False
        self.odom_stamp_ns = stamp_ns
        return True

    def time_delta_seconds(self) -> float | None:
        """返回当前地图和里程计快照的时间差"""
        if self.grid_stamp_ns is None or self.odom_stamp_ns is None:
            return None
        return abs(self.grid_stamp_ns - self.odom_stamp_ns) / 1.0e9


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
        self._message_cache = GraphMessageCache()
        self.visualizer = GraphVisualizer(
            show_radius_markers=bool(
                self.config.get("viz_show_radius_markers", False)
            )
        )

        self.latest_grid = None
        self.latest_odom = None
        self._input_freshness = InputFreshnessGate()
        self._latest_grid_sequence = 0
        self._processed_grid_sequence = 0
        self._stale_grid_messages = 0
        self._stale_odom_messages = 0
        self._last_input_warning = 0.0
        self._logged_first_grid = False
        self._logged_first_odom = False
        self._logged_first_publish = False
        self._last_slow_warning = 0.0
        self._last_viz_publish_time = -math.inf
        self._publish_rate = EventRate()
        self._timings = {
            name: TimingWindow()
            for name in ("classify", "update", "message", "visualize", "total")
        }
        self._graph_update_timings = {
            name: TimingWindow()
            for name in (
                "preprocess",
                "dirty",
                "distance",
                "nodes",
                "sampling",
                "frontier",
                "current",
                "edges",
            )
        }
        self._latest_graph_update_stats = None

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
            1,
        )

        self.create_subscription(
            Odometry,
            self.config["odom_topic"],
            self._on_odom,
            1,
        )

        publish_rate = float(self.config["publish_rate_hz"])
        self.create_timer(1.0 / publish_rate, self._on_timer)
        diagnostics_period = max(
            float(self.config["diagnostics_log_period_sec"]),
            5.0,
        )
        self.create_timer(diagnostics_period, self._report_diagnostics)

        self.get_logger().info(
            f"Graph construction 已启动, grid_map={self.config['grid_map_topic']}, "
            f"odom={self.config['odom_topic']}"
        )

    def _on_grid_map(self, msg: GridMap) -> None:
        """缓存最新 GridMap, 用 elevation 和 traversability layer 构建图"""
        stamp_ns = _header_stamp_nanoseconds(msg.header)
        previous_stamp_ns = self._input_freshness.grid_stamp_ns
        if not self._input_freshness.accept_grid(stamp_ns):
            self._stale_grid_messages += 1
            self._warn_input_freshness(
                "忽略时间戳未前进的 GridMap",
                f"当前={_nanoseconds_to_seconds(stamp_ns):.6f}s, "
                f"已接收={_nanoseconds_to_seconds(previous_stamp_ns):.6f}s",
            )
            return
        self.latest_grid = msg
        self._latest_grid_sequence += 1
        if not self._logged_first_grid:
            self.get_logger().info(
                f"收到第一帧 GridMap, frame={msg.header.frame_id}, layers={list(msg.layers)}"
            )
            self._logged_first_grid = True

    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新 odom, current_node_idx 计算依赖它"""
        stamp_ns = _header_stamp_nanoseconds(msg.header)
        previous_stamp_ns = self._input_freshness.odom_stamp_ns
        if not self._input_freshness.accept_odom(stamp_ns):
            self._stale_odom_messages += 1
            self._warn_input_freshness(
                "忽略时间戳未前进的 odom",
                f"当前={_nanoseconds_to_seconds(stamp_ns):.6f}s, "
                f"已接收={_nanoseconds_to_seconds(previous_stamp_ns):.6f}s",
            )
            return
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

        节点使用时间差在容限内的最新 grid 和 odom 快照
        地图解码或图更新失败时保留历史图, 当前周期不发布部分结果
        graph message 和 marker 使用同一份更新结果, 避免可视化与规划图不一致
        """
        inputs = self._take_latest_inputs()
        if inputs is None:
            return
        grid_msg, odom_msg = inputs

        cycle_started = time.perf_counter()
        try:
            stage_started = time.perf_counter()
            classified_grid = self._classify_grid(grid_msg)
            self._timings["classify"].add_seconds(time.perf_counter() - stage_started)
            stage_started = time.perf_counter()
            update_result = self.builder.update(
                classified_grid,
                _robot_position(odom_msg),
                _stamp_to_seconds(grid_msg.header),
            )
            self._timings["update"].add_seconds(time.perf_counter() - stage_started)
            self._latest_graph_update_stats = update_result.stats
            for name, elapsed_seconds in update_result.stats.stage_seconds.items():
                self._graph_update_timings[name].add_seconds(elapsed_seconds)
        except Exception as exc:
            self.get_logger().error(f"更新导航图失败: {exc}")
            return

        stage_started = time.perf_counter()
        header = self._graph_header(grid_msg.header, classified_grid.frame_id)
        nav_graph = graph_to_msg(
            update_result.graph,
            header,
            TRAVERSABILITY_CLASS,
            cache=self._message_cache,
        )
        self._timings["message"].add_seconds(time.perf_counter() - stage_started)

        self.nav_graph_pub.publish(nav_graph)

        now = time.monotonic()
        viz_rate = max(float(self.config["viz_publish_rate_hz"]), 0.01)
        if (
            self.viz_pub.get_subscription_count() > 0
            and now - self._last_viz_publish_time >= 1.0 / viz_rate
        ):
            stage_started = time.perf_counter()
            self.viz_pub.publish(
                self.visualizer.build_markers(
                    update_result.graph,
                    header,
                )
            )
            self._timings["visualize"].add_seconds(
                time.perf_counter() - stage_started
            )
            self._last_viz_publish_time = now
        total_seconds = time.perf_counter() - cycle_started
        self._timings["total"].add_seconds(total_seconds)
        self._publish_rate.tick()
        self._warn_if_slow(total_seconds)

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

    def _warn_if_slow(self, elapsed_seconds: float) -> None:
        """Throttle slow-cycle warnings to avoid hiding normal diagnostics"""
        threshold_ms = float(self.config["slow_cycle_warning_ms"])
        now = time.monotonic()
        if elapsed_seconds * 1000.0 < threshold_ms or now - self._last_slow_warning < 30.0:
            return
        self._last_slow_warning = now
        self.get_logger().warn(
            f"导航图构建耗时偏高, total={elapsed_seconds * 1000.0:.1f}ms, "
            f"threshold={threshold_ms:.1f}ms"
        )

    def _report_diagnostics(self) -> None:
        """Report per-stage timing without per-cycle log traffic"""
        summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._timings.items()
        }
        if not summaries["total"].count:
            return
        total = summaries["total"]
        update_summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._graph_update_timings.items()
        }
        workload = self._format_graph_update_workload()
        self.get_logger().info(
            "导航图性能, "
            f"频率={self._publish_rate.sample(reset=True):.2f}Hz, "
            f"总耗时=平均{total.average_ms:.0f}/95%上限{total.p95_ms:.0f}/"
            f"最大{total.maximum_ms:.0f}ms, "
            "阶段平均耗时="
            f"地图分类{summaries['classify'].average_ms:.0f}ms/"
            f"图更新{summaries['update'].average_ms:.0f}ms/"
            f"消息转换{summaries['message'].average_ms:.0f}ms/"
            f"可视化{summaries['visualize'].average_ms:.0f}ms, "
            "图内平均耗时="
            f"预处理{update_summaries['preprocess'].average_ms:.0f}ms/"
            f"变化检测{update_summaries['dirty'].average_ms:.0f}ms/"
            f"距离场{update_summaries['distance'].average_ms:.0f}ms/"
            f"节点{update_summaries['nodes'].average_ms:.0f}ms/"
            f"采样{update_summaries['sampling'].average_ms:.0f}ms/"
            f"Frontier{update_summaries['frontier'].average_ms:.0f}ms/"
            f"边{update_summaries['edges'].average_ms:.0f}ms"
            f"{workload}"
        )

    def _format_graph_update_workload(self) -> str:
        """格式化最近一帧局部更新工作量"""
        stats = self._latest_graph_update_stats
        if stats is None:
            return ""
        return (
            ", 最近工作量="
            f"局部节点{stats.local_node_count}/{stats.total_node_count}, "
            f"变化栅格{stats.dirty_cell_count}, "
            f"新增free栅格{stats.newly_free_cell_count}, "
            f"新增障碍栅格{stats.newly_obstacle_cell_count}, "
            f"重建边节点{stats.edge_rebuild_node_count}, "
            f"新free附近旧节点{stats.newly_free_rebuild_node_count}, "
            f"受影响边{stats.affected_edge_count}/{stats.total_edge_count}, "
            f"障碍附近边{stats.obstacle_affected_edge_count}, "
            f"边候选{stats.edge_candidate_pair_count}, "
            f"边碰撞检查{stats.edge_clearance_check_count}, "
            f"历史边复查{stats.historical_edge_check_count}, "
            f"unknown候选{stats.blocked_unknown_candidate_count}, "
            f"unknown开放重试{stats.blocked_unknown_retry_count}, "
            f"低连接节点重试{stats.low_degree_retry_node_count}, "
            f"Frontier候选{stats.frontier_candidate_count}, "
            f"活动Frontier owner{stats.active_frontier_owner_count}"
        )

    def _take_latest_inputs(self):
        """每个新 GridMap 只处理一次, 并等待时间匹配的 odom 快照"""
        if self.latest_grid is None or self.latest_odom is None:
            return None
        if self._latest_grid_sequence == self._processed_grid_sequence:
            return None

        time_delta = self._input_freshness.time_delta_seconds()
        max_time_delta = float(self.config["max_grid_odom_time_delta_sec"])
        if time_delta is None or time_delta > max_time_delta:
            self._warn_input_freshness(
                "等待时间匹配的 GridMap 和 odom",
                f"地图={_nanoseconds_to_seconds(self._input_freshness.grid_stamp_ns):.6f}s, "
                f"里程计={_nanoseconds_to_seconds(self._input_freshness.odom_stamp_ns):.6f}s, "
                f"差值={time_delta * 1000.0:.1f}ms, "
                f"上限={max_time_delta * 1000.0:.1f}ms",
            )
            return None

        self._processed_grid_sequence = self._latest_grid_sequence
        return self.latest_grid, self.latest_odom

    def _warn_input_freshness(
        self,
        reason: str,
        detail: str,
    ) -> None:
        """低频报告旧数据或输入时间不匹配"""
        now = time.monotonic()
        if now - self._last_input_warning < 30.0:
            return
        self._last_input_warning = now
        self.get_logger().warn(
            f"{reason}, {detail}, "
            f"旧地图丢弃={self._stale_grid_messages}, "
            f"旧里程计丢弃={self._stale_odom_messages}"
        )

    def _classify_grid(self, grid_msg):
        """按配置的输入后端把 ROS 地图解码为统一 ClassifiedGrid

        GridMap 的 rolling buffer 已由 grid_adapter 解包, 这里固定启用经过验证的
        拓扑后处理和标准轴约定, 避免重新开放会破坏坐标一致性的历史调试开关
        """
        return classify_grid_map(
            grid_msg,
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
            fill_hole_min_free_neighbor_ratio=self.config[
                "grid_map_fill_hole_min_free_neighbor_ratio"
            ],
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
    if float(resolved["viz_publish_rate_hz"]) <= 0.0:
        raise ValueError("viz_publish_rate_hz must be greater than 0")
    if float(resolved["max_grid_odom_time_delta_sec"]) <= 0.0:
        raise ValueError("max_grid_odom_time_delta_sec must be greater than 0")
    if float(resolved["diagnostics_log_period_sec"]) <= 0.0:
        raise ValueError("diagnostics_log_period_sec must be greater than 0")
    if float(resolved["slow_cycle_warning_ms"]) <= 0.0:
        raise ValueError("slow_cycle_warning_ms must be greater than 0")
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
        f"人工free={stats.get('robot_blind_zone_artificial_free', 0)}, "
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


def _header_stamp_nanoseconds(header: Any) -> int:
    """将 ROS header stamp 转成整数纳秒"""
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _nanoseconds_to_seconds(stamp_ns: int | None) -> float:
    """将可选整数纳秒转成日志使用的秒"""
    return float(stamp_ns or 0) / 1.0e9


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
