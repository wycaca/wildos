import copy
import heapq
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from object_search_msgs.msg import ObjectSearchStatus, TargetEstimate

from visual_navigation.object_search_types import (
    ObjectSearchState,
    coarse_target_evidence_ready,
    normalize_object_search_target,
)
from visual_navigation.object_search_goal_policy import (
    GoalMode,
    GoalSelectionInput,
    ObjectSearchGoalPolicy,
)


_TARGET_CHANGE_REACHED_GUARD_SEC = 1.0


class ObjectSearchGoalMux(Node):
    """在初始探索、视觉粗目标、稳定目标和完成状态之间选择唯一 goal"""

    def __init__(self):
        super().__init__("object_search_goal_mux")

        self.declare_parameter("output_goal_topic", "/spot1/graphnav_goal_pose")
        self.declare_parameter("status_topic", "/spot1/object_search_status")
        self.declare_parameter("object_target_estimate_topic", "/spot1/object_target_estimate")
        self.declare_parameter("object_reached_topic", "/spot1/object_search_reached")
        self.declare_parameter("completion_topic", "/spot1/object_search_completed")
        self.declare_parameter("object_search_target_topic", "/spot1/object_search_target")
        self.declare_parameter("odom_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("initial_goal_distance", 30.0)
        self.declare_parameter("initial_goal_heading_deg", 0.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("object_reached_timeout_sec", 2.0)
        self.declare_parameter("object_reached_max_target_distance", 2.0)
        self.declare_parameter("lidar_reached_timeout_sec", 2.0)
        self.declare_parameter("coarse_target_min_views", 2)
        self.declare_parameter("coarse_target_min_confidence", 0.45)
        self.declare_parameter("coarse_target_max_distance", 30.0)
        self.declare_parameter("coarse_target_timeout_sec", 3.0)
        self.declare_parameter("target_max_vertical_offset", 1.5)
        self.declare_parameter("coarse_target_max_horizontal_std", 8.0)
        self.declare_parameter("target_update_min_distance", 0.75)
        self.declare_parameter("stable_target_update_min_distance", 0.3)
        self.declare_parameter("coarse_observation_distance", 2.75)
        self.declare_parameter("coarse_observation_entry_tolerance", 0.4)
        self.declare_parameter("target_observation_duration_sec", 2.0)
        self.declare_parameter("target_observation_reposition_distance", 0.75)
        self.declare_parameter("target_observation_reposition_tolerance", 0.4)
        self.declare_parameter("target_observation_lost_timeout_sec", 2.0)
        self.declare_parameter("target_observation_scan_yaw_deg", 20.0)
        self.declare_parameter("target_observation_scan_tolerance_deg", 5.0)
        self.declare_parameter("target_observation_scan_hold_sec", 0.5)
        self.declare_parameter("pending_evidence_protection_sec", 3.0)
        self.declare_parameter("pending_observation_duration_sec", 1.5)
        self.declare_parameter("pending_reposition_distance", 0.6)
        self.declare_parameter("pending_reposition_tolerance", 0.3)
        self.declare_parameter(
            "pending_reposition_visibility_timeout_sec",
            1.5,
        )
        self.declare_parameter("final_observation_distance", 1.75)
        self.declare_parameter("final_observation_entry_tolerance", 0.4)
        self.declare_parameter("final_observation_duration_sec", 2.0)
        self.declare_parameter("final_observation_lost_timeout_sec", 2.0)
        self.declare_parameter("final_observation_scan_yaw_deg", 15.0)
        self.declare_parameter("final_observation_scan_tolerance_deg", 5.0)
        self.declare_parameter("final_observation_scan_hold_sec", 0.5)
        self.declare_parameter("final_observation_reposition_distance", 0.9)
        self.declare_parameter("final_observation_reposition_tolerance", 0.35)
        self.declare_parameter("nav_graph_topic", "/spot1/nav_graph")
        self.declare_parameter("scored_nav_graph_topic", "/spot1/scored_nav_graph")
        self.declare_parameter("startup_observation_enabled", True)
        self.declare_parameter("startup_warmup_sec", 3.0)
        self.declare_parameter("startup_min_nav_graph_frames", 6)
        self.declare_parameter("startup_min_scored_graph_frames", 3)
        self.declare_parameter("startup_min_forward_nodes", 3)
        self.declare_parameter("startup_min_forward_frontiers", 1)
        self.declare_parameter("startup_forward_range", 8.0)
        self.declare_parameter("startup_forward_half_angle_deg", 70.0)
        self.declare_parameter("startup_graph_timeout_sec", 2.0)
        self.declare_parameter("startup_scan_enabled", True)
        self.declare_parameter("startup_scan_trigger_frames", 3)
        self.declare_parameter("startup_scan_yaw_deg", 35.0)
        self.declare_parameter("startup_scan_yaw_tolerance_deg", 5.0)
        self.declare_parameter("startup_scan_hold_sec", 0.5)
        self.declare_parameter("startup_scan_phase_timeout_sec", 4.0)

        self.output_goal_topic = self._param_str("output_goal_topic")
        self.status_topic = self._param_str("status_topic")
        self.object_target_estimate_topic = self._param_str("object_target_estimate_topic")
        self.object_reached_topic = self._param_str("object_reached_topic")
        self.completion_topic = self._param_str("completion_topic")
        self.object_search_target_topic = self._param_str("object_search_target_topic")
        self.odom_topic = self._param_str("odom_topic")
        self.frame_id = self._param_str("frame_id")
        self.initial_goal_distance = self._param_float("initial_goal_distance")
        self.initial_goal_heading_deg = self._param_float("initial_goal_heading_deg")
        self.publish_rate = max(self._param_float("publish_rate"), 0.1)
        self.object_reached_timeout_sec = max(self._param_float("object_reached_timeout_sec"), 0.0)
        self.object_reached_max_target_distance = max(
            self._param_float("object_reached_max_target_distance"),
            0.1,
        )
        self.lidar_reached_timeout_sec = max(
            self._param_float("lidar_reached_timeout_sec"),
            0.1,
        )
        self.coarse_target_min_views = max(
            int(self.get_parameter("coarse_target_min_views").value),
            2,
        )
        self.coarse_target_min_confidence = max(
            self._param_float("coarse_target_min_confidence"),
            0.0,
        )
        self.coarse_target_max_distance = max(
            self._param_float("coarse_target_max_distance"),
            0.1,
        )
        self.coarse_target_timeout_sec = max(
            self._param_float("coarse_target_timeout_sec"),
            0.1,
        )
        self.target_max_vertical_offset = max(
            self._param_float("target_max_vertical_offset"),
            0.0,
        )
        self.coarse_target_max_horizontal_std = max(
            self._param_float("coarse_target_max_horizontal_std"),
            0.0,
        )
        self.target_update_min_distance = max(
            self._param_float("target_update_min_distance"),
            0.0,
        )
        self.stable_target_update_min_distance = max(
            self._param_float("stable_target_update_min_distance"),
            0.0,
        )
        self.coarse_observation_distance = max(
            self._param_float("coarse_observation_distance"),
            0.5,
        )
        self.coarse_observation_entry_tolerance = max(
            self._param_float("coarse_observation_entry_tolerance"),
            0.0,
        )
        self.target_observation_duration_sec = max(
            self._param_float("target_observation_duration_sec"),
            0.1,
        )
        self.target_observation_reposition_distance = max(
            self._param_float("target_observation_reposition_distance"),
            0.3,
        )
        self.target_observation_reposition_tolerance = max(
            self._param_float("target_observation_reposition_tolerance"),
            0.1,
        )
        self.target_observation_lost_timeout_sec = max(
            self._param_float("target_observation_lost_timeout_sec"),
            0.1,
        )
        self.target_observation_scan_yaw = math.radians(
            min(max(self._param_float("target_observation_scan_yaw_deg"), 5.0), 45.0)
        )
        self.target_observation_scan_tolerance = math.radians(
            min(
                max(self._param_float("target_observation_scan_tolerance_deg"), 1.0),
                15.0,
            )
        )
        self.target_observation_scan_hold_sec = max(
            self._param_float("target_observation_scan_hold_sec"),
            0.0,
        )
        self.pending_evidence_protection_sec = max(
            self._param_float("pending_evidence_protection_sec"),
            0.0,
        )
        self.pending_observation_duration_sec = max(
            self._param_float("pending_observation_duration_sec"),
            0.1,
        )
        self.pending_reposition_distance = max(
            self._param_float("pending_reposition_distance"),
            0.3,
        )
        self.pending_reposition_tolerance = max(
            self._param_float("pending_reposition_tolerance"),
            0.1,
        )
        self.pending_reposition_visibility_timeout_sec = max(
            self._param_float("pending_reposition_visibility_timeout_sec"),
            0.1,
        )
        self.final_observation_distance = max(
            self._param_float("final_observation_distance"),
            0.5,
        )
        self.final_observation_entry_tolerance = max(
            self._param_float("final_observation_entry_tolerance"),
            0.0,
        )
        self.final_observation_duration_sec = max(
            self._param_float("final_observation_duration_sec"),
            0.1,
        )
        self.final_observation_lost_timeout_sec = max(
            self._param_float("final_observation_lost_timeout_sec"),
            0.1,
        )
        self.final_observation_scan_yaw = math.radians(
            min(
                max(self._param_float("final_observation_scan_yaw_deg"), 5.0),
                30.0,
            )
        )
        self.final_observation_scan_tolerance = math.radians(
            min(
                max(
                    self._param_float(
                        "final_observation_scan_tolerance_deg"
                    ),
                    1.0,
                ),
                15.0,
            )
        )
        self.final_observation_scan_hold_sec = max(
            self._param_float("final_observation_scan_hold_sec"),
            0.0,
        )
        self.final_observation_reposition_distance = max(
            self._param_float("final_observation_reposition_distance"),
            0.3,
        )
        self.final_observation_reposition_tolerance = max(
            self._param_float("final_observation_reposition_tolerance"),
            0.1,
        )
        self.nav_graph_topic = self._param_str("nav_graph_topic")
        self.scored_nav_graph_topic = self._param_str("scored_nav_graph_topic")
        self.startup_observation_enabled = bool(
            self.get_parameter("startup_observation_enabled").value
        )
        self.startup_warmup_sec = max(self._param_float("startup_warmup_sec"), 0.0)
        self.startup_min_nav_graph_frames = max(
            int(self.get_parameter("startup_min_nav_graph_frames").value),
            1,
        )
        self.startup_min_scored_graph_frames = max(
            int(self.get_parameter("startup_min_scored_graph_frames").value),
            1,
        )
        self.startup_min_forward_nodes = max(
            int(self.get_parameter("startup_min_forward_nodes").value),
            1,
        )
        self.startup_min_forward_frontiers = max(
            int(self.get_parameter("startup_min_forward_frontiers").value),
            0,
        )
        self.startup_forward_range = max(
            self._param_float("startup_forward_range"),
            0.1,
        )
        self.startup_forward_half_angle = math.radians(
            min(max(self._param_float("startup_forward_half_angle_deg"), 1.0), 89.0)
        )
        self.startup_graph_timeout_sec = max(
            self._param_float("startup_graph_timeout_sec"),
            0.1,
        )
        self.startup_scan_enabled = bool(
            self.get_parameter("startup_scan_enabled").value
        )
        self.startup_scan_trigger_frames = max(
            int(self.get_parameter("startup_scan_trigger_frames").value),
            2,
        )
        self.startup_scan_yaw = math.radians(
            min(max(self._param_float("startup_scan_yaw_deg"), 5.0), 60.0)
        )
        self.startup_scan_yaw_tolerance = math.radians(
            min(max(self._param_float("startup_scan_yaw_tolerance_deg"), 1.0), 15.0)
        )
        self.startup_scan_hold_sec = max(
            self._param_float("startup_scan_hold_sec"),
            0.0,
        )
        self.startup_scan_phase_timeout_sec = max(
            self._param_float("startup_scan_phase_timeout_sec"),
            self.startup_scan_hold_sec + 0.1,
        )

        self.latest_odom: Odometry | None = None
        self.metric_target: PoseStamped | None = None
        self.metric_target_confidence = 0.0
        self.metric_target_source = TargetEstimate.SOURCE_NONE
        self.metric_target_stable = False
        self.metric_target_state = "EMPTY"
        self.initial_search_goal: PoseStamped | None = None
        self.exploration_heading_yaw: float | None = None
        self.latest_object_reached = False
        self.latest_object_reached_time = None
        self.reached_latched = False
        self.reached_hold_goal: PoseStamped | None = None
        self.current_target: str | None = None
        self._target_changed_stamp_sec: float | None = None
        self._last_state = ""
        self._last_reached_gate_reason = ""
        self._warned_frame_mismatch = False
        self.startup_started_time = None
        self.startup_observation_position = None
        self.startup_completed = not self.startup_observation_enabled
        self.startup_nav_graph_frames = 0
        self.startup_scored_graph_frames = 0
        self.startup_forward_nodes = 0
        self.startup_forward_frontiers = 0
        self.startup_insufficient_frames = 0
        self.startup_nav_graph_time = None
        self.startup_scored_graph_time = None
        self.latest_nav_graph: NavigationGraph | None = None
        self.used_reposition_goals: list[tuple[float, float]] = []
        self.startup_scan_phase = "WARMUP"
        self.startup_scan_phase_time = None
        self.startup_scan_phase_scored_frames = 0
        self.latest_target_estimate_time = None
        self.target_observation_started_time = None
        self.target_observation_phase = "ALIGN"
        self.target_observation_phase_time = None
        self.target_reposition_goal: PoseStamped | None = None
        self.target_reposition_attempts = 0
        self.pending_evidence_time = None
        self.pending_observation_started_time = None
        self.pending_observation_complete = False
        self.pending_bearing_yaw: float | None = None
        self.pending_reposition_goal: PoseStamped | None = None
        self.pending_reposition_attempts = 0
        self.final_observation_started_time = None
        self.final_observation_phase = "ALIGN"
        self.final_observation_phase_time = None
        self.final_approach_goal: PoseStamped | None = None
        self.final_reposition_goal: PoseStamped | None = None
        self.final_reposition_attempts = 0
        self.policy = ObjectSearchGoalPolicy()

        self.goal_pub = self.create_publisher(PoseStamped, self.output_goal_topic, 10)
        self.status_pub = self.create_publisher(
            ObjectSearchStatus,
            self.status_topic,
            10,
        )
        self.completion_pub = self.create_publisher(Bool, self.completion_topic, 10)
        self.metric_target_sub = self.create_subscription(
            TargetEstimate,
            self.object_target_estimate_topic,
            self._on_target_estimate,
            10,
        )
        self.reached_sub = self.create_subscription(
            Bool,
            self.object_reached_topic,
            self._on_object_reached,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            10,
        )
        self.nav_graph_sub = self.create_subscription(
            NavigationGraph,
            self.nav_graph_topic,
            self._on_nav_graph,
            10,
        )
        self.scored_nav_graph_sub = self.create_subscription(
            NavigationGraph,
            self.scored_nav_graph_topic,
            self._on_scored_nav_graph,
            10,
        )
        self.object_target_sub = self.create_subscription(
            String,
            self.object_search_target_topic,
            self._on_object_search_target,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            "目标搜索目标仲裁器已启动, "
            f"输出目标={self.output_goal_topic}, 融合估计={self.object_target_estimate_topic}, "
            f"视觉到达={self.object_reached_topic}, 里程计={self.odom_topic}, "
            f"初始探索距离={self.initial_goal_distance:.1f}m, "
            f"启动观察={'启用' if self.startup_observation_enabled else '禁用'}, "
            f"发布频率={self.publish_rate:.1f}Hz"
        )

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _param_float(self, name: str) -> float:
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"参数 {name} 必须是数字, 当前值={value}") from exc

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.startup_started_time is None:
            self.startup_started_time = self.get_clock().now()
            self.startup_observation_position = copy.deepcopy(
                msg.pose.pose.position
            )
        if self.exploration_heading_yaw is None:
            self.exploration_heading_yaw = (
                _yaw_from_quaternion(msg.pose.pose.orientation)
                + math.radians(self.initial_goal_heading_deg)
            )

    def _on_nav_graph(self, msg: NavigationGraph) -> None:
        """累计连续有效原始图帧, 作为启动地图就绪条件"""
        if not self._graph_is_valid(msg):
            self.startup_nav_graph_frames = 0
            self.latest_nav_graph = None
            return
        self.startup_nav_graph_frames += 1
        self.startup_nav_graph_time = self.get_clock().now()
        self.latest_nav_graph = msg

    def _on_scored_nav_graph(self, msg: NavigationGraph) -> None:
        """累计连续有效评分图并检查初始朝向前方是否可探索"""
        if not self._graph_is_valid(msg) or not msg.trav_classes:
            self.startup_scored_graph_frames = 0
            return
        self.startup_scored_graph_frames += 1
        self.startup_scored_graph_time = self.get_clock().now()
        self.startup_forward_nodes, self.startup_forward_frontiers = (
            self._forward_graph_counts(msg)
        )
        if self._startup_forward_region_ready():
            self.startup_insufficient_frames = 0
        else:
            self.startup_insufficient_frames += 1

    @staticmethod
    def _graph_is_valid(msg: NavigationGraph) -> bool:
        return bool(msg.nodes) and int(msg.current_node_idx) < len(msg.nodes)

    def _forward_graph_counts(self, msg: NavigationGraph) -> tuple[int, int]:
        """统计固定初始朝向扇区内的节点和 Frontier"""
        if self.latest_odom is None or self.exploration_heading_yaw is None:
            return 0, 0
        if msg.header.frame_id and msg.header.frame_id != self.latest_odom.header.frame_id:
            return 0, 0
        origin_x = self.latest_odom.pose.pose.position.x
        origin_y = self.latest_odom.pose.pose.position.y
        heading_x = math.cos(self.exploration_heading_yaw)
        heading_y = math.sin(self.exploration_heading_yaw)
        minimum_cosine = math.cos(self.startup_forward_half_angle)
        frontier_class_index = 0
        if "default" in msg.trav_classes:
            frontier_class_index = list(msg.trav_classes).index("default")

        forward_nodes = 0
        forward_frontiers = 0
        for node in msg.nodes:
            dx = node.pose.position.x - origin_x
            dy = node.pose.position.y - origin_y
            distance = math.hypot(dx, dy)
            if distance < 0.25 or distance > self.startup_forward_range:
                continue
            if (dx * heading_x + dy * heading_y) / distance < minimum_cosine:
                continue
            forward_nodes += 1
            if (
                frontier_class_index < len(node.trav_properties)
                and node.trav_properties[frontier_class_index].is_frontier
            ):
                forward_frontiers += 1
        return forward_nodes, forward_frontiers

    def _on_target_estimate(self, msg: TargetEstimate) -> None:
        """两视角粗定位先引导导航, 稳定估计随后提升精度"""
        estimate_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (
            self._target_changed_stamp_sec is not None
            and estimate_stamp > 0.0
            and estimate_stamp < self._target_changed_stamp_sec
        ):
            return
        if self._reached_latch_is_active():
            return
        self.latest_target_estimate_time = self.get_clock().now()
        target = PoseStamped()
        target.header = copy.deepcopy(msg.header)
        target.pose = copy.deepcopy(msg.pose.pose)
        if not self._pose_is_finite(target):
            self.get_logger().warn("收到包含非有限数的融合目标, 已忽略")
            return

        coarse_ready = coarse_target_evidence_ready(
            msg.state,
            msg.accepted_views,
            msg.confidence,
            self.coarse_target_min_views,
            self.coarse_target_min_confidence,
        )
        if (
            not msg.stable
            and not coarse_ready
            and int(msg.accepted_views) == 1
            and msg.state == "PENDING"
        ):
            protection_was_active = self._pending_evidence_protection_active(
                self.latest_target_estimate_time
            )
            self.pending_evidence_time = self.latest_target_estimate_time
            pending_bearing = self._pending_target_bearing(msg)
            if pending_bearing is not None:
                self.pending_bearing_yaw = pending_bearing
            if not protection_was_active and self.pending_bearing_yaw is not None:
                self.pending_observation_started_time = (
                    self.latest_target_estimate_time
                )
                self.pending_observation_complete = False
                self.pending_reposition_goal = None
                self.pending_reposition_attempts = 0
                self.get_logger().info(
                    "单视角目标候选观察已启用, "
                    f"观察={self.pending_observation_duration_sec:.1f}s, "
                    f"保护={self.pending_evidence_protection_sec:.1f}s, "
                    "只使用目标方向并保持当前位置"
                )
        if not msg.stable and not coarse_ready:
            return
        if self.metric_target_stable and not msg.stable:
            return
        rejection_reason = self._target_rejection_reason(msg, target)
        if rejection_reason:
            self.get_logger().warn(f"目标估计未通过物理门控, 已忽略: {rejection_reason}")
            return

        source_improved = int(msg.source) > int(self.metric_target_source)
        quality_improved = bool(msg.stable) and not self.metric_target_stable
        confidence_improved = (
            self.metric_target is None
            or float(msg.confidence) >= self.metric_target_confidence + 0.1
        )
        update_min_distance = (
            self.stable_target_update_min_distance
            if msg.stable
            else self.target_update_min_distance
        )
        moved_enough = (
            self.metric_target is None
            or self._pose_xy_distance(self.metric_target, target)
            >= update_min_distance
        )
        if self.metric_target is not None and not (
            quality_improved or source_improved or confidence_improved or moved_enough
        ):
            return

        first_metric_target = self.metric_target is None
        target_moved = (
            self.metric_target is not None
            and self._pose_xy_distance(self.metric_target, target) >= update_min_distance
        )
        self.metric_target = target
        self.metric_target_confidence = float(msg.confidence)
        self.metric_target_source = int(msg.source)
        self.metric_target_stable = bool(msg.stable)
        self.metric_target_state = str(msg.state)
        self._clear_pending_observation()
        if target_moved:
            self.used_reposition_goals.clear()
        if msg.stable:
            self._reset_target_observation()
            if target_moved:
                self.final_approach_goal = None
                self.final_reposition_goal = None
        elif target_moved and self.target_reposition_goal is not None:
            self.target_reposition_goal = None
        if first_metric_target and not msg.stable:
            self.get_logger().info(
                "导航目标已切换为视觉粗定位, "
                f"位置=({target.pose.position.x:.2f}, {target.pose.position.y:.2f}, "
                f"{target.pose.position.z:.2f}), "
                f"融合状态={_estimate_state_name(msg.state)}, "
                f"置信度={self.metric_target_confidence:.2f}, "
                f"有效视角={int(msg.accepted_views)}"
            )
        elif first_metric_target or quality_improved or source_improved:
            self.get_logger().info(
                "导航目标已切换为稳定融合定位, "
                f"位置=({target.pose.position.x:.2f}, {target.pose.position.y:.2f}, "
                f"{target.pose.position.z:.2f}), "
                f"来源={_estimate_source_name(msg.source)}, "
                f"置信度={self.metric_target_confidence:.2f}"
            )

    def _on_object_reached(self, msg: Bool) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1e-9
        if (
            msg.data
            and self._target_changed_stamp_sec is not None
            and now_seconds - self._target_changed_stamp_sec
            < _TARGET_CHANGE_REACHED_GUARD_SEC
        ):
            return
        self.latest_object_reached = bool(msg.data)
        self.latest_object_reached_time = now
        if not self.latest_object_reached:
            return
        reason_code, reason_text = self._object_reached_gate_reason(
            self.latest_object_reached_time
        )
        if reason_code is None:
            self._activate_reached_latch(self.latest_object_reached_time)
            return
        if reason_code != self._last_reached_gate_reason:
            self.get_logger().info(
                f"视觉近距离证据已收到, 完成门控等待中, 原因={reason_text}"
            )
            self._last_reached_gate_reason = reason_code

    def _on_object_search_target(self, msg: String) -> None:
        """切换任务时清除旧定位、完成锁存和旧探索方向"""
        target = normalize_object_search_target(msg.data)
        if target is None:
            self.get_logger().warn("Goal Mux 收到空搜索目标, 已忽略")
            return
        if target == self.current_target:
            return

        self.current_target = target
        self._target_changed_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self._clear_target_search_state()
        self.latest_object_reached = False
        self.latest_object_reached_time = None
        self.reached_latched = False
        self.reached_hold_goal = None
        self.initial_search_goal = None
        self.exploration_heading_yaw = None
        self.target_reposition_attempts = 0
        self.used_reposition_goals.clear()
        self._last_state = ""
        self._last_reached_gate_reason = ""
        self.completion_pub.publish(Bool(data=False))
        self.get_logger().info(f"目标搜索状态已重置, target={target!r}")

    def _on_timer(self) -> None:
        state, goal = self._select_goal()
        self.completion_pub.publish(Bool(data=self.reached_latched))
        self._publish_status(state, goal)
        if goal is not None:
            self.goal_pub.publish(goal)

    def _select_goal(self) -> tuple[str, PoseStamped | None]:
        """两视角粗目标出现前持续使用固定初始探索目标"""
        now = self.get_clock().now()
        target_age = self._age_seconds(now, self.latest_target_estimate_time)
        mode = self.policy.select(
            GoalSelectionInput(
                reached_latched=self._reached_latch_is_active(),
                reached_hold_available=self.reached_hold_goal is not None,
                odom_available=self.latest_odom is not None,
                completion_ready=self._object_reached_is_active(now),
                metric_target_available=self.metric_target is not None,
                metric_target_stable=self.metric_target_stable,
                metric_target_age_sec=target_age,
                coarse_target_timeout_sec=self.coarse_target_timeout_sec,
            )
        )
        if mode == GoalMode.WAIT_FOR_ODOM:
            return ObjectSearchState.WAIT_FOR_ODOM, None
        if mode == GoalMode.HOLD_REACHED:
            return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)
        if mode == GoalMode.ACTIVATE_REACHED:
            self._activate_reached_latch(now)
            return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)
        if mode == GoalMode.FOLLOW_STABLE_TARGET:
            return self._select_final_target_goal(now)
        if mode == GoalMode.FOLLOW_COARSE_TARGET:
            return self._select_coarse_target_goal(now)
        if mode == GoalMode.EXPIRE_COARSE_TARGET:
            self.get_logger().info(
                "视觉粗目标已过期, 恢复原探索分支, "
                f"age={target_age:.2f}s"
            )
            self._clear_metric_target()

        pending_state, pending_goal = self._select_pending_target_goal(now)
        if pending_state is not None:
            return pending_state, pending_goal

        if not self._startup_observation_is_complete(now):
            return (
                ObjectSearchState.STARTUP_OBSERVATION,
                self._build_startup_observation_goal(now),
            )

        return ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL, self._build_initial_goal(now)

    def _pending_target_bearing(self, msg: TargetEstimate) -> float | None:
        """读取相机 Mask 质心射线方向, 不使用粒子距离"""
        if not msg.bearing_valid:
            return None
        bearing_x = float(msg.bearing.x)
        bearing_y = float(msg.bearing.y)
        if not math.isfinite(bearing_x) or not math.isfinite(bearing_y):
            return None
        if math.hypot(bearing_x, bearing_y) < 1e-6:
            return None
        return math.atan2(bearing_y, bearing_x)

    def _select_reposition_position(
        self,
        bearing_yaw: float,
        distance: float,
        tolerance: float,
    ) -> tuple[float, float] | None:
        """从当前连通图的安全覆盖区域选择未访问横向观察点

        导航图节点稀疏, free_radius 才表示节点周围已确认安全的连续区域
        候选同时比较左右两侧, 墙边无安全覆盖时会自然选择开阔侧
        """
        graph = self.latest_nav_graph
        odom = self.latest_odom
        if graph is None or odom is None or self.startup_nav_graph_time is None:
            return None
        if self._age_seconds(
            self.get_clock().now(),
            self.startup_nav_graph_time,
        ) > self.startup_graph_timeout_sec:
            return None
        if graph.header.frame_id and graph.header.frame_id != odom.header.frame_id:
            return None

        graph_costs = self._reachable_graph_costs(graph)
        if not graph_costs:
            return None

        robot_x = float(odom.pose.pose.position.x)
        robot_y = float(odom.pose.pose.position.y)
        self._remember_reposition_goal(robot_x, robot_y, tolerance)
        tangent_x = -math.sin(bearing_yaw)
        tangent_y = math.cos(bearing_yaw)
        bearing_x = math.cos(bearing_yaw)
        bearing_y = math.sin(bearing_yaw)
        trav_index = self._default_traversability_index(graph)
        best = None

        for side in (1.0, -1.0):
            ideal_x = robot_x + side * tangent_x * distance
            ideal_y = robot_y + side * tangent_y * distance
            for node_index, graph_cost in graph_costs.items():
                node = graph.nodes[node_index]
                free_radius = self._node_free_radius(node, trav_index)
                safe_radius = free_radius - tolerance
                if safe_radius <= 0.0:
                    continue

                node_x = float(node.pose.position.x)
                node_y = float(node.pose.position.y)
                to_ideal_x = ideal_x - node_x
                to_ideal_y = ideal_y - node_y
                to_ideal = math.hypot(to_ideal_x, to_ideal_y)
                scale = min(1.0, safe_radius / max(to_ideal, 1e-9))
                candidate_x = node_x + to_ideal_x * scale
                candidate_y = node_y + to_ideal_y * scale
                delta_x = candidate_x - robot_x
                delta_y = candidate_y - robot_y
                lateral = side * (delta_x * tangent_x + delta_y * tangent_y)
                radial = abs(delta_x * bearing_x + delta_y * bearing_y)
                move_distance = math.hypot(delta_x, delta_y)
                if (
                    lateral < tolerance
                    or radial > tolerance
                    or move_distance > distance + tolerance
                    or self._reposition_goal_was_used(
                        candidate_x,
                        candidate_y,
                        tolerance,
                    )
                ):
                    continue

                node_to_candidate = math.hypot(
                    candidate_x - node_x,
                    candidate_y - node_y,
                )
                remaining_clearance = free_radius - node_to_candidate
                score = (
                    math.hypot(candidate_x - ideal_x, candidate_y - ideal_y),
                    radial,
                    graph_cost + node_to_candidate,
                    -remaining_clearance,
                )
                if best is None or score < best[0]:
                    best = (score, candidate_x, candidate_y)

        if best is None:
            return None
        self.used_reposition_goals.append((best[1], best[2]))
        return best[1], best[2]

    @staticmethod
    def _default_traversability_index(graph: NavigationGraph) -> int:
        if "default" in graph.trav_classes:
            return list(graph.trav_classes).index("default")
        return 0

    @classmethod
    def _reachable_graph_costs(
        cls,
        graph: NavigationGraph,
    ) -> dict[int, float]:
        """使用公开边代价计算 current node 所在连通分量"""
        node_count = len(graph.nodes)
        start = int(graph.current_node_idx)
        if start >= node_count:
            return {}
        trav_index = cls._default_traversability_index(graph)
        adjacency: list[list[tuple[int, float]]] = [
            [] for _ in range(node_count)
        ]
        for edge in graph.edges:
            source = int(edge.from_idx)
            target = int(edge.to_idx)
            if source >= node_count or target >= node_count:
                continue
            weight = math.hypot(
                graph.nodes[source].pose.position.x
                - graph.nodes[target].pose.position.x,
                graph.nodes[source].pose.position.y
                - graph.nodes[target].pose.position.y,
            )
            if trav_index < len(edge.traversability):
                configured = float(
                    edge.traversability[trav_index].traversability_cost
                )
                if math.isfinite(configured) and configured > 0.0:
                    weight = configured
            if not math.isfinite(weight) or weight <= 0.0:
                continue
            adjacency[source].append((target, weight))
            adjacency[target].append((source, weight))

        costs = {start: 0.0}
        queue = [(0.0, start)]
        while queue:
            current_cost, node_index = heapq.heappop(queue)
            if current_cost > costs[node_index]:
                continue
            for neighbor, weight in adjacency[node_index]:
                candidate_cost = current_cost + weight
                if candidate_cost >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = candidate_cost
                heapq.heappush(queue, (candidate_cost, neighbor))
        return costs

    @staticmethod
    def _node_free_radius(node, trav_index: int) -> float:
        if trav_index >= len(node.trav_properties):
            return 0.0
        value = float(node.trav_properties[trav_index].free_radius)
        return value if math.isfinite(value) and value > 0.0 else 0.0

    def _remember_reposition_goal(
        self,
        x: float,
        y: float,
        tolerance: float,
    ) -> None:
        if not self._reposition_goal_was_used(x, y, tolerance):
            self.used_reposition_goals.append((x, y))

    def _reposition_goal_was_used(
        self,
        x: float,
        y: float,
        tolerance: float,
    ) -> bool:
        return any(
            math.hypot(x - used_x, y - used_y) <= tolerance
            for used_x, used_y in self.used_reposition_goals
        )

    def _pending_observation_is_active(self, now) -> bool:
        """判断当前单视角静止观察阶段是否仍在进行"""
        if (
            self.pending_bearing_yaw is None
            or self.pending_observation_started_time is None
            or self.pending_observation_complete
            or not self._pending_evidence_protection_active(now)
        ):
            return False
        if self._age_seconds(now, self.pending_observation_started_time) < (
            self.pending_observation_duration_sec
        ):
            return True
        self.pending_observation_complete = True
        self.get_logger().info(
            "单视角目标候选静止观察结束, "
            "准备换位或恢复探索"
        )
        return False

    def _select_pending_target_goal(
        self,
        now,
    ) -> tuple[str | None, PoseStamped | None]:
        """先静止观察单视角候选, 仍可见时选择新的安全观察点"""
        if (
            self.latest_odom is None
            or self.pending_bearing_yaw is None
            or not self._pending_evidence_protection_active(now)
        ):
            return None, None

        evidence_age = self._age_seconds(
            now,
            self.pending_evidence_time,
        )
        evidence_fresh = (
            evidence_age <= self.pending_reposition_visibility_timeout_sec
        )
        if self.pending_reposition_goal is not None:
            if not evidence_fresh:
                self.pending_reposition_goal = None
                self.pending_observation_complete = True
                self.get_logger().info(
                    "单视角目标在换位途中失去新鲜证据, "
                    "恢复原探索分支"
                )
                return None, None
            if self._odom_distance_to_pose(self.pending_reposition_goal) > (
                self.pending_reposition_tolerance
            ):
                return (
                    ObjectSearchState.TARGET_PENDING_REPOSITION,
                    self._retime_pose(self.pending_reposition_goal, now),
                )
            self.pending_reposition_goal = None
            self.pending_observation_started_time = now
            self.pending_observation_complete = False
            self.get_logger().info(
                "已到达单视角横向观察点, 继续面向目标观察"
            )

        if self._pending_observation_is_active(now):
            return (
                ObjectSearchState.TARGET_PENDING_OBSERVATION,
                self._build_pending_observation_goal(now),
            )

        if evidence_fresh:
            reposition_goal = self._start_pending_reposition(now)
            if reposition_goal is not None:
                return (
                    ObjectSearchState.TARGET_PENDING_REPOSITION,
                    reposition_goal,
                )
            self.pending_observation_started_time = now
            self.pending_observation_complete = False
            return (
                ObjectSearchState.TARGET_PENDING_OBSERVATION,
                self._build_pending_observation_goal(now),
            )
        return None, None

    def _start_pending_reposition(self, now) -> PoseStamped | None:
        """沿目标射线切向选择可达的新观察点"""
        assert self.latest_odom is not None
        assert self.pending_bearing_yaw is not None
        robot = self.latest_odom.pose.pose.position
        position = self._select_reposition_position(
            self.pending_bearing_yaw,
            self.pending_reposition_distance,
            self.pending_reposition_tolerance,
        )
        if position is None:
            self.get_logger().info(
                "单视角目标附近暂无新的可达观察点, 保持当前位置"
            )
            return None
        goal = PoseStamped()
        goal.header.frame_id = (
            self.frame_id or self.latest_odom.header.frame_id
        )
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = robot.z
        self._set_pose_yaw(goal, self.pending_bearing_yaw)
        self.pending_reposition_attempts += 1
        self.pending_reposition_goal = copy.deepcopy(goal)
        self.get_logger().info(
            "单视角目标持续可见但证据不足, "
            "横向更换观察点, "
            f"距离={self.pending_reposition_distance:.2f}m, "
            f"attempt={self.pending_reposition_attempts}"
        )
        return goal

    def _build_pending_observation_goal(self, now) -> PoseStamped:
        """保持当前位置并朝向单视角候选"""
        assert self.latest_odom is not None
        assert self.pending_bearing_yaw is not None
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id or self.latest_odom.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position = copy.deepcopy(self.latest_odom.pose.pose.position)
        self._set_pose_yaw(goal, self.pending_bearing_yaw)
        return goal

    def _select_final_target_goal(self, now) -> tuple[str, PoseStamped]:
        """先到稳定目标外的安全位置, 再静止确认或小范围换位"""
        assert self.metric_target is not None
        assert self.latest_odom is not None

        if self.final_reposition_goal is not None:
            if self._odom_distance_to_pose(self.final_reposition_goal) > (
                self.final_observation_reposition_tolerance
            ):
                return (
                    ObjectSearchState.TARGET_FINAL_REPOSITION,
                    self._retime_pose(self.final_reposition_goal, now),
                )
            self.final_reposition_goal = None
            self._start_final_observation(now, "到达新观察点")

        entry_distance = (
            self.final_observation_distance
            + self.final_observation_entry_tolerance
        )
        if (
            self.final_observation_started_time is None
            and self._target_distance(self.metric_target) > entry_distance
        ):
            return (
                ObjectSearchState.TARGET_APPROACH_METRIC,
                self._build_final_approach_goal(now),
            )

        if self.final_observation_started_time is None:
            self._start_final_observation(now, "进入最终观察距离")

        reposition_goal = self._update_final_observation(now)
        if reposition_goal is not None:
            return ObjectSearchState.TARGET_FINAL_REPOSITION, reposition_goal
        return (
            ObjectSearchState.TARGET_FINAL_OBSERVATION,
            self._build_final_observation_goal(now),
        )

    def _start_final_observation(self, now, reason: str) -> None:
        self.final_observation_started_time = now
        self.final_observation_phase = "ALIGN"
        self.final_observation_phase_time = now
        self.get_logger().info(
            "开始稳定目标最终观察, "
            f"原因={reason}, 距离={self._target_distance(self.metric_target):.2f}m"
        )

    def _update_final_observation(self, now) -> PoseStamped | None:
        """目标可见时静止确认, 丢失后小角度重捕获并更换观察点"""
        target_age = self._age_seconds(now, self.latest_target_estimate_time)
        if target_age <= self.final_observation_lost_timeout_sec:
            if self.final_observation_phase != "ALIGN":
                self.final_observation_phase = "ALIGN"
                self.final_observation_phase_time = now
            if self._age_seconds(now, self.final_observation_started_time) < (
                self.final_observation_duration_sec
            ):
                return None
            return self._start_final_reposition(now, "静止确认后证据仍不足")

        if self.final_observation_phase == "ALIGN":
            self._set_final_observation_phase("SCAN_LEFT", now)
            return None
        if not self._final_observation_yaw_reached():
            return None
        if self._age_seconds(now, self.final_observation_phase_time) < (
            self.final_observation_scan_hold_sec
        ):
            return None

        next_phase = {
            "SCAN_LEFT": "SCAN_RIGHT",
            "SCAN_RIGHT": "SCAN_RETURN",
        }.get(self.final_observation_phase)
        if next_phase is not None:
            self._set_final_observation_phase(next_phase, now)
            return None
        return self._start_final_reposition(now, "局部重捕获后证据仍不足")

    def _set_final_observation_phase(self, phase: str, now) -> None:
        self.final_observation_phase = phase
        self.final_observation_phase_time = now
        self.get_logger().info(
            f"最终观察重捕获阶段切换, phase={phase}, "
            f"target_yaw={math.degrees(self._final_observation_yaw()):.1f}deg"
        )

    def _start_final_reposition(self, now, reason: str) -> PoseStamped | None:
        """沿目标切向选择可达且未访问的附近观察点"""
        assert self.metric_target is not None
        assert self.latest_odom is not None
        robot = self.latest_odom.pose.pose.position
        position = self._select_reposition_position(
            self._bearing_to_target(robot),
            self.final_observation_reposition_distance,
            self.final_observation_reposition_tolerance,
        )
        if position is None:
            self.final_observation_started_time = now
            self.get_logger().info(
                "稳定目标附近暂无新的可达观察点, 保持当前位置"
            )
            return None

        goal = PoseStamped()
        goal.header.frame_id = self.metric_target.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = robot.z
        self._set_pose_yaw(goal, self._bearing_to_target(goal.pose.position))
        self.final_reposition_attempts += 1
        self.final_reposition_goal = copy.deepcopy(goal)
        self.final_observation_started_time = None
        self.get_logger().info(
            f"最终观察需要新位置, 原因={reason}, "
            f"横向移动={self.final_observation_reposition_distance:.2f}m, "
            f"attempt={self.final_reposition_attempts}"
        )
        return goal

    def _build_final_approach_goal(self, now) -> PoseStamped:
        """在稳定目标外生成一次固定的安全观察位姿"""
        if self.final_approach_goal is not None:
            return self._retime_pose(self.final_approach_goal, now)
        assert self.metric_target is not None
        assert self.latest_odom is not None
        target = self.metric_target.pose.position
        robot = self.latest_odom.pose.pose.position
        from_target_x = robot.x - target.x
        from_target_y = robot.y - target.y
        distance = max(math.hypot(from_target_x, from_target_y), 1e-6)

        goal = PoseStamped()
        goal.header.frame_id = self.metric_target.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = (
            target.x
            + from_target_x / distance * self.final_observation_distance
        )
        goal.pose.position.y = (
            target.y
            + from_target_y / distance * self.final_observation_distance
        )
        goal.pose.position.z = robot.z
        self._set_pose_yaw(goal, self._bearing_to_target(goal.pose.position))
        self.final_approach_goal = copy.deepcopy(goal)
        return goal

    def _build_final_observation_goal(self, now) -> PoseStamped:
        assert self.latest_odom is not None
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id or self.latest_odom.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position = copy.deepcopy(self.latest_odom.pose.pose.position)
        self._set_pose_yaw(goal, self._final_observation_yaw())
        return goal

    def _final_observation_yaw(self) -> float:
        assert self.latest_odom is not None
        target_yaw = self._bearing_to_target(self.latest_odom.pose.pose.position)
        offset = {
            "SCAN_LEFT": self.final_observation_scan_yaw,
            "SCAN_RIGHT": -self.final_observation_scan_yaw,
        }.get(self.final_observation_phase, 0.0)
        return _normalize_angle(target_yaw + offset)

    def _final_observation_yaw_reached(self) -> bool:
        assert self.latest_odom is not None
        current_yaw = _yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        error = _normalize_angle(current_yaw - self._final_observation_yaw())
        return abs(error) <= self.final_observation_scan_tolerance

    def _startup_observation_is_complete(self, now) -> bool:
        """完成静止预热, 必要时依次执行左右小角度观察"""
        if self.startup_completed:
            return True
        if not self._startup_inputs_ready(now):
            return False
        if self.startup_scan_phase == "WARMUP":
            if self._startup_forward_region_ready():
                self._complete_startup_observation("前向区域可规划")
                return True
            if not self.startup_scan_enabled:
                self._complete_startup_observation("条件扫描已禁用")
                return True
            if self.startup_insufficient_frames < self.startup_scan_trigger_frames:
                return False
            self._set_startup_scan_phase("SCAN_LEFT", now)
            self.get_logger().info(
                "启动前向区域不足, 开始左右小角度观察, "
                f"前向节点={self.startup_forward_nodes}, "
                f"前向Frontier={self.startup_forward_frontiers}"
            )
            return False

        phase_age = self._age_seconds(now, self.startup_scan_phase_time)
        target_reached = self._startup_scan_target_reached()
        phase_timed_out = phase_age >= self.startup_scan_phase_timeout_sec
        if not target_reached and not phase_timed_out:
            return False
        new_scored_frames = (
            self.startup_scored_graph_frames - self.startup_scan_phase_scored_frames
        )
        if (
            target_reached
            and not phase_timed_out
            and (
                phase_age < self.startup_scan_hold_sec
                or new_scored_frames < 1
            )
        ):
            return False
        if phase_timed_out and not target_reached:
            self.get_logger().warning(
                "启动观察转向超时, 跳过当前朝向, "
                f"phase={self.startup_scan_phase}, "
                f"等待={phase_age:.1f}s"
            )

        next_phase = {
            "SCAN_LEFT": "SCAN_RIGHT",
            "SCAN_RIGHT": "SCAN_RETURN",
            "SCAN_RETURN": "COMPLETE",
        }.get(self.startup_scan_phase)
        if next_phase == "COMPLETE":
            self._complete_startup_observation("条件扫描完成")
            return True
        if next_phase is not None:
            self._set_startup_scan_phase(next_phase, now)
        return False

    def _startup_inputs_ready(self, now) -> bool:
        if self.startup_started_time is None:
            return False
        if self._age_seconds(now, self.startup_started_time) < self.startup_warmup_sec:
            return False
        if self.startup_nav_graph_frames < self.startup_min_nav_graph_frames:
            return False
        if self.startup_scored_graph_frames < self.startup_min_scored_graph_frames:
            return False
        return (
            self._age_seconds(now, self.startup_nav_graph_time)
            <= self.startup_graph_timeout_sec
            and self._age_seconds(now, self.startup_scored_graph_time)
            <= self.startup_graph_timeout_sec
        )

    def _startup_forward_region_ready(self) -> bool:
        return (
            self.startup_forward_nodes >= self.startup_min_forward_nodes
            and self.startup_forward_frontiers >= self.startup_min_forward_frontiers
        )

    def _set_startup_scan_phase(self, phase: str, now) -> None:
        self.startup_scan_phase = phase
        self.startup_scan_phase_time = now
        self.startup_scan_phase_scored_frames = self.startup_scored_graph_frames
        target_yaw = self._startup_observation_yaw()
        self.get_logger().info(
            f"启动观察阶段切换, phase={phase}, target_yaw={math.degrees(target_yaw):.1f}deg"
        )

    def _startup_scan_target_reached(self) -> bool:
        if self.latest_odom is None:
            return False
        current_yaw = _yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        return abs(_normalize_angle(current_yaw - self._startup_observation_yaw())) <= (
            self.startup_scan_yaw_tolerance
        )

    def _startup_observation_yaw(self) -> float:
        initial_yaw = self.exploration_heading_yaw or 0.0
        offset = {
            "SCAN_LEFT": self.startup_scan_yaw,
            "SCAN_RIGHT": -self.startup_scan_yaw,
        }.get(self.startup_scan_phase, 0.0)
        return _normalize_angle(initial_yaw + offset)

    def _complete_startup_observation(self, reason: str) -> None:
        self.startup_completed = True
        self.startup_scan_phase = "COMPLETE"
        self.get_logger().info(
            "启动观察完成, "
            f"原因={reason}, 原始图帧={self.startup_nav_graph_frames}, "
            f"评分图帧={self.startup_scored_graph_frames}, "
            f"前向节点={self.startup_forward_nodes}, "
            f"前向Frontier={self.startup_forward_frontiers}"
        )

    def _select_coarse_target_goal(self, now) -> tuple[str, PoseStamped]:
        """先到安全观察距离, 再对准目标观察或移动形成新视差"""
        assert self.metric_target is not None
        assert self.latest_odom is not None

        if self.target_reposition_goal is not None:
            if self._odom_distance_to_pose(self.target_reposition_goal) > (
                self.target_observation_reposition_tolerance
            ):
                return (
                    ObjectSearchState.TARGET_APPROACH_COARSE,
                    self._retime_pose(self.target_reposition_goal, now),
                )
            self.target_reposition_goal = None
            self._start_target_observation(now, "到达新观察点")

        target_distance = self._target_distance(self.metric_target)
        observation_entry_distance = (
            self.coarse_observation_distance
            + self.coarse_observation_entry_tolerance
        )
        if (
            self.target_observation_started_time is None
            and target_distance > observation_entry_distance
        ):
            return (
                ObjectSearchState.TARGET_APPROACH_COARSE,
                self._build_coarse_observation_approach_goal(now),
            )

        if self.target_observation_started_time is None:
            self._start_target_observation(now, "进入安全观察距离")

        reposition_goal = self._update_target_observation(now)
        if reposition_goal is not None:
            return ObjectSearchState.TARGET_APPROACH_COARSE, reposition_goal
        return (
            ObjectSearchState.TARGET_OBSERVATION,
            self._build_target_observation_goal(now),
        )

    def _start_target_observation(self, now, reason: str) -> None:
        self.target_observation_started_time = now
        self.target_observation_phase = "ALIGN"
        self.target_observation_phase_time = now
        self.get_logger().info(
            f"开始粗目标观察, 原因={reason}, 距离={self._target_distance(self.metric_target):.2f}m"
        )

    def _update_target_observation(self, now) -> PoseStamped | None:
        """目标可见时对准等待, 失联时局部扫描, 超时后横向换观察点"""
        target_age = self._age_seconds(now, self.latest_target_estimate_time)
        if target_age <= self.target_observation_lost_timeout_sec:
            if self.target_observation_phase != "ALIGN":
                self.target_observation_phase = "ALIGN"
                self.target_observation_phase_time = now
            if self._age_seconds(now, self.target_observation_started_time) < (
                self.target_observation_duration_sec
            ):
                return None
            return self._start_target_reposition(now, "静止观察后仍未稳定")

        if self.target_observation_phase == "ALIGN":
            self._set_target_observation_phase("SCAN_LEFT", now)
            return None
        if not self._target_observation_yaw_reached():
            return None
        if self._age_seconds(now, self.target_observation_phase_time) < (
            self.target_observation_scan_hold_sec
        ):
            return None

        next_phase = {
            "SCAN_LEFT": "SCAN_RIGHT",
            "SCAN_RIGHT": "SCAN_RETURN",
        }.get(self.target_observation_phase)
        if next_phase is not None:
            self._set_target_observation_phase(next_phase, now)
            return None
        return self._start_target_reposition(now, "局部重捕获后仍未稳定")

    def _set_target_observation_phase(self, phase: str, now) -> None:
        self.target_observation_phase = phase
        self.target_observation_phase_time = now
        self.get_logger().info(
            f"粗目标局部重捕获阶段切换, phase={phase}, "
            f"target_yaw={math.degrees(self._target_observation_yaw()):.1f}deg"
        )

    def _start_target_reposition(self, now, reason: str) -> PoseStamped | None:
        """沿目标切向选择可达且未访问的新观察点"""
        assert self.metric_target is not None
        assert self.latest_odom is not None
        robot = self.latest_odom.pose.pose.position
        position = self._select_reposition_position(
            self._bearing_to_target(robot),
            self.target_observation_reposition_distance,
            self.target_observation_reposition_tolerance,
        )
        if position is None:
            self.target_observation_started_time = now
            self.get_logger().info(
                "粗目标附近暂无新的可达观察点, 保持当前位置"
            )
            return None

        goal = PoseStamped()
        goal.header.frame_id = self.metric_target.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = robot.z
        self._set_pose_yaw(goal, self._bearing_to_target(goal.pose.position))
        self.target_reposition_attempts += 1
        self.target_reposition_goal = copy.deepcopy(goal)
        self.target_observation_started_time = None
        self.get_logger().info(
            f"粗目标观察需要新视差, 原因={reason}, "
            f"横向移动={self.target_observation_reposition_distance:.2f}m, "
            f"attempt={self.target_reposition_attempts}"
        )
        return goal

    def _build_coarse_observation_approach_goal(self, now) -> PoseStamped:
        """在目标与机器人连线上生成安全观察位姿"""
        assert self.metric_target is not None
        assert self.latest_odom is not None
        target = self.metric_target.pose.position
        robot = self.latest_odom.pose.pose.position
        from_target_x = robot.x - target.x
        from_target_y = robot.y - target.y
        distance = max(math.hypot(from_target_x, from_target_y), 1e-6)
        goal = PoseStamped()
        goal.header.frame_id = self.metric_target.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = target.x + from_target_x / distance * self.coarse_observation_distance
        goal.pose.position.y = target.y + from_target_y / distance * self.coarse_observation_distance
        goal.pose.position.z = robot.z
        self._set_pose_yaw(goal, self._bearing_to_target(goal.pose.position))
        return goal

    def _build_target_observation_goal(self, now) -> PoseStamped:
        assert self.latest_odom is not None
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id or self.latest_odom.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position = copy.deepcopy(self.latest_odom.pose.pose.position)
        self._set_pose_yaw(goal, self._target_observation_yaw())
        return goal

    def _target_observation_yaw(self) -> float:
        assert self.latest_odom is not None
        target_yaw = self._bearing_to_target(self.latest_odom.pose.pose.position)
        offset = {
            "SCAN_LEFT": self.target_observation_scan_yaw,
            "SCAN_RIGHT": -self.target_observation_scan_yaw,
        }.get(self.target_observation_phase, 0.0)
        return _normalize_angle(target_yaw + offset)

    def _target_observation_yaw_reached(self) -> bool:
        assert self.latest_odom is not None
        current_yaw = _yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        error = _normalize_angle(current_yaw - self._target_observation_yaw())
        return abs(error) <= self.target_observation_scan_tolerance

    def _bearing_to_target(self, position) -> float:
        assert self.metric_target is not None
        return math.atan2(
            self.metric_target.pose.position.y - position.y,
            self.metric_target.pose.position.x - position.x,
        )

    def _odom_distance_to_pose(self, pose: PoseStamped) -> float:
        assert self.latest_odom is not None
        dx = pose.pose.position.x - self.latest_odom.pose.pose.position.x
        dy = pose.pose.position.y - self.latest_odom.pose.pose.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def _set_pose_yaw(pose: PoseStamped, yaw: float) -> None:
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

    def _reset_target_observation(self) -> None:
        self.target_observation_started_time = None
        self.target_observation_phase = "ALIGN"
        self.target_observation_phase_time = None
        self.target_reposition_goal = None

    def _reset_final_observation(self) -> None:
        self.final_observation_started_time = None
        self.final_observation_phase = "ALIGN"
        self.final_observation_phase_time = None
        self.final_approach_goal = None
        self.final_reposition_goal = None
        self.final_reposition_attempts = 0

    @staticmethod
    def _age_seconds(now, then) -> float:
        if then is None:
            return math.inf
        return (now - then).nanoseconds * 1e-9

    def _build_initial_goal(self, now) -> PoseStamped:
        """用当前 odom 朝向生成一个远处粗 goal, 驱动 planner 选择探索 frontier"""
        odom = self.latest_odom
        assert odom is not None

        if self.initial_search_goal is not None:
            return self._retime_pose(self.initial_search_goal, now)

        odom_frame = odom.header.frame_id
        goal_frame = self.frame_id or odom_frame
        if odom_frame and goal_frame != odom_frame and not self._warned_frame_mismatch:
            self.get_logger().warn(
                f"初始搜索目标坐标系={goal_frame}, 里程计坐标系={odom_frame}, "
                "请确认二者在同一全局坐标系"
            )
            self._warned_frame_mismatch = True

        yaw = self.exploration_heading_yaw
        if yaw is None:
            yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
            yaw += math.radians(self.initial_goal_heading_deg)
            self.exploration_heading_yaw = yaw
        goal = PoseStamped()
        goal.header.frame_id = goal_frame
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = (
            odom.pose.pose.position.x
            + self.initial_goal_distance * math.cos(yaw)
        )
        goal.pose.position.y = (
            odom.pose.pose.position.y
            + self.initial_goal_distance * math.sin(yaw)
        )
        goal.pose.position.z = odom.pose.pose.position.z
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        self.initial_search_goal = copy.deepcopy(goal)
        return goal

    def _build_startup_observation_goal(self, now) -> PoseStamped:
        """发布当前位置和观察朝向, 由 Planner 生成单点纯 yaw 路径"""
        odom = self.latest_odom
        assert odom is not None
        yaw = self._startup_observation_yaw()
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id or odom.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position = copy.deepcopy(
            self.startup_observation_position or odom.pose.pose.position
        )
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def _clear_target_search_state(self) -> None:
        self._clear_metric_target()
        self._clear_pending_observation()

    def _clear_metric_target(self) -> None:
        """清除粗目标或稳定目标, 保留独立的单视角候选状态"""
        self.metric_target = None
        self.metric_target_confidence = 0.0
        self.metric_target_source = TargetEstimate.SOURCE_NONE
        self.metric_target_stable = False
        self.metric_target_state = "EMPTY"
        self.latest_target_estimate_time = None
        self._reset_target_observation()
        self._reset_final_observation()

    def _clear_pending_observation(self) -> None:
        self.pending_evidence_time = None
        self.pending_observation_started_time = None
        self.pending_observation_complete = False
        self.pending_bearing_yaw = None
        self.pending_reposition_goal = None
        self.pending_reposition_attempts = 0

    def _build_hold_goal(self, now) -> PoseStamped:
        """到达目标观察点后发布当前位置, 让 planner 不再继续追远处点"""
        if self.reached_hold_goal is not None:
            return self._retime_pose(self.reached_hold_goal, now)

        odom = self.latest_odom
        assert odom is not None

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id or odom.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position = copy.deepcopy(odom.pose.pose.position)
        goal.pose.orientation = copy.deepcopy(odom.pose.pose.orientation)
        return goal

    def _object_reached_is_active(self, now) -> bool:
        reason_code, _ = self._object_reached_gate_reason(now)
        return reason_code is None

    def _object_reached_gate_reason(self, now) -> tuple[str | None, str]:
        """返回视觉连续证据或 LiDAR 锁定尚不能完成任务的原因"""
        evidence_source, evidence_age = self._completion_evidence(now)
        if evidence_source is None:
            return "no_completion_evidence", "无新鲜视觉证据或 LiDAR 锁定"
        target_pose = self._active_reached_target_pose()
        if target_pose is None:
            return "no_stable_target", "尚无稳定融合目标"
        target_distance = self._target_distance(target_pose)
        if not math.isfinite(target_distance):
            return "no_odom", "缺少 odom, 无法计算目标距离"
        if target_distance > self.object_reached_max_target_distance:
            return (
                "target_too_far",
                f"目标距离={target_distance:.2f}m, "
                f"limit={self.object_reached_max_target_distance:.2f}m",
            )
        return None, f"{evidence_source}证据已通过, age={evidence_age:.2f}s"

    def _completion_evidence(self, now) -> tuple[str | None, float]:
        """选择当前新鲜的连续视觉证据或连续 LiDAR 锁定"""
        visual_age = self._age_seconds(now, self.latest_object_reached_time)
        if (
            self.latest_object_reached
            and visual_age <= self.object_reached_timeout_sec
        ):
            return "连续视觉", visual_age
        lidar_age = self._age_seconds(now, self.latest_target_estimate_time)
        if (
            self.metric_target_state == "LIDAR_LOCKED"
            and lidar_age <= self.lidar_reached_timeout_sec
        ):
            return "LiDAR锁定", lidar_age
        return None, math.inf

    def _active_reached_target_pose(self) -> PoseStamped | None:
        """到达距离门控只使用稳定融合目标"""
        return self.metric_target if self.metric_target_stable else None

    def _activate_reached_latch(self, now) -> None:
        """到达确认后锁定停止 goal, 不再被后续 mask False 拉回搜索"""
        if self.reached_latched:
            return
        target_pose = self._active_reached_target_pose()
        target_distance = (
            self._target_distance(target_pose)
            if target_pose is not None
            else math.inf
        )
        self.reached_latched = True
        self._last_reached_gate_reason = ""
        self.reached_hold_goal = None
        if self.latest_odom is not None:
            self.reached_hold_goal = self._build_hold_goal(now)
        evidence_source, evidence_age = self._completion_evidence(now)
        self.get_logger().info(
            f"目标完成门控已通过, 目标距离={target_distance:.2f}m, "
            f"证据={evidence_source or '未知'}, 证据年龄={evidence_age:.2f}s"
        )
        self._clear_target_search_state()

    def _reached_latch_is_active(self) -> bool:
        """任务完成是终态, 节点生命周期内不允许回到搜索状态"""
        return self.reached_latched

    @staticmethod
    def _pose_xy_distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)

    def _target_distance(self, target: PoseStamped) -> float:
        if self.latest_odom is None:
            return math.inf
        dx = target.pose.position.x - self.latest_odom.pose.pose.position.x
        dy = target.pose.position.y - self.latest_odom.pose.pose.position.y
        return math.hypot(dx, dy)

    def _target_rejection_reason(
        self,
        msg: TargetEstimate,
        target: PoseStamped,
    ) -> str | None:
        """检查融合目标的坐标系、高度、距离和粗定位不确定度"""
        if target.header.frame_id and target.header.frame_id != self.frame_id:
            return f"坐标系={target.header.frame_id}, 期望={self.frame_id}"
        if self.latest_odom is None:
            return None if msg.stable else "缺少 odom, 无法验证粗定位"

        vertical_offset = abs(
            target.pose.position.z - self.latest_odom.pose.pose.position.z
        )
        if vertical_offset > self.target_max_vertical_offset:
            return (
                f"垂直偏差={vertical_offset:.2f}m > "
                f"{self.target_max_vertical_offset:.2f}m"
            )
        if msg.stable:
            return None

        distance = self._target_distance(target)
        if distance > self.coarse_target_max_distance:
            return (
                f"水平距离={distance:.2f}m > "
                f"{self.coarse_target_max_distance:.2f}m"
            )
        horizontal_variance = max(float(msg.pose.covariance[0]), 0.0)
        horizontal_variance += max(float(msg.pose.covariance[7]), 0.0)
        horizontal_std = math.sqrt(horizontal_variance)
        if not math.isfinite(horizontal_std):
            return "水平标准差不是有限数"
        if horizontal_std > self.coarse_target_max_horizontal_std:
            return (
                f"水平标准差={horizontal_std:.2f}m > "
                f"{self.coarse_target_max_horizontal_std:.2f}m"
            )
        return None

    def _retime_pose(self, pose: PoseStamped, now) -> PoseStamped:
        goal = PoseStamped()
        goal.header = copy.deepcopy(pose.header)
        goal.header.stamp = now.to_msg()
        goal.pose = copy.deepcopy(pose.pose)
        return goal

    def _pose_is_finite(self, pose: PoseStamped) -> bool:
        values = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        return all(math.isfinite(value) for value in values)

    def _publish_status(self, state: str, goal: PoseStamped | None) -> None:
        now = self.get_clock().now()
        typed_status = ObjectSearchStatus()
        typed_status.header.stamp = now.to_msg()
        typed_status.header.frame_id = goal.header.frame_id if goal else self.frame_id
        typed_status.state = _object_search_state_code(state)
        typed_status.pending_protection = (
            state in {
                ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL,
                ObjectSearchState.TARGET_PENDING_OBSERVATION,
                ObjectSearchState.TARGET_PENDING_REPOSITION,
            }
            and self._pending_evidence_protection_active(now)
        )
        self.status_pub.publish(typed_status)
        if state == self._last_state:
            return
        self._last_state = state
        state_name = _object_search_state_name(state)
        if goal is None:
            self.get_logger().info(f"目标搜索状态变化, 状态={state_name}")
            return
        self.get_logger().info(
            f"目标搜索状态变化, 状态={state_name}, "
            f"目标坐标系={goal.header.frame_id}, "
            f"导航目标=({goal.pose.position.x:.2f}, "
            f"{goal.pose.position.y:.2f}, "
            f"{goal.pose.position.z:.2f})"
        )

    def _pending_evidence_protection_active(self, now) -> bool:
        return self._age_seconds(now, self.pending_evidence_time) <= (
            self.pending_evidence_protection_sec
        )


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _estimate_source_name(source: int) -> str:
    return {
        TargetEstimate.SOURCE_VISION: "视觉粒子滤波(VISION)",
        TargetEstimate.SOURCE_FUSED: "视觉雷达融合(FUSED)",
        TargetEstimate.SOURCE_LIDAR: "雷达锁定(LIDAR)",
    }.get(int(source), f"未知来源({int(source)})")


def _estimate_state_name(state: str) -> str:
    return {
        "PENDING": "等待多视角(PENDING)",
        "TRACKING": "多视角跟踪(TRACKING)",
        "STABLE_VISION": "视觉稳定(STABLE_VISION)",
        "LIDAR_LOCKED": "雷达锁定(LIDAR_LOCKED)",
        "REACHED": "任务完成(REACHED)",
    }.get(state, f"未知状态({state})")


def _object_search_state_name(state: str) -> str:
    return {
        ObjectSearchState.WAIT_FOR_ODOM: "等待里程计(WAIT_FOR_ODOM)",
        ObjectSearchState.STARTUP_OBSERVATION: (
            "启动静止预热或条件扫描(STARTUP_OBSERVATION)"
        ),
        ObjectSearchState.TARGET_PENDING_OBSERVATION: (
            "单视角目标短时观察(TARGET_PENDING_OBSERVATION)"
        ),
        ObjectSearchState.TARGET_PENDING_REPOSITION: (
            "单视角目标横向换位(TARGET_PENDING_REPOSITION)"
        ),
        ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL: (
            "按初始方向探索(SEARCHING_WITH_INITIAL_GOAL)"
        ),
        ObjectSearchState.TARGET_APPROACH_COARSE: (
            "接近视觉粗目标(TARGET_APPROACH_COARSE)"
        ),
        ObjectSearchState.TARGET_OBSERVATION: (
            "面向视觉粗目标观察(TARGET_OBSERVATION)"
        ),
        ObjectSearchState.TARGET_APPROACH_METRIC: (
            "接近稳定融合目标(TARGET_APPROACH_METRIC)"
        ),
        ObjectSearchState.TARGET_FINAL_OBSERVATION: (
            "稳定目标最终观察(TARGET_FINAL_OBSERVATION)"
        ),
        ObjectSearchState.TARGET_FINAL_REPOSITION: (
            "更换最终观察点(TARGET_FINAL_REPOSITION)"
        ),
        ObjectSearchState.TARGET_REACHED_VIEWPOINT: (
            "目标到达观察点(TARGET_REACHED_VIEWPOINT)"
        ),
    }.get(state, f"未知状态({state})")


def _object_search_state_code(state: str) -> int:
    return {
        ObjectSearchState.WAIT_FOR_ODOM: ObjectSearchStatus.WAIT_FOR_ODOM,
        ObjectSearchState.STARTUP_OBSERVATION: ObjectSearchStatus.STARTUP_OBSERVATION,
        ObjectSearchState.TARGET_PENDING_OBSERVATION: ObjectSearchStatus.TARGET_PENDING_OBSERVATION,
        ObjectSearchState.TARGET_PENDING_REPOSITION: ObjectSearchStatus.TARGET_PENDING_REPOSITION,
        ObjectSearchState.TARGET_APPROACH_COARSE: ObjectSearchStatus.TARGET_APPROACH_COARSE,
        ObjectSearchState.TARGET_OBSERVATION: ObjectSearchStatus.TARGET_OBSERVATION,
        ObjectSearchState.TARGET_APPROACH_METRIC: ObjectSearchStatus.TARGET_APPROACH_METRIC,
        ObjectSearchState.TARGET_FINAL_OBSERVATION: ObjectSearchStatus.TARGET_FINAL_OBSERVATION,
        ObjectSearchState.TARGET_FINAL_REPOSITION: ObjectSearchStatus.TARGET_FINAL_REPOSITION,
        ObjectSearchState.TARGET_REACHED_VIEWPOINT: ObjectSearchStatus.TARGET_REACHED_VIEWPOINT,
        ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL: ObjectSearchStatus.SEARCHING_WITH_INITIAL_GOAL,
    }.get(state, ObjectSearchStatus.UNKNOWN)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSearchGoalMux()
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
