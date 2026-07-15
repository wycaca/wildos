import copy
import math
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from object_search_msgs.msg import TargetEstimate

from visual_navigation.object_search_types import ObjectSearchState


class ObjectSearchGoalMux(Node):
    """在初始探索、视觉粗目标、稳定目标和完成状态之间选择唯一 goal"""

    def __init__(self):
        super().__init__("object_search_goal_mux")

        self.declare_parameter("output_goal_topic", "/spot1/graphnav_goal_pose")
        self.declare_parameter("goal_viz_topic", "/spot1/object_search_goal_viz")
        self.declare_parameter("status_topic", "/spot1/object_search_status")
        self.declare_parameter("object_target_estimate_topic", "/spot1/object_target_estimate")
        self.declare_parameter("object_reached_topic", "/spot1/object_search_reached")
        self.declare_parameter("completion_topic", "/spot1/object_search_completed")
        self.declare_parameter("odom_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("initial_goal_mode", "heading")
        self.declare_parameter("initial_goal_distance", 30.0)
        self.declare_parameter("initial_goal_heading_deg", 0.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("object_reached_timeout_sec", 2.0)
        self.declare_parameter("object_reached_require_target_distance", True)
        self.declare_parameter("object_reached_max_target_distance", 2.0)
        self.declare_parameter("coarse_target_min_views", 2)
        self.declare_parameter("coarse_target_min_confidence", 0.35)
        self.declare_parameter("metric_target_update_distance", 0.75)

        self.output_goal_topic = self._param_str("output_goal_topic")
        self.goal_viz_topic = self._param_str("goal_viz_topic")
        self.status_topic = self._param_str("status_topic")
        self.object_target_estimate_topic = self._param_str("object_target_estimate_topic")
        self.object_reached_topic = self._param_str("object_reached_topic")
        self.completion_topic = self._param_str("completion_topic")
        self.odom_topic = self._param_str("odom_topic")
        self.frame_id = self._param_str("frame_id")
        self.initial_goal_mode = self._param_str("initial_goal_mode")
        self.initial_goal_distance = self._param_float("initial_goal_distance")
        self.initial_goal_heading_deg = self._param_float("initial_goal_heading_deg")
        self.publish_rate = max(self._param_float("publish_rate"), 0.1)
        self.object_reached_timeout_sec = max(self._param_float("object_reached_timeout_sec"), 0.0)
        self.object_reached_require_target_distance = self._param_bool(
            "object_reached_require_target_distance"
        )
        self.object_reached_max_target_distance = max(
            self._param_float("object_reached_max_target_distance"),
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
        self.metric_target_update_distance = max(
            self._param_float("metric_target_update_distance"),
            0.0,
        )

        if self.initial_goal_mode != "heading":
            raise ValueError(
                f"暂不支持 initial_goal_mode={self.initial_goal_mode}, "
                "当前只支持 heading"
            )

        self.latest_odom: Odometry | None = None
        self.metric_target: PoseStamped | None = None
        self.metric_target_confidence = 0.0
        self.metric_target_source = TargetEstimate.SOURCE_NONE
        self.metric_target_stable = False
        self.initial_search_goal: PoseStamped | None = None
        self.exploration_heading_yaw: float | None = None
        self.latest_object_reached = False
        self.latest_object_reached_time = None
        self.reached_latched = False
        self.reached_hold_goal: PoseStamped | None = None
        self._last_state = ""
        self._last_status_text = ""
        self._warned_frame_mismatch = False

        self.goal_pub = self.create_publisher(PoseStamped, self.output_goal_topic, 10)
        self.goal_viz_pub = self.create_publisher(MarkerArray, self.goal_viz_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
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
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            "目标搜索 goal mux 已启动, "
            f"output={self.output_goal_topic}, estimate={self.object_target_estimate_topic}, "
            f"reached={self.object_reached_topic}, "
            f"viz={self.goal_viz_topic}, odom={self.odom_topic}, "
            f"initial_distance={self.initial_goal_distance:.1f}m, "
            f"publish_rate={self.publish_rate:.1f}Hz"
        )

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _param_float(self, name: str) -> float:
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"参数 {name} 必须是数字, 当前值={value}") from exc

    def _param_bool(self, name: str) -> bool:
        value: Any = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        normalized = str(value).strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False
        raise ValueError(f"参数 {name} 必须是布尔值, 当前值={value}")

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.exploration_heading_yaw is None:
            self.exploration_heading_yaw = (
                _yaw_from_quaternion(msg.pose.pose.orientation)
                + math.radians(self.initial_goal_heading_deg)
            )

    def _on_target_estimate(self, msg: TargetEstimate) -> None:
        """两视角粗定位先引导导航, 稳定估计随后提升精度"""
        if self._reached_latch_is_active():
            return
        coarse_ready = (
            int(msg.accepted_views) >= self.coarse_target_min_views
            and float(msg.confidence) >= self.coarse_target_min_confidence
            and msg.state in {"TRACKING", "STABLE_VISION", "LIDAR_LOCKED"}
        )
        if not msg.stable and not coarse_ready:
            return
        if self.metric_target_stable and not msg.stable:
            return
        target = PoseStamped()
        target.header = copy.deepcopy(msg.header)
        target.pose = copy.deepcopy(msg.pose.pose)
        if not self._pose_is_finite(target):
            self.get_logger().warn("收到无效 metric target estimate, 已忽略")
            return

        source_improved = int(msg.source) > int(self.metric_target_source)
        quality_improved = bool(msg.stable) and not self.metric_target_stable
        confidence_improved = (
            self.metric_target is None
            or float(msg.confidence) >= self.metric_target_confidence + 0.1
        )
        moved_enough = (
            self.metric_target is None
            or self._pose_xy_distance(self.metric_target, target)
            >= self.metric_target_update_distance
        )
        if self.metric_target is not None and not (
            quality_improved or source_improved or confidence_improved or moved_enough
        ):
            return

        first_metric_target = self.metric_target is None
        self.metric_target = target
        self.metric_target_confidence = float(msg.confidence)
        self.metric_target_source = int(msg.source)
        self.metric_target_stable = bool(msg.stable)
        if first_metric_target and not msg.stable:
            self.get_logger().info(
                "远距离视觉粗目标已锁定, "
                f"confidence={self.metric_target_confidence:.2f}, "
                f"views={int(msg.accepted_views)}"
            )
        elif first_metric_target or quality_improved or source_improved:
            self.get_logger().info(
                "稳定融合目标已锁定, "
                f"source={_estimate_source_name(msg.source)}, "
                f"confidence={self.metric_target_confidence:.2f}"
            )

    def _on_object_reached(self, msg: Bool) -> None:
        self.latest_object_reached = bool(msg.data)
        self.latest_object_reached_time = self.get_clock().now()
        if self.latest_object_reached and self._object_reached_is_active(
            self.latest_object_reached_time
        ):
            self._activate_reached_latch(self.latest_object_reached_time)

    def _on_timer(self) -> None:
        state, goal = self._select_goal()
        self.completion_pub.publish(Bool(data=self.reached_latched))
        self._publish_status(state, goal)
        if goal is None:
            self.goal_viz_pub.publish(_delete_goal_markers())
            return
        self.goal_pub.publish(goal)
        self.goal_viz_pub.publish(_goal_markers(state, goal))

    def _select_goal(self) -> tuple[str, PoseStamped | None]:
        """两视角粗目标出现前持续使用固定初始探索目标"""
        now = self.get_clock().now()
        if self._reached_latch_is_active():
            if self.reached_hold_goal is None and self.latest_odom is None:
                return ObjectSearchState.WAIT_FOR_ODOM, None
            return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)

        if self.latest_odom is not None and self._object_reached_is_active(now):
            self._activate_reached_latch(now)
            return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)

        if self.metric_target is not None:
            state = (
                ObjectSearchState.TARGET_APPROACH_METRIC
                if self.metric_target_stable
                else ObjectSearchState.TARGET_APPROACH_COARSE
            )
            return (
                state,
                self._retime_pose(self.metric_target, now),
            )

        if self.latest_odom is None:
            return ObjectSearchState.WAIT_FOR_ODOM, None

        return ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL, self._build_initial_goal(now)

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
                f"初始搜索 goal frame={goal_frame}, odom frame={odom_frame}, "
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

    def _clear_target_search_state(self) -> None:
        self.metric_target = None
        self.metric_target_confidence = 0.0
        self.metric_target_source = TargetEstimate.SOURCE_NONE
        self.metric_target_stable = False

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
        if not self.latest_object_reached:
            return False
        if (
            self._age_seconds(now, self.latest_object_reached_time)
            > self.object_reached_timeout_sec
        ):
            return False
        if not self.object_reached_require_target_distance:
            return True
        target_pose = self._active_reached_target_pose()
        if target_pose is None:
            return False
        return self._target_distance(target_pose) <= self.object_reached_max_target_distance

    def _active_reached_target_pose(self) -> PoseStamped | None:
        """到达距离门控只使用稳定融合目标"""
        return self.metric_target if self.metric_target_stable else None

    def _activate_reached_latch(self, now) -> None:
        """到达确认后锁定停止 goal, 不再被后续 mask False 拉回搜索"""
        self.reached_latched = True
        self.reached_hold_goal = None
        if self.latest_odom is not None:
            self.reached_hold_goal = self._build_hold_goal(now)
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
        status = String()
        status.data = self._status_text(state, goal)
        self.status_pub.publish(status)
        if state == self._last_state:
            self._last_status_text = status.data
            return
        self._last_state = state
        self._last_status_text = status.data
        if goal is None:
            self.get_logger().info(f"目标搜索状态={state}")
            return
        self.get_logger().info(
            f"目标搜索状态={state}, goal_frame={goal.header.frame_id}, "
            f"goal=({goal.pose.position.x:.2f}, "
            f"{goal.pose.position.y:.2f}, "
            f"{goal.pose.position.z:.2f})"
        )

    def _status_text(self, state: str, goal: PoseStamped | None) -> str:
        if goal is None:
            return f"state={state}"
        parts = [
            f"state={state}",
            f"goal_frame={goal.header.frame_id}",
            f"goal=({goal.pose.position.x:.2f},"
            f"{goal.pose.position.y:.2f},"
            f"{goal.pose.position.z:.2f})",
        ]
        return ", ".join(parts)


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _delete_goal_markers() -> MarkerArray:
    markers = MarkerArray()
    for marker_id in (0, 1):
        marker = Marker()
        marker.ns = "object_search_goal"
        marker.id = marker_id
        marker.action = Marker.DELETE
        markers.markers.append(marker)
    return markers


def _goal_markers(state: str, goal: PoseStamped) -> MarkerArray:
    """生成 RViz 目标搜索 marker, 避免把 PoseStamped 和 graph marker 混在一起"""
    color = _goal_color(state)
    markers = MarkerArray()
    sphere = Marker()
    sphere.header = goal.header
    sphere.ns = "object_search_goal"
    sphere.id = 0
    sphere.type = Marker.SPHERE
    sphere.action = Marker.ADD
    sphere.pose = copy.deepcopy(goal.pose)
    sphere.pose.position.z += 0.35
    sphere.scale.x = 0.65
    sphere.scale.y = 0.65
    sphere.scale.z = 0.65
    sphere.color.r, sphere.color.g, sphere.color.b = color
    sphere.color.a = 0.95
    markers.markers.append(sphere)

    arrow = Marker()
    arrow.header = goal.header
    arrow.ns = "object_search_goal"
    arrow.id = 1
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose = copy.deepcopy(goal.pose)
    arrow.pose.position.z += 0.65
    arrow.scale.x = 1.4
    arrow.scale.y = 0.18
    arrow.scale.z = 0.18
    arrow.color.r, arrow.color.g, arrow.color.b = color
    arrow.color.a = 0.95
    markers.markers.append(arrow)
    return markers


def _goal_color(state: str) -> tuple[float, float, float]:
    if state == ObjectSearchState.TARGET_REACHED_VIEWPOINT:
        return 0.0, 0.85, 0.2
    if state == ObjectSearchState.TARGET_APPROACH_METRIC:
        return 0.75, 0.1, 1.0
    if state == ObjectSearchState.TARGET_APPROACH_COARSE:
        return 0.0, 0.8, 1.0
    return 1.0, 0.78, 0.0


def _estimate_source_name(source: int) -> str:
    return {
        TargetEstimate.SOURCE_VISION: "vision_particle_filter",
        TargetEstimate.SOURCE_FUSED: "vision_lidar_fusion",
        TargetEstimate.SOURCE_LIDAR: "lidar_locked",
    }.get(int(source), "unknown")


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
