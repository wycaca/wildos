import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from object_search_msgs.msg import TargetEstimate

from visual_navigation.object_search_types import ObjectSearchState


class ObjectSearchGoalMux(Node):
    """在初始探索、视觉粗目标、稳定目标和完成状态之间选择唯一 goal"""

    def __init__(self):
        super().__init__("object_search_goal_mux")

        self.declare_parameter("output_goal_topic", "/spot1/graphnav_goal_pose")
        self.declare_parameter("status_topic", "/spot1/object_search_status")
        self.declare_parameter("object_target_estimate_topic", "/spot1/object_target_estimate")
        self.declare_parameter("object_reached_topic", "/spot1/object_search_reached")
        self.declare_parameter("completion_topic", "/spot1/object_search_completed")
        self.declare_parameter("odom_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("initial_goal_distance", 30.0)
        self.declare_parameter("initial_goal_heading_deg", 0.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("object_reached_timeout_sec", 2.0)
        self.declare_parameter("object_reached_max_target_distance", 2.0)
        self.declare_parameter("coarse_target_min_views", 2)
        self.declare_parameter("coarse_target_min_confidence", 0.5)
        self.declare_parameter("coarse_target_max_distance", 30.0)
        self.declare_parameter("target_max_vertical_offset", 1.5)
        self.declare_parameter("coarse_target_max_horizontal_std", 8.0)
        self.declare_parameter("target_update_min_distance", 0.75)
        self.declare_parameter("stable_target_update_min_distance", 0.3)

        self.output_goal_topic = self._param_str("output_goal_topic")
        self.status_topic = self._param_str("status_topic")
        self.object_target_estimate_topic = self._param_str("object_target_estimate_topic")
        self.object_reached_topic = self._param_str("object_reached_topic")
        self.completion_topic = self._param_str("completion_topic")
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
        self._last_reached_gate_reason = ""
        self._warned_frame_mismatch = False

        self.goal_pub = self.create_publisher(PoseStamped, self.output_goal_topic, 10)
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
            "目标搜索目标仲裁器已启动, "
            f"输出目标={self.output_goal_topic}, 融合估计={self.object_target_estimate_topic}, "
            f"视觉到达={self.object_reached_topic}, 里程计={self.odom_topic}, "
            f"初始探索距离={self.initial_goal_distance:.1f}m, "
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
            self.get_logger().warn("收到包含非有限数的融合目标, 已忽略")
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
        self.metric_target = target
        self.metric_target_confidence = float(msg.confidence)
        self.metric_target_source = int(msg.source)
        self.metric_target_stable = bool(msg.stable)
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
        self.latest_object_reached = bool(msg.data)
        self.latest_object_reached_time = self.get_clock().now()
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

    def _on_timer(self) -> None:
        state, goal = self._select_goal()
        self.completion_pub.publish(Bool(data=self.reached_latched))
        self._publish_status(state, goal)
        if goal is not None:
            self.goal_pub.publish(goal)

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
        reason_code, _ = self._object_reached_gate_reason(now)
        return reason_code is None

    def _object_reached_gate_reason(self, now) -> tuple[str | None, str]:
        """返回视觉到达证据尚不能触发最终完成的原因"""
        if not self.latest_object_reached:
            return "no_visual_evidence", "无当前视觉近距离证据"
        evidence_age = self._age_seconds(now, self.latest_object_reached_time)
        if evidence_age > self.object_reached_timeout_sec:
            return (
                "visual_evidence_expired",
                f"视觉证据已过期, age={evidence_age:.2f}s, "
                f"limit={self.object_reached_timeout_sec:.2f}s",
            )
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
        return None, "全部门控已通过"

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
        self.get_logger().info(
            f"目标完成门控已通过, 目标距离={target_distance:.2f}m, "
            f"视觉证据年龄={self._age_seconds(now, self.latest_object_reached_time):.2f}s"
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
        status = String()
        status.data = self._status_text(state, goal)
        self.status_pub.publish(status)
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
        ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL: (
            "按初始方向探索(SEARCHING_WITH_INITIAL_GOAL)"
        ),
        ObjectSearchState.TARGET_APPROACH_COARSE: (
            "接近视觉粗目标(TARGET_APPROACH_COARSE)"
        ),
        ObjectSearchState.TARGET_APPROACH_METRIC: (
            "接近稳定融合目标(TARGET_APPROACH_METRIC)"
        ),
        ObjectSearchState.TARGET_REACHED_VIEWPOINT: (
            "目标到达观察点(TARGET_REACHED_VIEWPOINT)"
        ),
    }.get(state, f"未知状态({state})")


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
