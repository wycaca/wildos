import copy
import math
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class ObjectSearchGoalMux(Node):
    """目标搜索 goal 管理, 在初始搜索 goal 和目标 frontier 之间切换"""

    def __init__(self):
        super().__init__("object_search_goal_mux")

        self.declare_parameter("output_goal_topic", "/goal_pose")
        self.declare_parameter("goal_viz_topic", "/spot1/object_search_goal_viz")
        self.declare_parameter("object_target_pose_topic", "/spot1/object_search_target_pose")
        self.declare_parameter("object_reached_topic", "/spot1/object_search_reached")
        self.declare_parameter("odom_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("initial_goal_mode", "heading")
        self.declare_parameter("initial_goal_distance", 30.0)
        self.declare_parameter("initial_goal_heading_deg", 0.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("target_timeout_sec", 3.0)
        self.declare_parameter("latch_target_after_first_detection", False)
        self.declare_parameter("latch_target_timeout_sec", 3.0)
        self.declare_parameter("memory_timeout_sec", 10.0)
        self.declare_parameter("memory_goal_distance", 10.0)
        self.declare_parameter("target_reached_radius", 1.5)
        self.declare_parameter("object_reached_timeout_sec", 2.0)
        self.declare_parameter("require_subscriber", True)

        self.output_goal_topic = self._param_str("output_goal_topic")
        self.goal_viz_topic = self._param_str("goal_viz_topic")
        self.object_target_pose_topic = self._param_str("object_target_pose_topic")
        self.object_reached_topic = self._param_str("object_reached_topic")
        self.odom_topic = self._param_str("odom_topic")
        self.frame_id = self._param_str("frame_id")
        self.initial_goal_mode = self._param_str("initial_goal_mode")
        self.initial_goal_distance = self._param_float("initial_goal_distance")
        self.initial_goal_heading_deg = self._param_float("initial_goal_heading_deg")
        self.publish_rate = max(self._param_float("publish_rate"), 0.1)
        self.target_timeout_sec = max(self._param_float("target_timeout_sec"), 0.0)
        self.latch_target_after_first_detection = self._param_bool("latch_target_after_first_detection")
        self.latch_target_timeout_sec = max(self._param_float("latch_target_timeout_sec"), 0.0)
        self.memory_timeout_sec = max(self._param_float("memory_timeout_sec"), 0.0)
        self.memory_goal_distance = max(self._param_float("memory_goal_distance"), 0.1)
        self.target_reached_radius = max(self._param_float("target_reached_radius"), 0.1)
        self.object_reached_timeout_sec = max(self._param_float("object_reached_timeout_sec"), 0.0)
        self.require_subscriber = self._param_bool("require_subscriber")

        if self.output_goal_topic == self.object_target_pose_topic:
            raise ValueError("output_goal_topic 和 object_target_pose_topic 不能相同, 否则会形成 topic 回环")
        if self.initial_goal_mode != "heading":
            raise ValueError(f"暂不支持 initial_goal_mode={self.initial_goal_mode}, 当前只支持 heading")

        self.latest_odom: Odometry | None = None
        self.latest_target: PoseStamped | None = None
        self.latest_target_time = None
        self.latched_target: PoseStamped | None = None
        self.latched_target_time = None
        self.memory_target: PoseStamped | None = None
        self.memory_direction: tuple[float, float] | None = None
        self.memory_target_frame = ""
        self.memory_target_time = None
        self.latest_object_reached = False
        self.latest_object_reached_time = None
        self._last_state = ""
        self._warned_frame_mismatch = False

        self.goal_pub = self.create_publisher(PoseStamped, self.output_goal_topic, 10)
        self.goal_viz_pub = self.create_publisher(MarkerArray, self.goal_viz_topic, 10)
        self.target_sub = self.create_subscription(
            PoseStamped,
            self.object_target_pose_topic,
            self._on_target_pose,
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
            f"output={self.output_goal_topic}, target={self.object_target_pose_topic}, "
            f"reached={self.object_reached_topic}, viz={self.goal_viz_topic}, odom={self.odom_topic}, "
            f"initial_distance={self.initial_goal_distance:.1f}m, publish_rate={self.publish_rate:.1f}Hz"
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

    def _on_target_pose(self, msg: PoseStamped) -> None:
        if not self._pose_is_finite(msg):
            self.get_logger().warn("收到无效 object target pose, 已忽略")
            return
        self.latest_target = msg
        self.latest_target_time = self.get_clock().now()
        self._update_target_memory(msg, self.latest_target_time)
        if self.latch_target_after_first_detection:
            self.latched_target = msg
            self.latched_target_time = self.latest_target_time

    def _on_object_reached(self, msg: Bool) -> None:
        self.latest_object_reached = bool(msg.data)
        self.latest_object_reached_time = self.get_clock().now()

    def _on_timer(self) -> None:
        state, goal = self._select_goal()
        self._log_state_change(state, goal)
        if goal is None:
            self.goal_viz_pub.publish(_delete_goal_markers())
            return
        self.goal_pub.publish(goal)
        self.goal_viz_pub.publish(_goal_markers(state, goal))

    def _select_goal(self) -> tuple[str, PoseStamped | None]:
        """按目标优先级选择 goal, target frontier 高于初始搜索 goal"""
        if self.require_subscriber and self.goal_pub.get_subscription_count() == 0:
            return "WAIT_FOR_SUBSCRIBER", None

        now = self.get_clock().now()
        if self.latest_odom is not None and self._object_reached_is_active(now):
            return "TARGET_REACHED_VIEWPOINT", self._build_hold_goal(now)

        if self.latest_target is not None and self.latest_target_time is not None:
            target_age = (now - self.latest_target_time).nanoseconds * 1e-9
            if target_age <= self.target_timeout_sec:
                if self.latest_odom is not None and self._target_is_reached(self.latest_target):
                    return "TARGET_REACHED_VIEWPOINT", self._build_hold_goal(now)
                return "TARGET_FRONTIER_ACTIVE", self._retime_pose(self.latest_target, now)

        if self.latch_target_after_first_detection and self.latched_target is not None:
            latch_age = self._age_seconds(now, self.latched_target_time)
            if latch_age <= self.latch_target_timeout_sec:
                return "TARGET_FRONTIER_LATCHED", self._retime_pose(self.latched_target, now)
            self.latched_target = None
            self.latched_target_time = None

        if self.latest_odom is None:
            return "WAIT_FOR_ODOM", None

        if self.memory_direction is not None:
            memory_age = self._age_seconds(now, self.memory_target_time)
            if memory_age <= self.memory_timeout_sec:
                return "TARGET_MEMORY_GUIDED_SEARCH", self._build_memory_goal(now)
            self.memory_target = None
            self.memory_direction = None
            self.memory_target_time = None
            self.memory_target_frame = ""

        return "SEARCHING_WITH_INITIAL_GOAL", self._build_initial_goal(now)

    @staticmethod
    def _age_seconds(now, then) -> float:
        if then is None:
            return math.inf
        return (now - then).nanoseconds * 1e-9

    def _build_initial_goal(self, now) -> PoseStamped:
        """用当前 odom 朝向生成一个远处粗 goal, 驱动 planner 选择探索 frontier"""
        odom = self.latest_odom
        assert odom is not None

        odom_frame = odom.header.frame_id
        goal_frame = self.frame_id or odom_frame
        if odom_frame and goal_frame != odom_frame and not self._warned_frame_mismatch:
            self.get_logger().warn(
                f"初始搜索 goal frame={goal_frame}, odom frame={odom_frame}, 请确认二者在同一全局坐标系"
            )
            self._warned_frame_mismatch = True

        yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        yaw += math.radians(self.initial_goal_heading_deg)
        goal = PoseStamped()
        goal.header.frame_id = goal_frame
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = odom.pose.pose.position.x + self.initial_goal_distance * math.cos(yaw)
        goal.pose.position.y = odom.pose.pose.position.y + self.initial_goal_distance * math.sin(yaw)
        goal.pose.position.z = odom.pose.pose.position.z
        goal.pose.orientation = odom.pose.pose.orientation
        return goal

    def _build_memory_goal(self, now) -> PoseStamped:
        """复用最近一次目标 frontier 点, 让路径继续落在安全 graph 节点上"""
        odom = self.latest_odom
        assert odom is not None

        if self.memory_target is not None:
            goal = self._retime_pose(self.memory_target, now)
            dx = goal.pose.position.x - odom.pose.pose.position.x
            dy = goal.pose.position.y - odom.pose.pose.position.y
            yaw = math.atan2(dy, dx)
            goal.pose.orientation.z = math.sin(yaw * 0.5)
            goal.pose.orientation.w = math.cos(yaw * 0.5)
            return goal

        assert self.memory_direction is not None
        direction_x, direction_y = self.memory_direction
        yaw = math.atan2(direction_y, direction_x)
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id or odom.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position.x = odom.pose.pose.position.x + self.memory_goal_distance * direction_x
        goal.pose.position.y = odom.pose.pose.position.y + self.memory_goal_distance * direction_y
        goal.pose.position.z = odom.pose.pose.position.z
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)
        return goal

    def _build_hold_goal(self, now) -> PoseStamped:
        """到达目标观察点后发布当前位置, 让 planner 不再继续追远处点"""
        odom = self.latest_odom
        assert odom is not None

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id or odom.header.frame_id
        goal.header.stamp = now.to_msg()
        goal.pose.position = copy.deepcopy(odom.pose.pose.position)
        goal.pose.orientation = copy.deepcopy(odom.pose.pose.orientation)
        return goal

    def _update_target_memory(self, target: PoseStamped, now) -> None:
        if self.latest_odom is None:
            return
        dx = target.pose.position.x - self.latest_odom.pose.pose.position.x
        dy = target.pose.position.y - self.latest_odom.pose.pose.position.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return
        self.memory_target = copy.deepcopy(target)
        self.memory_direction = (dx / norm, dy / norm)
        self.memory_target_time = now
        self.memory_target_frame = target.header.frame_id

    def _target_is_reached(self, target: PoseStamped) -> bool:
        if self.latest_odom is None:
            return False
        dx = target.pose.position.x - self.latest_odom.pose.pose.position.x
        dy = target.pose.position.y - self.latest_odom.pose.pose.position.y
        return math.hypot(dx, dy) <= self.target_reached_radius

    def _object_reached_is_active(self, now) -> bool:
        if not self.latest_object_reached:
            return False
        return self._age_seconds(now, self.latest_object_reached_time) <= self.object_reached_timeout_sec

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

    def _log_state_change(self, state: str, goal: PoseStamped | None) -> None:
        if state == self._last_state:
            return
        self._last_state = state
        if goal is None:
            self.get_logger().info(f"目标搜索状态={state}")
            return
        self.get_logger().info(
            f"目标搜索状态={state}, goal_frame={goal.header.frame_id}, "
            f"goal=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}, {goal.pose.position.z:.2f})"
        )


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
    if state == "TARGET_REACHED_VIEWPOINT":
        return 0.0, 0.85, 0.2
    if state == "TARGET_MEMORY_GUIDED_SEARCH":
        return 0.0, 0.65, 1.0
    if state.startswith("TARGET_FRONTIER"):
        return 1.0, 0.18, 0.02
    return 1.0, 0.78, 0.0


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
