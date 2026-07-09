import copy
import math
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from visual_navigation.object_search_types import ObjectSearchState, TargetHypothesis
from visual_navigation.stable_frontier_selector import (
    StableFrontierSelector,
    StableFrontierSelectorConfig,
)


class ObjectSearchGoalMux(Node):
    """目标搜索 goal 管理, 在稳定 frontier, 初始 goal 和目标记忆之间切换"""

    def __init__(self):
        super().__init__("object_search_goal_mux")

        self.declare_parameter("output_goal_topic", "/spot1/graphnav_goal_pose")
        self.declare_parameter("goal_viz_topic", "/spot1/object_search_goal_viz")
        self.declare_parameter("selected_frontier_topic", "/spot1/object_search_selected_frontier")
        self.declare_parameter("status_topic", "/spot1/object_search_status")
        self.declare_parameter("nav_graph_topic", "/spot1/scored_nav_graph")
        self.declare_parameter("object_target_pose_topic", "/spot1/object_search_target_pose")
        self.declare_parameter("object_reached_topic", "/spot1/object_search_reached")
        self.declare_parameter("odom_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("traversability_class", "default")
        self.declare_parameter("initial_goal_mode", "heading")
        self.declare_parameter("initial_goal_distance", 30.0)
        self.declare_parameter("initial_goal_heading_deg", 0.0)
        self.declare_parameter("initial_goal_latch_timeout_sec", 20.0)
        self.declare_parameter("initial_goal_reached_radius", 3.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("target_timeout_sec", 3.0)
        self.declare_parameter("latch_target_after_first_detection", False)
        self.declare_parameter("latch_target_timeout_sec", 3.0)
        self.declare_parameter("memory_timeout_sec", 10.0)
        self.declare_parameter("memory_goal_distance", 10.0)
        self.declare_parameter("target_reached_radius", 1.5)
        self.declare_parameter("object_reached_timeout_sec", 2.0)
        self.declare_parameter("reached_latch_timeout_sec", 60.0)
        self.declare_parameter("object_reached_require_target_distance", False)
        self.declare_parameter("object_reached_max_target_distance", 2.0)
        self.declare_parameter("require_subscriber", True)
        self.declare_parameter("enable_graph_frontier_selection", True)
        self.declare_parameter("frontier_min_dwell_sec", 8.0)
        self.declare_parameter("frontier_switch_min_score_margin", 0.15)
        self.declare_parameter("frontier_progress_timeout_sec", 12.0)
        self.declare_parameter("frontier_progress_min_delta", 0.25)
        self.declare_parameter("frontier_reached_radius", 1.5)
        self.declare_parameter("frontier_same_position_radius", 1.2)
        self.declare_parameter("deadend_blacklist_timeout_sec", 20.0)
        self.declare_parameter("frontier_score_weight", 1.0)
        self.declare_parameter("frontier_distance_weight", 0.06)
        self.declare_parameter("frontier_switch_penalty", 0.2)
        self.declare_parameter("frontier_forward_weight", 0.8)
        self.declare_parameter("frontier_min_forward_dot", 0.0)
        self.declare_parameter("frontier_forward_fallback_to_any", True)

        self.output_goal_topic = self._param_str("output_goal_topic")
        self.goal_viz_topic = self._param_str("goal_viz_topic")
        self.selected_frontier_topic = self._param_str("selected_frontier_topic")
        self.status_topic = self._param_str("status_topic")
        self.nav_graph_topic = self._param_str("nav_graph_topic")
        self.object_target_pose_topic = self._param_str("object_target_pose_topic")
        self.object_reached_topic = self._param_str("object_reached_topic")
        self.odom_topic = self._param_str("odom_topic")
        self.frame_id = self._param_str("frame_id")
        self.traversability_class = self._param_str("traversability_class")
        self.initial_goal_mode = self._param_str("initial_goal_mode")
        self.initial_goal_distance = self._param_float("initial_goal_distance")
        self.initial_goal_heading_deg = self._param_float("initial_goal_heading_deg")
        self.initial_goal_latch_timeout_sec = max(
            self._param_float("initial_goal_latch_timeout_sec"),
            0.0,
        )
        self.initial_goal_reached_radius = max(
            self._param_float("initial_goal_reached_radius"),
            0.1,
        )
        self.publish_rate = max(self._param_float("publish_rate"), 0.1)
        self.target_timeout_sec = max(self._param_float("target_timeout_sec"), 0.0)
        self.latch_target_after_first_detection = self._param_bool(
            "latch_target_after_first_detection"
        )
        self.latch_target_timeout_sec = max(self._param_float("latch_target_timeout_sec"), 0.0)
        self.memory_timeout_sec = max(self._param_float("memory_timeout_sec"), 0.0)
        self.memory_goal_distance = max(self._param_float("memory_goal_distance"), 0.1)
        self.target_reached_radius = max(self._param_float("target_reached_radius"), 0.1)
        self.object_reached_timeout_sec = max(self._param_float("object_reached_timeout_sec"), 0.0)
        self.reached_latch_timeout_sec = max(self._param_float("reached_latch_timeout_sec"), 0.0)
        self.object_reached_require_target_distance = self._param_bool(
            "object_reached_require_target_distance"
        )
        self.object_reached_max_target_distance = max(
            self._param_float("object_reached_max_target_distance"),
            self.target_reached_radius,
        )
        self.require_subscriber = self._param_bool("require_subscriber")
        self.enable_graph_frontier_selection = self._param_bool("enable_graph_frontier_selection")
        self.frontier_min_dwell_sec = max(self._param_float("frontier_min_dwell_sec"), 0.0)
        self.frontier_switch_min_score_margin = max(
            self._param_float("frontier_switch_min_score_margin"),
            0.0,
        )
        self.frontier_progress_timeout_sec = max(
            self._param_float("frontier_progress_timeout_sec"),
            0.1,
        )
        self.frontier_progress_min_delta = max(
            self._param_float("frontier_progress_min_delta"),
            0.0,
        )
        self.frontier_reached_radius = max(self._param_float("frontier_reached_radius"), 0.1)
        self.frontier_same_position_radius = max(
            self._param_float("frontier_same_position_radius"),
            0.1,
        )
        self.deadend_blacklist_timeout_sec = max(
            self._param_float("deadend_blacklist_timeout_sec"),
            0.0,
        )
        self.frontier_score_weight = self._param_float("frontier_score_weight")
        self.frontier_distance_weight = max(self._param_float("frontier_distance_weight"), 0.0)
        self.frontier_switch_penalty = max(self._param_float("frontier_switch_penalty"), 0.0)
        self.frontier_forward_weight = self._param_float("frontier_forward_weight")
        self.frontier_min_forward_dot = max(
            min(self._param_float("frontier_min_forward_dot"), 1.0),
            -1.0,
        )
        self.frontier_forward_fallback_to_any = self._param_bool(
            "frontier_forward_fallback_to_any"
        )

        if self.output_goal_topic == self.object_target_pose_topic:
            raise ValueError(
                "output_goal_topic 和 object_target_pose_topic 不能相同, "
                "否则会形成 topic 回环"
            )
        if self.initial_goal_mode != "heading":
            raise ValueError(
                f"暂不支持 initial_goal_mode={self.initial_goal_mode}, "
                "当前只支持 heading"
            )

        self.latest_odom: Odometry | None = None
        self.latest_graph: NavigationGraph | None = None
        self.latest_graph_time = None
        self.latest_target: PoseStamped | None = None
        self.latest_target_time = None
        self.latest_target_hypothesis: TargetHypothesis | None = None
        self.latched_target: PoseStamped | None = None
        self.latched_target_time = None
        self.latched_target_hypothesis: TargetHypothesis | None = None
        self.memory_target: PoseStamped | None = None
        self.memory_target_hypothesis: TargetHypothesis | None = None
        self.memory_direction: tuple[float, float] | None = None
        self.memory_target_frame = ""
        self.memory_target_time = None
        self.initial_search_goal: PoseStamped | None = None
        self.initial_search_goal_time = None
        self.latest_object_reached = False
        self.latest_object_reached_time = None
        self.reached_latched = False
        self.reached_latched_time = None
        self.reached_hold_goal: PoseStamped | None = None
        self.frontier_selector = StableFrontierSelector(
            StableFrontierSelectorConfig(
                traversability_class=self.traversability_class,
                frame_id=self.frame_id,
                min_dwell_sec=self.frontier_min_dwell_sec,
                switch_min_score_margin=self.frontier_switch_min_score_margin,
                progress_timeout_sec=self.frontier_progress_timeout_sec,
                progress_min_delta=self.frontier_progress_min_delta,
                reached_radius=self.frontier_reached_radius,
                same_position_radius=self.frontier_same_position_radius,
                deadend_blacklist_timeout_sec=self.deadend_blacklist_timeout_sec,
                score_weight=self.frontier_score_weight,
                distance_weight=self.frontier_distance_weight,
                switch_penalty=self.frontier_switch_penalty,
                forward_weight=self.frontier_forward_weight,
                min_forward_dot=self.frontier_min_forward_dot,
                forward_fallback_to_any=self.frontier_forward_fallback_to_any,
            ),
            logger=self.get_logger(),
        )
        self._last_state = ""
        self._last_frontier_uuid = ""
        self._last_status_text = ""
        self._warned_frame_mismatch = False

        self.goal_pub = self.create_publisher(PoseStamped, self.output_goal_topic, 10)
        self.goal_viz_pub = self.create_publisher(MarkerArray, self.goal_viz_topic, 10)
        self.selected_frontier_pub = self.create_publisher(
            PoseStamped,
            self.selected_frontier_topic,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
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
        self.graph_sub = self.create_subscription(
            NavigationGraph,
            self.nav_graph_topic,
            self._on_nav_graph,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            "目标搜索 goal mux 已启动, "
            f"output={self.output_goal_topic}, target={self.object_target_pose_topic}, "
            f"graph={self.nav_graph_topic}, reached={self.object_reached_topic}, "
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

    @property
    def selected_frontier(self):
        return self.frontier_selector.selected

    @property
    def blacklisted_frontier_count(self) -> int:
        return self.frontier_selector.blacklisted_count

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_nav_graph(self, msg: NavigationGraph) -> None:
        self.latest_graph = msg
        self.latest_graph_time = self.get_clock().now()

    def _on_target_pose(self, msg: PoseStamped) -> None:
        if not self._pose_is_finite(msg):
            self.get_logger().warn("收到无效 object target pose, 已忽略")
            return
        now = self.get_clock().now()
        if self._reached_latch_is_active(now):
            return
        now_sec = _time_seconds(now)
        self.latest_target = msg
        self.latest_target_time = now
        self.latest_target_hypothesis = TargetHypothesis(
            pose=copy.deepcopy(msg),
            last_seen_sec=now_sec,
        )
        self._clear_initial_search_goal()
        self._update_target_memory(msg, self.latest_target_time)
        if self.latch_target_after_first_detection:
            self._update_latched_target(msg, now)

    def _on_object_reached(self, msg: Bool) -> None:
        self.latest_object_reached = bool(msg.data)
        self.latest_object_reached_time = self.get_clock().now()
        if self.latest_object_reached and self._object_reached_is_active(self.latest_object_reached_time):
            self._activate_reached_latch(self.latest_object_reached_time)

    def _on_timer(self) -> None:
        state, goal = self._select_goal()
        self._publish_status(state, goal)
        if goal is None:
            self.goal_viz_pub.publish(_delete_goal_markers())
            return
        self.goal_pub.publish(goal)
        self.goal_viz_pub.publish(_goal_markers(state, goal))
        if self.selected_frontier is not None:
            self.selected_frontier_pub.publish(
                self._retime_pose(
                    self.selected_frontier.pose,
                    self.get_clock().now(),
                )
            )

    def _select_goal(self) -> tuple[str, PoseStamped | None]:
        """按目标状态选择稳定 goal, 优先使用 graph frontier"""
        if self.require_subscriber and self.goal_pub.get_subscription_count() == 0:
            return ObjectSearchState.WAIT_FOR_SUBSCRIBER, None

        now = self.get_clock().now()
        if self._reached_latch_is_active(now):
            if self.reached_hold_goal is None and self.latest_odom is None:
                return ObjectSearchState.WAIT_FOR_ODOM, None
            return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)

        if self.latest_odom is not None and self._object_reached_is_active(now):
            self._activate_reached_latch(now)
            return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)

        has_fresh_target = self._fresh_target_is_active(now)
        target_pose = self._active_target_pose(now)
        if (
            target_pose is not None
            and self.latest_odom is not None
            and self._target_is_reached(target_pose)
        ):
            self._activate_reached_latch(now)
            return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)

        if target_pose is not None and (has_fresh_target or self._latched_target_is_active(now)):
            self._clear_initial_search_goal()
            self._clear_selected_frontier()
            return ObjectSearchState.TARGET_APPROACH, self._retime_pose(target_pose, now)

        if self.enable_graph_frontier_selection and self.latest_odom is not None:
            graph_goal = self._select_graph_frontier_goal(now, target_pose)
            if graph_goal is not None:
                self._clear_initial_search_goal()
                if has_fresh_target:
                    return ObjectSearchState.VISION_GUIDED_FRONTIER, graph_goal
                if self._memory_is_active(now):
                    return ObjectSearchState.TARGET_MEMORY_GUIDED_FRONTIER, graph_goal
                return ObjectSearchState.GEOMETRIC_EXPLORE, graph_goal

        if self.latest_target is not None and self.latest_target_time is not None:
            target_age = (now - self.latest_target_time).nanoseconds * 1e-9
            if target_age <= self.target_timeout_sec:
                if self.latest_odom is not None and self._target_is_reached(self.latest_target):
                    self._clear_initial_search_goal()
                    return ObjectSearchState.TARGET_REACHED_VIEWPOINT, self._build_hold_goal(now)
                return ObjectSearchState.TARGET_APPROACH, self._retime_pose(self.latest_target, now)

        if self.latch_target_after_first_detection and self.latched_target is not None:
            latch_age = self._age_seconds(now, self.latched_target_time)
            if latch_age <= self.latch_target_timeout_sec:
                return (
                    ObjectSearchState.TARGET_APPROACH,
                    self._retime_pose(self.latched_target, now),
                )
            self.latched_target = None
            self.latched_target_time = None
            self.latched_target_hypothesis = None

        if self.latest_odom is None:
            return ObjectSearchState.WAIT_FOR_ODOM, None

        if self.memory_direction is not None:
            memory_age = self._age_seconds(now, self.memory_target_time)
            if memory_age <= self.memory_timeout_sec:
                self._clear_initial_search_goal()
                return ObjectSearchState.TARGET_MEMORY_GUIDED_SEARCH, self._build_memory_goal(now)
            self.memory_target = None
            self.memory_target_hypothesis = None
            self.memory_direction = None
            self.memory_target_time = None
            self.memory_target_frame = ""

        return ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL, self._build_initial_goal(now)

    def _active_target_pose(self, now) -> PoseStamped | None:
        if self._latched_target_is_active(now):
            return self.latched_target

        if self.latest_target is not None and self.latest_target_time is not None:
            if self._age_seconds(now, self.latest_target_time) <= self.target_timeout_sec:
                return self.latest_target

        if self._memory_is_active(now):
            return self.memory_target
        return None

    def _update_latched_target(self, target: PoseStamped, now) -> None:
        """看到目标后短时间固定目标 frontier, 避免每帧换到反方向节点"""
        if not self._latched_target_is_active(now):
            self.latched_target = copy.deepcopy(target)
            self.latched_target_time = now
            self.latched_target_hypothesis = copy.deepcopy(self.latest_target_hypothesis)
            return
        assert self.latched_target is not None
        if self._pose_xy_distance(self.latched_target, target) <= self.target_reached_radius:
            self.latched_target = copy.deepcopy(target)
            self.latched_target_time = now
            self.latched_target_hypothesis = copy.deepcopy(self.latest_target_hypothesis)

    def _latched_target_is_active(self, now) -> bool:
        if not self.latch_target_after_first_detection or self.latched_target is None:
            return False
        latch_age = self._age_seconds(now, self.latched_target_time)
        if latch_age <= self.latch_target_timeout_sec:
            return True
        self.latched_target = None
        self.latched_target_time = None
        self.latched_target_hypothesis = None
        return False

    def _memory_is_active(self, now) -> bool:
        return (
            self.memory_target is not None
            and self._age_seconds(now, self.memory_target_time) <= self.memory_timeout_sec
        )

    def _fresh_target_is_active(self, now) -> bool:
        return (
            self.latest_target is not None
            and self._age_seconds(now, self.latest_target_time) <= self.target_timeout_sec
        )

    @staticmethod
    def _age_seconds(now, then) -> float:
        if then is None:
            return math.inf
        return (now - then).nanoseconds * 1e-9

    def _select_graph_frontier_goal(
        self,
        now,
        target_pose: PoseStamped | None,
    ) -> PoseStamped | None:
        """调用纯 selector 选择 graph frontier, mux 只负责状态仲裁"""
        graph = self.latest_graph
        if graph is None or not graph.nodes:
            self._clear_selected_frontier()
            return None

        selected = self.frontier_selector.select(
            graph=graph,
            reference_xy=self._reference_xy(graph),
            now_sec=_time_seconds(now),
            stamp=now.to_msg(),
            target_pose=target_pose,
            heading_yaw=self._frontier_search_heading_yaw(target_pose),
        )
        if selected is None:
            return None
        return self._retime_pose(selected.pose, now)

    def _frontier_search_heading_yaw(self, target_pose: PoseStamped | None) -> float | None:
        """目标未知时用 odom 朝向约束几何搜索方向"""
        if target_pose is not None or self.latest_odom is None:
            return None
        yaw = _yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        return yaw + math.radians(self.initial_goal_heading_deg)

    def _clear_selected_frontier(self) -> None:
        self.frontier_selector.clear()

    def _reference_xy(self, graph: NavigationGraph) -> tuple[float, float]:
        if graph.current_node_idx < len(graph.nodes):
            node = graph.nodes[graph.current_node_idx]
            return node.pose.position.x, node.pose.position.y
        if self.latest_odom is not None:
            return self.latest_odom.pose.pose.position.x, self.latest_odom.pose.pose.position.y
        return 0.0, 0.0

    def _build_initial_goal(self, now) -> PoseStamped:
        """用当前 odom 朝向生成一个远处粗 goal, 驱动 planner 选择探索 frontier"""
        odom = self.latest_odom
        assert odom is not None

        cached_goal = self._valid_initial_search_goal(now)
        if cached_goal is not None:
            return cached_goal

        odom_frame = odom.header.frame_id
        goal_frame = self.frame_id or odom_frame
        if odom_frame and goal_frame != odom_frame and not self._warned_frame_mismatch:
            self.get_logger().warn(
                f"初始搜索 goal frame={goal_frame}, odom frame={odom_frame}, "
                "请确认二者在同一全局坐标系"
            )
            self._warned_frame_mismatch = True

        yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        yaw += math.radians(self.initial_goal_heading_deg)
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
        goal.pose.orientation = odom.pose.pose.orientation
        self.initial_search_goal = copy.deepcopy(goal)
        self.initial_search_goal_time = now
        return goal

    def _valid_initial_search_goal(self, now) -> PoseStamped | None:
        """复用初始搜索 goal, 避免机器人转向时 goal 围着机器人旋转"""
        if self.initial_search_goal is None or self.latest_odom is None:
            return None
        age = self._age_seconds(now, self.initial_search_goal_time)
        if age > self.initial_goal_latch_timeout_sec:
            self._clear_initial_search_goal()
            return None
        dx = self.initial_search_goal.pose.position.x - self.latest_odom.pose.pose.position.x
        dy = self.initial_search_goal.pose.position.y - self.latest_odom.pose.pose.position.y
        if math.hypot(dx, dy) <= self.initial_goal_reached_radius:
            self._clear_initial_search_goal()
            return None
        return self._retime_pose(self.initial_search_goal, now)

    def _clear_initial_search_goal(self) -> None:
        self.initial_search_goal = None
        self.initial_search_goal_time = None

    def _clear_target_search_state(self) -> None:
        self.latest_target = None
        self.latest_target_time = None
        self.latest_target_hypothesis = None
        self.latched_target = None
        self.latched_target_time = None
        self.latched_target_hypothesis = None
        self.memory_target = None
        self.memory_target_hypothesis = None
        self.memory_direction = None
        self.memory_target_time = None
        self.memory_target_frame = ""

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

    def _update_target_memory(self, target: PoseStamped, now) -> None:
        if self.latest_odom is None:
            return
        dx = target.pose.position.x - self.latest_odom.pose.pose.position.x
        dy = target.pose.position.y - self.latest_odom.pose.pose.position.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return
        self.memory_target = copy.deepcopy(target)
        self.memory_target_hypothesis = TargetHypothesis(
            pose=copy.deepcopy(target),
            last_seen_sec=_time_seconds(now),
            source="object_memory_frontier",
        )
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
        if self._age_seconds(now, self.latest_object_reached_time) > self.object_reached_timeout_sec:
            return False
        if not self.object_reached_require_target_distance:
            return True
        target_pose = self._active_reached_target_pose(now)
        if target_pose is None:
            return False
        return self._target_distance(target_pose) <= self.object_reached_max_target_distance

    def _active_reached_target_pose(self, now) -> PoseStamped | None:
        """到达确认只接受当前目标或目标 latch, 不用 memory 目标触发停止"""
        if self._latched_target_is_active(now):
            return self.latched_target
        if self.latest_target is not None and self.latest_target_time is not None:
            if self._age_seconds(now, self.latest_target_time) <= self.target_timeout_sec:
                return self.latest_target
        return None

    def _activate_reached_latch(self, now) -> None:
        """到达确认后锁定停止 goal, 不再被后续 mask False 拉回搜索"""
        self.reached_latched = True
        self.reached_latched_time = now
        self.reached_hold_goal = None
        if self.latest_odom is not None:
            self.reached_hold_goal = self._build_hold_goal(now)
        self._clear_initial_search_goal()
        self._clear_selected_frontier()
        self._clear_target_search_state()

    def _reached_latch_is_active(self, now) -> bool:
        if not self.reached_latched:
            return False
        if self.reached_latch_timeout_sec <= 0.0:
            return True
        if self._age_seconds(now, self.reached_latched_time) <= self.reached_latch_timeout_sec:
            return True
        self.reached_latched = False
        self.reached_latched_time = None
        self.reached_hold_goal = None
        return False

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
        frontier_uuid = self.selected_frontier.uuid if self.selected_frontier else ""
        status = String()
        status.data = self._status_text(state, goal, frontier_uuid)
        self.status_pub.publish(status)
        if state == self._last_state and frontier_uuid == self._last_frontier_uuid:
            self._last_status_text = status.data
            return
        self._last_state = state
        self._last_frontier_uuid = frontier_uuid
        self._last_status_text = status.data
        if goal is None:
            self.get_logger().info(f"目标搜索状态={state}")
            return
        extra = ""
        if self.selected_frontier is not None:
            extra = (
                f", frontier={self.selected_frontier.uuid}, "
                f"score={self.selected_frontier.score:.2f}, "
                f"distance={self.selected_frontier.distance:.2f}, "
                f"blacklisted={self.blacklisted_frontier_count}"
            )
        self.get_logger().info(
            f"目标搜索状态={state}, goal_frame={goal.header.frame_id}, "
            f"goal=({goal.pose.position.x:.2f}, "
            f"{goal.pose.position.y:.2f}, "
            f"{goal.pose.position.z:.2f}){extra}"
        )

    def _status_text(self, state: str, goal: PoseStamped | None, frontier_uuid: str) -> str:
        if goal is None:
            return f"state={state}"
        parts = [
            f"state={state}",
            f"goal_frame={goal.header.frame_id}",
            f"goal=({goal.pose.position.x:.2f},"
            f"{goal.pose.position.y:.2f},"
            f"{goal.pose.position.z:.2f})",
        ]
        if frontier_uuid:
            parts.extend(
                [
                    f"frontier={frontier_uuid}",
                    f"frontier_score={self.selected_frontier.score:.2f}",
                    f"frontier_distance={self.selected_frontier.distance:.2f}",
                    f"blacklisted={self.blacklisted_frontier_count}",
                ]
            )
        return ", ".join(parts)


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _time_seconds(stamp) -> float:
    return stamp.nanoseconds * 1e-9


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
    if state in {
        ObjectSearchState.TARGET_MEMORY_GUIDED_FRONTIER,
        ObjectSearchState.TARGET_MEMORY_GUIDED_SEARCH,
    }:
        return 0.0, 0.65, 1.0
    if state in {
        ObjectSearchState.VISION_GUIDED_FRONTIER,
        ObjectSearchState.TARGET_APPROACH,
    }:
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
