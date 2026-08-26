import copy
import math

from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Bool, String
from graphnav_msgs.msg import (
    Edge,
    EdgeTraversability,
    NavigationGraph,
    Node,
    NodeTraversabilityProperties,
)
from object_search_msgs.msg import ObjectSearchStatus, TargetEstimate

from visual_navigation.object_search_goal_mux import (
    ObjectSearchGoalMux,
    _object_search_state_code,
)
from visual_navigation.object_search_types import ObjectSearchState


@pytest.fixture
def mux_node():
    """创建 goal mux 测试节点"""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "initial_goal_distance:=20.0",
            "-p",
            "frame_id:=odom",
            "-p",
            "startup_observation_enabled:=false",
        ]
    )
    node = ObjectSearchGoalMux()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _odom(x: float, y: float, yaw: float, z: float = 0.0) -> Odometry:
    """构造给定位置和偏航角的 odom"""
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
    return msg


def _yaw(pose) -> float:
    return 2.0 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)


def _target_estimate(
    x: float,
    y: float,
    *,
    stable: bool = True,
    confidence: float = 0.8,
    accepted_views: int = 2,
    state: str | None = None,
    z: float = 0.0,
    horizontal_std: float = 0.0,
    bearing_yaw: float | None = None,
) -> TargetEstimate:
    """构造视觉粗目标或稳定融合目标估计"""
    msg = TargetEstimate()
    msg.header.frame_id = "odom"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = horizontal_std * horizontal_std * 0.5
    msg.pose.covariance[7] = horizontal_std * horizontal_std * 0.5
    msg.confidence = confidence
    msg.source = TargetEstimate.SOURCE_VISION
    msg.stable = stable
    msg.accepted_views = accepted_views
    msg.state = state or ("STABLE_VISION" if stable else "TRACKING")
    if bearing_yaw is not None:
        msg.bearing.x = math.cos(bearing_yaw)
        msg.bearing.y = math.sin(bearing_yaw)
        msg.bearing_valid = True
    return msg


def _graph(*, forward: bool, stamp: int = 1) -> NavigationGraph:
    """构造有前向候选或仅含机器人节点的评分图"""
    positions = [(0.0, 0.0)]
    if forward:
        positions.extend([(1.0, 0.0), (2.0, 0.3), (3.0, -0.3)])
    graph = _safe_graph(
        positions,
        [2.0] * len(positions),
        [(index - 1, index) for index in range(1, len(positions))],
        stamp=stamp,
    )
    for index in range(len(positions)):
        graph.nodes[index].trav_properties[0].is_frontier = (
            forward and index == len(positions) - 1
        )
    return graph


def _safe_graph(
    positions: list[tuple[float, float]],
    free_radii: list[float],
    edges: list[tuple[int, int]],
    *,
    current_node_idx: int = 0,
    stamp: int = 1,
) -> NavigationGraph:
    """构造带安全覆盖半径和无向边的原始导航图"""
    graph = NavigationGraph()
    graph.header.frame_id = "odom"
    graph.header.stamp.sec = stamp
    graph.trav_classes = ["default"]
    for index, ((x, y), free_radius) in enumerate(
        zip(positions, free_radii)
    ):
        node = Node()
        node.uuid.id = [0] * 15 + [index + 1]
        node.pose.position.x = x
        node.pose.position.y = y
        properties = NodeTraversabilityProperties()
        properties.free_radius = free_radius
        node.trav_properties = [properties]
        graph.nodes.append(node)
    for source, target in edges:
        edge = Edge()
        edge.from_idx = source
        edge.to_idx = target
        traversability = EdgeTraversability()
        traversability.traversability_cost = math.hypot(
            positions[source][0] - positions[target][0],
            positions[source][1] - positions[target][1],
        )
        edge.traversability = [traversability]
        graph.edges.append(edge)
    graph.current_node_idx = current_node_idx
    return graph


def _enable_test_startup(node: ObjectSearchGoalMux) -> None:
    """启用零等待启动观察参数便于单元测试"""
    node.startup_observation_enabled = True
    node.startup_completed = False
    node.startup_warmup_sec = 0.0
    node.startup_min_nav_graph_frames = 2
    node.startup_min_scored_graph_frames = 2
    node.startup_scan_trigger_frames = 3
    node.startup_scan_hold_sec = 0.0


def test_all_goal_mux_states_have_unique_typed_codes():
    states = [
        value
        for name, value in vars(ObjectSearchState).items()
        if name.isupper()
    ]
    codes = [_object_search_state_code(state) for state in states]

    assert len(codes) == len(set(codes))
    assert ObjectSearchStatus.UNKNOWN not in codes
    assert _object_search_state_code("INVALID") == ObjectSearchStatus.UNKNOWN


def test_initial_coarse_goal_is_computed_once(mux_node):
    """机器人后续移动和转向不能让粗目标围绕当前位置重算"""
    mux_node._on_odom(_odom(1.0, 2.0, 0.0))
    first_state, first_goal = mux_node._select_goal()

    mux_node._on_odom(_odom(8.0, 9.0, math.pi))
    second_state, second_goal = mux_node._select_goal()

    assert first_state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert second_state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert first_goal.pose.position.x == pytest.approx(21.0)
    assert first_goal.pose.position.y == pytest.approx(2.0)
    assert second_goal.pose.position.x == pytest.approx(first_goal.pose.position.x)
    assert second_goal.pose.position.y == pytest.approx(first_goal.pose.position.y)


def test_target_change_clears_old_goal_and_completion_state(mux_node):
    """运行时切换目标后必须从当前位置重新开始搜索"""
    mux_node._on_odom(_odom(1.0, 2.0, 0.0))
    mux_node._on_target_estimate(_target_estimate(4.0, 2.0))
    mux_node.reached_latched = True
    mux_node.reached_hold_goal = copy.deepcopy(mux_node.metric_target)
    mux_node.initial_search_goal = copy.deepcopy(mux_node.metric_target)
    mux_node.exploration_heading_yaw = 1.0

    mux_node._on_object_search_target(String(data="new target"))

    assert mux_node.current_target == "new target"
    assert mux_node.metric_target is None
    assert not mux_node.reached_latched
    assert mux_node.reached_hold_goal is None
    assert mux_node.initial_search_goal is None
    assert mux_node.exploration_heading_yaw is None
    state, goal = mux_node._select_goal()
    assert state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert goal is not None


def test_startup_observation_holds_until_graph_and_scoring_are_ready(mux_node):
    """原始图和评分图未连续就绪时禁止提交探索目标"""
    _enable_test_startup(mux_node)
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))

    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.STARTUP_OBSERVATION
    assert goal.pose.position.x == pytest.approx(0.0)
    assert goal.pose.position.y == pytest.approx(0.0)


def test_startup_observation_starts_directly_when_forward_graph_is_ready(mux_node):
    """前向区域稳定可规划时预热后直接开始探索"""
    _enable_test_startup(mux_node)
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    for stamp in (1, 2):
        mux_node._on_nav_graph(_graph(forward=True, stamp=stamp))
        mux_node._on_scored_nav_graph(_graph(forward=True, stamp=stamp))

    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert goal.pose.position.x == pytest.approx(20.0)
    assert mux_node.startup_scan_phase == "COMPLETE"


def test_startup_scan_requires_repeated_insufficient_forward_graph(mux_node):
    """单帧前向不足不能触发旋转, 连续不足才进入左侧观察"""
    _enable_test_startup(mux_node)
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    for stamp in (1, 2):
        mux_node._on_nav_graph(_graph(forward=False, stamp=stamp))
        mux_node._on_scored_nav_graph(_graph(forward=False, stamp=stamp))

    first_state, first_goal = mux_node._select_goal()
    mux_node._on_scored_nav_graph(_graph(forward=False, stamp=3))
    scan_state, scan_goal = mux_node._select_goal()

    assert first_state == ObjectSearchState.STARTUP_OBSERVATION
    assert _yaw(first_goal) == pytest.approx(0.0)
    assert scan_state == ObjectSearchState.STARTUP_OBSERVATION
    assert _yaw(scan_goal) == pytest.approx(math.radians(35.0))
    assert mux_node.startup_scan_phase == "SCAN_LEFT"

    mux_node._on_odom(_odom(0.0, 0.0, math.radians(35.0)))
    mux_node._on_scored_nav_graph(_graph(forward=False, stamp=4))
    _, right_goal = mux_node._select_goal()
    assert _yaw(right_goal) == pytest.approx(math.radians(-35.0))

    mux_node._on_odom(_odom(0.0, 0.0, math.radians(-35.0)))
    mux_node._on_scored_nav_graph(_graph(forward=False, stamp=5))
    _, return_goal = mux_node._select_goal()
    assert _yaw(return_goal) == pytest.approx(0.0)

    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_scored_nav_graph(_graph(forward=False, stamp=6))
    complete_state, _ = mux_node._select_goal()
    assert complete_state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL


def test_startup_scan_keeps_anchor_and_times_out_without_rotation(mux_node):
    """底层未执行纯转向时也要结束启动观察并保持固定观察位置"""
    _enable_test_startup(mux_node)
    mux_node.startup_scan_phase_timeout_sec = 0.1
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    for stamp in (1, 2):
        mux_node._on_nav_graph(_graph(forward=False, stamp=stamp))
        mux_node._on_scored_nav_graph(_graph(forward=False, stamp=stamp))
    mux_node._on_scored_nav_graph(_graph(forward=False, stamp=3))

    _, left_goal = mux_node._select_goal()
    mux_node._on_odom(_odom(0.02, -0.01, 0.0))
    _, repeated_goal = mux_node._select_goal()

    assert mux_node.startup_scan_phase == "SCAN_LEFT"
    assert repeated_goal.pose.position.x == pytest.approx(
        left_goal.pose.position.x
    )
    assert repeated_goal.pose.position.y == pytest.approx(
        left_goal.pose.position.y
    )

    for expected_phase in ("SCAN_RIGHT", "SCAN_RETURN"):
        mux_node.startup_scan_phase_time = (
            mux_node.get_clock().now() - Duration(seconds=1.0)
        )
        state, _ = mux_node._select_goal()
        assert state == ObjectSearchState.STARTUP_OBSERVATION
        assert mux_node.startup_scan_phase == expected_phase

    mux_node.startup_scan_phase_time = (
        mux_node.get_clock().now() - Duration(seconds=1.0)
    )
    state, _ = mux_node._select_goal()

    assert state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert mux_node.startup_scan_phase == "COMPLETE"


def test_initial_goal_orientation_matches_configured_heading(mux_node):
    """粗目标姿态必须携带与目标位置一致的固定探索方向"""
    mux_node.initial_goal_heading_deg = 90.0
    mux_node._on_odom(_odom(1.0, 2.0, 0.0))

    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert goal.pose.position.x == pytest.approx(1.0)
    assert goal.pose.position.y == pytest.approx(22.0)
    assert goal.pose.orientation.z == pytest.approx(math.sin(math.pi * 0.25))
    assert goal.pose.orientation.w == pytest.approx(math.cos(math.pi * 0.25))


def test_reached_event_permanently_holds_current_pose(mux_node):
    """首次 reached 后 False 消息不能解除当前位置停止 goal"""
    mux_node._on_odom(_odom(4.0, -2.0, 0.5))
    mux_node._on_target_estimate(_target_estimate(5.0, -2.0))

    mux_node._on_object_reached(Bool(data=True))
    reached_state, reached_goal = mux_node._select_goal()

    mux_node._on_object_reached(Bool(data=False))
    later_state, later_goal = mux_node._select_goal()

    assert reached_state == ObjectSearchState.TARGET_REACHED_VIEWPOINT
    assert later_state == ObjectSearchState.TARGET_REACHED_VIEWPOINT
    assert reached_goal.pose.position.x == pytest.approx(4.0)
    assert reached_goal.pose.position.y == pytest.approx(-2.0)
    assert later_goal.pose.position.x == pytest.approx(4.0)
    assert later_goal.pose.position.y == pytest.approx(-2.0)


def test_reached_requires_metric_target_within_distance(mux_node):
    """视觉 reached 不能在融合目标仍较远时提前停止导航"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(_target_estimate(5.0, 0.0))

    mux_node._on_object_reached(Bool(data=True))
    far_state, _ = mux_node._select_goal()

    mux_node._on_target_estimate(_target_estimate(1.5, 0.0))
    mux_node._on_object_reached(Bool(data=True))
    near_state, _ = mux_node._select_goal()

    assert far_state == ObjectSearchState.TARGET_APPROACH_METRIC
    assert near_state == ObjectSearchState.TARGET_REACHED_VIEWPOINT


def test_reached_gate_reports_stable_target_and_distance_reasons(mux_node):
    """完成门控诊断需要区分稳定目标缺失和目标距离过远"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node.latest_object_reached = True
    mux_node.latest_object_reached_time = mux_node.get_clock().now()

    reason_code, _ = mux_node._object_reached_gate_reason(
        mux_node.latest_object_reached_time
    )
    assert reason_code == "no_stable_target"

    mux_node._on_target_estimate(_target_estimate(5.0, 0.0))
    reason_code, reason_text = mux_node._object_reached_gate_reason(
        mux_node.latest_object_reached_time
    )
    assert reason_code == "target_too_far"
    assert "5.00m" in reason_text


def test_single_view_pending_estimate_holds_position_and_uses_bearing(mux_node):
    """单视角候选只能短暂停留并使用方向, 不能导航到估计坐标"""
    mux_node._on_odom(_odom(2.0, 1.0, 0.0))

    estimate = _target_estimate(
        12.0,
        6.0,
        stable=False,
        accepted_views=1,
        state="PENDING",
        bearing_yaw=math.atan2(5.0, 10.0),
    )
    mux_node._on_target_estimate(estimate)
    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_PENDING_OBSERVATION
    assert goal.pose.position.x == pytest.approx(2.0)
    assert goal.pose.position.y == pytest.approx(1.0)
    assert _yaw(goal) == pytest.approx(math.atan2(5.0, 10.0))
    assert mux_node._pending_evidence_protection_active(
        mux_node.get_clock().now()
    )

    mux_node.pending_evidence_protection_sec = -1.0
    assert not mux_node._pending_evidence_protection_active(
        mux_node.get_clock().now()
    )


def test_stale_pending_observation_resumes_original_exploration_goal(mux_node):
    """单视角候选不再可见后恢复原探索方向"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    initial_state, initial_goal = mux_node._select_goal()
    mux_node._on_target_estimate(
        _target_estimate(
            12.0,
            3.0,
            stable=False,
            accepted_views=1,
            state="PENDING",
            bearing_yaw=math.atan2(3.0, 12.0),
        )
    )
    pending_state, _ = mux_node._select_goal()
    mux_node.pending_evidence_time = None

    resumed_state, resumed_goal = mux_node._select_goal()

    assert initial_state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert pending_state == ObjectSearchState.TARGET_PENDING_OBSERVATION
    assert resumed_state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert resumed_goal.pose.position.x == pytest.approx(
        initial_goal.pose.position.x
    )
    assert resumed_goal.pose.position.y == pytest.approx(
        initial_goal.pose.position.y
    )


def test_visible_pending_target_repositions_to_safe_new_view(mux_node):
    """单视角静止观察后目标仍可见时选择安全横向位置"""
    mux_node.pending_observation_duration_sec = 100.0
    mux_node.pending_reposition_visibility_timeout_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_nav_graph(_graph(forward=False))
    mux_node._on_target_estimate(
        _target_estimate(
            12.0,
            0.0,
            stable=False,
            accepted_views=1,
            state="PENDING",
            bearing_yaw=0.0,
        )
    )
    first_state, _ = mux_node._select_goal()
    mux_node.pending_observation_duration_sec = 0.0

    reposition_state, reposition_goal = mux_node._select_goal()

    assert first_state == ObjectSearchState.TARGET_PENDING_OBSERVATION
    assert reposition_state == ObjectSearchState.TARGET_PENDING_REPOSITION
    assert reposition_goal.pose.position.x == pytest.approx(0.0)
    assert reposition_goal.pose.position.y == pytest.approx(0.6)
    assert _yaw(reposition_goal) == pytest.approx(0.0)
    assert mux_node.pending_reposition_attempts == 1

    mux_node.pending_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.6, 0.0))
    observed_state, observed_goal = mux_node._select_goal()

    assert observed_state == ObjectSearchState.TARGET_PENDING_OBSERVATION
    assert observed_goal.pose.position.x == pytest.approx(0.0)
    assert observed_goal.pose.position.y == pytest.approx(0.6)


def test_two_view_tracking_estimate_replaces_initial_goal(mux_node):
    """两视角粗定位形成后先导航到目标外侧安全观察点"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    estimate = _target_estimate(
        12.0,
        3.0,
        stable=False,
        confidence=0.46,
        accepted_views=2,
        state="TRACKING",
    )

    mux_node._on_target_estimate(estimate)
    state, coarse_goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_APPROACH_COARSE
    target_distance = math.hypot(
        coarse_goal.pose.position.x - 12.0,
        coarse_goal.pose.position.y - 3.0,
    )
    assert target_distance == pytest.approx(2.75)
    assert _yaw(coarse_goal) == pytest.approx(math.atan2(3.0, 12.0))


def test_tracking_below_coarse_confidence_does_not_take_over(mux_node):
    """两视角置信度低于 0.45 时继续原探索"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(
        _target_estimate(
            12.0,
            3.0,
            stable=False,
            confidence=0.44,
            accepted_views=2,
            state="TRACKING",
        )
    )

    state, _ = mux_node._select_goal()

    assert state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL


def test_stale_coarse_target_restores_initial_exploration_goal(mux_node):
    """粗目标失效后恢复被抢占前的初始探索方向"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    _, initial_goal = mux_node._select_goal()
    mux_node._on_target_estimate(
        _target_estimate(
            12.0,
            3.0,
            stable=False,
            confidence=0.46,
            accepted_views=2,
            state="TRACKING",
        )
    )
    takeover_state, _ = mux_node._select_goal()
    mux_node.latest_target_estimate_time = None

    resumed_state, resumed_goal = mux_node._select_goal()

    assert takeover_state == ObjectSearchState.TARGET_APPROACH_COARSE
    assert resumed_state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert resumed_goal.pose.position.x == pytest.approx(
        initial_goal.pose.position.x
    )
    assert resumed_goal.pose.position.y == pytest.approx(
        initial_goal.pose.position.y
    )


def test_near_coarse_target_enters_facing_observation(mux_node):
    """到达粗目标安全距离后保持位置并面向目标"""
    mux_node.target_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(1.0, 1.0, math.pi))
    mux_node._on_target_estimate(
        _target_estimate(3.8, 1.0, stable=False, confidence=0.51)
    )

    state, observation_goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_OBSERVATION
    assert observation_goal.pose.position.x == pytest.approx(1.0)
    assert observation_goal.pose.position.y == pytest.approx(1.0)
    assert _yaw(observation_goal) == pytest.approx(0.0)


def test_unstable_observation_moves_sideways_for_new_view(mux_node):
    """静止观察超时后横向移动而不是完整原地旋转"""
    mux_node.target_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_nav_graph(_graph(forward=False))
    mux_node._on_target_estimate(
        _target_estimate(2.8, 0.0, stable=False, confidence=0.51)
    )
    first_state, _ = mux_node._select_goal()
    mux_node.target_observation_duration_sec = 0.0

    next_state, reposition_goal = mux_node._select_goal()

    assert first_state == ObjectSearchState.TARGET_OBSERVATION
    assert next_state == ObjectSearchState.TARGET_APPROACH_COARSE
    assert reposition_goal.pose.position.x == pytest.approx(0.0)
    assert abs(reposition_goal.pose.position.y) == pytest.approx(0.75)
    assert _yaw(reposition_goal) == pytest.approx(
        math.atan2(-reposition_goal.pose.position.y, 2.8)
    )


def test_wall_side_without_safe_coverage_uses_open_side(mux_node):
    """墙侧没有安全覆盖时只选择开阔侧观察点"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_nav_graph(
        _safe_graph(
            [(0.0, 0.0), (0.0, -0.75), (0.0, 0.75)],
            [0.45, 0.8, 0.2],
            [(0, 1), (0, 2)],
        )
    )

    position = mux_node._select_reposition_position(0.0, 0.75, 0.4)

    assert position is not None
    assert position[0] == pytest.approx(0.0)
    assert position[1] == pytest.approx(-0.75)


def test_disconnected_wall_side_candidate_is_rejected(mux_node):
    """几何位置合适但与 current node 断开的观察区域不可使用"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_nav_graph(
        _safe_graph(
            [(0.0, 0.0), (0.0, -0.75), (0.0, 0.75)],
            [0.45, 0.8, 2.0],
            [(0, 1)],
        )
    )

    position = mux_node._select_reposition_position(0.0, 0.75, 0.4)

    assert position is not None
    assert position[1] == pytest.approx(-0.75)


def test_retries_choose_new_observation_position(mux_node):
    """持续重试可以选择新位置但不能返回已观察位置"""
    first_graph = _safe_graph(
        [(0.0, 0.0), (0.0, 0.75), (0.0, -0.75)],
        [0.45, 0.8, 0.8],
        [(0, 1), (0, 2)],
    )
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_nav_graph(first_graph)
    first = mux_node._select_reposition_position(0.0, 0.75, 0.4)
    assert first is not None

    mux_node._on_odom(_odom(first[0], first[1], 0.0))
    mux_node._on_nav_graph(
        _safe_graph(
            [(0.0, 0.0), (0.0, 0.75), (0.0, -0.75), (0.0, 1.5)],
            [0.45, 0.8, 0.8, 0.8],
            [(0, 1), (0, 2), (1, 3)],
            current_node_idx=1,
            stamp=2,
        )
    )
    second = mux_node._select_reposition_position(0.0, 0.75, 0.4)

    assert second is not None
    assert second[1] == pytest.approx(1.5)
    assert math.hypot(second[0], second[1]) > 0.75


def test_no_safe_reposition_position_keeps_observing(mux_node):
    """附近没有安全覆盖时保持当前位置而不是发布墙内目标"""
    mux_node.target_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_nav_graph(
        _safe_graph([(0.0, 0.0)], [0.2], [])
    )
    mux_node._on_target_estimate(
        _target_estimate(2.8, 0.0, stable=False, confidence=0.51)
    )
    mux_node._select_goal()
    mux_node.target_observation_duration_sec = 0.0

    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_OBSERVATION
    assert goal.pose.position.x == pytest.approx(0.0)
    assert goal.pose.position.y == pytest.approx(0.0)


def test_observation_switches_to_metric_approach_when_target_stabilizes(mux_node):
    """观察期间获得稳定融合结果后立即恢复精确目标接近"""
    mux_node.target_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(
        _target_estimate(2.8, 0.0, stable=False, confidence=0.51)
    )
    observation_state, _ = mux_node._select_goal()

    mux_node._on_target_estimate(_target_estimate(2.8, 0.0, stable=True))
    metric_state, metric_goal = mux_node._select_goal()

    assert observation_state == ObjectSearchState.TARGET_OBSERVATION
    assert metric_state == ObjectSearchState.TARGET_APPROACH_METRIC
    assert metric_goal.pose.position.x == pytest.approx(2.8 - 1.75)
    assert _yaw(metric_goal) == pytest.approx(0.0)


def test_stable_target_enters_facing_final_observation(mux_node):
    """到达稳定目标安全距离后保持位置并面向目标"""
    mux_node.final_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, math.pi))
    mux_node._on_target_estimate(_target_estimate(1.8, 0.0, stable=True))

    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_FINAL_OBSERVATION
    assert goal.pose.position.x == pytest.approx(0.0)
    assert goal.pose.position.y == pytest.approx(0.0)
    assert _yaw(goal) == pytest.approx(0.0)
    assert not mux_node.reached_latched


def test_final_observation_repositions_when_evidence_stays_insufficient(mux_node):
    """静止最终观察后仍无完成证据时横向更换观察点"""
    mux_node.final_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_nav_graph(_graph(forward=False))
    mux_node._on_target_estimate(_target_estimate(1.8, 0.0, stable=True))
    first_state, _ = mux_node._select_goal()
    mux_node.final_observation_duration_sec = -1.0

    next_state, reposition_goal = mux_node._select_goal()

    assert first_state == ObjectSearchState.TARGET_FINAL_OBSERVATION
    assert next_state == ObjectSearchState.TARGET_FINAL_REPOSITION
    assert reposition_goal.pose.position.x == pytest.approx(0.0)
    assert abs(reposition_goal.pose.position.y) == pytest.approx(0.9)


def test_visible_stable_target_does_not_start_scan(mux_node):
    """稳定目标持续更新时只对准目标, 不执行完整旋转"""
    mux_node.final_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(_target_estimate(1.8, 0.0, stable=True))

    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_FINAL_OBSERVATION
    assert mux_node.final_observation_phase == "ALIGN"
    assert _yaw(goal) == pytest.approx(0.0)


def test_lost_stable_target_uses_small_final_scan(mux_node):
    """最终观察失联时只围绕稳定目标方向小角度重捕获"""
    mux_node.final_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(_target_estimate(1.8, 0.0, stable=True))
    mux_node.final_observation_lost_timeout_sec = -1.0

    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_FINAL_OBSERVATION
    assert mux_node.final_observation_phase == "SCAN_LEFT"
    assert _yaw(goal) == pytest.approx(math.radians(15.0))


def test_lidar_locked_target_can_complete_without_visual_reached(mux_node):
    """连续 LiDAR 锁定在近距离时可以完成最终门控"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(
        _target_estimate(
            1.5,
            0.0,
            stable=True,
            state="LIDAR_LOCKED",
        )
    )

    state, _ = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_REACHED_VIEWPOINT
    assert mux_node.reached_latched


def test_stable_vision_final_observation_then_reached(mux_node):
    """稳定视觉目标必须经过最终观察再由连续视觉证据完成"""
    mux_node.final_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(_target_estimate(1.8, 0.0, stable=True))
    observation_state, _ = mux_node._select_goal()

    mux_node._on_object_reached(Bool(data=True))
    reached_state, _ = mux_node._select_goal()

    assert observation_state == ObjectSearchState.TARGET_FINAL_OBSERVATION
    assert reached_state == ObjectSearchState.TARGET_REACHED_VIEWPOINT
    assert mux_node.reached_latched


def test_lost_target_uses_small_scan_around_predicted_bearing(mux_node):
    """观察期间目标失联时只围绕预测方向做小角度重捕获"""
    mux_node.target_observation_duration_sec = 100.0
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(
        _target_estimate(2.8, 0.0, stable=False, confidence=0.51)
    )
    mux_node.target_observation_lost_timeout_sec = -1.0

    state, scan_goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_OBSERVATION
    assert _yaw(scan_goal) == pytest.approx(math.radians(20.0))
    assert mux_node.target_observation_phase == "SCAN_LEFT"


def test_coarse_target_cannot_trigger_final_completion(mux_node):
    """粗目标即使距离很近也不能拥有最终完成权限"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    mux_node._on_target_estimate(
        _target_estimate(1.0, 0.0, stable=False, confidence=0.5)
    )

    mux_node._on_object_reached(Bool(data=True))
    state, _ = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_OBSERVATION


def test_small_metric_updates_do_not_move_goal(mux_node):
    """置信度相近的小幅粒子波动不能反复改变高层 goal"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))
    first = _target_estimate(10.0, 2.0)
    mux_node._on_target_estimate(first)

    jittered = copy.deepcopy(first)
    jittered.pose.pose.position.x = 10.2
    jittered.pose.pose.position.y = 2.1
    jittered.confidence = 0.82
    mux_node._on_target_estimate(jittered)

    assert mux_node.metric_target.pose.position.x == pytest.approx(10.0)
    assert mux_node.metric_target.pose.position.y == pytest.approx(2.0)


def test_impossible_coarse_target_keeps_initial_goal(mux_node):
    """高度和距离明显异常的粗定位不能接管导航 goal"""
    mux_node._on_odom(_odom(15.68, -9.64, 0.0, z=0.2))
    estimate = _target_estimate(
        44.82,
        -39.53,
        stable=False,
        confidence=0.51,
        accepted_views=2,
        state="TRACKING",
        z=-5.58,
    )

    mux_node._on_target_estimate(estimate)
    state, _ = mux_node._select_goal()

    assert state == ObjectSearchState.SEARCHING_WITH_INITIAL_GOAL
    assert mux_node.metric_target is None


def test_uncertain_coarse_target_keeps_initial_goal(mux_node):
    """水平不确定度过大的粗定位不能接管导航 goal"""
    mux_node._on_odom(_odom(0.0, 0.0, 0.0))

    mux_node._on_target_estimate(
        _target_estimate(
            12.0,
            3.0,
            stable=False,
            confidence=0.7,
            state="TRACKING",
            horizontal_std=9.0,
        )
    )

    assert mux_node.metric_target is None


def test_stable_target_can_correct_previous_stable_goal(mux_node):
    """稳定融合结果移动超过小门槛时允许纠正首次稳定 goal"""
    mux_node._on_odom(_odom(15.68, -9.64, 0.0, z=0.2))
    mux_node._on_target_estimate(_target_estimate(18.35, -12.25, z=-0.03))

    mux_node._on_target_estimate(
        _target_estimate(18.75, -12.68, confidence=0.69, z=-0.17)
    )

    assert mux_node.metric_target.pose.position.x == pytest.approx(18.75)
    assert mux_node.metric_target.pose.position.y == pytest.approx(-12.68)
