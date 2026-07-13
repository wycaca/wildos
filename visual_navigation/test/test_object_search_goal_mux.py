import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy

from visual_navigation.object_search_goal_mux import ObjectSearchGoalMux
from visual_navigation.object_search_types import ObjectSearchState


@pytest.fixture
def mux_node():
    """创建不依赖 planner subscriber 的 goal mux 测试节点"""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "require_subscriber:=false",
            "-p",
            "initial_goal_distance:=20.0",
        ]
    )
    node = ObjectSearchGoalMux()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _odom(x: float, y: float, yaw: float) -> Odometry:
    """构造给定位置和偏航角的 odom"""
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
    return msg


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


def test_confirmed_target_replaces_coarse_goal(mux_node):
    """有效目标进入后由 mux 直接切换为目标位置"""
    mux_node._on_odom(_odom(1.0, 2.0, 0.0))
    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.position.x = 12.0
    target.pose.position.y = -3.0
    target.pose.orientation.w = 1.0

    mux_node._on_target_pose(target)
    state, goal = mux_node._select_goal()

    assert state == ObjectSearchState.TARGET_APPROACH
    assert goal.pose.position.x == pytest.approx(12.0)
    assert goal.pose.position.y == pytest.approx(-3.0)
