import numpy as np

from wildos_navigation.controller_node import AdvancedGeometricFollower


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _controller(pose=(0.0, 0.0, 0.0)):
    node = AdvancedGeometricFollower.__new__(AdvancedGeometricFollower)
    node.path_received = True
    node.path_points = np.array([[0.0, 0.0], [1.0, 0.0]])
    node.latest_pose = list(pose)
    node.real_obstacles = []
    node.predicted_dynamic_obs = []
    node.v_target = 0.6
    node.w_max = 1.0
    node.lookahead_dist = 0.55
    node.safe_radius = 0.22
    node.dynamic_safe_zone = 0.45
    node.cmd_vel_publisher = _Publisher()
    return node


def test_controller_publishes_original_clear_path_speed():
    node = _controller()

    node.control_loop()

    command = node.cmd_vel_publisher.messages[-1]
    assert command.linear.x == 0.6
    assert command.angular.z == 0.0


def test_controller_stops_for_static_obstacle():
    node = _controller()
    node.real_obstacles = [[0.1, 0.0]]

    node.control_loop()

    assert node.cmd_vel_publisher.messages[-1].linear.x == 0.0


def test_controller_stops_and_releases_completed_path():
    node = _controller(pose=(0.9, 0.0, 0.0))

    node.control_loop()

    command = node.cmd_vel_publisher.messages[-1]
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0
    assert not node.path_received
