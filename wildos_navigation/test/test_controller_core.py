import math

import numpy as np
import pytest

from wildos_navigation.controller_core import (
    ControllerConfig,
    VelocityCommand,
    compute_velocity_command,
    filter_local_obstacles,
    limit_acceleration,
    transform_points,
)


def _config() -> ControllerConfig:
    return ControllerConfig(
        target_speed=0.2,
        turn_speed=0.1,
        max_angular_speed=0.5,
        max_linear_acceleration=0.4,
        max_angular_acceleration=1.0,
        lookahead_distance=0.6,
        goal_tolerance=0.25,
        slow_turn_angle=0.35,
        rotate_in_place_angle=0.8,
        safety_radius=0.35,
        simulation_horizon=1.0,
        simulation_step=0.1,
    )


def test_tracks_straight_path():
    command = compute_velocity_command(
        np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]),
        (0.0, 0.0, 0.0),
        np.empty((0, 2)),
        _config(),
    )

    assert command.reason == "tracking"
    assert command.linear_x == 0.2
    assert command.angular_z == 0.0


def test_rotates_when_path_is_behind_robot():
    command = compute_velocity_command(
        np.array([[0.0, 0.0], [-1.0, 0.0]]),
        (0.0, 0.0, 0.0),
        np.empty((0, 2)),
        _config(),
    )

    assert command.reason == "rotate_in_place"
    assert command.linear_x == 0.0
    assert abs(command.angular_z) == 0.5


def test_stops_before_local_obstacle():
    command = compute_velocity_command(
        np.array([[0.0, 0.0], [2.0, 0.0]]),
        (0.0, 0.0, 0.0),
        np.array([[0.4, 0.0]]),
        _config(),
    )

    assert command == VelocityCommand(0.0, 0.0, "obstacle")


def test_stops_at_path_goal():
    command = compute_velocity_command(
        np.array([[0.0, 0.0], [1.0, 0.0]]),
        (0.8, 0.0, 0.0),
        np.empty((0, 2)),
        _config(),
    )

    assert command == VelocityCommand(0.0, 0.0, "goal_reached")


def test_filters_ground_self_and_far_points():
    filtered = filter_local_obstacles(
        np.array(
            [
                [0.2, 0.0, 0.0],
                [0.8, 0.0, -0.3],
                [0.8, 0.0, -0.7],
                [5.0, 0.0, 0.0],
            ]
        ),
        min_height=-0.55,
        max_height=0.3,
        self_filter_radius=0.45,
        max_range=4.0,
        max_points=100,
    )

    np.testing.assert_allclose(filtered, [[0.8, 0.0]])


def test_transforms_cloud_to_robot_frame():
    half_angle = math.pi / 4.0
    transformed = transform_points(
        np.array([[1.0, 0.0, 0.0]]),
        (1.0, 2.0, 0.0),
        (0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
    )

    np.testing.assert_allclose(transformed, [[1.0, 3.0, 0.0]], atol=1.0e-9)


def test_acceleration_limit_does_not_delay_safety_stop():
    previous = VelocityCommand(0.2, 0.3, "tracking")
    stopped = limit_acceleration(
        previous,
        VelocityCommand(0.0, 0.0, "stale_odom"),
        elapsed=0.05,
        config=_config(),
    )
    accelerating = limit_acceleration(
        VelocityCommand(0.0, 0.0, "idle"),
        VelocityCommand(0.2, 0.5, "tracking"),
        elapsed=0.05,
        config=_config(),
    )

    assert stopped.stopped
    assert accelerating.linear_x == pytest.approx(0.02)
    assert accelerating.angular_z == pytest.approx(0.05)
