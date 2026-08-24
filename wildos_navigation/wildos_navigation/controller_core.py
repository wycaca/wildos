from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ControllerConfig:
    """路径跟踪和局部碰撞检查参数"""

    target_speed: float
    turn_speed: float
    max_angular_speed: float
    max_linear_acceleration: float
    max_angular_acceleration: float
    lookahead_distance: float
    goal_tolerance: float
    slow_turn_angle: float
    rotate_in_place_angle: float
    safety_radius: float
    simulation_horizon: float
    simulation_step: float


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float
    angular_z: float
    reason: str

    @property
    def stopped(self) -> bool:
        return self.linear_x == 0.0 and self.angular_z == 0.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion: tuple[float, float, float, float]) -> float:
    x, y, z, w = quaternion
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def quaternion_rotation_matrix(
    quaternion: tuple[float, float, float, float],
) -> np.ndarray:
    """构造三维旋转矩阵, 零范数输入回退到单位旋转"""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return np.eye(3)
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_points(
    points: np.ndarray,
    translation: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> np.ndarray:
    """应用 target <- source 刚体变换"""
    coordinates = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    if coordinates.size == 0:
        return coordinates
    rotation = quaternion_rotation_matrix(quaternion)
    return coordinates @ rotation.T + np.asarray(translation, dtype=np.float64)


def filter_local_obstacles(
    points: np.ndarray,
    min_height: float,
    max_height: float,
    self_filter_radius: float,
    max_range: float,
    max_points: int,
) -> np.ndarray:
    """在 base_link 中过滤地面、自身结构和远距离点"""
    coordinates = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    if coordinates.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    finite = np.isfinite(coordinates).all(axis=1)
    radial_distance = np.linalg.norm(coordinates[:, :2], axis=1)
    selected = coordinates[
        finite
        & (coordinates[:, 2] >= min_height)
        & (coordinates[:, 2] <= max_height)
        & (radial_distance >= self_filter_radius)
        & (radial_distance <= max_range)
    ]
    if selected.shape[0] > max_points:
        stride = int(math.ceil(selected.shape[0] / max_points))
        selected = selected[::stride][:max_points]
    return selected[:, :2]


def select_lookahead(
    path: np.ndarray,
    robot_xy: np.ndarray,
    lookahead_distance: float,
) -> np.ndarray:
    """从距机器人最近的路径点向前累计选择前视点"""
    points = np.asarray(path, dtype=np.float64).reshape((-1, 2))
    if points.shape[0] == 0:
        raise ValueError("path must contain at least one point")
    robot = np.asarray(robot_xy, dtype=np.float64).reshape(2)
    nearest_index = int(np.argmin(np.linalg.norm(points - robot, axis=1)))
    accumulated = 0.0
    for index in range(nearest_index, points.shape[0] - 1):
        accumulated += float(np.linalg.norm(points[index + 1] - points[index]))
        if accumulated >= lookahead_distance:
            return points[index + 1]
    return points[-1]


def global_to_local(
    point: np.ndarray,
    robot_pose: tuple[float, float, float],
) -> np.ndarray:
    x, y, yaw = robot_pose
    delta = np.asarray(point, dtype=np.float64).reshape(2) - np.array([x, y])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.array(
        [
            delta[0] * cos_yaw + delta[1] * sin_yaw,
            -delta[0] * sin_yaw + delta[1] * cos_yaw,
        ]
    )


def simulate_local_trajectory(
    linear_speed: float,
    angular_speed: float,
    horizon: float,
    step: float,
) -> np.ndarray:
    """在机器人当前局部坐标中推演恒定速度轨迹"""
    samples = max(1, int(math.ceil(horizon / step)))
    trajectory = np.empty((samples, 2), dtype=np.float64)
    x = 0.0
    y = 0.0
    yaw = 0.0
    for index in range(samples):
        x += linear_speed * math.cos(yaw) * step
        y += linear_speed * math.sin(yaw) * step
        yaw += angular_speed * step
        trajectory[index] = (x, y)
    return trajectory


def has_collision(
    trajectory: np.ndarray,
    obstacles: np.ndarray,
    safety_radius: float,
) -> bool:
    obstacle_points = np.asarray(obstacles, dtype=np.float64).reshape((-1, 2))
    if obstacle_points.size == 0:
        return False
    distances = np.linalg.norm(
        np.asarray(trajectory, dtype=np.float64)[:, None, :] - obstacle_points[None, :, :],
        axis=2,
    )
    return bool(np.any(distances < safety_radius))


def compute_velocity_command(
    path: np.ndarray,
    robot_pose: tuple[float, float, float],
    obstacles: np.ndarray,
    config: ControllerConfig,
) -> VelocityCommand:
    """计算一帧 Pure Pursuit 控制并在预测轨迹碰撞时立即停车"""
    points = np.asarray(path, dtype=np.float64).reshape((-1, 2))
    if points.shape[0] == 0:
        return VelocityCommand(0.0, 0.0, "no_path")
    robot_xy = np.asarray(robot_pose[:2], dtype=np.float64)
    if float(np.linalg.norm(points[-1] - robot_xy)) <= config.goal_tolerance:
        return VelocityCommand(0.0, 0.0, "goal_reached")

    lookahead = select_lookahead(points, robot_xy, config.lookahead_distance)
    local_target = global_to_local(lookahead, robot_pose)
    heading_error = math.atan2(local_target[1], local_target[0])
    angular_speed = max(
        -config.max_angular_speed,
        min(config.max_angular_speed, 2.0 * heading_error),
    )

    if abs(heading_error) >= config.rotate_in_place_angle:
        return VelocityCommand(0.0, angular_speed, "rotate_in_place")

    linear_speed = (
        config.turn_speed
        if abs(heading_error) >= config.slow_turn_angle
        else config.target_speed
    )
    trajectory = simulate_local_trajectory(
        linear_speed,
        angular_speed,
        config.simulation_horizon,
        config.simulation_step,
    )
    if has_collision(trajectory, obstacles, config.safety_radius):
        return VelocityCommand(0.0, 0.0, "obstacle")
    return VelocityCommand(linear_speed, angular_speed, "tracking")


def limit_acceleration(
    previous: VelocityCommand,
    desired: VelocityCommand,
    elapsed: float,
    config: ControllerConfig,
) -> VelocityCommand:
    """正常跟踪时限制速度变化, 安全停车不使用减速度斜坡"""
    if desired.stopped or elapsed <= 0.0:
        return desired
    linear_delta = config.max_linear_acceleration * elapsed
    angular_delta = config.max_angular_acceleration * elapsed
    linear_x = float(
        np.clip(
            desired.linear_x,
            previous.linear_x - linear_delta,
            previous.linear_x + linear_delta,
        )
    )
    angular_z = float(
        np.clip(
            desired.angular_z,
            previous.angular_z - angular_delta,
            previous.angular_z + angular_delta,
        )
    )
    return VelocityCommand(linear_x, angular_z, desired.reason)
