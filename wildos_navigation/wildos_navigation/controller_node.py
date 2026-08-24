from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from wildos_navigation.controller_core import (
    ControllerConfig,
    VelocityCommand,
    compute_velocity_command,
    filter_local_obstacles,
    limit_acceleration,
    transform_points,
    yaw_from_quaternion,
)


class NavigationController(Node):
    """执行 WildOS Path 并在局部点云中生成安全速度指令"""

    def __init__(self) -> None:
        super().__init__("wildos_navigation_controller")
        self._declare_parameters()
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.expected_cloud_frame = str(
            self.get_parameter("expected_cloud_frame").value
        )
        self.odom_timeout_sec = self._positive_parameter("odom_timeout_sec")
        self.cloud_timeout_sec = self._positive_parameter("cloud_timeout_sec")
        self.path_initial_max_age_sec = self._positive_parameter(
            "path_initial_max_age_sec"
        )
        self.future_tolerance_sec = self._nonnegative_parameter(
            "future_tolerance_sec"
        )
        self.tf_timeout_sec = self._positive_parameter("tf_timeout_sec")
        self.obstacle_min_height = float(
            self.get_parameter("obstacle_min_height").value
        )
        self.obstacle_max_height = float(
            self.get_parameter("obstacle_max_height").value
        )
        self.self_filter_radius = self._nonnegative_parameter(
            "self_filter_radius"
        )
        self.max_obstacle_range = self._positive_parameter(
            "max_obstacle_range"
        )
        self.max_obstacle_points = int(
            self.get_parameter("max_obstacle_points").value
        )
        if self.obstacle_min_height >= self.obstacle_max_height:
            raise ValueError("obstacle_min_height must be less than obstacle_max_height")
        if self.max_obstacle_points <= 0:
            raise ValueError("max_obstacle_points must be greater than 0")

        self.controller_config = ControllerConfig(
            target_speed=self._positive_parameter("target_linear_speed"),
            turn_speed=self._positive_parameter("turn_linear_speed"),
            max_angular_speed=self._positive_parameter("max_angular_speed"),
            max_linear_acceleration=self._positive_parameter(
                "max_linear_acceleration"
            ),
            max_angular_acceleration=self._positive_parameter(
                "max_angular_acceleration"
            ),
            lookahead_distance=self._positive_parameter("lookahead_distance"),
            goal_tolerance=self._positive_parameter("goal_tolerance"),
            slow_turn_angle=self._positive_parameter("slow_turn_angle"),
            rotate_in_place_angle=self._positive_parameter(
                "rotate_in_place_angle"
            ),
            safety_radius=self._positive_parameter("safety_radius"),
            simulation_horizon=self._positive_parameter("simulation_horizon"),
            simulation_step=self._positive_parameter("simulation_step"),
        )
        if self.controller_config.turn_speed > self.controller_config.target_speed:
            raise ValueError("turn_linear_speed must not exceed target_linear_speed")
        if (
            self.controller_config.slow_turn_angle
            >= self.controller_config.rotate_in_place_angle
        ):
            raise ValueError("slow_turn_angle must be less than rotate_in_place_angle")

        self._path = np.empty((0, 2), dtype=np.float64)
        self._robot_pose: tuple[float, float, float] | None = None
        self._obstacles = np.empty((0, 2), dtype=np.float64)
        self._last_odom_monotonic: float | None = None
        self._last_cloud_monotonic: float | None = None
        self._previous_command = VelocityCommand(0.0, 0.0, "startup")
        self._previous_control_monotonic = time.monotonic()
        self._last_reason = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            1,
        )
        self.path_subscription = self.create_subscription(
            Path,
            self.path_topic,
            self._on_path,
            1,
        )
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._on_cloud,
            cloud_qos,
        )
        control_frequency = self._positive_parameter("control_frequency")
        self.create_timer(1.0 / control_frequency, self._control)
        self.get_logger().info(
            "GO2 局部导航已启动, "
            f"path={self.path_topic}, odom={self.odom_topic}, "
            f"cloud={self.pointcloud_topic}, cmd={self.cmd_vel_topic}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "path_topic": "/spot1/graphnav_planner/path",
            "odom_topic": "/odom",
            "pointcloud_topic": "/cloud_registered",
            "cmd_vel_topic": "/wildos/cmd_vel",
            "world_frame": "odom",
            "base_frame": "base_link",
            "expected_cloud_frame": "dlio_odom",
            "control_frequency": 20.0,
            "target_linear_speed": 0.2,
            "turn_linear_speed": 0.1,
            "max_angular_speed": 0.5,
            "max_linear_acceleration": 0.4,
            "max_angular_acceleration": 1.0,
            "lookahead_distance": 0.6,
            "goal_tolerance": 0.25,
            "slow_turn_angle": 0.35,
            "rotate_in_place_angle": 0.8,
            "safety_radius": 0.35,
            "simulation_horizon": 1.0,
            "simulation_step": 0.1,
            "obstacle_min_height": -0.55,
            "obstacle_max_height": 0.3,
            "self_filter_radius": 0.45,
            "max_obstacle_range": 4.0,
            "max_obstacle_points": 512,
            "odom_timeout_sec": 0.5,
            "cloud_timeout_sec": 0.5,
            "path_initial_max_age_sec": 2.0,
            "future_tolerance_sec": 0.1,
            "tf_timeout_sec": 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f"{name} must be greater than 0")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f"{name} must not be negative")
        return value

    def _on_path(self, msg: Path) -> None:
        """只接受当前 odom 坐标系中时间有效的执行路径"""
        if msg.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"拒绝路径坐标系, expected={self.world_frame}, "
                f"actual={msg.header.frame_id}"
            )
            self._path = np.empty((0, 2), dtype=np.float64)
            return
        stamp = Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type)
        age = (self.get_clock().now() - stamp).nanoseconds / 1.0e9
        if age < -self.future_tolerance_sec or age > self.path_initial_max_age_sec:
            self.get_logger().warning(f"拒绝时间异常路径, age={age:.3f}s")
            self._path = np.empty((0, 2), dtype=np.float64)
            return
        points = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses],
            dtype=np.float64,
        ).reshape((-1, 2))
        if points.shape[0] < 2 or not np.isfinite(points).all():
            self._path = np.empty((0, 2), dtype=np.float64)
            self._publish_stop("hold_or_invalid_path")
            return
        self._path = points

    def _on_odom(self, msg: Odometry) -> None:
        """缓存全局机器人位姿, 非标准 frame 直接拒绝"""
        if msg.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"拒绝 odom 坐标系, expected={self.world_frame}, "
                f"actual={msg.header.frame_id}",
                throttle_duration_sec=5.0,
            )
            return
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not np.isfinite(values).all():
            return
        yaw = yaw_from_quaternion(tuple(values[2:6]))
        self._robot_pose = (float(values[0]), float(values[1]), yaw)
        self._last_odom_monotonic = time.monotonic()

    def _on_cloud(self, msg: PointCloud2) -> None:
        """按点云测量时刻转换到 base_link 并缓存局部障碍"""
        if self.expected_cloud_frame and msg.header.frame_id != self.expected_cloud_frame:
            self.get_logger().warning(
                f"拒绝点云坐标系, expected={self.expected_cloud_frame}, "
                f"actual={msg.header.frame_id}",
                throttle_duration_sec=5.0,
            )
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"点云 TF 不可用, error={exc}",
                throttle_duration_sec=5.0,
            )
            return
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        local_points = transform_points(
            points,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        self._obstacles = filter_local_obstacles(
            local_points,
            self.obstacle_min_height,
            self.obstacle_max_height,
            self.self_filter_radius,
            self.max_obstacle_range,
            self.max_obstacle_points,
        )
        self._last_cloud_monotonic = time.monotonic()

    def _control(self) -> None:
        """检查输入租约并计算一帧控制, 任一关键输入失效立即停车"""
        now = time.monotonic()
        reason = self._input_failure_reason(now)
        if reason:
            self._publish_stop(reason)
            return
        desired = compute_velocity_command(
            self._path,
            self._robot_pose,
            self._obstacles,
            self.controller_config,
        )
        if desired.reason == "goal_reached":
            # 到达终点后丢弃旧路线, 避免位姿漂移触发重复运动
            self._path = np.empty((0, 2), dtype=np.float64)
        command = limit_acceleration(
            self._previous_command,
            desired,
            now - self._previous_control_monotonic,
            self.controller_config,
        )
        self._publish_command(command, now)

    def _input_failure_reason(self, now: float) -> str:
        if self._path.shape[0] < 2:
            return "no_path"
        if self.path_subscription.get_publisher_count() == 0:
            return "planner_disconnected"
        if self._robot_pose is None or self._last_odom_monotonic is None:
            return "no_odom"
        if now - self._last_odom_monotonic > self.odom_timeout_sec:
            return "stale_odom"
        if self._last_cloud_monotonic is None:
            return "no_cloud"
        if now - self._last_cloud_monotonic > self.cloud_timeout_sec:
            return "stale_cloud"
        return ""

    def _publish_stop(self, reason: str) -> None:
        self._publish_command(VelocityCommand(0.0, 0.0, reason), time.monotonic())

    def _publish_command(self, command: VelocityCommand, now: float) -> None:
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        self.cmd_vel_publisher.publish(msg)
        self._previous_command = command
        self._previous_control_monotonic = now
        if command.reason != self._last_reason:
            self.get_logger().info(
                f"导航控制状态={command.reason}, "
                f"vx={command.linear_x:.3f}, wz={command.angular_z:.3f}"
            )
            self._last_reason = command.reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop("shutdown")
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
