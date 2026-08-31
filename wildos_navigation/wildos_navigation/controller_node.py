from collections import Counter
import math
import random
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from wildos_navigation.performance_stats import (
    EventRate,
    TimingWindow,
    diagnostic_status,
    message_age_ms,
    timing_metrics,
)


_TIMING_STAGES = (
    "scan_total",
    "scan_decode",
    "scan_filter",
    "scan_limit",
    "scan_cluster",
    "control_total",
    "control_publish",
)


def _controller_diagnostic_metrics(summaries, cloud_rate, counters, cloud_age_ms):
    """构造路径跟随与点云避障稳定指标名称"""
    metrics = {"input.cloud.rate_hz": cloud_rate}
    if cloud_age_ms is not None:
        metrics["input.cloud.age_ms"] = cloud_age_ms
    for stage, summary in summaries.items():
        prefix = stage.replace("_", ".", 1)
        metrics.update(timing_metrics(prefix, summary))
    metrics["cycle.total.average_ms"] = summaries["scan_total"].average_ms
    metrics["cycle.total.p95_ms"] = summaries["scan_total"].p95_ms
    metrics["cycle.total.maximum_ms"] = summaries["scan_total"].maximum_ms
    metrics.update({f"workload.{key}": value for key, value in counters.items()})
    return metrics


# 迁移自导航同事已测试工作区, 仅适配当前 ROS 接口
class SimpleKalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.X = np.zeros(4)  # [x, y, vx, vy]
        self.F = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.P = np.eye(4) * 10.0
        self.Q = np.eye(4) * 0.01
        self.R = np.eye(2) * 0.1
        self.is_initialized = False

    def update(self, z):
        if not self.is_initialized:
            self.X = np.array([z[0], z[1], 0.0, 0.0])
            self.is_initialized = True
            return self.X
        X_pred = self.F @ self.X
        P_pred = self.F @ self.P @ self.F.T + self.Q
        y = z - (self.H @ X_pred)
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.X = X_pred + K @ y
        self.P = (np.eye(4) - K @ self.H) @ P_pred
        return self.X


class AdvancedGeometricFollower(Node):
    def __init__(self):
        super().__init__("start_nav")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("lidar_topic", "/cloud_registered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("diagnostics_enabled", True)
        self.declare_parameter("diagnostics_period_sec", 30.0)
        self.declare_parameter("diagnostics_scan_budget_ms", 100.0)
        self.declare_parameter("diagnostics_control_budget_ms", 20.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.diagnostics_enabled = bool(
            self.get_parameter("diagnostics_enabled").value
        )
        self.diagnostics_period_sec = float(
            self.get_parameter("diagnostics_period_sec").value
        )
        self.diagnostics_scan_budget_ms = float(
            self.get_parameter("diagnostics_scan_budget_ms").value
        )
        self.diagnostics_control_budget_ms = float(
            self.get_parameter("diagnostics_control_budget_ms").value
        )
        if self.diagnostics_period_sec <= 0.0:
            raise ValueError("diagnostics_period_sec must be positive")

        # 原算法动力学与避障参数
        self.v_target = 0.6
        self.w_max = 1.0
        self.lookahead_dist = 0.55
        self.safe_radius = 0.22
        self.dynamic_safe_zone = 0.45

        self.current_v = 0.0
        self.current_w = 0.0
        self.path_points = None
        self.path_received = False
        self.latest_pose = None

        self.real_obstacles = []
        self.predicted_dynamic_obs = []
        self.kf_tracker = SimpleKalmanFilter(dt=0.1)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odometry_callback,
            10,
        )
        self.path_subscriber = self.create_subscription(
            Path,
            "path",
            self.path_callback,
            10,
        )
        self.scan_subscriber = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self._timings = {
            stage: TimingWindow(max_samples=512)
            for stage in _TIMING_STAGES
        }
        self._cloud_rate = EventRate()
        self._diagnostic_counters = Counter()
        self._last_cloud_stamp_ns = None
        if self.diagnostics_enabled:
            diagnostic_qos = QoSProfile(depth=1)
            diagnostic_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.diagnostics_pub = self.create_publisher(
                DiagnosticArray,
                "/diagnostics",
                diagnostic_qos,
            )
            self.diagnostics_timer = self.create_timer(
                self.diagnostics_period_sec,
                self._publish_diagnostics,
            )

        self.control_timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(
            "智能几何追随节点已启动, 融合静态安全盾与卡尔曼动态预测"
        )

    def path_callback(self, msg):
        if len(msg.poses) == 0:
            self.path_received = False
            return
        self.path_points_list = [
            [point.pose.position.x, point.pose.position.y]
            for point in msg.poses
        ]
        self.path_points = np.array(self.path_points_list)
        self.path_received = True

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odometry_callback(self, msg):
        robot_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            robot_yaw,
        ]

    def scan_callback(self, msg):
        total_started = time.perf_counter()
        self._record_cloud_input(msg)
        try:
            self._process_scan(msg)
        finally:
            self._record_timing("scan_total", total_started)

    def _process_scan(self, msg):
        """执行原始点云过滤和动态障碍预测"""
        if self.latest_pose is None:
            self._count("dropped_no_pose")
            return

        stage_started = time.perf_counter()
        points = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        self._record_timing("scan_decode", stage_started)
        self._count("input_points", len(points))
        stage_started = time.perf_counter()
        point_x = points["x"]
        point_y = points["y"]
        point_z = points["z"]
        distance_to_dog = np.hypot(
            point_x.astype(np.float64) - self.latest_pose[0],
            point_y.astype(np.float64) - self.latest_pose[1],
        )
        valid = (
            (point_z > 0.12)
            & (point_z < 0.8)
            & (distance_to_dog > 0.42)
            & (distance_to_dog < 4.0)
        )
        obstacles = np.column_stack((point_x[valid], point_y[valid]))
        self._record_timing("scan_filter", stage_started)
        self._count("filtered_points", len(obstacles))

        stage_started = time.perf_counter()
        if len(obstacles) > 120:
            sample_indices = random.sample(range(len(obstacles)), 120)
            obstacles = obstacles[sample_indices]
        if len(obstacles) > 35:
            distances = np.hypot(
                obstacles[:, 0].astype(np.float64) - self.latest_pose[0],
                obstacles[:, 1].astype(np.float64) - self.latest_pose[1],
            )
            nearest = np.argsort(distances, kind="stable")[:35]
            obstacles = obstacles[nearest]
        self._record_timing("scan_limit", stage_started)
        self._count("limited_points", len(obstacles))

        self.real_obstacles = obstacles.tolist()

        # 预测局部运动障碍在未来 1 秒内的位置
        stage_started = time.perf_counter()
        self.predicted_dynamic_obs = []
        if len(obstacles) > 0:
            # 距离矩阵最多处理35个候选, 避免整帧点云进入二次复杂度路径
            obstacle_xy = obstacles.astype(np.float64)
            distance_matrix = np.hypot(
                obstacle_xy[:, np.newaxis, 0] - obstacle_xy[np.newaxis, :, 0],
                obstacle_xy[:, np.newaxis, 1] - obstacle_xy[np.newaxis, :, 1],
            )
            neighbor_counts = np.count_nonzero(distance_matrix < 0.3, axis=1)
            for candidate_index, count_neighbors in enumerate(neighbor_counts):
                if 3 <= count_neighbors <= 20:
                    cluster_points = obstacles[
                        distance_matrix[candidate_index] < 0.4
                    ]
                    if len(cluster_points) >= 3:
                        centroid_x = np.mean(cluster_points[:, 0])
                        centroid_y = np.mean(cluster_points[:, 1])
                        estimated_state = self.kf_tracker.update(
                            np.array([centroid_x, centroid_y])
                        )
                        obs_vx, obs_vy = estimated_state[2], estimated_state[3]

                        if math.hypot(obs_vx, obs_vy) > 0.15:
                            for i in range(1, 4):
                                self.predicted_dynamic_obs.append(
                                    [
                                        centroid_x + obs_vx * 0.33 * i,
                                        centroid_y + obs_vy * 0.33 * i,
                                    ]
                                )
        else:
            self.kf_tracker.is_initialized = False
        self._record_timing("scan_cluster", stage_started)
        self._count("predicted_points", len(self.predicted_dynamic_obs))

    def control_loop(self):
        """执行原算法的路径跟踪、轨迹推演和动态障碍降速"""
        total_started = time.perf_counter()
        self._count("control_ticks")
        try:
            self._run_control_loop()
        finally:
            self._record_timing("control_total", total_started)

    def _run_control_loop(self):
        """保持原路径跟踪、停车和避障状态机"""
        if (
            not self.path_received
            or self.path_points is None
            or self.latest_pose is None
        ):
            return

        pose = self.latest_pose
        robot_x, robot_y, robot_yaw = pose[0], pose[1], pose[2]

        distance_to_end = np.linalg.norm(
            np.array([robot_x, robot_y]) - self.path_points[-1]
        )
        if distance_to_end < 0.20:
            self.stop_robot()
            return

        distances_to_path = np.linalg.norm(
            self.path_points - np.array([robot_x, robot_y]),
            axis=1,
        )
        nearest_idx = np.argmin(distances_to_path)

        lookahead_target = self.path_points[-1]
        accumulated_dist = 0.0
        for i in range(nearest_idx, len(self.path_points) - 1):
            accumulated_dist += np.linalg.norm(
                self.path_points[i + 1] - self.path_points[i]
            )
            if accumulated_dist > self.lookahead_dist:
                lookahead_target = self.path_points[i + 1]
                break

        dx = lookahead_target[0] - robot_x
        dy = lookahead_target[1] - robot_y

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        local_y = -dx * sin_yaw + dy * cos_yaw

        ld_sq = self.lookahead_dist**2
        raw_w = (
            (2.0 * self.v_target * local_y) / ld_sq
            if ld_sq > 0.001
            else 0.0
        )

        sim_traj = []
        sim_x, sim_y, sim_yaw = robot_x, robot_y, robot_yaw
        for _ in range(5):
            sim_x += self.v_target * math.cos(sim_yaw) * 0.1
            sim_y += self.v_target * math.sin(sim_yaw) * 0.1
            sim_yaw += raw_w * 0.1
            sim_traj.append([sim_x, sim_y])

        is_collision_risk = False
        is_dynamic_risk = False

        for p in sim_traj:
            for obs in self.real_obstacles:
                if (
                    math.hypot(p[0] - obs[0], p[1] - obs[1])
                    < self.safe_radius
                ):
                    is_collision_risk = True
                    break

        for p in sim_traj:
            for ghost in self.predicted_dynamic_obs:
                if (
                    math.hypot(p[0] - ghost[0], p[1] - ghost[1])
                    < self.dynamic_safe_zone
                ):
                    is_dynamic_risk = True
                    break

        target_angle = math.atan2(dy, dx)
        angle_diff = abs(
            math.atan2(
                math.sin(target_angle - robot_yaw),
                math.cos(target_angle - robot_yaw),
            )
        )

        speed = self.v_target

        if angle_diff > 0.9:
            speed = 0.0
        elif is_collision_risk or is_dynamic_risk:
            speed = 0.1 if is_dynamic_risk else 0.0
            self._count("static_risk_cycles", int(is_collision_risk))
            self._count("dynamic_risk_cycles", int(is_dynamic_risk))
        elif angle_diff > 0.4:
            speed = 0.4

        steering_angle = raw_w
        steering_angle = max(-self.w_max, min(self.w_max, steering_angle))

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(speed)
        cmd_vel_msg.angular.z = float(steering_angle)
        publish_started = time.perf_counter()
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self._record_timing("control_publish", publish_started)
        self._count("published_commands")

    def stop_robot(self):
        cmd_vel_msg = Twist()
        publish_started = time.perf_counter()
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self._record_timing("control_publish", publish_started)
        self._count("published_stops")
        self.path_received = False

    def _record_timing(self, stage, started):
        timings = getattr(self, "_timings", None)
        if timings is not None:
            timings[stage].add_seconds(time.perf_counter() - started)

    def _count(self, name, value=1):
        counters = getattr(self, "_diagnostic_counters", None)
        if counters is not None:
            counters[name] += value

    def _record_cloud_input(self, msg):
        rate = getattr(self, "_cloud_rate", None)
        if rate is None:
            return
        rate.tick()
        self._count("received_clouds")
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )
        self._last_cloud_stamp_ns = stamp_ns if stamp_ns > 0 else None

    def _cloud_age_ms(self):
        return message_age_ms(
            self.get_clock().now().nanoseconds,
            self._last_cloud_stamp_ns,
        )

    def _publish_diagnostics(self):
        """低频发布点云避障与路径控制阶段标量"""
        summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._timings.items()
        }
        metrics = _controller_diagnostic_metrics(
            summaries,
            self._cloud_rate.sample(reset=True),
            dict(self._diagnostic_counters),
            self._cloud_age_ms(),
        )
        slow = (
            summaries["scan_total"].p95_ms >= self.diagnostics_scan_budget_ms
            or summaries["control_total"].p95_ms
            >= self.diagnostics_control_budget_ms
        )
        status = diagnostic_status(
            "wildos/controller",
            metrics,
            level=DiagnosticStatus.WARN if slow else DiagnosticStatus.OK,
            message="processing budget exceeded" if slow else "OK",
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)
        self._diagnostic_counters.clear()


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedGeometricFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
