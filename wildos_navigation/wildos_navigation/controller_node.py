import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


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

        self.odom_topic = self.get_parameter("odom_topic").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

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
        if self.latest_pose is None:
            return

        obstacles_temp = []
        point_generator = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        for p in point_generator:
            obs_x, obs_y, obs_z = p[0], p[1], p[2]

            if 0.12 < obs_z < 0.8:
                dist_to_dog = math.hypot(
                    obs_x - self.latest_pose[0],
                    obs_y - self.latest_pose[1],
                )
                if 0.42 < dist_to_dog < 4.0:
                    obstacles_temp.append([obs_x, obs_y])

        if len(obstacles_temp) > 120:
            obstacles_temp = random.sample(obstacles_temp, 120)
        if len(obstacles_temp) > 35:
            obstacles_temp.sort(
                key=lambda obs: math.hypot(
                    obs[0] - self.latest_pose[0],
                    obs[1] - self.latest_pose[1],
                )
            )
            obstacles_temp = obstacles_temp[:35]

        self.real_obstacles = obstacles_temp

        # 预测局部运动障碍在未来 1 秒内的位置
        self.predicted_dynamic_obs = []
        if len(self.real_obstacles) > 0:
            for pt_candidate in self.real_obstacles:
                count_neighbors = sum(
                    1
                    for obs in self.real_obstacles
                    if math.hypot(
                        obs[0] - pt_candidate[0],
                        obs[1] - pt_candidate[1],
                    )
                    < 0.3
                )
                if 3 <= count_neighbors <= 20:
                    cluster_points = [
                        obs
                        for obs in self.real_obstacles
                        if math.hypot(
                            obs[0] - pt_candidate[0],
                            obs[1] - pt_candidate[1],
                        )
                        < 0.4
                    ]
                    if len(cluster_points) >= 3:
                        obs_array = np.array(cluster_points)
                        centroid_x = np.mean(obs_array[:, 0])
                        centroid_y = np.mean(obs_array[:, 1])
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

    def control_loop(self):
        """执行原算法的路径跟踪、轨迹推演和动态障碍降速"""
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
        local_x = dx * cos_yaw + dy * sin_yaw
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
            self.get_logger().warning(
                "避障生效, "
                f"static={is_collision_risk}, dynamic={is_dynamic_risk}"
            )
        elif angle_diff > 0.4:
            speed = 0.4

        speed = speed
        steering_angle = raw_w
        steering_angle = max(-self.w_max, min(self.w_max, steering_angle))

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(speed)
        cmd_vel_msg.angular.z = float(steering_angle)
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        if not hasattr(self, "log_counter"):
            self.log_counter = 0
        self.log_counter += 1
        if self.log_counter % 25 == 0:
            self.get_logger().info(
                f"导航速度, vx={speed:.2f}, wz={steering_angle:.2f}, "
                f"obstacles={len(self.real_obstacles)}, "
                f"predicted={len(self.predicted_dynamic_obs)}"
            )

    def stop_robot(self):
        cmd_vel_msg = Twist()
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.path_received = False


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
