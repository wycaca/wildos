from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool


def stamp_nanoseconds(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def is_strictly_new_stamp(
    current_stamp_ns: int,
    previous_stamp_ns: int | None,
) -> bool:
    """Accept only strictly increasing sensor timestamps"""
    return previous_stamp_ns is None or current_stamp_ns > previous_stamp_ns


class DlioInputFilter(Node):
    """Remove duplicate sensor stamps and stop input after DLIO divergence"""

    def __init__(self) -> None:
        super().__init__("dlio_input_filter")
        self.declare_parameter("input_pointcloud_topic", "/livox/lidar")
        self.declare_parameter("input_imu_topic", "/livox/imu")
        self.declare_parameter(
            "output_pointcloud_topic",
            "/spot1/dlio/odom_node/input/pointcloud",
        )
        self.declare_parameter(
            "output_imu_topic",
            "/spot1/dlio/odom_node/input/imu",
        )
        self.declare_parameter(
            "health_topic",
            "/spot1/dlio/odom_node/healthy",
        )
        self.declare_parameter("diagnostic_interval", 10.0)

        self.input_pointcloud_topic = str(
            self.get_parameter("input_pointcloud_topic").value
        )
        self.input_imu_topic = str(self.get_parameter("input_imu_topic").value)
        self.output_pointcloud_topic = str(
            self.get_parameter("output_pointcloud_topic").value
        )
        self.output_imu_topic = str(
            self.get_parameter("output_imu_topic").value
        )
        self.health_topic = str(self.get_parameter("health_topic").value)
        diagnostic_interval = float(
            self.get_parameter("diagnostic_interval").value
        )

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        health_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            self.output_pointcloud_topic,
            sensor_qos,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            self.output_imu_topic,
            sensor_qos,
        )
        self.create_subscription(
            PointCloud2,
            self.input_pointcloud_topic,
            self._on_pointcloud,
            sensor_qos,
        )
        self.create_subscription(
            Imu,
            self.input_imu_topic,
            self._on_imu,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            self.health_topic,
            self._on_health,
            health_qos,
        )
        self.create_timer(diagnostic_interval, self._report_diagnostics)

        self.last_pointcloud_stamp_ns: int | None = None
        self.last_imu_stamp_ns: int | None = None
        self.pointcloud_accepted = 0
        self.pointcloud_dropped = 0
        self.imu_accepted = 0
        self.imu_dropped = 0
        self.healthy = True
        self._logged_unhealthy = False
        self.get_logger().info(
            "DLIO input filter 已启动, "
            f"pointcloud={self.input_pointcloud_topic}"
            f"->{self.output_pointcloud_topic}, "
            f"imu={self.input_imu_topic}->{self.output_imu_topic}"
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        stamp_ns = stamp_nanoseconds(msg)
        if not is_strictly_new_stamp(stamp_ns, self.last_pointcloud_stamp_ns):
            self.pointcloud_dropped += 1
            return
        self.last_pointcloud_stamp_ns = stamp_ns
        if not self.healthy:
            return
        self.pointcloud_publisher.publish(msg)
        self.pointcloud_accepted += 1

    def _on_imu(self, msg: Imu) -> None:
        stamp_ns = stamp_nanoseconds(msg)
        if not is_strictly_new_stamp(stamp_ns, self.last_imu_stamp_ns):
            self.imu_dropped += 1
            return
        self.last_imu_stamp_ns = stamp_ns
        if not self.healthy:
            return
        self.imu_publisher.publish(msg)
        self.imu_accepted += 1

    def _on_health(self, msg: Bool) -> None:
        self.healthy = bool(msg.data)
        if self.healthy:
            return
        if not self._logged_unhealthy:
            self.get_logger().error(
                "DLIO 健康检查失败, 已停止转发 LiDAR 和 IMU, 需要重启链路"
            )
            self._logged_unhealthy = True

    def _report_diagnostics(self) -> None:
        self.get_logger().info(
            "DLIO 输入时间戳统计, "
            f"pointcloud accepted={self.pointcloud_accepted}, "
            f"dropped={self.pointcloud_dropped}, "
            f"imu accepted={self.imu_accepted}, dropped={self.imu_dropped}, "
            f"forwarding={self.healthy}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DlioInputFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
