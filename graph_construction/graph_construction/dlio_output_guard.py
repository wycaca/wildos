from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class DlioOutputGuard(Node):
    """Forward DLIO point clouds only while aligned odometry is healthy"""

    def __init__(self) -> None:
        super().__init__("dlio_output_guard")
        self.declare_parameter(
            "input_pointcloud_topic",
            "/spot1/dlio/odom_node/pointcloud/deskewed_raw",
        )
        self.declare_parameter(
            "output_pointcloud_topic",
            "/spot1/dlio/odom_node/pointcloud/deskewed",
        )
        self.declare_parameter(
            "health_topic",
            "/spot1/dlio/odom_node/healthy",
        )
        self.declare_parameter("diagnostic_interval", 10.0)

        self.input_pointcloud_topic = str(
            self.get_parameter("input_pointcloud_topic").value
        )
        self.output_pointcloud_topic = str(
            self.get_parameter("output_pointcloud_topic").value
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
        self.create_subscription(
            PointCloud2,
            self.input_pointcloud_topic,
            self._on_pointcloud,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            self.health_topic,
            self._on_health,
            health_qos,
        )
        self.create_timer(diagnostic_interval, self._report_diagnostics)

        self.forwarded = 0
        self.suppressed = 0
        self.healthy = False
        self.get_logger().info(
            "DLIO output guard 已启动, "
            f"pointcloud={self.input_pointcloud_topic}"
            f"->{self.output_pointcloud_topic}"
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        if not self.healthy:
            self.suppressed += 1
            return
        self.pointcloud_publisher.publish(msg)
        self.forwarded += 1

    def _on_health(self, msg: Bool) -> None:
        previous = self.healthy
        self.healthy = bool(msg.data)
        if self.healthy and not previous:
            self.get_logger().info("DLIO 位姿健康, 开始转发对齐点云")
        elif previous and not self.healthy:
            self.get_logger().error("DLIO 位姿异常, 已暂停转发对齐点云")

    def _report_diagnostics(self) -> None:
        self.get_logger().info(
            "DLIO 输出点云统计, "
            f"forwarded={self.forwarded}, suppressed={self.suppressed}, "
            f"healthy={self.healthy}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DlioOutputGuard()
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
