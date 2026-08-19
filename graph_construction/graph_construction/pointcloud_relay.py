from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudRelay(Node):
    """将跨机注册点云降频后转发给本机高开销节点"""

    def __init__(self) -> None:
        super().__init__("pointcloud_relay")

        self.declare_parameter("input_topic", "/cloud_registered")
        self.declare_parameter("output_topic", "/spot1/cloud_registered_local")
        self.declare_parameter("expected_frame", "dlio_odom")
        self.declare_parameter("max_output_rate_hz", 2.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.expected_frame = self.get_parameter("expected_frame").value
        self.max_output_rate_hz = float(
            self.get_parameter("max_output_rate_hz").value
        )
        self._last_publish_time_ns = None
        self._logged_first_cloud = False

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            output_qos,
        )
        self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._on_cloud,
            input_qos,
        )

        self.get_logger().info(
            f"PointCloud relay started, input={self.input_topic}, "
            f"output={self.output_topic}, expected_frame={self.expected_frame}, "
            f"max_rate={self.max_output_rate_hz:.1f} Hz"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        """限制发布频率并保留点云的全部字段"""
        relayed_msg = _relay_cloud(msg, self.expected_frame)
        if relayed_msg is None:
            self.get_logger().warning(
                "Dropped point cloud with unexpected frame, "
                f"expected={self.expected_frame}, actual={msg.header.frame_id}",
                throttle_duration_sec=10.0,
            )
            return
        now_ns = time.monotonic_ns()
        if not _publish_due(
            now_ns,
            self._last_publish_time_ns,
            self.max_output_rate_hz,
        ):
            return
        self._last_publish_time_ns = now_ns
        self.publisher.publish(relayed_msg)

        if not self._logged_first_cloud:
            self.get_logger().info(
                "Published first relayed cloud, "
                f"input_frame={msg.header.frame_id}, "
                f"output_frame={relayed_msg.header.frame_id}"
            )
            self._logged_first_cloud = True


def _publish_due(now_ns: int, last_ns: int | None, max_rate_hz: float) -> bool:
    """限制高开销下游的点云输入速率"""
    if max_rate_hz <= 0.0 or last_ns is None:
        return True
    return now_ns - last_ns >= 1.0e9 / max_rate_hz


def _relay_cloud(
    cloud: PointCloud2,
    expected_frame: str,
) -> PointCloud2 | None:
    """仅转发坐标系符合契约的原消息"""
    frame = str(expected_frame).strip()
    if frame and frame != cloud.header.frame_id:
        return None
    return cloud


def main(args=None) -> None:
    """ROS2 控制台脚本入口"""
    rclpy.init(args=args)
    node = PointCloudRelay()
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
