from __future__ import annotations

import copy
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
        self.declare_parameter("output_frame", "dlio_odom")
        self.declare_parameter("max_output_rate_hz", 2.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
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
            f"output={self.output_topic}, frame={self.output_frame}, "
            f"max_rate={self.max_output_rate_hz:.1f} Hz"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        """限制发布频率并保留点云的全部字段"""
        now_ns = time.monotonic_ns()
        if not _publish_due(
            now_ns,
            self._last_publish_time_ns,
            self.max_output_rate_hz,
        ):
            return
        self._last_publish_time_ns = now_ns
        relayed_msg = _relay_cloud(msg, self.output_frame)
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


def _relay_cloud(cloud: PointCloud2, output_frame: str) -> PointCloud2:
    """仅在 frame 变化时复制消息, 避免遍历高密度点云"""
    target_frame = str(output_frame).strip() or cloud.header.frame_id
    if target_frame == cloud.header.frame_id:
        return cloud
    adapted = copy.deepcopy(cloud)
    adapted.header.frame_id = target_frame
    return adapted


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
