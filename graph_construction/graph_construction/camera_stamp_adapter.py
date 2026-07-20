from __future__ import annotations

from functools import partial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage


CAMERA_NAMES = ("front", "left", "right")


def replace_header_stamp(msg, stamp):
    """Replace a sensor header stamp without changing payload or frame"""
    msg.header.stamp = stamp
    return msg


class CameraStampAdapter(Node):
    """Move Unity camera headers onto the active ROS clock"""

    def __init__(self) -> None:
        super().__init__("camera_stamp_adapter")
        self.declare_parameter(
            "input_image_topic_template",
            "/camera/{}/color/image/compressed",
        )
        self.declare_parameter(
            "input_info_topic_template",
            "/camera/{}/color/camera_info",
        )
        self.declare_parameter(
            "output_image_topic_template",
            "/spot1/camera_synced/{}/color/image/compressed",
        )
        self.declare_parameter(
            "output_info_topic_template",
            "/spot1/camera_synced/{}/color/camera_info",
        )

        input_image_template = str(
            self.get_parameter("input_image_topic_template").value
        )
        input_info_template = str(
            self.get_parameter("input_info_topic_template").value
        )
        output_image_template = str(
            self.get_parameter("output_image_topic_template").value
        )
        output_info_template = str(
            self.get_parameter("output_info_topic_template").value
        )
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._camera_publishers = []
        self._camera_subscriptions = []
        for camera_name in CAMERA_NAMES:
            image_publisher = self.create_publisher(
                CompressedImage,
                output_image_template.format(camera_name),
                qos,
            )
            info_publisher = self.create_publisher(
                CameraInfo,
                output_info_template.format(camera_name),
                qos,
            )
            self._camera_publishers.extend([image_publisher, info_publisher])
            self._camera_subscriptions.extend(
                [
                    self.create_subscription(
                        CompressedImage,
                        input_image_template.format(camera_name),
                        partial(self._restamp, publisher=image_publisher),
                        qos,
                    ),
                    self.create_subscription(
                        CameraInfo,
                        input_info_template.format(camera_name),
                        partial(self._restamp, publisher=info_publisher),
                        qos,
                    ),
                ]
            )

        self.forwarded = 0
        self._logged_first = False
        self.get_logger().info(
            "Unity camera stamp adapter 已启动, "
            f"images={input_image_template}->{output_image_template}, "
            f"info={input_info_template}->{output_info_template}"
        )

    def _restamp(self, msg, publisher) -> None:
        source_stamp = msg.header.stamp
        target_stamp = self.get_clock().now().to_msg()
        publisher.publish(replace_header_stamp(msg, target_stamp))
        self.forwarded += 1
        if not self._logged_first:
            source = source_stamp.sec + source_stamp.nanosec / 1.0e9
            target = target_stamp.sec + target_stamp.nanosec / 1.0e9
            self.get_logger().info(
                "已重打第一帧 Unity 相机时间戳, "
                f"source={source:.3f}s, target={target:.3f}s"
            )
            self._logged_first = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraStampAdapter()
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
