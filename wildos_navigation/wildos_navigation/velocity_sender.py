from __future__ import annotations

import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from wildos_navigation.velocity_protocol import encode_velocity


class VelocitySender(Node):
    """将 x86 导航速度发送到相机 AGX 专用链路"""

    def __init__(self) -> None:
        super().__init__("wildos_velocity_sender")
        self.declare_parameter("input_topic", "/wildos/cmd_vel")
        self.declare_parameter("target_ip", "192.168.50.2")
        self.declare_parameter("target_port", 9999)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.target = (
            str(self.get_parameter("target_ip").value),
            int(self.get_parameter("target_port").value),
        )
        if not 1 <= self.target[1] <= 65535:
            raise ValueError("target_port must be between 1 and 65535")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Twist, self.input_topic, self._on_velocity, 1)
        self.get_logger().info(
            f"速度发送器已启动, input={self.input_topic}, "
            f"target={self.target[0]}:{self.target[1]}"
        )

    def _on_velocity(self, msg: Twist) -> None:
        try:
            packet = encode_velocity(msg.linear.x, msg.linear.y, msg.angular.z)
        except ValueError as exc:
            self.get_logger().error(f"拒绝非法速度, error={exc}")
            return
        try:
            self.socket.sendto(packet, self.target)
        except OSError as exc:
            self.get_logger().warning(
                f"速度链路发送失败, error={exc}",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self) -> bool:
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocitySender()
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
