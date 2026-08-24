from __future__ import annotations

import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from wildos_navigation.velocity_protocol import clamp_velocity, decode_velocity


class VelocityReceiver(Node):
    """在相机 AGX 校验远端速度并发布给本地 GO2 motion 节点"""

    def __init__(self) -> None:
        super().__init__("wildos_velocity_receiver")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("bind_ip", "192.168.50.2")
        self.declare_parameter("bind_port", 9999)
        self.declare_parameter("expected_source_ip", "192.168.50.1")
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("max_linear_x", 0.2)
        self.declare_parameter("max_linear_y", 0.0)
        self.declare_parameter("max_angular_z", 0.5)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.bind_address = (
            str(self.get_parameter("bind_ip").value),
            int(self.get_parameter("bind_port").value),
        )
        self.expected_source_ip = str(
            self.get_parameter("expected_source_ip").value
        )
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.velocity_limits = (
            float(self.get_parameter("max_linear_x").value),
            float(self.get_parameter("max_linear_y").value),
            float(self.get_parameter("max_angular_z").value),
        )
        if not 1 <= self.bind_address[1] <= 65535:
            raise ValueError("bind_port must be between 1 and 65535")
        if self.command_timeout_sec <= 0.0:
            raise ValueError("command_timeout_sec must be greater than 0")
        clamp_velocity((0.0, 0.0, 0.0), *self.velocity_limits)

        self.publisher = self.create_publisher(Twist, self.output_topic, 1)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind(self.bind_address)
        self._last_command_monotonic: float | None = None
        self._timeout_stop_published = False
        self.create_timer(0.01, self._poll)
        self.get_logger().info(
            f"GO2 速度网关已启动, bind={self.bind_address[0]}:{self.bind_address[1]}, "
            f"source={self.expected_source_ip}, output={self.output_topic}"
        )

    def _poll(self) -> None:
        """清空接收队列并只发布最新合法速度, 断流时主动归零"""
        latest_velocity: tuple[float, float, float] | None = None
        # 限制单次排空数量, 避免异常流量长期占用 ROS executor
        for _ in range(64):
            try:
                packet, source = self.socket.recvfrom(128)
            except BlockingIOError:
                break
            if source[0] != self.expected_source_ip:
                self.get_logger().warning(
                    f"拒绝未知速度源, source={source[0]}",
                    throttle_duration_sec=5.0,
                )
                continue
            try:
                latest_velocity = clamp_velocity(
                    decode_velocity(packet),
                    *self.velocity_limits,
                )
            except ValueError as exc:
                self.get_logger().warning(
                    f"拒绝损坏速度报文, error={exc}",
                    throttle_duration_sec=5.0,
                )
        now = time.monotonic()
        if latest_velocity is not None:
            self._publish(latest_velocity)
            self._last_command_monotonic = now
            self._timeout_stop_published = False
            return
        if (
            self._last_command_monotonic is not None
            and now - self._last_command_monotonic > self.command_timeout_sec
            and not self._timeout_stop_published
        ):
            self._publish((0.0, 0.0, 0.0))
            self._timeout_stop_published = True
            self.get_logger().warning("远端速度断流, 已发布零速度")

    def _publish(self, velocity: tuple[float, float, float]) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self._publish((0.0, 0.0, 0.0))
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityReceiver()
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
