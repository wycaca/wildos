from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

from graph_construction.performance_stats import EventRate, TimingWindow


_DIAGNOSTICS_LOG_PERIOD_SEC = 30.0


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
        self.input_pointcloud_topic = str(
            self.get_parameter("input_pointcloud_topic").value
        )
        self.output_pointcloud_topic = str(
            self.get_parameter("output_pointcloud_topic").value
        )
        self.health_topic = str(self.get_parameter("health_topic").value)
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        self.create_timer(_DIAGNOSTICS_LOG_PERIOD_SEC, self._report_diagnostics)

        self.forwarded = 0
        self.suppressed = 0
        self._last_forwarded = 0
        self._last_suppressed = 0
        self._health_transitions = 0
        self._callback_timing = TimingWindow()
        self._input_rate = EventRate()
        self.healthy = False
        self.get_logger().info(
            "DLIO output guard 已启动, "
            f"pointcloud={self.input_pointcloud_topic}"
            f"->{self.output_pointcloud_topic}"
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        started = time.perf_counter()
        self._input_rate.tick()
        if not self.healthy:
            self.suppressed += 1
            self._callback_timing.add_seconds(time.perf_counter() - started)
            return
        self.pointcloud_publisher.publish(msg)
        self.forwarded += 1
        self._callback_timing.add_seconds(time.perf_counter() - started)

    def _on_health(self, msg: Bool) -> None:
        previous = self.healthy
        self.healthy = bool(msg.data)
        if self.healthy != previous:
            self._health_transitions += 1

    def _report_diagnostics(self) -> None:
        forwarded = self.forwarded - self._last_forwarded
        suppressed = self.suppressed - self._last_suppressed
        total = forwarded + suppressed
        suppressed_ratio = 100.0 * suppressed / total if total else 0.0
        timing = self._callback_timing.summary(reset=True)
        self.get_logger().info(
            "DLIO 点云输出, "
            f"输入频率={self._input_rate.sample(reset=True):.1f}Hz, "
            f"转发={forwarded}, 拦截={suppressed}({suppressed_ratio:.1f}%), "
            f"状态={'正常' if self.healthy else '暂停'}, "
            f"状态切换={self._health_transitions}次, "
            f"回调耗时=平均{timing.average_ms:.1f}/95%上限{timing.p95_ms:.1f}/"
            f"最大{timing.maximum_ms:.1f}ms"
        )
        self._last_forwarded = self.forwarded
        self._last_suppressed = self.suppressed


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
