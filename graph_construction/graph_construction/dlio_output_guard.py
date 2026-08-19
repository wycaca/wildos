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


class HealthLease:
    """使用 monotonic 时间限制健康状态有效期"""

    def __init__(self, timeout_sec: float) -> None:
        self.timeout_sec = float(timeout_sec)
        if self.timeout_sec <= 0.0:
            raise ValueError("health_timeout_sec must be greater than 0")
        self.healthy = False
        self.last_heartbeat_time: float | None = None
        self.transitions = 0
        self.timeouts = 0

    def update(self, healthy: bool, now: float) -> None:
        """更新最新心跳并记录状态切换"""
        self.is_healthy(now)
        previous = self.healthy
        self.last_heartbeat_time = float(now)
        self.healthy = bool(healthy)
        if self.healthy != previous:
            self.transitions += 1

    def is_healthy(self, now: float) -> bool:
        """租约过期时立即降为不健康"""
        if (
            self.healthy
            and self.last_heartbeat_time is not None
            and float(now) - self.last_heartbeat_time > self.timeout_sec
        ):
            self.healthy = False
            self.transitions += 1
            self.timeouts += 1
        return self.healthy

    def heartbeat_age(self, now: float) -> float | None:
        if self.last_heartbeat_time is None:
            return None
        return max(0.0, float(now) - self.last_heartbeat_time)


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
        self.declare_parameter("health_timeout_sec", 1.5)
        self.input_pointcloud_topic = str(
            self.get_parameter("input_pointcloud_topic").value
        )
        self.output_pointcloud_topic = str(
            self.get_parameter("output_pointcloud_topic").value
        )
        self.health_topic = str(self.get_parameter("health_topic").value)
        self.health_lease = HealthLease(
            float(self.get_parameter("health_timeout_sec").value)
        )
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        health_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
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
        self._callback_timing = TimingWindow()
        self._input_rate = EventRate()
        self.get_logger().info(
            "DLIO output guard 已启动, "
            f"pointcloud={self.input_pointcloud_topic}"
            f"->{self.output_pointcloud_topic}"
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        started = time.perf_counter()
        self._input_rate.tick()
        if not self.health_lease.is_healthy(time.monotonic()):
            self.suppressed += 1
            self._callback_timing.add_seconds(time.perf_counter() - started)
            return
        self.pointcloud_publisher.publish(msg)
        self.forwarded += 1
        self._callback_timing.add_seconds(time.perf_counter() - started)

    def _on_health(self, msg: Bool) -> None:
        self.health_lease.update(bool(msg.data), time.monotonic())

    def _report_diagnostics(self) -> None:
        now = time.monotonic()
        healthy = self.health_lease.is_healthy(now)
        heartbeat_age = self.health_lease.heartbeat_age(now)
        heartbeat_age_text = (
            "未收到" if heartbeat_age is None else f"{heartbeat_age:.2f}s"
        )
        forwarded = self.forwarded - self._last_forwarded
        suppressed = self.suppressed - self._last_suppressed
        total = forwarded + suppressed
        suppressed_ratio = 100.0 * suppressed / total if total else 0.0
        timing = self._callback_timing.summary(reset=True)
        self.get_logger().info(
            "DLIO 点云输出, "
            f"输入频率={self._input_rate.sample(reset=True):.1f}Hz, "
            f"转发={forwarded}, 拦截={suppressed}({suppressed_ratio:.1f}%), "
            f"状态={'正常' if healthy else '暂停'}, "
            f"心跳年龄={heartbeat_age_text}, "
            f"心跳超时={self.health_lease.timeouts}次, "
            f"状态切换={self.health_lease.transitions}次, "
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
