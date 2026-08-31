from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def format_status(status: DiagnosticStatus) -> str:
    """只展开关键指标, 完整标量仍保留在 diagnostics 消息中"""
    values = {item.key: item.value for item in status.values}
    parts = [f"name={status.name}", f"level={status.level}"]
    for key in (
        "publish.rate_hz",
        "cycle.total.average_ms",
        "cycle.total.p95_ms",
        "cycle.total.maximum_ms",
        "workload.total_node_count",
        "workload.total_edge_count",
    ):
        if key in values:
            parts.append(f"{key}={values[key]}")
    return ", ".join(parts)


class PipelinePerformanceMonitor(Node):
    """独立汇总核心进程发布的轻量 diagnostics"""

    def __init__(self) -> None:
        super().__init__("pipeline_performance_monitor")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        topic = str(self.get_parameter("diagnostics_topic").value)
        self.create_subscription(
            DiagnosticArray,
            topic,
            self._on_diagnostics,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.get_logger().info(f"性能监控已启动, diagnostics={topic}")

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            self.get_logger().info(format_status(status))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PipelinePerformanceMonitor()
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
