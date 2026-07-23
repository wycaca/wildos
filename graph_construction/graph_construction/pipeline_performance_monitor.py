from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from graphnav_msgs.msg import NavigationGraph
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry, Path
from object_search_msgs.msg import ObjectMaskWithTf, TargetEstimate
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from graph_construction.performance_stats import EventRate, TimingWindow


_DIAGNOSTICS_LOG_PERIOD_SEC = 30.0


@dataclass
class TopicMetrics:
    rate: EventRate
    age: TimingWindow
    interval_count: int = 0


class PipelinePerformanceMonitor(Node):
    """Observe cross-node rates and source-stamp age without changing the pipeline"""

    TOPICS = (
        ("lidar", "raw_lidar_topic", "/livox/lidar", PointCloud2, True),
        ("imu", "raw_imu_topic", "/livox/imu", Imu, True),
        ("cloud", "aligned_pointcloud_topic", "/livox/lidar_aligned", PointCloud2, True),
        ("odom", "odom_topic", "/spot1/odom_for_scoring", Odometry, False),
        ("map", "grid_map_topic", "/elevation_mapping_node/elevation_map_raw", GridMap, False),
        ("graph", "nav_graph_topic", "/spot1/nav_graph", NavigationGraph, False),
        ("scored", "scored_nav_graph_topic", "/spot1/scored_nav_graph", NavigationGraph, False),
        ("mask", "object_mask_topic", "/spot1/object_mask", ObjectMaskWithTf, False),
        ("target", "target_estimate_topic", "/spot1/object_target_estimate", TargetEstimate, False),
        ("goal", "goal_topic", "/spot1/goal_pose", PoseStamped, False),
        ("path", "path_topic", "/spot1/path", Path, False),
    )
    TOPIC_LABELS = {
        "lidar": "雷达",
        "imu": "IMU",
        "cloud": "对齐点云",
        "odom": "里程计",
        "map": "高程图",
        "graph": "导航图",
        "scored": "视觉评分图",
        "mask": "目标Mask",
        "target": "融合目标",
        "goal": "规划目标",
        "path": "规划路径",
    }

    def __init__(self) -> None:
        super().__init__("pipeline_performance_monitor")
        self._metrics = {
            name: TopicMetrics(EventRate(), TimingWindow())
            for name, _, _, _, _ in self.TOPICS
        }
        for name, parameter, default_topic, message_type, sensor_qos in self.TOPICS:
            self.declare_parameter(parameter, default_topic)
            topic = str(self.get_parameter(parameter).value)
            if not topic:
                continue
            qos = qos_profile_sensor_data if sensor_qos else 10
            self.create_subscription(
                message_type,
                topic,
                lambda msg, metric_name=name: self._on_message(metric_name, msg),
                qos,
            )
        self.create_timer(_DIAGNOSTICS_LOG_PERIOD_SEC, self._report)
        self.get_logger().info(
            f"链路性能监测已启动, period={_DIAGNOSTICS_LOG_PERIOD_SEC:.0f}s"
        )

    def _on_message(self, name: str, msg) -> None:
        metrics = self._metrics[name]
        metrics.rate.tick()
        metrics.interval_count += 1
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns > 0 and now_ns >= stamp_ns:
            metrics.age.add_seconds((now_ns - stamp_ns) / 1.0e9)

    def _report(self) -> None:
        """Emit two compact lines only for active topics"""
        samples = {}
        for name, metrics in self._metrics.items():
            rate = metrics.rate.sample(reset=True)
            age = metrics.age.summary(reset=True)
            if metrics.interval_count:
                samples[name] = (rate, age)
            metrics.interval_count = 0
        if not samples:
            return
        rates = [
            f"{self.TOPIC_LABELS[name]}={rate:.1f}Hz"
            for name, (rate, _) in samples.items()
        ]
        ages = []
        for name, (_, summary) in samples.items():
            if summary.count:
                ages.append(
                    f"{self.TOPIC_LABELS[name]}={summary.average_ms:.0f}/"
                    f"{summary.p95_ms:.0f}/{summary.maximum_ms:.0f}ms"
                )
        self.get_logger().info("链路频率, " + ", ".join(rates))
        if ages:
            self.get_logger().info(
                "链路延迟(平均/95%上限/最大), " + ", ".join(ages)
            )


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
