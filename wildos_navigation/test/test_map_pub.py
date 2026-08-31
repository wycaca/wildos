import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from wildos_navigation.map_pub import (
    LocalObstacleGridNode,
    _map_diagnostic_metrics,
)
from wildos_navigation.performance_stats import TimingSummary


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _NonIterablePoints(np.ndarray):
    def __iter__(self):
        raise AssertionError("点云解码结果不能经过 Python 逐点迭代")


def _node():
    node = LocalObstacleGridNode.__new__(LocalObstacleGridNode)
    node.width_cells = 5
    node.height_cells = 5
    node.center_u = 2
    node.center_v = 2
    node.resolution = 0.1
    node.min_height = 0.1
    node.max_height = 1.0
    node.obstacle_radius = 0.0
    node.obstacle_height_threshold = 0.25
    node.odom_frame_id = "odom"
    node.odom_data = Odometry()
    node.odom_data.pose.pose.position.x = 0.25
    node.odom_data.pose.pose.position.y = 0.25
    node.grid_combined = OccupancyGrid()
    node.grid_combined.info.width = node.width_cells
    node.grid_combined.info.height = node.height_cells
    node.grid_combined.info.resolution = node.resolution
    node.grid_combined_pub = _Publisher()
    return node


def _assert_grid_contract(result):
    grid = np.asarray(result.data, dtype=np.int16).reshape(5, 5)
    assert result.header.stamp.sec == 7
    assert result.header.frame_id == "odom"
    assert result.info.origin.position.x == 0.0
    assert result.info.origin.position.y == 0.0
    assert grid[0, 0] == 100
    assert np.all(grid[1:4, 1:4] == 0)


def test_map_pub_uses_structured_array_without_python_point_iteration(monkeypatch):
    points = np.array(
        [
            (0.01, 0.01, 0.2),
            (0.01, 0.01, 0.6),
            (0.4, 0.4, 0.2),
        ],
        dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
    ).view(_NonIterablePoints)
    monkeypatch.setattr(
        "wildos_navigation.map_pub.pc2.read_points",
        lambda *_args, **_kwargs: points,
    )
    node = _node()
    cloud = PointCloud2(header=Header(stamp=Time(sec=7), frame_id="odom"))

    node.pointcloud_callback(cloud)

    assert len(node.grid_combined_pub.messages) == 1
    _assert_grid_contract(node.grid_combined_pub.messages[0])


def test_map_pub_reads_real_pointcloud2_message():
    node = _node()
    cloud = point_cloud2.create_cloud_xyz32(
        Header(stamp=Time(sec=7), frame_id="odom"),
        [
            (0.01, 0.01, 0.2),
            (0.01, 0.01, 0.6),
            (0.4, 0.4, 0.2),
        ],
    )

    node.pointcloud_callback(cloud)

    assert len(node.grid_combined_pub.messages) == 1
    _assert_grid_contract(node.grid_combined_pub.messages[0])


def test_map_diagnostics_report_stages_and_workload():
    summaries = {
        stage: TimingSummary(3, 5.0, 8.0, 9.0)
        for stage in ("total", "decode", "filter", "project", "grid", "publish")
    }

    metrics = _map_diagnostic_metrics(
        summaries,
        cloud_rate=10.0,
        counters={"input_points": 200_000, "published_grids": 3},
        cloud_age_ms=12.0,
    )

    assert metrics["cycle.total.p95_ms"] == 8.0
    assert metrics["stage.grid.average_ms"] == 5.0
    assert metrics["workload.input_points"] == 200_000
    assert metrics["input.cloud.age_ms"] == 12.0
