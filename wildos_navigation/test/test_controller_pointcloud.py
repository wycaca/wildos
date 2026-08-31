import random

import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from wildos_navigation.controller_node import AdvancedGeometricFollower


class _NonIterablePoints(np.ndarray):
    def __iter__(self):
        raise AssertionError("控制器点云不能经过 Python 逐点迭代")


class _Tracker:
    def __init__(self, velocity=0.0):
        self.is_initialized = True
        self.updates = []
        self.velocity = velocity

    def update(self, position):
        self.updates.append(np.asarray(position))
        return np.array([position[0], position[1], self.velocity, 0.0])


def _controller(tracker=None):
    node = AdvancedGeometricFollower.__new__(AdvancedGeometricFollower)
    node.latest_pose = [0.0, 0.0, 0.0]
    node.real_obstacles = []
    node.predicted_dynamic_obs = []
    node.kf_tracker = tracker if tracker is not None else _Tracker()
    return node


def test_controller_filters_and_limits_structured_pointcloud(monkeypatch):
    valid_x = np.linspace(0.5, 3.0, 130, dtype=np.float32)
    valid = np.column_stack(
        (valid_x, np.zeros_like(valid_x), np.full_like(valid_x, 0.2))
    )
    rejected = np.array(
        [
            [0.2, 0.0, 0.2],
            [1.0, 0.0, 0.1],
            [4.1, 0.0, 0.2],
        ],
        dtype=np.float32,
    )
    xyz = np.vstack((valid, rejected))
    points = np.empty(
        len(xyz),
        dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
    )
    points["x"], points["y"], points["z"] = xyz.T
    points = points.view(_NonIterablePoints)
    monkeypatch.setattr(
        "wildos_navigation.controller_node.pc2.read_points",
        lambda *_args, **_kwargs: points,
    )
    node = _controller()
    random.seed(7)

    node.scan_callback(object())

    obstacles = np.asarray(node.real_obstacles)
    assert obstacles.shape == (35, 2)
    assert np.all(obstacles[:, 0] > 0.42)
    assert np.all(obstacles[:, 0] < 4.0)
    assert np.all(np.diff(obstacles[:, 0]) >= 0.0)


def test_controller_preserves_dynamic_obstacle_prediction():
    tracker = _Tracker(velocity=0.2)
    node = _controller(tracker)
    cloud = point_cloud2.create_cloud_xyz32(
        Header(frame_id="odom"),
        [
            (1.0, -0.05, 0.2),
            (1.0, 0.00, 0.2),
            (1.0, 0.05, 0.2),
            (1.0, 0.10, 0.2),
        ],
    )

    node.scan_callback(cloud)

    assert len(node.real_obstacles) == 4
    assert len(tracker.updates) == 4
    assert len(node.predicted_dynamic_obs) == 12
    assert np.allclose(
        node.predicted_dynamic_obs[:3],
        [[1.066, 0.025], [1.132, 0.025], [1.198, 0.025]],
    )


def test_controller_resets_tracker_when_cloud_has_no_obstacles():
    tracker = _Tracker()
    node = _controller(tracker)
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id="odom"), [])

    node.scan_callback(cloud)

    assert node.real_obstacles == []
    assert node.predicted_dynamic_obs == []
    assert not tracker.is_initialized
