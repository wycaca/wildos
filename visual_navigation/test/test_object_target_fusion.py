import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import MultiArrayDimension
from visualization_msgs.msg import Marker

from object_search_msgs.msg import ObjectMaskWithTf
from triangulation3d.target_particle_filter import TargetEstimate
from visual_navigation.explorfm_triangulation.obj_mask_triangulation import (
    _mask_array,
    _target_surface_measurement,
    _target_marker,
)
from visual_navigation.utils.object_search_utils import reference_image_stamp


def test_mask_array_decodes_object_mask_message():
    """融合节点必须按 ObjectMaskWithTf 中的 B,1,H,W 布局恢复 Mask"""
    expected = np.arange(24, dtype=np.uint8).reshape(3, 1, 2, 4)
    msg = ObjectMaskWithTf()
    msg.object_mask.layout.dim = [
        MultiArrayDimension(label="batch", size=3, stride=24),
        MultiArrayDimension(label="channel", size=1, stride=8),
        MultiArrayDimension(label="height", size=2, stride=8),
        MultiArrayDimension(label="width", size=4, stride=4),
    ]
    msg.object_mask.data = expected.reshape(-1).tolist()

    decoded = _mask_array(msg)

    assert np.array_equal(decoded, expected)


def _estimate(
    stable: bool,
    covariance: np.ndarray,
    state: str | None = None,
) -> TargetEstimate:
    return TargetEstimate(
        position=np.array([8.0, 2.0, 1.0]),
        covariance=covariance,
        confidence=0.8,
        source=1,
        stable=stable,
        accepted_views=2,
        lidar_support=0,
        state=state or ("STABLE_VISION" if stable else "PENDING"),
    )


def test_pending_estimate_deletes_misleading_position_marker():
    marker = _target_marker(_estimate(False, np.eye(3) * 100.0), "odom", Time())

    assert marker.action == Marker.DELETE


def test_stable_marker_scale_is_bounded():
    covariance = np.diag([100.0, 4.0, 0.25])
    marker = _target_marker(_estimate(True, covariance), "odom", Time())

    assert marker.action == Marker.ADD
    assert marker.pose.position.x == 8.0
    assert 0.3 <= marker.scale.x <= 1.5
    assert 0.3 <= marker.scale.y <= 1.5
    assert 0.3 <= marker.scale.z <= 1.5


def test_two_view_tracking_estimate_publishes_coarse_marker():
    marker = _target_marker(
        _estimate(False, np.eye(3) * 16.0, state="TRACKING"),
        "odom",
        Time(),
    )

    assert marker.action == Marker.ADD
    assert marker.color.r == 1.0
    assert marker.color.g == 0.75


def test_reference_image_stamp_uses_middle_camera_time():
    image_msgs = [Image(), Image(), Image()]
    image_msgs[0].header.stamp = Time(sec=10, nanosec=100)
    image_msgs[1].header.stamp = Time(sec=12, nanosec=100)
    image_msgs[2].header.stamp = Time(sec=11, nanosec=100)

    stamp = reference_image_stamp(image_msgs)

    assert stamp.sec == 11
    assert stamp.nanosec == 100


def test_lidar_measurement_rejects_ground_points_inside_mask():
    rng = np.random.default_rng(7)
    ground = np.column_stack(
        (
            rng.normal(0.0, 0.35, 45),
            rng.normal(-7.0, 0.35, 45),
            rng.normal(0.0, 0.015, 45),
        )
    )
    target = np.column_stack(
        (
            rng.normal(0.0, 0.18, 24),
            rng.normal(-9.0, 0.18, 24),
            rng.normal(0.55, 0.08, 24),
        )
    )

    measurement = _target_surface_measurement(
        np.vstack((ground, target)),
        minimum_support=30,
    )

    assert measurement is not None
    position, support = measurement
    assert support >= 10
    assert np.linalg.norm(position[:2] - np.array([0.0, -9.0])) < 0.5
    assert position[2] > 0.3
