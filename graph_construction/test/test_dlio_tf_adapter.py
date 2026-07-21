import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
from tf2_msgs.msg import TFMessage

from graph_construction.dlio_tf_adapter import (
    HealthStateFilter,
    align_odometry,
    alignment_from_odometry,
    alignment_to_transform,
    is_odometry_healthy,
    odom_to_transform,
    relay_extrinsic_transforms,
)


def test_health_filter_ignores_single_bad_sample():
    health_filter = HealthStateFilter(
        unhealthy_confirm_frames=3,
        healthy_confirm_frames=2,
    )

    assert health_filter.update(True) is True
    assert health_filter.update(False) is None
    assert health_filter.update(True) is None
    assert health_filter.state is True


def test_health_filter_requires_stable_failure_and_recovery():
    health_filter = HealthStateFilter(
        unhealthy_confirm_frames=2,
        healthy_confirm_frames=3,
    )

    assert health_filter.update(True) is True
    assert health_filter.update(False) is None
    assert health_filter.update(False) is False
    assert health_filter.update(True) is None
    assert health_filter.update(False) is None
    assert health_filter.update(True) is None
    assert health_filter.update(True) is None
    assert health_filter.update(True) is True


def test_health_filter_immediately_rejects_nonfinite_state():
    health_filter = HealthStateFilter(5, 2)

    assert health_filter.update(True) is True
    assert health_filter.update(False, force_unhealthy=True) is False


def test_odom_to_transform_preserves_pose_and_stamp():
    odom = Odometry()
    odom.header.stamp.sec = 12
    odom.pose.pose.position.x = 1.2
    odom.pose.pose.position.y = -3.4
    odom.pose.pose.orientation.w = 1.0

    transform = odom_to_transform(odom, "odom_3D", "base_link")

    assert transform.header.stamp.sec == 12
    assert transform.header.frame_id == "odom_3D"
    assert transform.child_frame_id == "base_link"
    assert transform.transform.translation.x == 1.2
    assert transform.transform.translation.y == -3.4
    assert transform.transform.rotation.w == 1.0


def test_relay_extrinsics_drops_stale_dlio_pose_tf():
    pose_tf = _transform("dlio_odom", "base_link")
    lidar_tf = _transform("base_link", "livox_frame")

    output = relay_extrinsic_transforms(
        TFMessage(transforms=[pose_tf, lidar_tf]),
        "dlio_odom",
        "base_link",
    )

    assert len(output.transforms) == 1
    assert output.transforms[0].header.frame_id == "base_link"
    assert output.transforms[0].child_frame_id == "livox_frame"


def test_alignment_anchors_first_dlio_pose_to_reference_pose():
    reference = _odom(2.0, -1.0, 0.3, 90.0)
    local = _odom(0.2, 0.1, 0.0, 15.0)

    alignment = alignment_from_odometry(reference, local)
    aligned = align_odometry(local, alignment, "odom_3D", "base_link")

    assert aligned.header.frame_id == "odom_3D"
    assert aligned.child_frame_id == "base_link"
    assert aligned.pose.pose.position.x == pytest.approx(2.0)
    assert aligned.pose.pose.position.y == pytest.approx(-1.0)
    assert aligned.pose.pose.position.z == pytest.approx(0.3)
    assert _yaw_deg(aligned) == pytest.approx(90.0)


def test_alignment_rotates_later_dlio_motion_into_global_frame():
    reference = _odom(2.0, -1.0, 0.0, 90.0)
    local_initial = _odom(0.0, 0.0, 0.0, 0.0)
    local_later = _odom(1.0, 0.0, 0.0, 0.0)

    alignment = alignment_from_odometry(reference, local_initial)
    aligned = align_odometry(local_later, alignment, "odom_3D", "base_link")
    transform = alignment_to_transform(
        alignment,
        aligned.header.stamp,
        "odom_3D",
        "dlio_odom",
    )

    assert aligned.pose.pose.position.x == pytest.approx(2.0)
    assert aligned.pose.pose.position.y == pytest.approx(0.0)
    assert transform.header.frame_id == "odom_3D"
    assert transform.child_frame_id == "dlio_odom"


def test_health_check_accepts_small_simulation_error():
    aligned = _odom(1.1, 2.0, 0.2, 12.0)
    reference = _odom(1.0, 2.0, 0.2, 10.0)
    aligned.twist.twist.linear.x = 0.5

    healthy, metrics = is_odometry_healthy(
        aligned,
        reference,
        max_position_error=1.0,
        max_orientation_error_deg=20.0,
        max_linear_speed=5.0,
    )

    assert healthy
    assert metrics[0] == pytest.approx(0.1)
    assert metrics[1] == pytest.approx(2.0)
    assert metrics[2] == pytest.approx(0.5)


@pytest.mark.parametrize(
    "aligned_values,reference_values",
    [
        ((3.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0, 45.0), (0.0, 0.0, 0.0, 0.0)),
    ],
)
def test_health_check_rejects_pose_divergence(
    aligned_values,
    reference_values,
):
    aligned = _odom(*aligned_values)
    reference = _odom(*reference_values)
    healthy, _ = is_odometry_healthy(
        aligned,
        reference,
        max_position_error=1.0,
        max_orientation_error_deg=20.0,
        max_linear_speed=5.0,
    )

    assert not healthy


def test_health_check_rejects_excessive_speed():
    aligned = _odom(0.0, 0.0, 0.0, 0.0)
    aligned.twist.twist.linear.x = 6.0

    healthy, metrics = is_odometry_healthy(
        aligned,
        None,
        max_position_error=1.0,
        max_orientation_error_deg=20.0,
        max_linear_speed=5.0,
    )

    assert not healthy
    assert metrics[2] == pytest.approx(6.0)


def _odom(x: float, y: float, z: float, yaw_deg: float) -> Odometry:
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = z
    yaw = math.radians(yaw_deg)
    odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
    odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return odom


def _yaw_deg(odom: Odometry) -> float:
    orientation = odom.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )
    return math.degrees(yaw)


def _transform(parent: str, child: str) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.rotation.w = 1.0
    return transform
