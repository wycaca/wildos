from types import SimpleNamespace

import pytest

from graph_construction.pointcloud_axis_adapter import _axis_transform, _identity_cloud


def test_identity_cloud_preserves_message_when_frame_matches():
    cloud = SimpleNamespace(header=SimpleNamespace(frame_id="odom"))

    assert _identity_cloud(cloud, "odom") is cloud
    assert _identity_cloud(cloud, "") is cloud


def test_identity_cloud_only_relabels_a_copy():
    cloud = SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"))

    adapted = _identity_cloud(cloud, "odom")

    assert adapted is not cloud
    assert adapted.header.frame_id == "odom"
    assert cloud.header.frame_id == "camera_init"


def test_axis_transform_rejects_removed_legacy_modes():
    assert _axis_transform("isaac_lidar_to_base")((1.0, 2.0, 3.0)) == (-1.0, -2.0, 3.0)
    with pytest.raises(ValueError, match="Unsupported axis_mode"):
        _axis_transform("negate_xyz")
