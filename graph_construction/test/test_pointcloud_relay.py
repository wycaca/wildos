from types import SimpleNamespace

from graph_construction.pointcloud_relay import (
    _publish_due,
    _relay_cloud,
)


def test_identity_cloud_preserves_message_when_frame_matches():
    cloud = SimpleNamespace(header=SimpleNamespace(frame_id="odom"))

    assert _relay_cloud(cloud, "odom") is cloud
    assert _relay_cloud(cloud, "") is cloud


def test_identity_cloud_rejects_frame_mismatch():
    cloud = SimpleNamespace(header=SimpleNamespace(frame_id="camera_init"))

    adapted = _relay_cloud(cloud, "odom")

    assert adapted is None
    assert cloud.header.frame_id == "camera_init"


def test_output_rate_limit_keeps_only_due_clouds():
    assert _publish_due(1_000_000_000, None, 2.0)
    assert not _publish_due(1_499_999_999, 1_000_000_000, 2.0)
    assert _publish_due(1_500_000_000, 1_000_000_000, 2.0)
    assert _publish_due(1_000_000_001, 1_000_000_000, 0.0)
