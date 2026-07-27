from types import SimpleNamespace

from graph_construction.pointcloud_axis_adapter import _identity_cloud


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
