from pathlib import Path

import pytest
import yaml

from graph_construction.localization import resolve_localization_wiring


REPO_ROOT = Path(__file__).resolve().parents[2]


def _unity_profile():
    config_path = REPO_ROOT / "graph_construction" / "configs" / "topic_profiles.yaml"
    with config_path.open("r", encoding="utf-8") as config_stream:
        return yaml.safe_load(config_stream)["profiles"]["unity"]


def _unity_dlio_config():
    config_path = REPO_ROOT / "graph_construction" / "configs" / "dlio" / "unity.yaml"
    with config_path.open("r", encoding="utf-8") as config_stream:
        return yaml.safe_load(config_stream)["/**"]["ros__parameters"]


def test_unity_platform_localization_keeps_existing_topics():
    wiring = resolve_localization_wiring(_unity_profile(), "platform")

    assert wiring.odom_input_topic == "/unity/odom"
    assert wiring.mapping_pointcloud_topic == "/livox/lidar_aligned"
    assert wiring.dlio_pointcloud_input_topic == "/livox/lidar"
    assert wiring.dlio_imu_input_topic == "/livox/imu"
    assert wiring.use_pointcloud_axis_adapter is True
    assert wiring.isolate_platform_tf is False


def test_unity_dlio_localization_uses_raw_lidar_and_imu():
    wiring = resolve_localization_wiring(_unity_profile(), "dlio")

    assert wiring.dlio_pointcloud_input_topic == "/livox/lidar"
    assert wiring.dlio_imu_input_topic == "/livox/imu"
    assert wiring.odom_input_topic == "/spot1/dlio/odom_node/odom"
    assert wiring.dlio_aligned_odom_topic == "/spot1/dlio/odom_node/aligned_odom"
    assert wiring.dlio_reference_odom_topic == "/unity/odom"
    assert wiring.dlio_local_frame == "dlio_odom"
    assert wiring.dlio_alignment_delay == pytest.approx(3.2)
    assert wiring.mapping_pointcloud_topic == "/spot1/dlio/odom_node/pointcloud/deskewed"
    assert wiring.use_pointcloud_axis_adapter is False
    assert wiring.isolate_platform_tf is True


def test_unity_profile_normalizes_camera_stamps():
    assert _unity_profile()["camera_stamp_mode"] == "now"


def test_unity_dlio_disables_unstable_adaptive_gicp():
    config = _unity_dlio_config()

    assert config["adaptive"] is False
    assert config["pointcloud/deskew"] is False


def test_unity_dlio_uses_gravity_alignment_and_bounded_accel_bias():
    config = _unity_dlio_config()

    assert config["odom/publishRate"] == 20.0
    assert config["odom/imu/approximateGravity"] is True
    assert config["odom/geo/abias_max"] == 1.0


def test_unity_dlio_matches_unity_livox_extrinsics():
    config = _unity_dlio_config()

    expected_translation = [0.093, 0.0, 0.334]
    expected_rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert config["extrinsics/baselink2imu/t"] == expected_translation
    assert config["extrinsics/baselink2lidar/t"] == expected_translation
    assert config["extrinsics/baselink2imu/R"] == expected_rotation
    assert config["extrinsics/baselink2lidar/R"] == expected_rotation
    assert config["frames/imu"] == "livox_frame"
    assert config["frames/lidar"] == "livox_frame"


def test_unknown_localization_backend_is_rejected():
    with pytest.raises(ValueError, match="Unsupported localization_backend"):
        resolve_localization_wiring(_unity_profile(), "unknown")
