from pathlib import Path

import pytest
import yaml

from graph_construction.localization import resolve_localization_wiring


REPO_ROOT = Path(__file__).resolve().parents[2]


def _topic_profile_config():
    config_path = REPO_ROOT / "graph_construction" / "configs" / "topic_profiles.yaml"
    with config_path.open("r", encoding="utf-8") as config_stream:
        return yaml.safe_load(config_stream)


def _unity_profile():
    return _topic_profile_config()["profiles"]["unity"]


def _unity_dlio_config():
    config_path = REPO_ROOT / "graph_construction" / "configs" / "dlio" / "unity.yaml"
    with config_path.open("r", encoding="utf-8") as config_stream:
        return yaml.safe_load(config_stream)["/**"]["ros__parameters"]


def test_unity_platform_localization_keeps_existing_topics():
    wiring = resolve_localization_wiring(_unity_profile(), "platform")

    assert wiring.odom_input_topic == "/unity/odom"
    assert wiring.mapping_pointcloud_topic == "/livox/lidar_aligned"
    assert wiring.pointcloud_input_topic == "/livox/lidar"
    assert wiring.dlio_imu_input_topic == ""
    assert wiring.use_pointcloud_axis_adapter is True
    assert wiring.isolate_platform_tf is False


def test_unity_dlio_localization_uses_raw_lidar_and_imu():
    wiring = resolve_localization_wiring(_unity_profile(), "dlio")

    assert wiring.pointcloud_input_topic == "/livox/lidar"
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


def test_profiles_share_internal_topic_contract():
    config = _topic_profile_config()
    common_contract = config["common_contract"]
    internal_topic_keys = {
        "odom_output_topic",
        "elevation_grid_map_topic",
        "nav_graph_topic",
        "scored_nav_graph_topic",
        "planner_path_topic",
        "object_mask_topic",
        "object_target_estimate_topic",
        "object_search_completed_topic",
    }

    assert set(config["profiles"]) == {"isaac", "unity", "robot"}
    for profile in config["profiles"].values():
        assert {
            key: profile[key]
            for key in internal_topic_keys
        } == {
            key: common_contract[key]
            for key in internal_topic_keys
        }


def test_every_profile_declares_environment_inputs_and_frames():
    required_keys = {
        "ros_domain_id",
        "rmw_implementation",
        "global_frame",
        "base_frame",
        "lidar_frame",
        "pointcloud_input_topic",
        "aligned_lidar_topic",
        "odom_input_topic",
        "camera_img_topic",
        "camera_info_topic",
        "camera_stamp_mode",
    }

    for profile in _topic_profile_config()["profiles"].values():
        assert required_keys <= set(profile)
        assert all(str(profile[key]).strip() for key in required_keys)


def test_unity_planner_publishes_source_path_topic():
    assert _unity_profile()["planner_path_topic"] == "/spot1/graphnav_planner/path"


def test_robot_profile_preserves_platform_odometry_and_registered_cloud_frame():
    robot = _topic_profile_config()["profiles"]["robot"]

    assert robot["pointcloud_input_topic"] == "/cloud_registered"
    assert robot["aligned_lidar_topic"] == "/spot1/cloud_registered"
    assert robot["pointcloud_output_frame"] == "odom_3D"
    assert robot["odom_stamp_mode"] == "preserve"
    assert robot["odom_pose_source"] == "message"


def test_unity_dlio_disables_unstable_adaptive_gicp():
    config = _unity_dlio_config()

    assert config["adaptive"] is False
    assert config["pointcloud/deskew"] is False
    assert "map/sparse/leafSize" not in config


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
