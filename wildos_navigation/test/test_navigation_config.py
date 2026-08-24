from pathlib import Path

import yaml


def test_go2_topic_contract_and_safety_limits():
    config_file = Path(__file__).parents[1] / "config" / "navigation.yaml"
    config = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    controller = config["wildos_navigation_controller"]["ros__parameters"]
    sender = config["wildos_velocity_sender"]["ros__parameters"]
    receiver = config["wildos_velocity_receiver"]["ros__parameters"]

    assert controller["path_topic"] == "/spot1/graphnav_planner/path"
    assert controller["odom_topic"] == "/odom"
    assert controller["pointcloud_topic"] == "/cloud_registered"
    assert controller["cmd_vel_topic"] == "/wildos/cmd_vel"
    assert sender["input_topic"] == controller["cmd_vel_topic"]
    assert sender["target_ip"] == receiver["bind_ip"] == "192.168.50.2"
    assert receiver["expected_source_ip"] == "192.168.50.1"
    assert receiver["output_topic"] == "/cmd_vel"
    assert receiver["command_timeout_sec"] < 0.5
    assert receiver["max_linear_x"] <= controller["target_linear_speed"]
