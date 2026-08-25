from pathlib import Path

import yaml


def test_migrated_navigation_topic_contract():
    config_file = Path(__file__).parents[1] / "config" / "navigation.yaml"
    config = yaml.safe_load(config_file.read_text(encoding="utf-8"))
    planner = config["astar"]["ros__parameters"]
    costmap = config["map_pub"]["ros__parameters"]
    controller = config["start_nav"]["ros__parameters"]
    sender = config["wildos_velocity_sender"]["ros__parameters"]
    receiver = config["wildos_velocity_receiver"]["ros__parameters"]

    assert planner["goal_topic"] == "/goal_pose"
    assert planner["grid_map_topic"] == "combined_grid"
    assert planner["odom_topic"] == "/odom"
    assert planner["odom_frame"] == "odom"
    assert costmap["lidar_topic"] == "/cloud_registered"
    assert costmap["odom_topic"] == planner["odom_topic"]
    assert costmap["odom_frame"] == planner["odom_frame"]
    assert costmap["grid_width"] == costmap["grid_height"] == 10.0
    assert costmap["resolution"] == 0.1
    assert controller["odom_topic"] == "/odom"
    assert controller["lidar_topic"] == "/cloud_registered"
    assert controller["cmd_vel_topic"] == "/cmd_vel"
    assert sender["input_topic"] == controller["cmd_vel_topic"]
    assert sender["target_ip"] == receiver["bind_ip"] == "192.168.50.2"
    assert receiver["expected_source_ip"] == "192.168.50.1"
    assert receiver["output_topic"] == "/cmd_vel"
    assert receiver["command_timeout_sec"] < 0.5
    assert receiver["max_linear_x"] == 0.6
    assert receiver["max_angular_z"] == 1.0

    launch_file = Path(__file__).parents[1] / "launch" / "navigation.launch.py"
    launch_source = launch_file.read_text(encoding="utf-8")
    for executable in ("map_pub", "astar", "controller", "velocity_sender"):
        assert f'executable="{executable}"' in launch_source
    assert "/spot1/graphnav_planner/path" not in launch_source
