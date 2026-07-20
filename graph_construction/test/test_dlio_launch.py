import importlib.util
import os
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_PATH = (
    REPO_ROOT
    / "graph_construction"
    / "launch"
    / "elevation_visual_navigation_sim.launch.py"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("wildos_elevation_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context_with_defaults(module):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(
                context,
                action.default_value,
            )
    return context


def _expanded(context, substitutions):
    if isinstance(substitutions, str):
        return substitutions
    return perform_substitutions(context, substitutions)


def test_unity_dlio_launch_owns_dlio_and_skips_xyz_adapter():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/wildos_test_ros_log")
    module = _load_launch_module()
    context = _context_with_defaults(module)
    context.launch_configurations["localization_backend"] = "dlio"
    context.launch_configurations["launch_dlio"] = "true"

    actions = module._launch_setup(context)
    launched_nodes = {
        (
            _expanded(context, action.node_package),
            _expanded(context, action.node_executable),
        )
        for action in actions
        if isinstance(action, Node)
    }

    assert (
        "direct_lidar_inertial_odometry",
        "dlio_odom_node",
    ) in launched_nodes
    assert ("graph_construction", "dlio_tf_adapter") in launched_nodes
    assert ("graph_construction", "dlio_input_filter") in launched_nodes
    assert ("graph_construction", "pointcloud_axis_adapter") not in launched_nodes


def test_dlio_odom_adapter_defaults_to_message_pose():
    module = _load_launch_module()
    context = _context_with_defaults(module)

    assert module._odom_pose_source(context, "dlio") == "message"
    assert module._odom_pose_source(context, "platform") == "tf"


def test_dlio_scoring_uses_globally_aligned_odom():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    context.launch_configurations["localization_backend"] = "dlio"
    context.launch_configurations["launch_dlio"] = "true"

    profile = module.load_topic_profile("unity")
    wiring = module.resolve_localization_wiring(profile, "dlio")

    assert module._scoring_odom_input_topic(wiring) == "/spot1/dlio/odom_node/aligned_odom"
    assert wiring.dlio_local_frame == "dlio_odom"
