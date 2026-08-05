import importlib.util
import os
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction
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


def _remappings(context, node):
    return {
        (
            perform_substitutions(context, source),
            perform_substitutions(context, target),
        )
        for source, target in node._Node__remappings
    }


def _all_nodes(actions):
    """展开 TimerAction, 收集主链路中的所有节点"""
    for action in actions:
        if isinstance(action, Node):
            yield action
        elif isinstance(action, TimerAction):
            yield from _all_nodes(action._TimerAction__actions)


def _node_prefix(context, node):
    description = node._ExecuteLocal__process_description
    return perform_substitutions(
        context,
        description._Executable__prefix,
    )


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
    assert ("graph_construction", "dlio_output_guard") in launched_nodes
    assert ("graph_construction", "camera_stamp_adapter") in launched_nodes
    assert ("graph_construction", "pointcloud_axis_adapter") not in launched_nodes

    dlio_node = next(
        action
        for action in actions
        if isinstance(action, Node)
        and _expanded(context, action.node_executable) == "dlio_odom_node"
    )
    remappings = _remappings(context, dlio_node)
    assert _node_prefix(context, dlio_node).strip().endswith(
        ".venv/bin/python3 -m graph_construction.quiet_stdout"
    )
    assert ("pointcloud", "/livox/lidar") in remappings
    assert ("imu", "/livox/imu") in remappings
    assert (
        "deskewed",
        "/spot1/dlio/odom_node/pointcloud/deskewed_raw",
    ) in remappings
    assert (
        "/tf",
        "/spot1/dlio/odom_node/tf_raw",
    ) in remappings
    assert (
        "/tf_static",
        "/spot1/dlio/odom_node/tf_static_raw",
    ) in remappings


def test_robot_platform_launch_skips_registered_cloud_adapter():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    context.launch_configurations["topic_profile"] = "robot"
    context.launch_configurations["localization_backend"] = "platform"

    executables = {
        _expanded(context, action.node_executable)
        for action in module._launch_setup(context)
        if isinstance(action, Node)
    }

    assert "pointcloud_axis_adapter" not in executables
    assert "elevation_mapping_node.py" in executables


def test_robot_profile_preserves_camera_stamps():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    context.launch_configurations["topic_profile"] = "robot"

    actions = module._launch_setup(context)
    executables = {
        _expanded(context, action.node_executable)
        for action in actions
        if isinstance(action, Node)
    }

    assert "camera_stamp_adapter" not in executables


def test_topic_and_frame_overrides_use_custom_profiles():
    module = _load_launch_module()
    argument_names = {
        action.name
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert "topic_profile_file" in argument_names
    assert "pointcloud_input_topic" not in argument_names
    assert "global_frame" not in argument_names


def test_unity_planner_output_is_remapped_to_source_path():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    planner = next(
        node
        for node in _all_nodes(module._launch_setup(context))
        if _expanded(context, node.node_executable) == "planner_node"
    )

    assert ("~/path", "/spot1/graphnav_planner/path") in _remappings(context, planner)


def test_dlio_odom_adapter_defaults_to_message_pose():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    unity = module.load_topic_profile("unity")
    robot = module.load_topic_profile("robot")

    assert module._odom_pose_source(context, unity, "dlio") == "message"
    assert module._odom_pose_source(context, unity, "platform") == "tf"
    assert module._odom_pose_source(context, robot, "platform") == "message"


def test_dlio_scoring_uses_globally_aligned_odom():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    context.launch_configurations["localization_backend"] = "dlio"
    context.launch_configurations["launch_dlio"] = "true"

    profile = module.load_topic_profile("unity")
    wiring = module.resolve_localization_wiring(profile, "dlio")

    assert module._scoring_odom_input_topic(wiring) == "/spot1/dlio/odom_node/aligned_odom"
    assert wiring.dlio_local_frame == "dlio_odom"


def test_all_python_nodes_use_uv_environment_python():
    module = _load_launch_module()
    expected_python = str(REPO_ROOT / ".venv/bin/python3")
    python_executables = {
        "camera_stamp_adapter",
        "dlio_output_guard",
        "dlio_tf_adapter",
        "elevation_mapping_node.py",
        "graph_construction",
        "object_search_goal_mux",
        "object_target_fusion",
        "odom_frame_adapter",
        "pipeline_performance_monitor",
        "pointcloud_axis_adapter",
        "wildos",
    }
    launched_python = set()

    for backend in ("platform", "dlio"):
        context = _context_with_defaults(module)
        context.launch_configurations["localization_backend"] = backend
        context.launch_configurations["launch_dlio"] = "true"
        for node in _all_nodes(module._launch_setup(context)):
            executable = _expanded(context, node.node_executable)
            if executable not in python_executables:
                continue
            launched_python.add(executable)
            assert _node_prefix(context, node).strip() == expected_python

    assert launched_python == python_executables


def test_system_python_is_rejected_for_python_nodes():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    context.launch_configurations["wildos_python_executable"] = "/usr/bin/python3"

    with pytest.raises(RuntimeError, match="不是 uv 创建"):
        module._launch_setup(context)


def test_static_transform_publishers_use_named_arguments():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    nodes = list(_all_nodes(module._launch_setup(context)))
    static_nodes = [
        node
        for node in nodes
        if _expanded(context, node.node_executable) == "static_transform_publisher"
    ]

    assert static_nodes
    for node in static_nodes:
        arguments = [
            _expanded(context, argument)
            for argument in node._Node__arguments
        ]
        assert "--frame-id" in arguments
        assert "--child-frame-id" in arguments


def test_wildos_starts_before_heavy_pipeline_nodes():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    actions = module._launch_setup(context)

    wildos_index = next(
        index
        for index, action in enumerate(actions)
        if isinstance(action, Node)
        and _expanded(context, action.node_executable) == "wildos"
    )
    elevation_index = next(
        index
        for index, action in enumerate(actions)
        if isinstance(action, Node)
        and _expanded(context, action.node_executable) == "elevation_mapping_node.py"
    )
    delayed_executables = {
        _expanded(context, node.node_executable)
        for action in actions
        if isinstance(action, TimerAction)
        for node in _all_nodes([action])
    }

    assert wildos_index < elevation_index
    assert "wildos" not in delayed_executables
    assert "object_target_fusion" in delayed_executables
