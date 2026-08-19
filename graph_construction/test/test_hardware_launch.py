import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_PATH = (
    REPO_ROOT
    / "graph_construction"
    / "launch"
    / "elevation_visual_navigation.launch.py"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "wildos_elevation_launch",
        LAUNCH_PATH,
    )
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
    if not isinstance(substitutions, (list, tuple)):
        substitutions = [substitutions]
    return perform_substitutions(context, substitutions)


def _all_nodes(actions):
    """展开延迟动作, 返回实机主链的全部节点"""
    for action in actions:
        if isinstance(action, Node):
            yield action
        elif isinstance(action, TimerAction):
            yield from _all_nodes(action._TimerAction__actions)


def _remappings(context, node):
    return {
        (
            perform_substitutions(context, source),
            perform_substitutions(context, target),
        )
        for source, target in node._Node__remappings
    }


def _node_prefix(context, node):
    description = node._ExecuteLocal__process_description
    return perform_substitutions(
        context,
        description._Executable__prefix,
    )


def test_launch_defaults_are_hardware_only():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    argument_names = {
        action.name
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert context.launch_configurations["topic_profile"] == "robot"
    assert context.launch_configurations["elevation_config"] == "elevation_mapping.yaml"
    assert context.launch_configurations["visual_config"] == "wildos_nav_conf.yaml"
    assert "use_sim_time" not in argument_names
    assert "localization_backend" not in argument_names
    assert "launch_dlio" not in argument_names
    assert "publish_camera_static_tf" not in argument_names


def test_hardware_launch_relays_registered_cloud_locally():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    actions = module._launch_setup(context)
    nodes = list(_all_nodes(actions))
    executables = {
        _expanded(context, node.node_executable)
        for node in nodes
    }
    monitor = next(
        node
        for node in nodes
        if _expanded(context, node.node_executable)
        == "pipeline_performance_monitor"
    )
    monitor_parameters = {
        key: value
        for parameters in evaluate_parameters(context, monitor._Node__parameters)
        for key, value in parameters.items()
    }
    relay = next(
        node
        for node in nodes
        if _expanded(context, node.node_executable) == "pointcloud_relay"
    )
    relay_parameters = {
        key: value
        for parameters in evaluate_parameters(context, relay._Node__parameters)
        for key, value in parameters.items()
    }

    assert "pointcloud_relay" in executables
    assert "elevation_mapping_node.py" in executables
    assert "dlio_odom_node" not in executables
    assert "static_transform_publisher" not in executables
    assert monitor_parameters["raw_lidar_topic"] == ""
    assert monitor_parameters["aligned_pointcloud_topic"] == (
        "/spot1/cloud_registered_local"
    )
    assert relay_parameters["expected_frame"] == "dlio_odom"
    assert "output_frame" not in relay_parameters


def test_planner_uses_the_canonical_path_topic():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    planner = next(
        node
        for node in _all_nodes(module._launch_setup(context))
        if _expanded(context, node.node_executable) == "planner_node"
    )

    assert (
        "~/path",
        "/spot1/graphnav_planner/path",
    ) in _remappings(context, planner)


def test_all_python_nodes_use_uv_environment_python():
    module = _load_launch_module()
    context = _context_with_defaults(module)
    expected_python = str(REPO_ROOT / ".venv/bin/python3")
    python_executables = {
        "elevation_mapping_node.py",
        "graph_construction",
        "object_search_goal_mux",
        "object_target_fusion",
        "odom_frame_adapter",
        "pipeline_performance_monitor",
        "pointcloud_relay",
        "wildos",
    }
    launched_python = set()

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
        and _expanded(context, action.node_executable)
        == "elevation_mapping_node.py"
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


def test_runtime_target_topic_reaches_all_object_search_nodes():
    """三个目标搜索节点必须使用同一个运行时目标 topic"""
    module = _load_launch_module()
    context = _context_with_defaults(module)
    nodes = list(_all_nodes(module._launch_setup(context)))
    target_topic = "/spot1/object_search_target"

    for executable in ("object_target_fusion", "object_search_goal_mux"):
        node = next(
            item
            for item in nodes
            if _expanded(context, item.node_executable) == executable
        )
        parameters = {
            key: value
            for values in evaluate_parameters(context, node._Node__parameters)
            if isinstance(values, dict)
            for key, value in values.items()
        }
        assert parameters["object_search_target_topic"] == target_topic

    wildos = next(
        item
        for item in nodes
        if _expanded(context, item.node_executable) == "wildos"
    )
    arguments = [
        _expanded(context, argument)
        for argument in wildos._Node__arguments
    ]
    assert f'object_search_target_topic="{target_topic}"' in arguments
