import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from graph_construction.topic_profiles import load_topic_profile


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topic_profile",
                default_value="robot",
                description="实机 topic 和 frame 配置",
            ),
            DeclareLaunchArgument(
                "topic_profile_file",
                default_value="",
                description="Optional absolute path to a custom topic profile YAML",
            ),
            DeclareLaunchArgument(
                "elevation_config",
                default_value="elevation_mapping.yaml",
                description="Base config file for the elevation mapping backend",
            ),
            DeclareLaunchArgument(
                "graph_config",
                default_value="graph_construction_elevation.yaml",
                description="Base config file for graph_construction",
            ),
            DeclareLaunchArgument(
                "visual_config",
                default_value="wildos_nav_conf.yaml",
                description="Base config file installed by visual_navigation",
            ),
            DeclareLaunchArgument("do_object_search", default_value="false", description="Enable object search"),
            DeclareLaunchArgument(
                "launch_paper_rviz",
                default_value="false",
                description="Launch the paper-style RViz view and lightweight point cloud",
            ),
            DeclareLaunchArgument(
                "launch_performance_monitor",
                default_value="true",
                description="Launch the pipeline performance monitor",
            ),
            DeclareLaunchArgument(
                "paper_rviz_config",
                default_value="wildos_paper.rviz",
                description="RViz config name installed by graph_construction or an absolute path",
            ),
            DeclareLaunchArgument(
                "graph_start_delay",
                default_value="3.0",
                description="Delay graph_construction startup so the elevation GridMap is advertised first",
            ),
            DeclareLaunchArgument(
                "visual_start_delay",
                default_value="6.0",
                description="Delay target fusion startup while WildOS loads immediately",
            ),
            DeclareLaunchArgument(
                "planner_start_delay",
                default_value="7.0",
                description="Delay planner startup until graph and visual scoring are active",
            ),
            DeclareLaunchArgument("log_level", default_value="INFO", description="Logging level"),
            DeclareLaunchArgument(
                "wildos_python_executable",
                default_value="",
                description="uv environment Python used by every Python node",
            ),
            DeclareLaunchArgument(
                "launch_odom_adapter",
                default_value="true",
                description="Publish odom with frame ids expected by WildOS",
            ),
            DeclareLaunchArgument(
                "odom_fallback_to_message",
                default_value="false",
                description="Fallback to source odom pose if TF pose is unavailable",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="",
                description="ROS domain override, empty uses topic profile",
            ),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="",
                description="RMW override, empty uses topic profile",
            ),
            DeclareLaunchArgument(
                "fastdds_profile",
                default_value="",
                description="可选 FastDDS 配置文件",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context):
    profile_name = _arg(context, "topic_profile")
    profile_file = _arg(context, "topic_profile_file") or None
    profile = load_topic_profile(profile_name, profile_file)

    ns = str(profile["namespace"])
    elevation_config = _arg(context, "elevation_config")
    graph_config = _arg(context, "graph_config")
    visual_config = _arg(context, "visual_config")
    global_frame = _value(context, profile, "global_frame", "global_frame")
    odom_output_topic = _value(context, profile, "odom_output_topic", "odom_output_topic")
    nav_graph_topic = _value(context, profile, "nav_graph_topic", "nav_graph_topic")
    pointcloud_input_topic = _value(
        context,
        profile,
        "pointcloud_input_topic",
        "pointcloud_input_topic",
    )
    aligned_lidar_topic = _value(
        context,
        profile,
        "aligned_lidar_topic",
        "aligned_lidar_topic",
    )
    camera_img_topic = _value(
        context,
        profile,
        "camera_img_topic",
        "camera_img_topic",
    )
    camera_info_topic = _value(
        context,
        profile,
        "camera_info_topic",
        "camera_info_topic",
    )
    pointcloud_expected_frame = _value(
        context,
        profile,
        "pointcloud_expected_frame",
        "pointcloud_expected_frame",
    )

    graph_overrides = _config_override_args(
        {
            "global_frame": global_frame,
            "odom_topic": odom_output_topic,
            "grid_map_topic": _value(context, profile, "elevation_grid_map_topic", "elevation_grid_map_topic"),
            "nav_graph_topic": nav_graph_topic,
            "viz_topic": _value(context, profile, "graph_construction_viz_topic", "graph_construction_viz_topic"),
        }
    )
    visual_overrides = _config_override_args(
        {
            "parent_frame": _value(context, profile, "parent_frame", "parent_frame"),
            "cam_frame": _value(context, profile, "cam_frame", "cam_frame"),
            "camera_img_topic": camera_img_topic,
            "camera_info_topic": camera_info_topic,
            "odometry_topic": odom_output_topic,
            "navigation_graph_topic": nav_graph_topic,
            "scored_navgraph_topic": _value(context, profile, "scored_nav_graph_topic", "scored_nav_graph_topic"),
            "model_viz_topic": _value(context, profile, "model_viz_topic", "model_viz_topic"),
            "valid_geofrontiers_topic": _value(context, profile, "valid_geofrontiers_topic", "valid_geofrontiers_topic"),
            "score_ring_topic": _value(context, profile, "score_ring_topic", "score_ring_topic"),
            "object_mask_topic": _value(context, profile, "object_mask_topic", "object_mask_topic"),
            "object_search_target_topic": _value(
                context,
                profile,
                "object_search_target_topic",
                "object_search_target_topic",
            ),
            "object_reached_topic": _value(context, profile, "object_reached_topic", "object_reached_topic"),
            "object_completed_topic": _value(context, profile, "object_search_completed_topic", "object_search_completed_topic"),
            "object_search_config.mask_threshold": _value(
                context,
                profile,
                "object_search_mask_threshold",
                "object_search_mask_threshold",
            ),
            "frontiers_range": _float_value(
                context,
                profile,
                "visual_frontiers_range",
                "visual_frontiers_range",
            ),
            "frontier_threshold": _float_value(
                context,
                profile,
                "visual_frontier_threshold",
                "visual_frontier_threshold",
            ),
        }
    )

    use_sim_time = False
    log_level = LaunchConfiguration("log_level")
    planner_odom_topic = _value(context, profile, "planner_odom_topic", "planner_odom_topic")
    goal_pose_topic = _value(context, profile, "goal_pose_topic", "goal_pose_topic")
    scored_nav_graph_topic = _value(context, profile, "scored_nav_graph_topic", "scored_nav_graph_topic")
    repo_root = _repo_root()
    python_executable = _uv_python_executable(context, repo_root)
    python_node_extra_args = {
        "prefix": f"{python_executable} ",
        "additional_env": {
            "PYTHONPATH": _prepend_pythonpath(repo_root),
            "PYTHONNOUSERSITE": "1",
            "VIRTUAL_ENV": str(Path(python_executable).parent.parent),
            "PATH": _prepend_path(str(Path(python_executable).parent)),
        },
    }

    elevation_share = get_package_share_directory("elevation_mapping_cupy")
    core_param = PathJoinSubstitution([TextSubstitution(text=elevation_share), "config", "core", "core_param.yaml"])
    base_frame = _value(context, profile, "base_frame", "base_frame")

    elevation_mapping = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        **python_node_extra_args,
        parameters=[
            core_param,
            _package_config_path("graph_construction", elevation_config),
            {"use_sim_time": use_sim_time},
            {"map_frame": global_frame},
            {"corrected_map_frame": global_frame},
            {"base_frame": base_frame},
            {"initialize_frame_id": [base_frame]},
            {"subscribers.livox.topic_name": aligned_lidar_topic},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/livox/lidar_aligned", aligned_lidar_topic),
        ],
    )

    graph_construction = Node(
        package="graph_construction",
        executable="graph_construction",
        output="screen",
        **python_node_extra_args,
        namespace=ns,
        arguments=["--config", graph_config, *graph_overrides, "--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    odom_adapter = Node(
        package="visual_navigation",
        executable="odom_frame_adapter",
        output="screen",
        **python_node_extra_args,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": _value(context, profile, "odom_input_topic", "odom_input_topic")},
            {"output_topic": odom_output_topic},
            {"parent_frame": _value(context, profile, "odom_parent_frame", "odom_parent_frame")},
            {"child_frame": _value(context, profile, "odom_child_frame", "odom_child_frame")},
            {"stamp_mode": str(profile.get("odom_stamp_mode", "preserve"))},
            {"pose_source": str(profile.get("odom_pose_source", "message"))},
            {"fallback_to_message": LaunchConfiguration("odom_fallback_to_message")},
        ],
        condition=IfCondition(LaunchConfiguration("launch_odom_adapter")),
    )

    pointcloud_relay = Node(
        package="graph_construction",
        executable="pointcloud_relay",
        output="screen",
        **python_node_extra_args,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": pointcloud_input_topic},
            {"output_topic": aligned_lidar_topic},
            {"expected_frame": pointcloud_expected_frame},
            {
                "max_output_rate_hz": float(
                    profile.get("pointcloud_output_rate_hz", 0.0)
                )
            },
        ],
    )

    paper_rviz_config = Path(_arg(context, "paper_rviz_config"))
    if not paper_rviz_config.is_absolute():
        paper_rviz_config = (
            Path(get_package_share_directory("graph_construction"))
            / "rviz"
            / paper_rviz_config
        )
    paper_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="wildos_paper_rviz",
        output="screen",
        arguments=["-d", str(paper_rviz_config), "-f", global_frame],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("launch_paper_rviz")),
    )

    wildos = Node(
        package="visual_navigation",
        executable="wildos",
        output="both",
        **python_node_extra_args,
        arguments=[
            "--config",
            visual_config,
            *visual_overrides,
            "--do_object_search",
            LaunchConfiguration("do_object_search"),
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    object_target_fusion = Node(
        package="visual_navigation",
        executable="object_target_fusion",
        name="object_target_fusion",
        output="screen",
        **python_node_extra_args,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"global_frame": global_frame},
            {"object_mask_topic": _value(context, profile, "object_mask_topic", "object_mask_topic")},
            {"lidar_topic": aligned_lidar_topic},
            {"target_estimate_topic": _value(context, profile, "object_target_estimate_topic", "object_target_estimate_topic")},
            {"target_marker_topic": _value(context, profile, "object_target_estimate_viz_topic", "object_target_estimate_viz_topic")},
            {"particle_topic": _value(context, profile, "object_target_particles_topic", "object_target_particles_topic")},
            {"completion_topic": _value(context, profile, "object_search_completed_topic", "object_search_completed_topic")},
            {
                "object_search_target_topic": _value(
                    context,
                    profile,
                    "object_search_target_topic",
                    "object_search_target_topic",
                )
            },
            {
                "max_depth": _float_value(
                    context,
                    profile,
                    "object_target_max_depth",
                    "object_target_max_depth",
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration("do_object_search")),
    )

    object_search_goal_mux = Node(
        package="visual_navigation",
        executable="object_search_goal_mux",
        output="screen",
        **python_node_extra_args,
        parameters=[
            _package_config_path("visual_navigation", "object_search_goal_mux.yaml"),
            {"use_sim_time": use_sim_time},
            {"output_goal_topic": goal_pose_topic},
            {"status_topic": _value(context, profile, "object_search_status_topic", "object_search_status_topic")},
            {"object_target_estimate_topic": _value(context, profile, "object_target_estimate_topic", "object_target_estimate_topic")},
            {"object_reached_topic": _value(context, profile, "object_reached_topic", "object_reached_topic")},
            {"completion_topic": _value(context, profile, "object_search_completed_topic", "object_search_completed_topic")},
            {
                "object_search_target_topic": _value(
                    context,
                    profile,
                    "object_search_target_topic",
                    "object_search_target_topic",
                )
            },
            {"odom_topic": odom_output_topic},
            {"nav_graph_topic": nav_graph_topic},
            {"scored_nav_graph_topic": scored_nav_graph_topic},
            {"frame_id": global_frame},
            {
                "object_reached_timeout_sec": _float_value(
                    context,
                    profile,
                    "object_search_object_reached_timeout_sec",
                    "object_search_object_reached_timeout_sec",
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration("do_object_search")),
    )

    planner_path_topic = _value(
        context,
        profile,
        "planner_path_topic",
        "planner_path_topic",
    )
    planner = _planner_node(
        ns,
        use_sim_time,
        planner_odom_topic,
        goal_pose_topic,
        scored_nav_graph_topic,
        _value(context, profile, "object_search_status_topic", "object_search_status_topic"),
        planner_path_topic,
    )

    pipeline_performance_monitor = Node(
        package="graph_construction",
        executable="pipeline_performance_monitor",
        output="screen",
        **python_node_extra_args,
        condition=IfCondition(LaunchConfiguration("launch_performance_monitor")),
        parameters=[
            {"use_sim_time": use_sim_time},
            {"raw_lidar_topic": ""},
            {"raw_imu_topic": ""},
            {"aligned_pointcloud_topic": aligned_lidar_topic},
            {"odom_topic": odom_output_topic},
            {
                "grid_map_topic": _value(
                    context,
                    profile,
                    "elevation_grid_map_topic",
                    "elevation_grid_map_topic",
                )
            },
            {"nav_graph_topic": nav_graph_topic},
            {"scored_nav_graph_topic": scored_nav_graph_topic},
            {
                "object_mask_topic": _value(
                    context,
                    profile,
                    "object_mask_topic",
                    "object_mask_topic",
                )
            },
            {
                "target_estimate_topic": _value(
                    context,
                    profile,
                    "object_target_estimate_topic",
                    "object_target_estimate_topic",
                )
            },
            {"goal_topic": goal_pose_topic},
            {"path_topic": planner_path_topic},
        ],
    )

    return [
        SetEnvironmentVariable("ROS_DOMAIN_ID", _value(context, profile, "ros_domain_id", "ros_domain_id")),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", _value(context, profile, "rmw_implementation", "rmw_implementation")),
        SetEnvironmentVariable(
            "FASTDDS_DEFAULT_PROFILES_FILE",
            LaunchConfiguration("fastdds_profile"),
            condition=LaunchConfigurationNotEquals("fastdds_profile", ""),
        ),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            LaunchConfiguration("fastdds_profile"),
            condition=LaunchConfigurationNotEquals("fastdds_profile", ""),
        ),
        wildos,
        pointcloud_relay,
        odom_adapter,
        paper_rviz,
        elevation_mapping,
        pipeline_performance_monitor,
        TimerAction(period=LaunchConfiguration("graph_start_delay"), actions=[graph_construction]),
        TimerAction(period=LaunchConfiguration("visual_start_delay"), actions=[object_target_fusion]),
        TimerAction(period=LaunchConfiguration("planner_start_delay"), actions=[object_search_goal_mux, planner]),
    ]


def _planner_node(
    ns,
    use_sim_time,
    odom_topic,
    goal_pose_topic,
    scored_nav_graph_topic,
    object_search_status_topic,
    planner_path_topic,
):
    return Node(
        package="graphnav_planner",
        executable="planner_node",
        name="graphnav_planner",
        output="screen",
        namespace=ns,
        parameters=[
            str(Path(get_package_share_directory("graphnav_planner")) / "config" / "planner.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("~/nav_graph", scored_nav_graph_topic),
            ("~/odom", odom_topic),
            ("~/goal_pose", goal_pose_topic),
            ("~/object_search_status", object_search_status_topic),
            ("~/path", planner_path_topic),
        ],
    )


def _config_override_args(overrides):
    args = []
    for key, value in overrides.items():
        args.extend(["--config-override", f"{key}={_config_scalar(value)}"])
    return args


def _config_scalar(value):
    """把字符串转成 YAML 安全标量, 兼容 OmegaConf dotlist"""
    if not isinstance(value, str):
        return str(value)
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def _repo_root():
    """从源码 symlink launch 路径推导仓库根目录"""
    return str(Path(__file__).resolve().parents[2])


def _uv_python_executable(context, repo_root):
    """只接受 uv 创建的 Python 环境, 避免 ROS entrypoint 回退系统 Python"""
    configured = _arg(context, "wildos_python_executable")
    candidate = Path(configured) if configured else Path(repo_root) / ".venv/bin/python3"
    if not candidate.is_absolute():
        candidate = Path(repo_root) / candidate
    venv_config = candidate.parent.parent / "pyvenv.cfg"
    if not candidate.is_file() or not os.access(candidate, os.X_OK):
        raise RuntimeError(f"uv Python 不可执行: {candidate}")
    if not venv_config.is_file() or not any(
        line.startswith("uv = ")
        for line in venv_config.read_text(encoding="utf-8").splitlines()
    ):
        raise RuntimeError(f"Python 环境不是 uv 创建的虚拟环境: {candidate.parent.parent}")
    return str(candidate.absolute())


def _prepend_pythonpath(path):
    """给所有 Python 节点补仓库根目录, 让本地包可导入"""
    current_pythonpath = os.environ.get("PYTHONPATH")
    if current_pythonpath:
        return f"{path}:{current_pythonpath}"
    return path


def _prepend_path(path):
    """让 Python 节点创建的子进程继续优先使用同一 uv 环境"""
    current_path = os.environ.get("PATH")
    if current_path:
        return f"{path}:{current_path}"
    return path


def _package_config_path(package_name, config_name):
    config_path = Path(config_name).expanduser()
    if config_path.is_absolute():
        return str(config_path)
    return str(Path(get_package_share_directory(package_name)) / "configs" / config_name)


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _value(context, profile, arg_name, profile_key):
    override = context.launch_configurations.get(arg_name, "")
    if override:
        return override
    return str(profile[profile_key])


def _float_value(context, profile, arg_name, profile_key):
    raw_value = _value(context, profile, arg_name, profile_key)
    try:
        return float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid float value for {arg_name}: {raw_value}") from exc
