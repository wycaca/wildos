import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from graph_construction.topic_profiles import load_topic_profile, profile_key_description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("topic_profile", default_value="isaac", description="Topic profile: isaac, unity, robot"),
            DeclareLaunchArgument(
                "topic_profile_file",
                default_value="",
                description="Optional absolute path to a custom topic profile YAML",
            ),
            DeclareLaunchArgument("ns", default_value="", description="Robot namespace, empty uses topic profile"),
            DeclareLaunchArgument(
                "elevation_config",
                default_value="elevation_mapping_sim.yaml",
                description="Base config file for the elevation mapping backend",
            ),
            DeclareLaunchArgument(
                "graph_config",
                default_value="graph_construction_elevation.yaml",
                description="Base config file for graph_construction",
            ),
            DeclareLaunchArgument(
                "visual_config",
                default_value="wildos_nav_sim_conf.yaml",
                description="Base config file installed by visual_navigation",
            ),
            DeclareLaunchArgument("do_object_search", default_value="false", description="Enable object search"),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
            DeclareLaunchArgument(
                "graph_start_delay",
                default_value="3.0",
                description="Delay graph_construction startup so the elevation GridMap is advertised first",
            ),
            DeclareLaunchArgument(
                "visual_start_delay",
                default_value="6.0",
                description="Delay visual navigation startup so the base graph is available first",
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
                description="Python executable used to launch WildOS, empty uses console script shebang",
            ),
            DeclareLaunchArgument(
                "launch_odom_adapter",
                default_value="true",
                description="Publish odom with frame ids expected by WildOS",
            ),
            DeclareLaunchArgument(
                "launch_pointcloud_axis_adapter",
                default_value="true",
                description="Publish an aligned point cloud for elevation mapping",
            ),
            DeclareLaunchArgument(
                "pointcloud_axis_mode",
                default_value="",
                description="Point cloud axis conversion mode, empty uses topic profile",
            ),
            _profile_arg("global_frame", "global_frame_3d"),
            _profile_arg("pointcloud_input_topic", "pointcloud_input_topic"),
            _profile_arg("pointcloud_output_topic", "aligned_lidar_topic"),
            _profile_arg("pointcloud_output_frame", "pointcloud_output_frame_3d"),
            _profile_arg("elevation_grid_map_topic", "elevation_grid_map_topic"),
            _profile_arg("odom_input_topic", "odom_input_topic"),
            _profile_arg("odom_output_topic", "odom_output_topic"),
            _profile_arg("odom_parent_frame", "odom_parent_frame"),
            _profile_arg("odom_child_frame", "odom_child_frame"),
            _profile_arg("base_frame", "base_frame"),
            DeclareLaunchArgument("odom_stamp_mode", default_value="now", description="Adapted odometry stamp mode"),
            DeclareLaunchArgument("odom_pose_source", default_value="tf", description="Adapted odometry pose source"),
            DeclareLaunchArgument(
                "odom_fallback_to_message",
                default_value="false",
                description="Fallback to source odom pose if TF pose is unavailable",
            ),
            DeclareLaunchArgument(
                "publish_lidar_static_tf",
                default_value="false",
                description="Publish a fallback static transform for the elevation point cloud frame",
            ),
            _profile_arg("lidar_parent_frame", "lidar_parent_frame"),
            _profile_arg("lidar_frame", "lidar_frame"),
            _profile_arg("camera_parent_frame", "camera_parent_frame"),
            _profile_arg("camera_static_tf_convention", "camera_static_tf_convention"),
            _profile_arg("camera_image_flip_x", "camera_image_flip_x"),
            _profile_arg("parent_frame", "parent_frame"),
            _profile_arg("cam_frame", "cam_frame"),
            _profile_arg("camera_img_topic", "camera_img_topic"),
            _profile_arg("camera_info_topic", "camera_info_topic"),
            _profile_arg("nav_graph_topic", "nav_graph_topic"),
            _profile_arg("graph_construction_viz_topic", "graph_construction_viz_topic"),
            _profile_arg("scored_nav_graph_topic", "scored_nav_graph_topic"),
            _profile_arg("model_viz_topic", "model_viz_topic"),
            _profile_arg("valid_geofrontiers_topic", "valid_geofrontiers_topic"),
            _profile_arg("score_ring_topic", "score_ring_topic"),
            _profile_arg("graph_viz_topic", "graph_viz_topic"),
            _profile_arg("object_mask_topic", "object_mask_topic"),
            _profile_arg("object_target_pose_topic", "object_target_pose_topic"),
            _profile_arg("object_target_viz_topic", "object_target_viz_topic"),
            _profile_arg("object_reached_topic", "object_reached_topic"),
            _profile_arg("object_search_initial_goal_distance", "object_search_initial_goal_distance"),
            _profile_arg("object_search_initial_goal_heading_deg", "object_search_initial_goal_heading_deg"),
            _profile_arg("object_search_mask_threshold", "object_search_mask_threshold"),
            _profile_arg("visual_frontiers_range", "visual_frontiers_range"),
            _profile_arg("visual_frontier_threshold", "visual_frontier_threshold"),
            _profile_arg("object_search_detection_debug_interval", "object_search_detection_debug_interval"),
            _profile_arg("object_search_target_log_period_sec", "object_search_target_log_period_sec"),
            _profile_arg("object_search_goal_publish_rate", "object_search_goal_publish_rate"),
            _profile_arg("object_search_target_timeout_sec", "object_search_target_timeout_sec"),
            _profile_arg("object_search_latch_target_after_first_detection", "object_search_latch_target_after_first_detection"),
            _profile_arg("object_search_latch_target_timeout_sec", "object_search_latch_target_timeout_sec"),
            _profile_arg("object_search_memory_timeout_sec", "object_search_memory_timeout_sec"),
            _profile_arg("object_search_memory_goal_distance", "object_search_memory_goal_distance"),
            _profile_arg("object_search_target_reached_radius", "object_search_target_reached_radius"),
            _profile_arg("object_search_object_reached_timeout_sec", "object_search_object_reached_timeout_sec"),
            _profile_arg("object_search_reached_latch_timeout_sec", "object_search_reached_latch_timeout_sec"),
            _profile_arg(
                "object_search_object_reached_require_target_distance",
                "object_search_object_reached_require_target_distance",
            ),
            _profile_arg(
                "object_search_object_reached_max_target_distance",
                "object_search_object_reached_max_target_distance",
            ),
            _profile_arg("object_search_reached_mask_fraction", "object_search_reached_mask_fraction"),
            _profile_arg("object_search_reached_min_pixel_count", "object_search_reached_min_pixel_count"),
            _profile_arg("object_search_reached_confirm_frames", "object_search_reached_confirm_frames"),
            _profile_arg("object_search_detection_min_peak_score", "object_search_detection_min_peak_score"),
            _profile_arg(
                "object_search_detection_min_component_pixels",
                "object_search_detection_min_component_pixels",
            ),
            _profile_arg(
                "object_search_detection_min_component_fraction",
                "object_search_detection_min_component_fraction",
            ),
            _profile_arg("object_search_detection_confirm_frames", "object_search_detection_confirm_frames"),
            _profile_arg("object_search_goal_viz_topic", "object_search_goal_viz_topic"),
            _profile_arg("object_search_status_topic", "object_search_status_topic"),
            _profile_arg("planner_odom_topic", "planner_odom_topic"),
            _profile_arg("goal_pose_topic", "goal_pose_topic"),
            _profile_arg("tracking_goal_pose_topic", "tracking_goal_pose_topic"),
            _profile_arg("path_topic", "path_topic"),
            DeclareLaunchArgument(
                "publish_camera_static_tf",
                default_value="true",
                description="Publish fallback static transforms for camera frames",
            ),
            _profile_arg("ros_domain_id", "ros_domain_id"),
            _profile_arg("rmw_implementation", "rmw_implementation_3d"),
            DeclareLaunchArgument(
                "fastdds_profile",
                default_value="",
                description="Optional FastDDS profile shared with Isaac Sim",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context):
    profile_name = _arg(context, "topic_profile")
    profile_file = _arg(context, "topic_profile_file") or None
    profile = load_topic_profile(profile_name, profile_file)

    ns = _value(context, profile, "ns", "namespace")
    elevation_config = _arg(context, "elevation_config")
    graph_config = _arg(context, "graph_config")
    visual_config = _arg(context, "visual_config")
    global_frame = _value(context, profile, "global_frame", "global_frame_3d")
    odom_output_topic = _value(context, profile, "odom_output_topic", "odom_output_topic")
    nav_graph_topic = _value(context, profile, "nav_graph_topic", "nav_graph_topic")
    aligned_lidar_topic = _value(context, profile, "pointcloud_output_topic", "aligned_lidar_topic")
    pointcloud_axis_mode = _value(context, profile, "pointcloud_axis_mode", "pointcloud_axis_mode_3d")
    pointcloud_output_frame = _value(context, profile, "pointcloud_output_frame", "pointcloud_output_frame_3d")

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
            "camera_img_topic": _value(context, profile, "camera_img_topic", "camera_img_topic"),
            "camera_info_topic": _value(context, profile, "camera_info_topic", "camera_info_topic"),
            "odometry_topic": odom_output_topic,
            "navigation_graph_topic": nav_graph_topic,
            "scored_navgraph_topic": _value(context, profile, "scored_nav_graph_topic", "scored_nav_graph_topic"),
            "model_viz_topic": _value(context, profile, "model_viz_topic", "model_viz_topic"),
            "valid_geofrontiers_topic": _value(context, profile, "valid_geofrontiers_topic", "valid_geofrontiers_topic"),
            "score_ring_topic": _value(context, profile, "score_ring_topic", "score_ring_topic"),
            "graph_viz_topic": _value(context, profile, "graph_viz_topic", "graph_viz_topic"),
            "object_mask_topic": _value(context, profile, "object_mask_topic", "object_mask_topic"),
            "object_target_pose_topic": _value(context, profile, "object_target_pose_topic", "object_target_pose_topic"),
            "object_target_viz_topic": _value(context, profile, "object_target_viz_topic", "object_target_viz_topic"),
            "object_reached_topic": _value(context, profile, "object_reached_topic", "object_reached_topic"),
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
            "object_search_config.detection_debug_interval": _value(
                context,
                profile,
                "object_search_detection_debug_interval",
                "object_search_detection_debug_interval",
            ),
            "object_search_config.detection_min_peak_score": _value(
                context,
                profile,
                "object_search_detection_min_peak_score",
                "object_search_detection_min_peak_score",
            ),
            "object_search_config.detection_min_component_pixels": _value(
                context,
                profile,
                "object_search_detection_min_component_pixels",
                "object_search_detection_min_component_pixels",
            ),
            "object_search_config.detection_min_component_fraction": _value(
                context,
                profile,
                "object_search_detection_min_component_fraction",
                "object_search_detection_min_component_fraction",
            ),
            "object_search_config.detection_confirm_frames": _value(
                context,
                profile,
                "object_search_detection_confirm_frames",
                "object_search_detection_confirm_frames",
            ),
            "object_search_config.target_log_period_sec": _value(
                context,
                profile,
                "object_search_target_log_period_sec",
                "object_search_target_log_period_sec",
            ),
            "object_search_config.reached_mask_fraction": _value(
                context,
                profile,
                "object_search_reached_mask_fraction",
                "object_search_reached_mask_fraction",
            ),
            "object_search_config.reached_min_pixel_count": _value(
                context,
                profile,
                "object_search_reached_min_pixel_count",
                "object_search_reached_min_pixel_count",
            ),
            "object_search_config.reached_confirm_frames": _value(
                context,
                profile,
                "object_search_reached_confirm_frames",
                "object_search_reached_confirm_frames",
            ),
            "camera_image_flip_x": _value(context, profile, "camera_image_flip_x", "camera_image_flip_x"),
        }
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    camera_parent_frame = _value(context, profile, "camera_parent_frame", "camera_parent_frame")
    camera_transforms = _camera_static_transforms(
        _value(context, profile, "camera_static_tf_convention", "camera_static_tf_convention")
    )
    planner_odom_topic = _value(context, profile, "planner_odom_topic", "planner_odom_topic")
    goal_pose_topic = _value(context, profile, "goal_pose_topic", "goal_pose_topic")
    scored_nav_graph_topic = _value(context, profile, "scored_nav_graph_topic", "scored_nav_graph_topic")
    publish_camera_static_tf = TextSubstitution(text=_arg(context, "publish_camera_static_tf"))
    wildos_python_executable = _arg(context, "wildos_python_executable")
    wildos_extra_args = {}
    if wildos_python_executable:
        wildos_extra_args["prefix"] = f"{wildos_python_executable} "
    repo_root = _repo_root()
    wildos_extra_args["additional_env"] = {
        "PYTHONPATH": _prepend_pythonpath(repo_root),
    }

    elevation_share = get_package_share_directory("elevation_mapping_cupy")
    core_param = PathJoinSubstitution([TextSubstitution(text=elevation_share), "config", "core", "core_param.yaml"])
    base_frame = _value(context, profile, "base_frame", "base_frame")

    elevation_mapping = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
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
            ("/unitree_go2/lidar/points_aligned", aligned_lidar_topic),
        ],
    )

    graph_construction = Node(
        package="graph_construction",
        executable="graph_construction",
        output="screen",
        namespace=ns,
        arguments=["--config", graph_config, *graph_overrides, "--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/tf", PathJoinSubstitution([TextSubstitution(text="/"), TextSubstitution(text=ns), TextSubstitution(text="tf")])),
            (
                "/tf_static",
                PathJoinSubstitution([TextSubstitution(text="/"), TextSubstitution(text=ns), TextSubstitution(text="tf_static")]),
            ),
        ],
    )

    odom_adapter = Node(
        package="visual_navigation",
        executable="odom_frame_adapter",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": _value(context, profile, "odom_input_topic", "odom_input_topic")},
            {"output_topic": odom_output_topic},
            {"parent_frame": _value(context, profile, "odom_parent_frame", "odom_parent_frame")},
            {"child_frame": _value(context, profile, "odom_child_frame", "odom_child_frame")},
            {"stamp_mode": LaunchConfiguration("odom_stamp_mode")},
            {"pose_source": LaunchConfiguration("odom_pose_source")},
            {"fallback_to_message": LaunchConfiguration("odom_fallback_to_message")},
        ],
        condition=IfCondition(LaunchConfiguration("launch_odom_adapter")),
    )

    pointcloud_axis_adapter = Node(
        package="graph_construction",
        executable="pointcloud_axis_adapter",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": _value(context, profile, "pointcloud_input_topic", "pointcloud_input_topic")},
            {"output_topic": aligned_lidar_topic},
            {"output_frame": pointcloud_output_frame},
            {"axis_mode": pointcloud_axis_mode},
        ],
        condition=IfCondition(LaunchConfiguration("launch_pointcloud_axis_adapter")),
    )

    wildos = Node(
        package="visual_navigation",
        executable="wildos",
        output="both",
        **wildos_extra_args,
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

    object_search_goal_mux = Node(
        package="visual_navigation",
        executable="object_search_goal_mux",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"output_goal_topic": goal_pose_topic},
            {"goal_viz_topic": _value(context, profile, "object_search_goal_viz_topic", "object_search_goal_viz_topic")},
            {"status_topic": _value(context, profile, "object_search_status_topic", "object_search_status_topic")},
            {"object_target_pose_topic": _value(context, profile, "object_target_pose_topic", "object_target_pose_topic")},
            {"object_reached_topic": _value(context, profile, "object_reached_topic", "object_reached_topic")},
            {"odom_topic": odom_output_topic},
            {"frame_id": global_frame},
            {
                "publish_rate": _float_value(
                    context,
                    profile,
                    "object_search_goal_publish_rate",
                    "object_search_goal_publish_rate",
                )
            },
            {
                "initial_goal_distance": _float_value(
                    context,
                    profile,
                    "object_search_initial_goal_distance",
                    "object_search_initial_goal_distance",
                )
            },
            {
                "initial_goal_heading_deg": _float_value(
                    context,
                    profile,
                    "object_search_initial_goal_heading_deg",
                    "object_search_initial_goal_heading_deg",
                )
            },
            {
                "target_timeout_sec": _float_value(
                    context,
                    profile,
                    "object_search_target_timeout_sec",
                    "object_search_target_timeout_sec",
                )
            },
            {
                "latch_target_after_first_detection": _bool_value(
                    context,
                    profile,
                    "object_search_latch_target_after_first_detection",
                    "object_search_latch_target_after_first_detection",
                )
            },
            {
                "latch_target_timeout_sec": _float_value(
                    context,
                    profile,
                    "object_search_latch_target_timeout_sec",
                    "object_search_latch_target_timeout_sec",
                )
            },
            {
                "memory_timeout_sec": _float_value(
                    context,
                    profile,
                    "object_search_memory_timeout_sec",
                    "object_search_memory_timeout_sec",
                )
            },
            {
                "memory_goal_distance": _float_value(
                    context,
                    profile,
                    "object_search_memory_goal_distance",
                    "object_search_memory_goal_distance",
                )
            },
            {
                "target_reached_radius": _float_value(
                    context,
                    profile,
                    "object_search_target_reached_radius",
                    "object_search_target_reached_radius",
                )
            },
            {
                "object_reached_timeout_sec": _float_value(
                    context,
                    profile,
                    "object_search_object_reached_timeout_sec",
                    "object_search_object_reached_timeout_sec",
                )
            },
            {
                "reached_latch_timeout_sec": _float_value(
                    context,
                    profile,
                    "object_search_reached_latch_timeout_sec",
                    "object_search_reached_latch_timeout_sec",
                )
            },
            {
                "object_reached_require_target_distance": _bool_value(
                    context,
                    profile,
                    "object_search_object_reached_require_target_distance",
                    "object_search_object_reached_require_target_distance",
                )
            },
            {
                "object_reached_max_target_distance": _float_value(
                    context,
                    profile,
                    "object_search_object_reached_max_target_distance",
                    "object_search_object_reached_max_target_distance",
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration("do_object_search")),
    )

    planner = _planner_node(ns, use_sim_time, planner_odom_topic, goal_pose_topic, scored_nav_graph_topic)
    path_follower = _path_follower_node(
        ns,
        use_sim_time,
        planner_odom_topic,
        _value(context, profile, "tracking_goal_pose_topic", "tracking_goal_pose_topic"),
        _value(context, profile, "path_topic", "path_topic"),
    )

    return [
        SetEnvironmentVariable("ROS_DOMAIN_ID", _value(context, profile, "ros_domain_id", "ros_domain_id")),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", _value(context, profile, "rmw_implementation", "rmw_implementation_3d")),
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
        odom_adapter,
        _lidar_static_tf(context, profile),
        pointcloud_axis_adapter,
        _camera_static_tf("front", camera_parent_frame, camera_transforms["front"], publish_camera_static_tf),
        _camera_static_tf("left", camera_parent_frame, camera_transforms["left"], publish_camera_static_tf),
        _camera_static_tf("right", camera_parent_frame, camera_transforms["right"], publish_camera_static_tf),
        elevation_mapping,
        TimerAction(period=LaunchConfiguration("graph_start_delay"), actions=[graph_construction]),
        TimerAction(period=LaunchConfiguration("visual_start_delay"), actions=[wildos]),
        TimerAction(period=LaunchConfiguration("planner_start_delay"), actions=[object_search_goal_mux, planner, path_follower]),
    ]


def _planner_node(ns, use_sim_time, odom_topic, goal_pose_topic, scored_nav_graph_topic):
    return Node(
        package="graphnav_planner",
        executable="planner_node",
        name="graphnav_planner",
        output="screen",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"frontier_dist_cost_factor": 2.0},
            {"goal_dist_cost_factor": 1.0},
            {"frontier_score_factor": 20.0},
            {"frontier_continuity_radius": 10.0},
            {"frontier_progress_timeout": 12.0},
            {"frontier_switch_margin": 2.0},
            {"revisit_cost_factor": 1.0},
            {"trav_class": "default"},
            {"goal_radius": 3.0},
            {"append_virtual_goal_to_path": False},
        ],
        remappings=[
            ("~/nav_graph", scored_nav_graph_topic),
            ("~/odom", odom_topic),
            ("~/goal_pose", goal_pose_topic),
        ],
    )


def _path_follower_node(ns, use_sim_time, odom_topic, tracking_goal_pose_topic, path_topic):
    return Node(
        package="graphnav_planner",
        executable="path_follower_node",
        name="graphnav_path_follower",
        output="screen",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"wp_lookahead_dist": 5.0},
        ],
        remappings=[
            ("~/path", path_topic),
            ("~/odom", odom_topic),
            ("~/goal_pose", tracking_goal_pose_topic),
        ],
    )


def _lidar_static_tf(context, profile):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="unitree_lidar_static_tf",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            _value(context, profile, "lidar_parent_frame", "lidar_parent_frame"),
            _value(context, profile, "lidar_frame", "lidar_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_lidar_static_tf")),
    )


def _camera_static_tf(name, parent_frame, transform_args, condition):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"unitree_{name}_camera_static_tf",
        output="screen",
        arguments=[*transform_args, parent_frame, f"{name}_camera"],
        condition=IfCondition(condition),
    )


def _camera_static_transforms(convention):
    """按仿真 profile 选择相机 optical frame 外参"""
    transforms = {
        "x_forward_y_left": {
            "front": ["0.30", "0.00", "0.20", "0.5", "-0.5", "0.5", "-0.5"],
            "left": ["0.00", "0.18", "0.20", "0.7071067812", "0.0", "0.0", "-0.7071067812"],
            "right": ["0.00", "-0.18", "0.20", "0.0", "0.7071067812", "-0.7071067812", "0.0"],
        },
        "y_forward_x_right": {
            "front": ["0.00", "0.30", "0.20", "0.7071067812", "0.0", "0.0", "-0.7071067812"],
            "left": ["-0.18", "0.00", "0.20", "0.5", "0.5", "-0.5", "-0.5"],
            "right": ["0.18", "0.00", "0.20", "0.5", "-0.5", "0.5", "-0.5"],
        },
        "negative_y_forward_x_right": {
            "front": ["0.00", "-0.30", "0.20", "0.0", "0.7071067812", "-0.7071067812", "0.0"],
            "left": ["-0.18", "0.00", "0.20", "0.5", "0.5", "-0.5", "-0.5"],
            "right": ["0.18", "0.00", "0.20", "0.5", "-0.5", "0.5", "-0.5"],
        },
    }
    if convention not in transforms:
        raise ValueError(f"Unsupported camera_static_tf_convention: {convention}")
    return transforms[convention]


def _profile_arg(name, profile_key):
    return DeclareLaunchArgument(
        name,
        default_value="",
        description=f"{profile_key_description(profile_key)}, 留空使用 topic profile",
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


def _prepend_pythonpath(path):
    """给 WildOS 子进程补仓库根目录, 让本地 explorfm 包可导入"""
    current_pythonpath = os.environ.get("PYTHONPATH")
    if current_pythonpath:
        return f"{path}:{current_pythonpath}"
    return path


def _package_config_path(package_name, config_name):
    config_path = Path(config_name).expanduser()
    if config_path.is_absolute():
        return str(config_path)
    return str(Path(get_package_share_directory(package_name)) / "configs" / config_name)


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _value(context, profile, arg_name, profile_key):
    override = _arg(context, arg_name)
    if override:
        return override
    return str(profile[profile_key])


def _bool_value(context, profile, arg_name, profile_key):
    raw_value = _value(context, profile, arg_name, profile_key)
    if isinstance(raw_value, bool):
        return raw_value
    normalized = str(raw_value).strip().lower()
    if normalized in {"true", "1", "yes", "on"}:
        return True
    if normalized in {"false", "0", "no", "off"}:
        return False
    raise ValueError(f"Invalid boolean value for {arg_name}: {raw_value}")


def _float_value(context, profile, arg_name, profile_key):
    raw_value = _value(context, profile, arg_name, profile_key)
    try:
        return float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid float value for {arg_name}: {raw_value}") from exc
