from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ns = LaunchConfiguration("ns")
    elevation_config = LaunchConfiguration("elevation_config")
    graph_config = LaunchConfiguration("graph_config")
    visual_config = LaunchConfiguration("visual_config")
    do_object_search = LaunchConfiguration("do_object_search")
    use_sim_time = LaunchConfiguration("use_sim_time")
    graph_start_delay = LaunchConfiguration("graph_start_delay")
    visual_start_delay = LaunchConfiguration("visual_start_delay")
    log_level = LaunchConfiguration("log_level")
    launch_odom_adapter = LaunchConfiguration("launch_odom_adapter")
    odom_input_topic = LaunchConfiguration("odom_input_topic")
    odom_output_topic = LaunchConfiguration("odom_output_topic")
    odom_parent_frame = LaunchConfiguration("odom_parent_frame")
    odom_child_frame = LaunchConfiguration("odom_child_frame")
    odom_stamp_mode = LaunchConfiguration("odom_stamp_mode")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    elevation_share = get_package_share_directory("elevation_mapping_cupy")
    graph_share = get_package_share_directory("graph_construction")
    core_param = PathJoinSubstitution([elevation_share, "config", "core", "core_param.yaml"])

    elevation_mapping = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping_node",
        output="screen",
        parameters=[
            core_param,
            PathJoinSubstitution([graph_share, "configs", elevation_config]),
            {"use_sim_time": use_sim_time},
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
    )

    graph_construction = Node(
        package="graph_construction",
        executable="graph_construction",
        output="screen",
        namespace=ns,
        arguments=[
            "--config",
            graph_config,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/tf", PathJoinSubstitution([TextSubstitution(text="/"), ns, TextSubstitution(text="tf")])),
            (
                "/tf_static",
                PathJoinSubstitution([TextSubstitution(text="/"), ns, TextSubstitution(text="tf_static")]),
            ),
        ],
    )

    odom_adapter = Node(
        package="visual_navigation",
        executable="odom_frame_adapter",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": odom_input_topic},
            {"output_topic": odom_output_topic},
            {"parent_frame": odom_parent_frame},
            {"child_frame": odom_child_frame},
            {"stamp_mode": odom_stamp_mode},
        ],
        condition=IfCondition(launch_odom_adapter),
    )

    wildos = Node(
        package="visual_navigation",
        executable="wildos",
        output="screen",
        arguments=[
            "--config",
            visual_config,
            "--do_object_search",
            do_object_search,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ns", default_value="spot1", description="Robot namespace"),
            DeclareLaunchArgument(
                "elevation_config",
                default_value="elevation_mapping_sim.yaml",
                description="Config file for the experimental elevation mapping backend",
            ),
            DeclareLaunchArgument(
                "graph_config",
                default_value="graph_construction_elevation.yaml",
                description="Config file for graph_construction",
            ),
            DeclareLaunchArgument(
                "visual_config",
                default_value="wildos_nav_sim_conf.yaml",
                description="Config file installed by visual_navigation",
            ),
            DeclareLaunchArgument(
                "do_object_search",
                default_value="false",
                description="Enable object search",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
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
            DeclareLaunchArgument("log_level", default_value="INFO", description="Logging level"),
            DeclareLaunchArgument(
                "launch_odom_adapter",
                default_value="true",
                description="Publish odom with frame ids expected by WildOS",
            ),
            DeclareLaunchArgument(
                "odom_input_topic",
                default_value="/unity/odom",
                description="Source odometry topic",
            ),
            DeclareLaunchArgument(
                "odom_output_topic",
                default_value="/spot1/odom_for_scoring",
                description="Adapted odometry topic",
            ),
            DeclareLaunchArgument(
                "odom_parent_frame",
                default_value="map",
                description="Adapted odometry header frame",
            ),
            DeclareLaunchArgument(
                "odom_child_frame",
                default_value="odom_fram",
                description="Adapted odometry child frame",
            ),
            DeclareLaunchArgument(
                "odom_stamp_mode",
                default_value="now",
                description="Use now or preserve for adapted odometry stamp",
            ),
            DeclareLaunchArgument("ros_domain_id", default_value="3", description="ROS domain used by simulator"),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="rmw_cyclonedds_cpp",
                description="RMW implementation used by simulator",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            elevation_mapping,
            TimerAction(period=graph_start_delay, actions=[graph_construction]),
            TimerAction(period=visual_start_delay, actions=[odom_adapter, wildos]),
        ]
    )
