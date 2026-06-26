from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration("ns")
    grid_config = LaunchConfiguration("grid_config")
    graph_config = LaunchConfiguration("graph_config")
    grid_use_sim_time = LaunchConfiguration("grid_use_sim_time")
    graph_use_sim_time = LaunchConfiguration("graph_use_sim_time")
    graph_start_delay = LaunchConfiguration("graph_start_delay")
    log_level = LaunchConfiguration("log_level")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    livox_grid_builder = Node(
        package="graph_construction",
        executable="livox_grid_builder",
        output="screen",
        arguments=[
            "--config",
            grid_config,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[
            {"use_sim_time": grid_use_sim_time},
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
            {"use_sim_time": graph_use_sim_time},
        ],
        remappings=[
            ("/tf", PathJoinSubstitution([TextSubstitution(text="/"), ns, TextSubstitution(text="tf")])),
            (
                "/tf_static",
                PathJoinSubstitution([TextSubstitution(text="/"), ns, TextSubstitution(text="tf_static")]),
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ns",
                default_value="spot1",
                description="Robot namespace",
            ),
            DeclareLaunchArgument(
                "grid_config",
                default_value="livox_grid_builder.yaml",
                description="Config file for the default LiDAR geometric traversability grid backend",
            ),
            DeclareLaunchArgument(
                "graph_config",
                default_value="graph_construction.yaml",
                description="Config file for graph_construction",
            ),
            DeclareLaunchArgument(
                "grid_use_sim_time",
                default_value="false",
                description="Use simulation clock for livox_grid_builder",
            ),
            DeclareLaunchArgument(
                "graph_use_sim_time",
                default_value="true",
                description="Use simulation clock for graph_construction",
            ),
            DeclareLaunchArgument(
                "graph_start_delay",
                default_value="1.0",
                description="Delay graph_construction startup so the grid topic is advertised first",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="3",
                description="ROS domain used by the simulator",
            ),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="rmw_cyclonedds_cpp",
                description="RMW implementation used by the simulator",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            livox_grid_builder,
            TimerAction(
                period=graph_start_delay,
                actions=[graph_construction],
            ),
        ]
    )
