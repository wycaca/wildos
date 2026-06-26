from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ns = LaunchConfiguration("ns")
    elevation_config = LaunchConfiguration("elevation_config")
    adapter_config = LaunchConfiguration("adapter_config")
    graph_config = LaunchConfiguration("graph_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    graph_start_delay = LaunchConfiguration("graph_start_delay")
    log_level = LaunchConfiguration("log_level")
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

    grid_adapter = Node(
        package="graph_construction",
        executable="grid_map_to_occupancy",
        output="screen",
        arguments=[
            "--config",
            adapter_config,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("ns", default_value="spot1", description="Robot namespace"),
            DeclareLaunchArgument(
                "elevation_config",
                default_value="elevation_mapping_sim.yaml",
                description="Config file for the experimental elevation mapping backend",
            ),
            DeclareLaunchArgument(
                "adapter_config",
                default_value="grid_map_to_occupancy.yaml",
                description="Config file for the experimental GridMap to OccupancyGrid adapter",
            ),
            DeclareLaunchArgument(
                "graph_config",
                default_value="graph_construction_elevation.yaml",
                description="Config file for graph_construction",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "graph_start_delay",
                default_value="3.0",
                description="Delay graph_construction startup so the experimental elevation grid is advertised first",
            ),
            DeclareLaunchArgument("log_level", default_value="INFO", description="Logging level"),
            DeclareLaunchArgument("ros_domain_id", default_value="3", description="ROS domain used by simulator"),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="rmw_cyclonedds_cpp",
                description="RMW implementation used by simulator",
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            elevation_mapping,
            grid_adapter,
            TimerAction(period=graph_start_delay, actions=[graph_construction]),
        ]
    )
