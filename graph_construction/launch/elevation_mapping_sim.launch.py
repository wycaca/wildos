from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    elevation_config = LaunchConfiguration("elevation_config")
    adapter_config = LaunchConfiguration("adapter_config")
    log_level = LaunchConfiguration("log_level")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    elevation_share = get_package_share_directory("elevation_mapping_cupy")
    core_param = PathJoinSubstitution([elevation_share, "config", "core", "core_param.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "elevation_config",
                default_value="elevation_mapping_sim.yaml",
                description="Graph construction package config for elevation_mapping_cupy",
            ),
            DeclareLaunchArgument(
                "adapter_config",
                default_value="grid_map_to_occupancy.yaml",
                description="Config file for GridMap to OccupancyGrid adapter",
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
            Node(
                package="elevation_mapping_cupy",
                executable="elevation_mapping_node.py",
                name="elevation_mapping_node",
                output="screen",
                parameters=[
                    core_param,
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("graph_construction"),
                            "configs",
                            elevation_config,
                        ]
                    ),
                    {"use_sim_time": use_sim_time},
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
            ),
            Node(
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
            ),
        ]
    )
