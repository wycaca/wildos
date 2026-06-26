from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = LaunchConfiguration("config")
    log_level = LaunchConfiguration("log_level")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "config",
                default_value="livox_grid_builder.yaml",
                description="Config file installed by graph_construction",
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
                package="graph_construction",
                executable="livox_grid_builder",
                output="screen",
                arguments=[
                    "--config",
                    config,
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
