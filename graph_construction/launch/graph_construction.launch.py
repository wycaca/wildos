from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    ns = LaunchConfiguration("ns")
    config = LaunchConfiguration("config")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "ns",
                default_value="spot1",
                description="Robot namespace",
            ),
            DeclareLaunchArgument(
                "config",
                default_value="graph_construction.yaml",
                description="Config file installed by graph_construction",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Logging level",
            ),
            Node(
                package="graph_construction",
                executable="graph_construction",
                output="screen",
                namespace=ns,
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
                remappings=[
                    ("/tf", PathJoinSubstitution([TextSubstitution(text="/"), ns, TextSubstitution(text="tf")])),
                    (
                        "/tf_static",
                        PathJoinSubstitution([TextSubstitution(text="/"), ns, TextSubstitution(text="tf_static")]),
                    ),
                ],
            ),
        ]
    )
