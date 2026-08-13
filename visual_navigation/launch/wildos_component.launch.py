from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config")
    do_object_search = LaunchConfiguration("do_object_search")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="wildos_nav_conf.yaml",
                description="Config file installed by visual_navigation",
            ),
            DeclareLaunchArgument(
                "do_object_search",
                default_value="false",
                description="Enable object search target fusion",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Logging level",
            ),
            Node(
                package="visual_navigation",
                executable="wildos",
                output="screen",
                arguments=[
                    "--config",
                    config,
                    "--do_object_search",
                    do_object_search,
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="visual_navigation",
                executable="object_target_fusion",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(do_object_search),
            ),
        ]
    )
