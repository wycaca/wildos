from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory("wildos_navigation"))
        / "config"
        / "navigation.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="wildos_navigation",
                executable="controller",
                name="wildos_navigation_controller",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
                respawn=True,
                respawn_delay=1.0,
            ),
            Node(
                package="wildos_navigation",
                executable="velocity_sender",
                name="wildos_velocity_sender",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
                respawn=True,
                respawn_delay=1.0,
            ),
        ]
    )
