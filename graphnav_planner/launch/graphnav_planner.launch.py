from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_path = str(
        Path(get_package_share_directory("graphnav_planner"))
        / "config"
        / "planner.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("nav_graph_topic", default_value="scored_nav_graph"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        DeclareLaunchArgument(
            "object_search_status_topic",
            default_value="/spot1/object_search_status",
        ),
        Node(
            package="graphnav_planner",
            executable="planner_node",
            name="graphnav_planner",
            namespace=LaunchConfiguration("ns"),
            output="screen",
            parameters=[
                config_path,
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    )
                },
            ],
            remappings=[
                ("~/nav_graph", LaunchConfiguration("nav_graph_topic")),
                ("~/odom", LaunchConfiguration("odom_topic")),
                ("~/goal_pose", LaunchConfiguration("goal_topic")),
                (
                    "~/object_search_status",
                    LaunchConfiguration("object_search_status_topic"),
                ),
            ],
        ),
    ])
