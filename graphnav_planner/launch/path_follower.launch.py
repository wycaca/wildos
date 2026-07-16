from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ns", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("path_topic", default_value="/path2"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        DeclareLaunchArgument("wp_lookahead_dist", default_value="5.0"),
        Node(
            package="graphnav_planner",
            executable="path_follower_node",
            name="graphnav_path_follower",
            namespace=LaunchConfiguration("ns"),
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    )
                },
                {
                    "wp_lookahead_dist": ParameterValue(
                        LaunchConfiguration("wp_lookahead_dist"),
                        value_type=float,
                    )
                },
            ],
            remappings=[
                ("~/path", LaunchConfiguration("path_topic")),
                ("~/odom", LaunchConfiguration("odom_topic")),
                ("~/goal_pose", LaunchConfiguration("goal_topic")),
            ],
        ),
    ])
