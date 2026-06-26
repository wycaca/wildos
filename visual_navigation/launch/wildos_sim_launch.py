from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = LaunchConfiguration("config")
    do_object_search = LaunchConfiguration("do_object_search")
    log_level = LaunchConfiguration("log_level")
    launch_odom_adapter = LaunchConfiguration("launch_odom_adapter")
    odom_input_topic = LaunchConfiguration("odom_input_topic")
    odom_output_topic = LaunchConfiguration("odom_output_topic")
    odom_parent_frame = LaunchConfiguration("odom_parent_frame")
    odom_child_frame = LaunchConfiguration("odom_child_frame")
    odom_stamp_mode = LaunchConfiguration("odom_stamp_mode")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "config",
                default_value="wildos_nav_sim_conf.yaml",
                description="Config file installed by visual_navigation",
            ),
            DeclareLaunchArgument(
                "do_object_search",
                default_value="false",
                description="Enable object search",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="INFO",
                description="Logging level",
            ),
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
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
