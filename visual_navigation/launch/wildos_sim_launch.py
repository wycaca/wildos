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
    odom_pose_source = LaunchConfiguration("odom_pose_source")
    odom_fallback_to_message = LaunchConfiguration("odom_fallback_to_message")
    publish_camera_static_tf = LaunchConfiguration("publish_camera_static_tf")
    camera_parent_frame = LaunchConfiguration("camera_parent_frame")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    front_camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="unitree_front_camera_static_tf",
        output="screen",
        arguments=[
            "0.30",
            "0.00",
            "0.20",
            "0.5",
            "-0.5",
            "0.5",
            "-0.5",
            camera_parent_frame,
            "front_camera",
        ],
        condition=IfCondition(publish_camera_static_tf),
    )

    left_camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="unitree_left_camera_static_tf",
        output="screen",
        arguments=[
            "0.00",
            "0.18",
            "0.20",
            "0.7071067812",
            "0.0",
            "0.0",
            "-0.7071067812",
            camera_parent_frame,
            "left_camera",
        ],
        condition=IfCondition(publish_camera_static_tf),
    )

    right_camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="unitree_right_camera_static_tf",
        output="screen",
        arguments=[
            "0.00",
            "-0.18",
            "0.20",
            "0.0",
            "0.7071067812",
            "-0.7071067812",
            "0.0",
            camera_parent_frame,
            "right_camera",
        ],
        condition=IfCondition(publish_camera_static_tf),
    )

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
                default_value="/odom",
                description="Source odometry topic",
            ),
            DeclareLaunchArgument(
                "odom_output_topic",
                default_value="/spot1/odom_for_scoring",
                description="Adapted odometry topic",
            ),
            DeclareLaunchArgument(
                "odom_parent_frame",
                default_value="odom",
                description="Adapted odometry header frame",
            ),
            DeclareLaunchArgument(
                "odom_child_frame",
                default_value="base_link",
                description="Adapted odometry child frame",
            ),
            DeclareLaunchArgument(
                "odom_stamp_mode",
                default_value="now",
                description="Use now or preserve for adapted odometry stamp",
            ),
            DeclareLaunchArgument(
                "odom_pose_source",
                default_value="tf",
                description="Use message or tf pose for adapted odometry",
            ),
            DeclareLaunchArgument(
                "odom_fallback_to_message",
                default_value="false",
                description="Fallback to input odom pose if TF pose is unavailable",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="3",
                description="ROS domain used by the simulator",
            ),
            DeclareLaunchArgument(
                "publish_camera_static_tf",
                default_value="true",
                description="Publish fallback static transforms for Isaac camera frames",
            ),
            DeclareLaunchArgument(
                "camera_parent_frame",
                default_value="base_link",
                description="Parent frame for fallback Isaac camera transforms",
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
                    {"pose_source": odom_pose_source},
                    {"fallback_to_message": odom_fallback_to_message},
                ],
                condition=IfCondition(launch_odom_adapter),
            ),
            front_camera_static_tf,
            left_camera_static_tf,
            right_camera_static_tf,
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
