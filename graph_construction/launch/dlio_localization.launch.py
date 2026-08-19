from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the x86 DLIO service and canonical WildOS outputs"""
    raw_odom_topic = "/wildos/dlio/odom_raw"
    raw_pointcloud_topic = "/wildos/dlio/pointcloud/deskewed_raw"
    raw_tf_topic = "/wildos/dlio/tf_raw"
    health_topic = "/wildos/dlio/healthy"

    arguments = [
        DeclareLaunchArgument("dlio_config_file"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
        DeclareLaunchArgument("output_pointcloud_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("output_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("output_tf_topic", default_value="/tf"),
        DeclareLaunchArgument("global_frame", default_value="odom"),
        DeclareLaunchArgument("local_frame", default_value="dlio_odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="lidar_link"),
        DeclareLaunchArgument("imu_frame", default_value="imu_link"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    dlio = Node(
        package="direct_lidar_inertial_odometry",
        executable="dlio_odom_node",
        name="dlio_odom_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration("dlio_config_file"),
            {
                "use_sim_time": False,
                "adaptive": False,
                "frames/odom": LaunchConfiguration("local_frame"),
                "frames/baselink": LaunchConfiguration("base_frame"),
                "frames/lidar": LaunchConfiguration("lidar_frame"),
                "frames/imu": LaunchConfiguration("imu_frame"),
            },
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        remappings=[
            ("pointcloud", LaunchConfiguration("pointcloud_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
            ("odom", raw_odom_topic),
            ("pose", "/wildos/dlio/pose"),
            ("path", "/wildos/dlio/path"),
            ("kf_pose", "/wildos/dlio/keyframes"),
            ("kf_cloud", "/wildos/dlio/pointcloud/keyframe"),
            ("deskewed", raw_pointcloud_topic),
            ("/tf", raw_tf_topic),
            ("/tf_static", "/wildos/dlio/tf_static_raw"),
        ],
    )

    tf_adapter = Node(
        package="graph_construction",
        executable="dlio_tf_adapter",
        name="dlio_tf_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_odom_topic": raw_odom_topic,
                "input_raw_tf_topic": raw_tf_topic,
                "reference_odom_topic": "",
                "output_aligned_odom_topic": LaunchConfiguration(
                    "output_odom_topic"
                ),
                "output_tf_topic": LaunchConfiguration("output_tf_topic"),
                "global_frame": LaunchConfiguration("global_frame"),
                "local_frame": LaunchConfiguration("local_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "alignment_delay": 0.0,
                "health_topic": health_topic,
                "health_heartbeat_hz": 2.0,
            }
        ],
    )

    output_guard = Node(
        package="graph_construction",
        executable="dlio_output_guard",
        name="dlio_output_guard",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_pointcloud_topic": raw_pointcloud_topic,
                "output_pointcloud_topic": LaunchConfiguration(
                    "output_pointcloud_topic"
                ),
                "health_topic": health_topic,
                "health_timeout_sec": 1.5,
            }
        ],
    )

    return LaunchDescription([*arguments, dlio, tf_adapter, output_guard])
