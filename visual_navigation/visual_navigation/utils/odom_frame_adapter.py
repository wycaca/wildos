import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class OdomFrameAdapter(Node):
    def __init__(self):
        super().__init__("odom_frame_adapter")

        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "odom_fram")
        self.declare_parameter("stamp_mode", "now")
        self.declare_parameter("pose_source", "message")
        self.declare_parameter("fallback_to_message", True)
        self.declare_parameter("tf_lookup_timeout", 0.05)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.stamp_mode = self.get_parameter("stamp_mode").value
        self.pose_source = self.get_parameter("pose_source").value
        self.fallback_to_message = self.get_parameter("fallback_to_message").value
        self.tf_lookup_timeout = float(self.get_parameter("tf_lookup_timeout").value)
        self._logged_first_message = False
        self._logged_tf_failure = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self._on_odom,
            10,
        )

        self.get_logger().info(
            f"odom frame adapter 已启动, input={self.input_topic}, output={self.output_topic}, "
            f"parent_frame={self.parent_frame}, child_frame={self.child_frame}, "
            f"stamp_mode={self.stamp_mode}, pose_source={self.pose_source}"
        )

    def _on_odom(self, msg: Odometry):
        # 统一 frame id, 必要时使用 TF 位姿替换 Isaac odom topic 位姿
        adapted_msg = copy.deepcopy(msg)
        adapted_msg.header.frame_id = self.parent_frame
        adapted_msg.child_frame_id = self.child_frame
        if self.pose_source == "tf":
            tf_msg = self._lookup_odom_tf()
            if tf_msg is None:
                if not self.fallback_to_message:
                    return
            else:
                self._apply_tf_pose(adapted_msg, tf_msg)
        elif self.pose_source != "message":
            self.get_logger().warn_once(
                f"未知 pose_source={self.pose_source}, 将使用消息中的 pose"
            )
        if self.stamp_mode == "now":
            adapted_msg.header.stamp = self.get_clock().now().to_msg()
        elif self.stamp_mode != "preserve":
            self.get_logger().warn_once(
                f"未知 stamp_mode={self.stamp_mode}, 将保留输入时间戳"
            )
        self.publisher.publish(adapted_msg)

        if not self._logged_first_message:
            self.get_logger().info(
                "已发布第一帧适配后的 odom, "
                f"input_frame={msg.header.frame_id}, input_child={msg.child_frame_id}, "
                f"output_frame={adapted_msg.header.frame_id}, output_child={adapted_msg.child_frame_id}, "
                f"position=({adapted_msg.pose.pose.position.x:.3f}, "
                f"{adapted_msg.pose.pose.position.y:.3f}, {adapted_msg.pose.pose.position.z:.3f})"
            )
            self._logged_first_message = True

    def _lookup_odom_tf(self) -> TransformStamped | None:
        """读取最新 TF 位姿, 避免 Isaac /odom topic 与 TF 坐标不一致"""
        try:
            return self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
        except Exception as exc:
            if not self._logged_tf_failure:
                self.get_logger().warn(
                    f"暂时无法读取 odom TF 位姿, target={self.parent_frame}, "
                    f"source={self.child_frame}, error={exc}"
                )
                self._logged_tf_failure = True
            return None

    def _apply_tf_pose(self, odom_msg: Odometry, tf_msg: TransformStamped) -> None:
        """用 TF translation 和 rotation 覆盖 odom pose"""
        odom_msg.pose.pose.position.x = tf_msg.transform.translation.x
        odom_msg.pose.pose.position.y = tf_msg.transform.translation.y
        odom_msg.pose.pose.position.z = tf_msg.transform.translation.z
        odom_msg.pose.pose.orientation = tf_msg.transform.rotation


def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
