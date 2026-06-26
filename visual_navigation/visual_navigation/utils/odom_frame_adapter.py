import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomFrameAdapter(Node):
    def __init__(self):
        super().__init__("odom_frame_adapter")

        self.declare_parameter("input_topic", "/unity/odom")
        self.declare_parameter("output_topic", "/spot1/odom_for_scoring")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "odom_fram")
        self.declare_parameter("stamp_mode", "now")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.child_frame = self.get_parameter("child_frame").value
        self.stamp_mode = self.get_parameter("stamp_mode").value
        self._logged_first_message = False

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self._on_odom,
            10,
        )

        self.get_logger().info(
            f"Adapting odom from {self.input_topic} to {self.output_topic}, "
            f"parent_frame={self.parent_frame}, child_frame={self.child_frame}, "
            f"stamp_mode={self.stamp_mode}"
        )

    def _on_odom(self, msg: Odometry):
        # Normalize frame ids and optionally align stale odom stamps to ROS time
        adapted_msg = copy.deepcopy(msg)
        adapted_msg.header.frame_id = self.parent_frame
        adapted_msg.child_frame_id = self.child_frame
        if self.stamp_mode == "now":
            adapted_msg.header.stamp = self.get_clock().now().to_msg()
        elif self.stamp_mode != "preserve":
            self.get_logger().warn_once(
                f"Unknown stamp_mode={self.stamp_mode}, preserving input stamp"
            )
        self.publisher.publish(adapted_msg)

        if not self._logged_first_message:
            self.get_logger().info(
                "Published first adapted odom message, "
                f"input_frame={msg.header.frame_id}, input_child={msg.child_frame_id}, "
                f"output_frame={adapted_msg.header.frame_id}, output_child={adapted_msg.child_frame_id}, "
                f"position=({adapted_msg.pose.pose.position.x:.3f}, "
                f"{adapted_msg.pose.pose.position.y:.3f}, {adapted_msg.pose.pose.position.z:.3f})"
            )
            self._logged_first_message = True


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
