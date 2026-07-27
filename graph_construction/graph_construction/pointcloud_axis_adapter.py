from __future__ import annotations

import copy
from typing import Callable, Iterable, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


PointXYZ = Tuple[float, float, float]


class PointCloudAxisAdapter(Node):
    """修正 Isaac LiDAR 点云轴向, 让 elevation mapping 使用 ROS 风格坐标"""

    def __init__(self) -> None:
        super().__init__("pointcloud_axis_adapter")

        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_aligned")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("axis_mode", "isaac_lidar_to_base")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.axis_mode = self.get_parameter("axis_mode").value
        self.transform_point = _axis_transform(self.axis_mode)
        self._logged_first_cloud = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)
        self.create_subscription(PointCloud2, self.input_topic, self._on_cloud, sensor_qos)

        self.get_logger().info(
            f"PointCloud axis adapter started, input={self.input_topic}, "
            f"output={self.output_topic}, frame={self.output_frame}, mode={self.axis_mode}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        """转换 XYZ 并丢弃非几何字段, elevation mapping 只需要点坐标"""
        if self.axis_mode == "identity":
            aligned_msg = _identity_cloud(msg, self.output_frame)
        else:
            points = (self.transform_point(point) for point in _iter_xyz(msg))
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.output_frame or msg.header.frame_id
            aligned_msg = point_cloud2.create_cloud(header, _xyz_fields(), points)
        self.publisher.publish(aligned_msg)

        if not self._logged_first_cloud:
            self.get_logger().info(
                f"Published first aligned cloud, input_frame={msg.header.frame_id}, "
                f"output_frame={aligned_msg.header.frame_id}"
            )
            self._logged_first_cloud = True


def _axis_transform(axis_mode: str) -> Callable[[PointXYZ], PointXYZ]:
    """返回固定轴向变换, 复杂坐标约定集中在这里便于联调"""
    if axis_mode == "identity":
        return lambda point: point
    if axis_mode in ("isaac_lidar_to_base", "neg_x_forward", "neg_xy_keep_z"):
        return lambda point: (-point[0], -point[1], point[2])
    if axis_mode == "negate_xyz":
        return lambda point: (-point[0], -point[1], -point[2])
    if axis_mode in ("isaac_y_forward_to_base", "y_forward"):
        return lambda point: (point[1], -point[0], point[2])
    raise ValueError(f"Unsupported axis_mode={axis_mode}")


def _identity_cloud(cloud: PointCloud2, output_frame: str) -> PointCloud2:
    """Preserve all fields and avoid Python point iteration for aligned input"""
    target_frame = str(output_frame).strip() or cloud.header.frame_id
    if target_frame == cloud.header.frame_id:
        return cloud
    adapted = copy.deepcopy(cloud)
    adapted.header.frame_id = target_frame
    return adapted


def _iter_xyz(cloud: PointCloud2) -> Iterable[PointXYZ]:
    """读取 PointCloud2 的 XYZ 字段"""
    points = point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
    for point in points:
        yield (float(point[0]), float(point[1]), float(point[2]))


def _xyz_fields() -> list[PointField]:
    """生成只包含 XYZ 的 PointCloud2 字段描述"""
    return [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]


def main(args=None) -> None:
    """ROS2 控制台脚本入口"""
    rclpy.init(args=args)
    node = PointCloudAxisAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
