from __future__ import annotations

import argparse
from dataclasses import dataclass, fields
from math import ceil, floor, hypot, isfinite
from pathlib import Path
from typing import Any, Dict, Iterator, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

from graph_construction.grid_adapter import bresenham_line


Point3D = Tuple[float, float, float]
GridIndex = Tuple[int, int]


DEFAULT_CONFIG: Dict[str, Any] = {
    "grid_frame": "map",
    "lidar_topic": "/livox/lidar",
    "odom_topic": "/unity/odom",
    "grid_topic": "/spot1/traversability_grid",
    "publish_rate_hz": 5.0,
    "resolution": 0.2,
    "local_width": 30.0,
    "local_height": 30.0,
    "min_range": 0.3,
    "max_range": 25.0,
    "min_obstacle_height": 0.15,
    "max_obstacle_height": 1.8,
    "obstacle_inflation_radius": 0.3,
    "unknown_value": -1,
    "free_value": 0,
    "obstacle_value": 100,
    "assume_input_in_grid_frame": False,
    "tf_timeout_sec": 0.1,
}


@dataclass
class LivoxGridBuilderConfig:
    """Livox 点云转局部 OccupancyGrid 的运行参数"""

    grid_frame: str = "map"
    lidar_topic: str = "/livox/lidar"
    odom_topic: str = "/unity/odom"
    grid_topic: str = "/spot1/traversability_grid"
    publish_rate_hz: float = 5.0
    resolution: float = 0.2
    local_width: float = 30.0
    local_height: float = 30.0
    min_range: float = 0.3
    max_range: float = 25.0
    min_obstacle_height: float = 0.15
    max_obstacle_height: float = 1.8
    obstacle_inflation_radius: float = 0.3
    unknown_value: int = -1
    free_value: int = 0
    obstacle_value: int = 100
    assume_input_in_grid_frame: bool = False
    tf_timeout_sec: float = 0.1


class LivoxGridBuilder(Node):
    """将 Livox PointCloud2 转换成 Graph Construction 使用的局部 OccupancyGrid"""

    def __init__(self, config: Dict[str, Any]) -> None:
        super().__init__("livox_grid_builder")
        self.config = _builder_config({**DEFAULT_CONFIG, **config})
        self.latest_cloud: Optional[PointCloud2] = None
        self.latest_odom: Optional[Odometry] = None
        self._logged_first_cloud = False
        self._logged_first_odom = False
        self._logged_first_grid = False
        self._logged_missing_tf = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grid_pub = self.create_publisher(OccupancyGrid, self.config.grid_topic, 10)
        self.create_subscription(PointCloud2, self.config.lidar_topic, self._on_cloud, 10)
        self.create_subscription(Odometry, self.config.odom_topic, self._on_odom, 10)

        publish_rate = max(0.1, float(self.config.publish_rate_hz))
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.get_logger().info(
            f"Livox grid builder started, lidar={self.config.lidar_topic}, grid={self.config.grid_topic}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        """缓存最新点云, 计算放在 timer 中执行"""
        self.latest_cloud = msg
        if not self._logged_first_cloud:
            self.get_logger().info(f"Received first cloud, frame={msg.header.frame_id}")
            self._logged_first_cloud = True

    def _on_odom(self, msg: Odometry) -> None:
        """缓存最新 odom, 用于确定局部 grid 中心"""
        self.latest_odom = msg
        if not self._logged_first_odom:
            self.get_logger().info(f"Received first odom, frame={msg.header.frame_id}")
            self._logged_first_odom = True

    def _on_timer(self) -> None:
        """周期性把最新点云投影成局部 OccupancyGrid"""
        if self.latest_cloud is None or self.latest_odom is None:
            return

        transform = self._lookup_cloud_transform(self.latest_cloud)
        if transform is None and not self.config.assume_input_in_grid_frame:
            return

        grid_msg = self._build_grid(self.latest_cloud, self.latest_odom, transform)
        self.grid_pub.publish(grid_msg)
        if not self._logged_first_grid:
            self.get_logger().info(
                f"Published first grid, frame={grid_msg.header.frame_id}, size={grid_msg.info.width}x{grid_msg.info.height}"
            )
            self._logged_first_grid = True

    def _lookup_cloud_transform(self, cloud: PointCloud2) -> Optional[TransformStamped]:
        """查询点云 frame 到 grid frame 的 TF, 简化模式下跳过查询"""
        if self.config.assume_input_in_grid_frame or cloud.header.frame_id == self.config.grid_frame:
            return None

        try:
            return self.tf_buffer.lookup_transform(
                self.config.grid_frame,
                cloud.header.frame_id,
                Time(),
                timeout=Duration(seconds=float(self.config.tf_timeout_sec)),
            )
        except Exception as exc:
            if not self._logged_missing_tf:
                self.get_logger().warn(
                    f"Missing TF from {cloud.header.frame_id} to {self.config.grid_frame}, error={exc}"
                )
                self._logged_missing_tf = True
            return None

    def _build_grid(
        self,
        cloud: PointCloud2,
        odom: Odometry,
        transform: Optional[TransformStamped],
    ) -> OccupancyGrid:
        """把点云投影到机器人附近局部 grid

        射线经过的 cell 标为 free, 命中点附近按膨胀半径标为 obstacle
        未被射线观测到的区域保持 unknown, 用于后续 frontier 检测
        """
        width = max(1, int(ceil(self.config.local_width / self.config.resolution)))
        height = max(1, int(ceil(self.config.local_height / self.config.resolution)))
        robot = _odom_position(odom)
        origin_x = robot[0] - width * self.config.resolution * 0.5
        origin_y = robot[1] - height * self.config.resolution * 0.5
        sensor_origin = self._sensor_origin(robot, transform)
        grid = np.full((height, width), int(self.config.unknown_value), dtype=np.int16)
        origin_index = _world_to_grid(sensor_origin[0], sensor_origin[1], origin_x, origin_y, self.config.resolution)

        for raw_point in _iter_cloud_xyz(cloud):
            point = _transform_point(raw_point, transform) if transform is not None else raw_point
            if not _is_valid_point(point):
                continue

            distance = hypot(point[0] - sensor_origin[0], point[1] - sensor_origin[1])
            if distance < self.config.min_range or distance > self.config.max_range:
                continue

            endpoint = _world_to_grid(point[0], point[1], origin_x, origin_y, self.config.resolution)

            self._mark_ray_free(grid, origin_index, endpoint)
            relative_z = point[2] - robot[2]
            if self.config.min_obstacle_height <= relative_z <= self.config.max_obstacle_height:
                self._mark_obstacle(grid, endpoint)

        return self._grid_message(cloud, grid, origin_x, origin_y, width, height)

    def _sensor_origin(self, robot: Point3D, transform: Optional[TransformStamped]) -> Point3D:
        """返回 grid frame 下的传感器原点"""
        if transform is None:
            return robot
        translation = transform.transform.translation
        return float(translation.x), float(translation.y), float(translation.z)

    def _mark_ray_free(self, grid: np.ndarray, start: GridIndex, end: GridIndex) -> None:
        """把传感器到命中点之间的 cell 标记为 free"""
        cells = list(bresenham_line(start[0], start[1], end[0], end[1]))
        for ix, iy in cells[:-1]:
            if 0 <= iy < grid.shape[0] and 0 <= ix < grid.shape[1]:
                if grid[iy, ix] != self.config.obstacle_value:
                    grid[iy, ix] = self.config.free_value

    def _mark_obstacle(self, grid: np.ndarray, center: GridIndex) -> None:
        """按配置半径膨胀障碍 cell"""
        radius_cells = max(0, int(ceil(self.config.obstacle_inflation_radius / self.config.resolution)))
        cx, cy = center
        for iy in range(cy - radius_cells, cy + radius_cells + 1):
            for ix in range(cx - radius_cells, cx + radius_cells + 1):
                if not (0 <= iy < grid.shape[0] and 0 <= ix < grid.shape[1]):
                    continue
                if hypot(ix - cx, iy - cy) * self.config.resolution <= self.config.obstacle_inflation_radius:
                    grid[iy, ix] = self.config.obstacle_value

    def _grid_message(
        self,
        cloud: PointCloud2,
        grid: np.ndarray,
        origin_x: float,
        origin_y: float,
        width: int,
        height: int,
    ) -> OccupancyGrid:
        """组装 nav_msgs/OccupancyGrid 消息"""
        msg = OccupancyGrid()
        msg.header.stamp = cloud.header.stamp
        msg.header.frame_id = self.config.grid_frame
        msg.info.resolution = float(self.config.resolution)
        msg.info.width = int(width)
        msg.info.height = int(height)
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.astype(np.int8).reshape(-1).tolist()
        return msg


def _iter_cloud_xyz(cloud: PointCloud2) -> Iterator[Point3D]:
    """从 PointCloud2 中迭代 XYZ 点, 兼容 tuple 和 structured array 返回值"""
    points = point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
    for point in points:
        if isinstance(point, np.void) and point.dtype.names:
            yield float(point["x"]), float(point["y"]), float(point["z"])
        else:
            yield float(point[0]), float(point[1]), float(point[2])


def _transform_point(point: Point3D, transform: TransformStamped) -> Point3D:
    """手写四元数旋转和平移, 避免额外依赖 tf2_sensor_msgs"""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    rotated = _rotate_point(point, (rotation.x, rotation.y, rotation.z, rotation.w))
    return (
        rotated[0] + float(translation.x),
        rotated[1] + float(translation.y),
        rotated[2] + float(translation.z),
    )


def _rotate_point(point: Point3D, quat: Tuple[float, float, float, float]) -> Point3D:
    """使用 q * p * q^-1 旋转点"""
    qx, qy, qz, qw = quat
    px, py, pz = point
    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)
    return (
        px + qw * tx + (qy * tz - qz * ty),
        py + qw * ty + (qz * tx - qx * tz),
        pz + qw * tz + (qx * ty - qy * tx),
    )


def _world_to_grid(x: float, y: float, origin_x: float, origin_y: float, resolution: float) -> GridIndex:
    """把世界坐标转换为局部 grid 索引, 允许索引落在地图外"""
    ix = int(floor((x - origin_x) / resolution))
    iy = int(floor((y - origin_y) / resolution))
    return ix, iy


def _is_valid_point(point: Point3D) -> bool:
    """过滤 NaN 和 Inf 点"""
    return isfinite(point[0]) and isfinite(point[1]) and isfinite(point[2])


def _odom_position(odom: Odometry) -> Point3D:
    """提取 odom 中的机器人位置"""
    position = odom.pose.pose.position
    return float(position.x), float(position.y), float(position.z)


def _builder_config(config: Dict[str, Any]) -> LivoxGridBuilderConfig:
    """从完整配置中提取 LivoxGridBuilderConfig 字段"""
    allowed = {field.name for field in fields(LivoxGridBuilderConfig)}
    values = {
        key: value
        for key, value in config.items()
        if key in allowed
    }
    return LivoxGridBuilderConfig(**values)


def _load_config(config_name: str) -> Dict[str, Any]:
    """加载安装目录或绝对路径中的 yaml 配置"""
    config_path = Path(config_name)
    if not config_path.is_absolute():
        share_dir = Path(get_package_share_directory("graph_construction"))
        config_path = share_dir / "configs" / config_name

    if not config_path.exists():
        return {}

    try:
        import yaml
    except ImportError:
        return {}

    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}
    return loaded


def main(args=None) -> None:
    """ROS2 console script 入口"""
    rclpy.init(args=args)
    custom_args = rclpy.utilities.remove_ros_args(args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="livox_grid_builder.yaml")
    parsed = parser.parse_args(custom_args[1:] if custom_args else [])

    node = LivoxGridBuilder(_load_config(parsed.config))
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
