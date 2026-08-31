from collections import Counter
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from wildos_navigation.performance_stats import (
    EventRate,
    TimingWindow,
    diagnostic_status,
    message_age_ms,
    timing_metrics,
)


_TIMING_STAGES = ("total", "decode", "filter", "project", "grid", "publish")


def _map_diagnostic_metrics(summaries, cloud_rate, counters, cloud_age_ms):
    """构造代价地图稳定指标名称"""
    metrics = {"input.cloud.rate_hz": cloud_rate}
    if cloud_age_ms is not None:
        metrics["input.cloud.age_ms"] = cloud_age_ms
    for stage, summary in summaries.items():
        metrics.update(timing_metrics(f"stage.{stage}", summary))
    metrics["cycle.total.average_ms"] = summaries["total"].average_ms
    metrics["cycle.total.p95_ms"] = summaries["total"].p95_ms
    metrics["cycle.total.maximum_ms"] = summaries["total"].maximum_ms
    metrics.update({f"workload.{key}": value for key, value in counters.items()})
    return metrics


class LocalObstacleGridNode(Node):
    def __init__(self):
        super().__init__('map_pub')
        # 声明并获取参数
        self.declare_parameter('grid_width', 20.0)   # 20m x 20m 局部动态窗口
        self.declare_parameter('grid_height', 20.0)  
        self.declare_parameter('resolution', 0.1)    # 0.1米/格
        self.declare_parameter('min_height', 0.1)    # 相对狗地面的滤除高度
        self.declare_parameter('max_height', 1.2)    # 相对狗地面的最大检测高度
        self.declare_parameter('obstacle_radius', 0.2)
        self.declare_parameter('obstacle_height_threshold', 0.15)
        self.declare_parameter('odom_topic', '/Odometry')  
        self.declare_parameter('lidar_topic', '/cloud_registered')
        self.declare_parameter('odom_frame', 'odom_3D')
        self.declare_parameter('diagnostics_enabled', True)
        self.declare_parameter('diagnostics_period_sec', 30.0)
        self.declare_parameter('diagnostics_budget_ms', 100.0)

        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.resolution = self.get_parameter('resolution').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.obstacle_height_threshold = self.get_parameter('obstacle_height_threshold').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame').value
        self.diagnostics_enabled = bool(
            self.get_parameter('diagnostics_enabled').value
        )
        self.diagnostics_period_sec = float(
            self.get_parameter('diagnostics_period_sec').value
        )
        self.diagnostics_budget_ms = float(
            self.get_parameter('diagnostics_budget_ms').value
        )
        if self.diagnostics_period_sec <= 0.0:
            raise ValueError('diagnostics_period_sec must be positive')
        self.get_logger().info(
            f'Initializing Perfect Pixel-Aligned Rolling Local Costmap ({self.odom_frame_id})'
        )

        # 使用奇数尺寸保证唯一中心栅格
        self.width_cells = int(self.grid_width / self.resolution)
        if self.width_cells % 2 == 0:
            self.width_cells += 1
            
        self.height_cells = int(self.grid_height / self.resolution)
        if self.height_cells % 2 == 0:
            self.height_cells += 1
        
        # 物理尺寸与整数栅格严格一致
        self.grid_width = self.width_cells * self.resolution
        self.grid_height = self.height_cells * self.resolution

        # 固定中心栅格索引
        self.center_u = self.width_cells // 2
        self.center_v = self.height_cells // 2

        self.odom_topic = self.get_parameter('odom_topic').value

        # 初始化 OccupancyGrid 结构消息体
        self.grid_combined = OccupancyGrid()
        self.grid_combined.header.frame_id = self.odom_frame_id
        self.grid_combined.info.width = self.width_cells
        self.grid_combined.info.height = self.height_cells
        self.grid_combined.info.resolution = self.resolution

        # 创建订阅者和发布者
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.grid_combined_pub = self.create_publisher(OccupancyGrid, 'combined_grid', 10)

        self._timings = {
            stage: TimingWindow(max_samples=512)
            for stage in _TIMING_STAGES
        }
        self._cloud_rate = EventRate()
        self._diagnostic_counters = Counter()
        self._last_cloud_stamp_ns = None
        if self.diagnostics_enabled:
            diagnostic_qos = QoSProfile(depth=1)
            diagnostic_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.diagnostics_pub = self.create_publisher(
                DiagnosticArray,
                '/diagnostics',
                diagnostic_qos,
            )
            self.diagnostics_timer = self.create_timer(
                self.diagnostics_period_sec,
                self._publish_diagnostics,
            )
        
        self.odom_data = None

    def odom_callback(self, msg):
        self.odom_data = msg

    def pointcloud_callback(self, msg):
        total_started = time.perf_counter()
        self._record_cloud_input(msg)
        try:
            self._process_pointcloud(msg)
        finally:
            self._record_timing("total", total_started)

    def _process_pointcloud(self, msg):
        """执行点云到局部代价地图的原始数据路径"""
        if self.odom_data is None:
            self._count("dropped_no_odom")
            return

        # 1. 实时捕获机器狗当前在世界坐标系下的绝对 3D 物理坐标
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        robot_z = self.odom_data.pose.pose.position.z

        # 原点按 resolution 离散, 避免滚动窗口产生亚栅格抖动
        origin_x = math.floor((robot_x - (self.center_u * self.resolution)) / self.resolution) * self.resolution
        origin_y = math.floor((robot_y - (self.center_v * self.resolution)) / self.resolution) * self.resolution
        origin_z = robot_z - 0.05  # 固定压低 5cm 贴紧地面

        # 2. 读取点云数据
        stage_started = time.perf_counter()
        points_struct = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        self._record_timing("decode", stage_started)
        self._count("input_points", len(points_struct))
        if len(points_struct) == 0:
            self._count("dropped_empty_cloud")
            return

        stage_started = time.perf_counter()
        X = points_struct['x'].flatten()
        Y = points_struct['y'].flatten()
        Z = points_struct['z'].flatten()
        
        # 动态高度范围筛选
        min_z_dynamic = robot_z + self.min_height
        max_z_dynamic = robot_z + self.max_height
        
        height_mask = (Z >= min_z_dynamic) & (Z <= max_z_dynamic)
        X, Y, Z = X[height_mask], Y[height_mask], Z[height_mask]
        self._record_timing("filter", stage_started)
        self._count("height_filtered_points", len(X))
        if len(X) == 0:
            self._count("dropped_height_filter")
            return
        
        # 使用对齐原点投影栅格坐标
        stage_started = time.perf_counter()
        gu = ((X - origin_x) / self.resolution).astype(np.int32)
        gv = ((Y - origin_y) / self.resolution).astype(np.int32)
        
        # 边界过滤, 直接丢弃局部窗口外点云
        valid_bounds_mask = (gu >= 0) & (gu < self.width_cells) & \
                            (gv >= 0) & (gv < self.height_cells)
        gu = gu[valid_bounds_mask]
        gv = gv[valid_bounds_mask]
        Z = Z[valid_bounds_mask]
        self._record_timing("project", stage_started)
        self._count("projected_points", len(gu))
        
        if len(gu) == 0:
            self._count("dropped_outside_grid")
            return
        
        # 3. 网格统计与高度差双重滤波
        stage_started = time.perf_counter()
        max_z_grid = np.full((self.height_cells, self.width_cells), -1000.0, dtype=np.float32)
        min_z_grid = np.full((self.height_cells, self.width_cells), 1000.0, dtype=np.float32)
        np.maximum.at(max_z_grid, (gv, gu), Z)
        np.minimum.at(min_z_grid, (gv, gu), Z)

        wall_mask = (max_z_grid - min_z_grid) > self.obstacle_height_threshold
        table_mask = (max_z_grid > -500) & (max_z_grid > (robot_z + 0.3))
        obstacle_mask = wall_mask | table_mask

        # OpenCV 极速形态学膨胀
        radius_cells = int(self.obstacle_radius / self.resolution)
        obs_img = obstacle_mask.astype(np.uint8)
        def get_dilated_mask(base_img, layer_factor):
            r = layer_factor * radius_cells
            if r <= 0: return base_img > 0
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
            return cv2.dilate(base_img, kernel) > 0
        
        mask_l1 = get_dilated_mask(obs_img, 1)
        mask_l2 = get_dilated_mask(obs_img, 2)
        mask_l3 = get_dilated_mask(obs_img, 3)

        # 4. 构建地图像素阵列
        grid_array = np.zeros((self.height_cells, self.width_cells), dtype=np.int8)
        grid_array[mask_l3] = -120       
        grid_array[mask_l2] = -8         
        grid_array[mask_l1] = 5          
        grid_array[obstacle_mask] = 100  

        # 中心 3x3 栅格保持可通行, 避免机器人自身点云触发障碍
        grid_array[self.center_v-1:self.center_v+2, self.center_u-1:self.center_u+2] = 0
        self._record_timing("grid", stage_started)
        self._count("obstacle_cells", int(np.count_nonzero(obstacle_mask)))

        # 5. 打包并即时发布
        self.grid_combined.header.stamp = msg.header.stamp
        self.grid_combined.header.frame_id = self.odom_frame_id
        
        # 将对齐后的离散原点写入地图元数据
        self.grid_combined.info.origin.position.x = float(origin_x)
        self.grid_combined.info.origin.position.y = float(origin_y)
        self.grid_combined.info.origin.position.z = float(origin_z) 
        self.grid_combined.info.origin.orientation.w = 1.0
        
        stage_started = time.perf_counter()
        self.grid_combined.data = grid_array.flatten().tolist()
        self.grid_combined_pub.publish(self.grid_combined)
        self._record_timing("publish", stage_started)
        self._count("published_grids")

    def _record_timing(self, stage, started):
        timings = getattr(self, "_timings", None)
        if timings is not None:
            timings[stage].add_seconds(time.perf_counter() - started)

    def _count(self, name, value=1):
        counters = getattr(self, "_diagnostic_counters", None)
        if counters is not None:
            counters[name] += value

    def _record_cloud_input(self, msg):
        rate = getattr(self, "_cloud_rate", None)
        if rate is None:
            return
        rate.tick()
        self._diagnostic_counters["received_clouds"] += 1
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )
        self._last_cloud_stamp_ns = stamp_ns if stamp_ns > 0 else None

    def _cloud_age_ms(self):
        return message_age_ms(
            self.get_clock().now().nanoseconds,
            self._last_cloud_stamp_ns,
        )

    def _publish_diagnostics(self):
        """低频发布点云代价地图阶段与工作量标量"""
        summaries = {
            name: timing.summary(reset=True)
            for name, timing in self._timings.items()
        }
        metrics = _map_diagnostic_metrics(
            summaries,
            self._cloud_rate.sample(reset=True),
            dict(self._diagnostic_counters),
            self._cloud_age_ms(),
        )
        slow = summaries["total"].p95_ms >= self.diagnostics_budget_ms
        status = diagnostic_status(
            "wildos/map_pub",
            metrics,
            level=DiagnosticStatus.WARN if slow else DiagnosticStatus.OK,
            message="processing budget exceeded" if slow else "OK",
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)
        self._diagnostic_counters.clear()


def main(args=None):
    rclpy.init(args=args)
    node = LocalObstacleGridNode() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
