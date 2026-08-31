import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import cv2
import math
from rclpy.qos import qos_profile_sensor_data

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

        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.resolution = self.get_parameter('resolution').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.obstacle_height_threshold = self.get_parameter('obstacle_height_threshold').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame').value
        self.get_logger().info(
            f'Initializing Perfect Pixel-Aligned Rolling Local Costmap ({self.odom_frame_id})'
        )

        # 精确计算网格的行列数，并强制让它们保持为“奇数”
        self.width_cells = int(self.grid_width / self.resolution)
        if self.width_cells % 2 == 0: self.width_cells += 1
            
        self.height_cells = int(self.grid_height / self.resolution)
        if self.height_cells % 2 == 0: self.height_cells += 1
        
        # 重新微调实际网格物理尺寸
        self.grid_width = self.width_cells * self.resolution
        self.grid_height = self.height_cells * self.resolution

        # 锁定中心像素点索引（奇数保证了有唯一确定的中心像素）
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
        
        self.odom_data = None

    def odom_callback(self, msg):
        self.odom_data = msg

    def pointcloud_callback(self, msg):
        if self.odom_data is None:
            return

        # 1. 实时捕获机器狗当前在世界坐标系下的绝对 3D 物理坐标
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        robot_z = self.odom_data.pose.pose.position.z

        # ==================== 🛠️ 彻底更改完全 A：分辨率取整对齐原点 ====================
        # 💡 这是消灭微观发抖、让狗百分之百居中死锁的最核心修改！
        # 使用 math.floor(...) * resolution 迫使连续变动的位置只能以 0.1 米的整数倍跳动。
        # 这样它的浮点数余数在任何时候都恒等于 0，完美消除了所有的离散截断视差！
        origin_x = math.floor((robot_x - (self.center_u * self.resolution)) / self.resolution) * self.resolution
        origin_y = math.floor((robot_y - (self.center_v * self.resolution)) / self.resolution) * self.resolution
        origin_z = robot_z - 0.05  # 固定压低 5cm 贴紧地面

        # 2. 读取点云数据
        points_struct = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        if len(points_struct) == 0:
            return

        X = points_struct['x'].flatten()
        Y = points_struct['y'].flatten()
        Z = points_struct['z'].flatten()
        
        # 动态高度范围筛选
        min_z_dynamic = robot_z + self.min_height
        max_z_dynamic = robot_z + self.max_height
        
        height_mask = (Z >= min_z_dynamic) & (Z <= max_z_dynamic)
        X, Y, Z = X[height_mask], Y[height_mask], Z[height_mask]
        if len(X) == 0:
            return
        
        # ==================== 🛠️ 基于绝对对准原点的安全网格投影 ====================
        gu = ((X - origin_x) / self.resolution).astype(np.int32)
        gv = ((Y - origin_y) / self.resolution).astype(np.int32)
        
        # 边界过滤，超出当前局域滚动窗口之内的点云直接剪裁掉
        valid_bounds_mask = (gu >= 0) & (gu < self.width_cells) & \
                            (gv >= 0) & (gv < self.height_cells)
        gu = gu[valid_bounds_mask]
        gv = gv[valid_bounds_mask]
        Z = Z[valid_bounds_mask]
        
        if len(gu) == 0:
            return
        
        # 3. 网格统计与高度差双重滤波
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

        # ==================== 🛠️ 彻底更改完全 B：强行刷白机器狗物理中心 ====================
        # 💡 在最终发布的地图矩阵里，强制把机器狗正中央的 3x3 像素格刷成 0（安全通行）
        # 配合上面的 floor 离散，在代数上直接彻底锁死：狗身体中心在任何时候都绝对在 center 格上。
        grid_array[self.center_v-1:self.center_v+2, self.center_u-1:self.center_u+2] = 0

        # 5. 打包并即时发布
        self.grid_combined.header.stamp = msg.header.stamp
        self.grid_combined.header.frame_id = self.odom_frame_id
        
        # 将对齐取整后的高精离散原点，丝毫不差地打包发给 Rviz 和 A* 节点
        self.grid_combined.info.origin.position.x = float(origin_x)
        self.grid_combined.info.origin.position.y = float(origin_y)
        self.grid_combined.info.origin.position.z = float(origin_z) 
        self.grid_combined.info.origin.orientation.w = 1.0
        
        self.grid_combined.data = grid_array.flatten().tolist()
        self.grid_combined_pub.publish(self.grid_combined)
    
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
