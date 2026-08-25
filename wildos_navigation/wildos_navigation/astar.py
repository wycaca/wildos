import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile
import scipy.interpolate as si
from scipy.interpolate import splprep, splev
from nav_msgs.msg import Odometry
from scipy.interpolate import BSpline
import time
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

expansion_size = 2  # 全局寻路膨胀（2格），防止狗蹭墙

# 处理成本图数据，扩展障碍物
def costmap(data, width, height):
    data = np.array(data, dtype=np.int8).reshape(height, width)  # 重塑数据为矩阵
    wall_mask = data == 100
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            shifted_mask = np.roll(wall_mask, (i, j), axis=(0, 1))
            data[shifted_mask] = 100
    return data

def bezier_smoothing(array, num_points):
    try:
        array = np.array(array)
        if len(array) < 4:
            return array  # B样条至少需要4个点
        x = array[:, 0]
        y = array[:, 1]
        
        dx = np.diff(x, prepend=x[0])
        dy = np.diff(y, prepend=y[0])
        chord_lengths = np.sqrt(dx**2 + dy**2)  # 弦长
        t = np.concatenate(([0], np.cumsum(chord_lengths)))  # 累积弦长作为参数t
        if t[-1] == 0:
            return array
        t /= t[-1]  # 规范化到[0, 1]
        
        k = min(3, len(array) - 1)  # 阶数不能超过点数-1
        
        t_knots = np.concatenate(([0]*k, t, [1]*k))
        x_padded = np.pad(x, (k, k), 'edge')
        y_padded = np.pad(y, (k, k), 'edge')
        
        spline_x = BSpline(t_knots, x_padded, k, extrapolate=False)
        spline_y = BSpline(t_knots, y_padded, k, extrapolate=False)
        
        t_new = np.linspace(0, 1, num_points)
        x_smoothed = spline_x(t_new)
        y_smoothed = spline_y(t_new)
        
        return np.column_stack((x_smoothed, y_smoothed))
    except Exception as e:
        return array

# A*算法
def astar(start, goal, grid):
    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    rows, cols = grid.shape
    
    if grid[start] == 100: grid[start] = 1
    if grid[goal] == 100: grid[goal] = 1
    
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    closed_set = set()
    
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            
            clean_path = [path[0]]
            for p in path[1:]:
                if p != clean_path[-1]:
                    clean_path.append(p)
                    
            if len(clean_path) > 3:
                path_array = np.array(clean_path)
                x = path_array[:, 0]
                y = path_array[:, 1]
                try:
                    tck, u = splprep([x, y], s=1.0, k=3)
                    num_samples = max(100, len(clean_path) * 3)
                    u_fine = np.linspace(0, 1, num_samples)
                    x_smooth, y_smooth = splev(u_fine, tck)
                    path = [(float(x_smooth[i]), float(y_smooth[i])) for i in range(len(x_smooth))]
                except Exception as e:
                    path = clean_path
            return path
            
        if current in closed_set:
            continue
        closed_set.add(current)
        for d in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 100:
                new_cost = cost_so_far[current] + grid[neighbor]
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current
    return []

# 标准世界坐标系全局路径生成节点（对接不转动的 map 滚动网格）
class NavigationControl(Node):
    def __init__(self):
        super().__init__('astar')

        self.declare_parameter('grid_map_topic', 'combined_grid')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('odom_frame', 'odom_3D')
        self.declare_parameter('goal_topic', '/goal_pose')
        
        
        self.odom_frame_id = self.get_parameter('odom_frame').value
        self.goal_topic = self.get_parameter('goal_topic').value
        # 🔍 核心修改 1：將預設話題參數改為你的真實定位話題 '/tracked_pose'
        # self.declare_parameter('odom_topic', '/tracked_pose')

        self.grid_map = self.get_parameter('grid_map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.map_subscription = self.create_subscription(OccupancyGrid, self.grid_map, self.map_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        self.path_publisher2 = self.create_publisher(Path, 'path2', 10)
        
        self.corrected_path_publisher = self.create_publisher(Path, 'corrected_path', 10)
        self.odom_subscriber = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        # self.odom_subscriber = self.create_subscription(PoseStamped, self.odom_topic, self.tracked_pose_callback, 10)
        
        self.global_path_sub = self.create_subscription(Path, 'multi_planned_path', self.global_path_callback, 10)
        
        # 🔍 重新恢复接收全局目标点（在 Rviz2 里的世界坐标点击）
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.corrected_global_path = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.goal = None              # 存储全局目标坐标 (world_goal_x, world_goal_y)
        self.full_global_path = None
        self.current_map_z = 0.0      # 地图当前的绝对 Z 高度
        
        self.create_timer(0.1, self.publish_path)
        self.path = None
        self.path2 = None
        
        
    def global_path_callback(self, msg):
        """當接收到外部多點全局路徑時觸發"""
        if len(msg.poses) == 0:
            return
        self.full_global_path = msg
        self.get_logger().info(f"✔ [局部追隨] 成功對接全局路徑，共包含 {len(msg.poses)} 個路徑點。")
        
        odom_path_msg = Path()
        odom_path_msg.header.stamp = msg.header.stamp
        odom_path_msg.header.frame_id = self.odom_frame_id  # 🔍 強行規範化坐標系

        for map_pose in msg.poses:
            odom_pose = PoseStamped()
            odom_pose.header.stamp = map_pose.header.stamp
            odom_pose.header.frame_id = self.odom_frame_id
            
            # 🔍 直接承接物理絕對坐標。
            # 如果你的 A* 尋路依然有偏差，請將下面兩行改為與狗當前位置的相對扣减：
            # odom_pose.pose.position.x = map_pose.pose.position.x - self.x + (20.0 / 2.0)
            odom_pose.pose.position.x = map_pose.pose.position.x
            odom_pose.pose.position.y = map_pose.pose.position.y
            odom_pose.pose.position.z = map_pose.pose.position.z
            
            odom_pose.pose.orientation = map_pose.pose.orientation
            odom_path_msg.poses.append(odom_pose)
            
        # 核心覆蓋：將這條乾淨的、鎖死在 odom_3D 坐標軸上的路徑送入寻路流
        self.corrected_global_path = odom_path_msg
        
        # 向 Rviz2 發布動態參考路徑
        self.corrected_path_publisher.publish(odom_path_msg)
        self.get_logger().info(f"✔ [世界軸幾何硬鎖死] 成功跳過 TF2 庫 Bug，全條路徑已強制對齊至 odom_3D 系！")
            
            
        # # # 立刻發布並固化到 map 系中
        # self.corrected_path_publisher.publish(msg)
        # self.get_logger().info(f"✔ [一次性定錨糾偏完成] 路徑已固化在 map 座標系中，共 {len(msg.poses)} 個點。")
        
    # def goal_callback(self, msg):
    #     self.full_global_path = None
        
    #     odom_pose = PoseStamped()
    #     odom_pose.header.stamp = msg.header.stamp
    #     odom_pose.header.frame_id = self.odom_frame_id
    #     odom_pose.pose.position.x = msg.pose.position.x
    #     odom_pose.pose.position.y = msg.pose.position.y
    #     odom_pose.pose.position.z = msg.pose.position.z
    #     odom_pose.pose.orientation = msg.pose.orientation
    #     self.goal = (odom_pose.pose.position.x, odom_pose.pose.position.y)
    #     self.get_logger().info(f"★ 成功设定绝对全局目标点: X={self.goal[0]:.2f}, Y={self.goal[1]:.2f}")
    def goal_callback(self, msg):
        self.full_global_path = None
        
        # ==================== 📐 🛠️ 核心修改：將目標點從 map 轉化為 odom_3D ====================
        # 💡 如果發布過來的目標點本身就是 odom_3D，就直接讀取；如果是 map，就通過 TF 動態轉換
        if msg.header.frame_id == self.odom_frame_id:
            # 情況 A：同坐標系，直接賦值
            target_goal_x = msg.pose.position.x
            target_goal_y = msg.pose.position.y
            self.get_logger().info(
                f"接收同 frame 目标点({self.odom_frame_id}), "
                f"x={target_goal_x:.2f}, y={target_goal_y:.2f}"
            )
        else:
            # 情況 B：跨坐標系，調用 TF2 標準庫進行動態變換
            try:
                # 尋找當前最即時的 map -> odom_3D 坐標系空間變換幾何矩陣
                transform = self.tf_buffer.lookup_transform(
                    self.odom_frame_id,      # 目標坐標系: odom_3D
                    msg.header.frame_id,     # 源坐標系: 消息自帶的 frame_id (例如 map)
                    rclpy.time.Time(), 
                    rclpy.duration.Duration(seconds=1.0) # 最多等待 1 秒
                )
                
                # 使用標準 tf2 幾何轉換函數處理平移和旋轉
                odom_pose = do_transform_pose_stamped(msg, transform)
                
                target_goal_x = odom_pose.pose.position.x
                target_goal_y = odom_pose.pose.position.y
                self.get_logger().info(
                    f"目标点已转换到 {self.odom_frame_id}"
                )
                
            except Exception as e:
                self.get_logger().error(f"❌ [TF2 轉換失敗] 無法將目標點從 {msg.header.frame_id} 轉換到 {self.odom_frame_id}: {str(e)}")
                # 🔍 降級防護：如果 TF 樹斷開，採用你原來的幾何硬賦值（防止節點直接崩潰）
                target_goal_x = msg.pose.position.x
                target_goal_y = msg.pose.position.y
                self.get_logger().warn("⚠️ [降級運行] 使用硬編碼坐標直接鎖定目標。")
        # ==================================================================================

        # 最終將轉換完全的 odom_3D 世界絕對坐標寫入 A* 局部導航的流中
        self.goal = (target_goal_x, target_goal_y)
        self.get_logger().info(
            f"A* 局部目标, x={self.goal[0]:.2f}, y={self.goal[1]:.2f}"
        )
        
    def odom_callback(self, msg):
        # 实时获取机器狗在世界 map 系下的绝对位置与高度
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        
    def tracked_pose_callback(self, msg):
        # PoseStamped 的數據結構沒有外層的 .pose.pose，直接是 .pose.position
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
    
    def map_callback(self, msg):
        if self.x == 0.0 and self.y == 0.0:
            return
        
        
        # if not hasattr(self, 'param_log_counter'):
        #     self.param_log_counter = 0
        # self.param_log_counter += 1
        
        # if self.param_log_counter % 20 == 0:
        #     # 實時從 ROS 2 內存中重新獲取最新的參數值
        #     current_grid_map_topic = self.get_parameter('grid_map_topic').value
        #     current_odom_topic = self.get_parameter('odom_topic').value
        #     current_odom_frame = self.get_parameter('odom_frame').value
            
        #     # 讀取地圖消息頭中傳過來的動態核心物理幾何參數
        #     msg_resolution = msg.info.resolution
        #     msg_width_cells = msg.info.width
        #     msg_height_cells = msg.info.height
        #     msg_origin_x = msg.info.origin.position.x
        #     msg_origin_y = msg.info.origin.position.y
        #     msg_frame_id = msg.header.frame_id

        #     self.get_logger().info(
        #         f'📋 [A* 外部參數實時狀態監測] \n'
        #         f'  ├─ 節點訂閱地圖話題 (grid_map_topic): {current_grid_map_topic}\n'
        #         f'  ├─ 節點訂閱里程話題 (odom_topic): {current_odom_topic}\n'
        #         f'  ├─ 地圖消息坐標系 (header.frame_id): {msg_frame_id} (🔍 必須是 {current_odom_frame})\n'
        #         f'  ├─ 地圖分辨率 (resolution): {msg_resolution} m/格\n'
        #         f'  ├─ 地圖格子尺寸 (cells): {msg_width_cells} x {msg_height_cells}\n'
        #         f'  └─ 地圖動態原點 (origin): X={msg_origin_x:.3f}, Y={msg_origin_y:.3f}'
        #     )
        
        # 💡 【核心修改】：動態滾動目標提取
        # 如果有外部長距離全局路徑，實時沿著路徑截取眼前 5.0 米處的點作為局部 Goal
        if self.corrected_global_path is not None and len(self.corrected_global_path.poses) > 0:
            poses = self.corrected_global_path.poses
            
            # 1. 轉成 numpy 陣列以便進行高效的距離矩陣計算
            path_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in poses])
            robot_pt = np.array([self.x, self.y])
            
            # 2. 尋找全局路徑上距離機器狗最近點的索引
            dists_to_robot = np.linalg.norm(path_pts - robot_pt, axis=1)
            nearest_idx = np.argmin(dists_to_robot)
            
            found_local_target = False
            lookahead_dist = 2.0 # 前瞻裁剪距離（米），確保目標點不會超出 20m 局部網格
            
            for i in range(nearest_idx, len(poses)):
                pt = poses[i].pose.position
                d = math.hypot(pt.x - self.x, pt.y - self.y)
                
                if d >= lookahead_dist:
                    self.goal = (pt.x, pt.y)
                    found_local_target = True
                    break
            
            # 如果剩餘的全局路徑很短（小於 5 米），直接鎖定最終終點
            if not found_local_target:
                final_pt = poses[-1].pose.position
                self.goal = (final_pt.x, final_pt.y)

        if self.goal is None:
            return
        
        # 計算與目前局部目標點的距離
        distance = math.hypot(self.x - self.goal[0], self.y - self.goal[1])
        
        # 💡 如果快要走到最終總終點了（距離小於 0.2 米），且外部路徑已消耗完，則清除路徑停下
        if self.corrected_global_path is not None:
            final_pose = self.corrected_global_path.poses[-1].pose.position
            total_remain_dist = math.hypot(self.x - final_pose.x, self.y - final_pose.y)
            if total_remain_dist <= 0.2:
                self.get_logger().info("🎯 機器狗已完全抵達多點全局總終點！")
                self.path = None
                self.path2 = None
                self.corrected_global_path = None
                self.full_global_path = None
                self.goal = None
                return
        
        
        # # 1. 计算当前机器狗绝对世界位置与绝对全局终点的直线距离
        # distance = abs(math.hypot(self.x - self.goal[0], self.y - self.goal[1]))
        if distance > 0.2:
            resolution = msg.info.resolution
            originX = msg.info.origin.position.x
            originY = msg.info.origin.position.y
            self.current_map_z = msg.info.origin.position.z  # 获取当前楼层高度 (1楼是0，2楼是3m)
            
            width = msg.info.width
            height = msg.info.height
            
            # ==================== 🛠️ 核心修改 1：利用滚动中心锁定绝对起点 ====================
            # 💡 既然上一级节点发布的局部图不带自转，且原点严格定义为 `robot_x - grid_width / 2`
            # 那么在数学上，这帧地图像素矩阵的正中央格，就百分之百完美对应机器狗的身体轴心！
            # 这样直接写死为中心点，能够彻底根除由于 odom 频率和 map 频率不同步导致起点的抖动与错位。
            column = width // 2
            row = height // 2
            
            # 全局绝对目标位置（Goal）通过相对当前地图绝对左下角的偏移投影转换为像素索引
            columnH = int((self.goal[0] - originX) / resolution)
            rowH = int((self.goal[1] - originY) / resolution)
            
            # 边界安全限幅，防止在 Rviz 里点到 20m x 20m 局部窗口外部导致索引越界崩溃
            column = max(0, min(column, width - 1))
            row = max(0, min(row, height - 1))
            columnH = max(0, min(columnH, width - 1))
            rowH = max(0, min(rowH, height - 1))
            # =========================================================================
            
            # 2. 处理成本图
            data = costmap(msg.data, width, height)
            
            map_grid = np.full((height, width), 100, dtype=np.int32)
            passable_mask = (data == 0) | (data == 5)
            map_grid[passable_mask] = 1
            
            # 强行解锁起点和终点
            map_grid[row][column] = 1 
            map_grid[rowH][columnH] = 1
            
            start = (row, column)
            goal = (rowH, columnH)
            
            # 3. 调用 A* 算法寻路
            path = astar(start, goal, map_grid)
            if len(path) == 0:
                return
            
            # ==================== 🛠️ 核心修改 2：反向恢复绝对全局坐标 ====================
            # 💡 坐标轴没有旋转，公式恢复为经典的 map 系恢复公式
            # (像素索引 * 分辨率) + 地图绝对左下角原点 = 全局绝对坐标
            paths = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
            # =========================================================================
            
            self.path = paths
            self.path2 = bezier_smoothing(paths, len(paths))
        else:
           if self.full_global_path is None: # 僅在單點模式下直接清空
                self.path = None
                self.path2 = None

    def publish_path(self):
        if self.path is None or len(self.path) == 0:
            return
            
        # 1. 发布 A* 全局原始路径
        path_msg = Path()
        path_msg.header.frame_id = self.odom_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in self.path:
            pose = PoseStamped()
            pose.header.frame_id = self.odom_frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(self.current_map_z)  # 🔍 核心：让路径高度在 3D 空间完美跟着狗爬楼梯上二楼
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)

        # 2. 发布平滑后的全局路径
        path2_msg = Path()
        path2_msg.header.frame_id = self.odom_frame_id
        path2_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in self.path2:
            pose2 = PoseStamped()
            pose2.header.frame_id = self.odom_frame_id
            pose2.pose.position.x = float(x)
            pose2.pose.position.y = float(y)
            pose2.pose.position.z = float(self.current_map_z)
            pose2.pose.orientation.w = 1.0
            path2_msg.poses.append(pose2)
        self.path_publisher2.publish(path2_msg)

def main(args=None):
    rclpy.init(args=args)
    navigation_control = NavigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
