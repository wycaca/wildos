# Livox PointCloud2 转 OccupancyGrid 实现说明

日期: 2026-06-24

目标: 新增一个轻量的 LiDAR grid builder, 将 `/livox/lidar` 点云转换为 `/spot1/traversability_grid`, 给 `graph_construction` 提供第一版可用的局部可通行栅格输入

## 背景

当前 `graph_construction` 已经按实现文档接收:

```text
/spot1/traversability_grid, nav_msgs/OccupancyGrid
/unity/odom, nav_msgs/Odometry
```

但实测当前仿真环境中原本设计的 `/spot1/traversability_grid` 没有 publisher

这会导致 `graph_construction` 只能收到 odom, 不能收到 grid, 因而不会进入完整构图流程

当前环境已有:

```text
/livox/lidar, sensor_msgs/PointCloud2
```

因此需要新增一个转换节点, 把 LiDAR 点云降维成局部 2D `OccupancyGrid`, 先补齐 Graph Construction 第一版所需的 grid 输入

## 为什么需要新增这个实现

Graph Construction 第一版不直接处理原始点云

它的输入契约是 `OccupancyGrid`, 内部逻辑依赖 free, obstacle, unknown 三类 cell:

```text
OccupancyGrid + Odometry
    -> classify free, obstacle, unknown
    -> sample graph nodes
    -> detect frontier
    -> build collision-free edges
    -> publish NavigationGraph
```

如果没有 `/spot1/traversability_grid`, 后续链路会断在最前面:

```text
/livox/lidar exists
/spot1/traversability_grid missing
graph_construction waits for grid
/spot1/nav_graph not published
visual scoring and graph planner cannot run
```

新增 LiDAR grid builder 的作用是把当前已有的传感器数据转换成 Graph Construction 能消费的中间表达

这不是论文级完整 traversability mapping, 而是为了先跑通几何图, frontier, planner 联调闭环

## 实现原理

节点订阅点云和里程计, 周期发布机器人附近的局部栅格

核心步骤:

```text
PointCloud2
    -> transform to grid frame
    -> crop by range and height
    -> allocate local OccupancyGrid
    -> ray trace sensor origin to each point, mark free cells
    -> mark point hit cells as obstacle
    -> keep unobserved cells as unknown
    -> publish /spot1/traversability_grid
```

栅格数值约定:

```text
-1   unknown, 未观测区域
0    free, 射线穿过且无障碍区域
100  obstacle, 点云命中的障碍区域
```

高度过滤用于避免地面点或过高点错误生成障碍:

```text
min_obstacle_height <= point.z <= max_obstacle_height
```

距离过滤用于限制局部地图范围:

```text
min_range <= sqrt(x^2 + y^2) <= max_range
```

射线清空用于生成 free space:

```text
for each valid point:
    cells from sensor origin to point are free
    endpoint cell is obstacle
```

如果暂时没有完整 TF, 可以先提供简化模式:

```text
assume_input_in_grid_frame: true
```

该模式假设 `/livox/lidar` 点云已经在 `map` 或可直接用于局部 grid 的坐标系下

## 作用

新增节点的直接作用:

- 发布 `/spot1/traversability_grid`, 类型为 `nav_msgs/OccupancyGrid`
- 让 `graph_construction` 能收到 grid 输入并发布 `/spot1/nav_graph`
- 给 frontier 检测提供 free 和 unknown 的边界
- 给 edge builder 提供 obstacle collision check
- 给 current node 匹配提供当前局部地图约束

它在系统链路中的位置:

```text
/livox/lidar
    -> livox_grid_builder
    -> /spot1/traversability_grid
    -> graph_construction
    -> /spot1/nav_graph
    -> visual_navigation
    -> /spot1/scored_nav_graph
    -> graphnav_planner
```

## 输入

必需输入:

```text
/livox/lidar, sensor_msgs/PointCloud2
/unity/odom, nav_msgs/Odometry
```

可选输入:

```text
/tf, tf2_msgs/TFMessage
/tf_static, tf2_msgs/TFMessage
```

如果启用 TF, 节点应将点云转换到 `grid_frame`

第一版建议配置:

```text
grid_frame: map
lidar_topic: /livox/lidar
odom_topic: /unity/odom
grid_topic: /spot1/traversability_grid
```

## 输出

主要输出:

```text
/spot1/traversability_grid, nav_msgs/OccupancyGrid
```

消息字段要求:

```text
header.frame_id = grid_frame
info.resolution = configured resolution
info.width = local_width / resolution
info.height = local_height / resolution
info.origin = robot-centered local origin in grid_frame
data = -1, 0, 100
```

可选调试输出:

```text
/spot1/traversability_grid/filtered_points, sensor_msgs/PointCloud2
/spot1/traversability_grid/obstacle_points, sensor_msgs/PointCloud2
```

调试输出只用于 RViz 检查, 不应作为 Graph Construction 的依赖

## 建议参数

第一版建议从保守参数开始:

```text
publish_rate_hz: 5.0
resolution: 0.2
local_width: 30.0
local_height: 30.0
min_range: 0.3
max_range: 25.0
min_obstacle_height: 0.15
max_obstacle_height: 1.8
obstacle_inflation_radius: 0.3
raytrace_max_range: 25.0
unknown_value: -1
free_value: 0
obstacle_value: 100
grid_frame: map
assume_input_in_grid_frame: false
```

如果点云 frame 和 TF 暂时不稳定, 可以先设置:

```text
assume_input_in_grid_frame: true
```

但该模式只能用于临时联调, 后续需要恢复 TF 转换

## 后续需要修改的内容

建议新增文件:

```text
graph_construction/graph_construction/livox_grid_builder.py
graph_construction/configs/livox_grid_builder.yaml
graph_construction/launch/livox_grid_builder.launch.py
```

`setup.py` 需要新增 console script:

```text
livox_grid_builder = graph_construction.livox_grid_builder:main
```

`package.xml` 需要确认依赖:

```text
rclpy
sensor_msgs
nav_msgs
geometry_msgs
tf2_ros
tf2_sensor_msgs
numpy
```

如果当前环境缺少 `tf2_sensor_msgs`, 第一版可以手写 PointCloud2 字段读取和坐标转换, 或先只支持 `assume_input_in_grid_frame`

当前已经新增合并 launch:

```text
graph_construction/launch/graph_construction_sim.launch.py
```

合并 launch 会同时启动 `livox_grid_builder` 和 `graph_construction`, 并通过 `graph_start_delay` 让 graph node 稍后启动

脚本入口:

```text
scripts/start_graph_construction.sh
```

两个独立 launch 仍保留, 用于单独排查 grid 或 graph

## 风险点

### TF 不完整

如果 `/livox/lidar` 的 `header.frame_id` 无法转换到 `map`, grid 会错位或无法发布

缓解方式:

- 先打印首帧点云 frame
- 检查 `/tf` 是否包含 lidar 到 odom 的链路
- 临时启用 `assume_input_in_grid_frame`

### 地面点误判为障碍

Livox 点云会包含地面, 如果高度过滤不合理, 大量地面 cell 会被标为 obstacle

缓解方式:

- 调整 `min_obstacle_height`
- 后续加入地面拟合或坡度估计
- 保留 RViz 调试点云

### free space 过少

如果只标记点云命中 cell 为 obstacle, 不做 ray tracing, 大部分区域会保持 unknown

这会导致 frontier 过多, graph edges 过少

缓解方式:

- 必须做 sensor origin 到 hit point 的 ray tracing
- 对 raytrace range 做上限
- 对障碍 inflation 做保守调参

### 动态障碍残留

OccupancyGrid 是局部瞬时地图, 动态物体或噪点可能造成障碍残留

缓解方式:

- 第一版每帧重建局部 grid
- 后续增加时间衰减或多帧融合
- 对孤立 obstacle cell 做过滤

### 2D 栅格表达能力有限

2D `OccupancyGrid` 无法表达完整 2.5D 地形, 坡度, 台阶, 粗糙度

这和当前 Graph Construction 第一版风险一致

缓解方式:

- 第一版只用于跑通几何图闭环
- 后续接入 elevation map 或 terrain traversability layer
- edge cost 后续叠加坡度和粗糙度代价

### 共享 topic 影响其他节点

如果 `/spot1/traversability_grid` 被其他节点订阅, 新 publisher 会影响已有规划链路

缓解方式:

- 联调前确认是否允许发布 `/spot1/traversability_grid`
- 可先发布到 `/graph_construction/test_grid`
- 验证稳定后再切到 `/spot1/traversability_grid`

## 验证步骤

以下步骤用于验证当前基础链路:

```text
/livox/lidar
    -> livox_grid_builder
    -> /spot1/traversability_grid
    -> graph_construction
    -> /spot1/nav_graph
```

### 1. 准备环境

每个终端都先加载 ROS 和工作空间:

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2. 编译模块

在 workspace 根目录执行:

```bash
cd /home/ks-server3/han/wildos_ws
colcon build --packages-select graph_construction --symlink-install
```

期望结果:

```text
Summary: 1 package finished
```

### 3. 检查 LiDAR 和 odom topic

确认 LiDAR 点云存在:

```bash
ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --field header
```

期望结果:

```text
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
frame_id: livox_frame
```

确认 odom 存在:

```bash
ros2 topic echo /unity/odom --once --field pose.pose
```

当前环境中 `/unity/odom` 的 `header.frame_id` 为空, 但 pose 与 TF 中的 `map -> odom_fram` 一致

### 4. 检查 TF frame

确认 LiDAR 可以转换到 `map`:

```bash
python3 - <<'PY'
import time
import rclpy
import tf2_ros
from rclpy.time import Time

rclpy.init()
node = rclpy.create_node("tf_check")
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer, node)
end_time = time.time() + 5.0
while time.time() < end_time:
    rclpy.spin_once(node, timeout_sec=0.1)

print(buffer.all_frames_as_yaml())
transform = buffer.lookup_transform("map", "livox_frame", Time())
t = transform.transform.translation
print(f"map -> livox_frame: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
node.destroy_node()
rclpy.shutdown()
PY
```

当前实测 TF 树:

```text
map
  -> odom_fram
      -> livox_frame
      -> camera_frame
      -> imu_link
```

### 5. 启动阶段一链路

推荐方式:

```bash
cd /home/ks-server3/han/wildos_ws/src/nebula2-wildos
scripts/start_graph_construction.sh
```

等价 launch:

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch graph_construction graph_construction_sim.launch.py
```

手动排查时可以拆分启动

启动 LiDAR grid builder:

终端 1:

```bash
ros2 launch graph_construction livox_grid_builder.launch.py
```

期望日志:

```text
Livox grid builder started, lidar=/livox/lidar, grid=/spot1/traversability_grid
Received first odom, frame=
Received first cloud, frame=livox_frame
Published first grid, frame=map, size=150x150
```

启动早期可能短暂出现一次:

```text
Missing TF from livox_frame to map
```

如果随后出现 `Published first grid`, 表示 TF buffer 已经填充完成, 可继续测试

检查 grid topic:

```bash
ros2 topic info /spot1/traversability_grid
ros2 topic echo /spot1/traversability_grid --once --field header
```

期望结果:

```text
Type: nav_msgs/msg/OccupancyGrid
Publisher count: 1
frame_id: map
```

### 6. 启动 Graph Construction

终端 2:

```bash
ros2 launch graph_construction graph_construction.launch.py
```

期望日志:

```text
Graph construction started, grid=/spot1/traversability_grid, odom=/unity/odom
Received first grid, frame=map, size=150x150
Received first odom, frame=
Published first graph, nodes=<nonzero>, edges=<nonzero>
```

本次实测结果:

```text
Published first graph, nodes=146, edges=330-340
```

### 7. 检查 NavigationGraph 输出

终端 3:

```bash
ros2 topic info /spot1/nav_graph
ros2 topic echo /spot1/nav_graph --once --field header
```

期望结果:

```text
Type: graphnav_msgs/msg/NavigationGraph
Publisher count: 1
frame_id: map
```

如果需要统计节点, 边和 frontier:

```bash
python3 - <<'PY'
import rclpy
from graphnav_msgs.msg import NavigationGraph

rclpy.init()
node = rclpy.create_node("nav_graph_check")

def callback(msg):
    frontier_nodes = sum(
        1
        for graph_node in msg.nodes
        if graph_node.trav_properties and graph_node.trav_properties[0].is_frontier
    )
    frontier_points = sum(
        len(graph_node.trav_properties[0].frontier_points)
        for graph_node in msg.nodes
        if graph_node.trav_properties
    )
    print(f"frame={msg.header.frame_id}")
    print(f"nodes={len(msg.nodes)}")
    print(f"edges={len(msg.edges)}")
    print(f"current_node_idx={msg.current_node_idx}")
    print(f"frontier_nodes={frontier_nodes}")
    print(f"frontier_points={frontier_points}")
    node.destroy_node()
    rclpy.shutdown()

node.create_subscription(NavigationGraph, "/spot1/nav_graph", callback, 10)
rclpy.spin(node)
PY
```

### 8. 当前测试结论

基础端到端链路已经跑通:

```text
/livox/lidar
    -> /spot1/traversability_grid
    -> /spot1/nav_graph
```

已确认:

- `/spot1/traversability_grid` 有 publisher
- `/spot1/traversability_grid.header.frame_id = map`
- `graph_construction` 日志出现 `Received first grid`
- `/spot1/nav_graph` 有 publisher
- `/spot1/nav_graph.header.frame_id = map`
- `NavigationGraph.nodes` 和 `NavigationGraph.edges` 非空

仍需人工或 RViz 继续检查:

- `/spot1/traversability_grid` 中 free, obstacle, unknown 是否符合真实场景
- `/spot1/graph_construction_viz` 中 frontier node 是否位于 free 和 unknown 边界附近
- 图连通性是否足够支撑后续 `graphnav_planner`
- 视觉 scoring 是否能正确投影 frontier

## 当前结论

新增 LiDAR 到 OccupancyGrid 的转换节点是当前环境中补齐 `/spot1/traversability_grid` 的最小可行方案

它不替代后续 elevation mapping 或完整 traversability mapping, 但能让 Graph Construction 第一版从真实传感器数据进入构图流程

后续重点不是只发布 grid, 而是验证 grid 的 free, obstacle, unknown 语义是否足够稳定, 是否能生成连通 graph 和合理 frontier
