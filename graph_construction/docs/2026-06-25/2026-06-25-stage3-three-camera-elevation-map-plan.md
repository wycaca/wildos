# 阶段三实现方案: 3 相机视觉适配与几何地图后端分离

日期: 2026-06-25

目标: 在阶段一 graph construction 和阶段二 visual scoring 已经跑通的基础上, 将当前实现向论文系统靠近, 优先补齐 3 相机视觉输入, 同时把几何 traversability map 后端作为独立轨道验证, 暂不接入 Nav2 local planning/control

2026-06-26 修正:

- `/spot1/traversability_grid` 属于 LiDAR / 几何地图链路, 不属于 3 相机视觉链路
- 3 相机只影响 `visual_navigation / WildOS` 的图像模型输入和 frontier scoring
- 日常阶段三联调默认继续使用 `livox_grid_builder` 生成 `/spot1/traversability_grid`
- `elevation_mapping_cupy -> grid_map_to_occupancy` 是可选实验后端, 需要单独标定, 不能和 3 相机效果混为一谈

## 设计边界

本阶段不实现 cmd_vel controller, 不接 Nav2, 不改变 `graphnav_planner` 的高层图规划逻辑

本阶段拆成两个互相独立的轨道:

```text
轨道 A, 默认联调路径
visual_navigation 从单前向相机适配为 front, left, right 三相机
graph_construction 继续使用 LiDAR baseline /spot1/traversability_grid

轨道 B, 可选实验路径
graph_construction 从 livox_grid_builder baseline 过渡到 elevation_mapping_cupy backend
该路径只用于地图质量评估和后续替换, 不作为 3 相机适配的前置条件
```

阶段三完成后的链路目标:

```text
front camera
left camera
right camera
    -> visual_navigation/wildos

/livox/lidar + /unity/odom + /tf
    -> livox_grid_builder
    -> /spot1/traversability_grid
    -> graph_construction
    -> /spot1/nav_graph
    -> visual_navigation/wildos
    -> /spot1/scored_nav_graph
    -> graphnav_planner
    -> /spot1/graphnav_planner/path
```

Nav2 仍然是论文系统中的 local planning/control 依赖, 但本阶段先不接

原因:

- 当前系统刚验证到 graph planner path, 直接接 Nav2 会同时引入 lifecycle, costmap, controller, behavior tree 等配置问题
- 3 相机直接影响 scoring 覆盖率, 应优先在稳定几何图上验证
- elevation/traversability map 后端会改变 `/spot1/traversability_grid` 质量, 应独立评估, 不应和 3 相机改动耦合
- 当前仿真环境已有 `/unity/odom`, `/tf`, `/livox/lidar`, 三相机 topic, 足够先做默认联调

## 与论文和原仓库的对应关系

原 WildOS 系统依赖:

- front, left, right 三个 RGB camera
- local geometric traversability map
- elevation mapping 相关组件
- graph planner 做高层路径选择
- Nav2 做 local planning/control

论文中的 local geometric traversability map 与 RGB image 是两个输入:

```text
RGB image I_t
    -> ExploRFM visual traversability, visual frontiers, object similarity
    -> frontier scoring

local geometric traversability map T_geo_t
    -> graph construction / local geometric safety
```

因此, 相机数量变化不应直接改变 `/spot1/traversability_grid`

本仓库原始配置也体现了这一点:

```yaml
num_cameras: 3
cam_frame: spot1/realsense/{}_color_optical_frame
camera_img_topic: /spot1/realsense/{}/color/image_raw/compressed
camera_info_topic: /spot1/realsense/{}/color/camera_info
```

当前仿真适配配置曾临时改成:

```yaml
num_cameras: 1
cam_frame: camera_frame
camera_img_topic: /camera/color/image/compressed
camera_info_topic: /camera/color/camera_info
```

阶段三需要把仿真配置恢复到 3 相机结构, 但 topic 和 frame 使用当前 Unity 仿真实际发布名称

## 当前状态

已完成:

- `/livox/lidar` 可转为 `/spot1/traversability_grid`
- `/spot1/traversability_grid` 可驱动 `graph_construction`
- `/spot1/nav_graph` 可生成节点, 边, frontier
- `wildos` 可消费单相机图像并发布 `/spot1/scored_nav_graph`
- `graphnav_planner` 可从 `/spot1/imgnav_waypoint` 生成 `/spot1/graphnav_planner/path`

已知不足:

- 当前 `livox_grid_builder` 输出的是简化 `nav_msgs/OccupancyGrid`, 但它是默认稳定 baseline
- 当前 grid 只表达 free, occupied, unknown, 没有 elevation, slope, roughness, traversability cost layers
- 当前 graph edge cost 主要来自 2D 可通行性和距离, 没有充分利用地形代价
- 当前视觉评分中, 相机后方或侧方 frontier 很多只能走 default score

## 3 相机适配方案

### 输入 topic

需要先用当前仿真确认三相机 topic 名称:

```bash
ros2 topic list | grep camera
ros2 topic list | grep image
ros2 topic list | grep camera_info
```

当前已知仿真 topic:

```text
/camera/front/color/image/compressed

/camera/left/color/camera_info
/camera/left/color/image
/camera/left/color/image/compressed

/camera/right/color/camera_info
/camera/right/color/image
/camera/right/color/image/compressed
```

已确认当前三路 `image` 和 `camera_info` 的 `header.frame_id` 均为:

```text
camera_frame
```

因此当前 `wildos_nav_sim_conf.yaml` 先使用:

```yaml
cam_frame: camera_frame
```

这可以保证当前 TF 查询可用, 但不等价于真实 front, left, right 三相机外参, 后续仿真需要补充独立相机 frame

目标结构建议保持 WildOS 原始模板:

```yaml
num_cameras: 3
camera_img_topic: /camera/{}/color/image/compressed
camera_info_topic: /camera/{}/color/camera_info
cam_frame: camera_frame
```

其中 `{}` 由代码映射为:

```text
0 -> front
1 -> left
2 -> right
```

如果 Unity 当前 topic 是固定展开形式, 可以采用以下两种方案之一:

### 方案 A: topic 名称适配 WildOS 模板

将仿真三相机 topic 通过 remap 或 relay 统一成:

```text
/camera/front/color/image/compressed
/camera/front/color/camera_info
/camera/left/color/image/compressed
/camera/left/color/camera_info
/camera/right/color/image/compressed
/camera/right/color/camera_info
```

配置:

```yaml
num_cameras: 3
cams_inverted: false
cam_frame: camera_frame
camera_img_topic: /camera/{}/color/image/compressed
camera_info_topic: /camera/{}/color/camera_info
```

优点:

- 最接近现有 `CAMERA_MAPPING`
- `wildos/nav.py` 不需要大改
- 后续切回真实 Spot/Realsense topic 更容易

缺点:

- 需要新增 relay/remap launch

### 方案 B: 配置支持显式三相机列表

将配置从 format string 扩展为列表:

```yaml
cameras:
  - name: front
    image_topic: /camera/front/color/image/compressed
    camera_info_topic: /camera/front/color/camera_info
    frame: camera_front_frame
  - name: left
    image_topic: /camera/left/color/image/compressed
    camera_info_topic: /camera/left/color/camera_info
    frame: camera_left_frame
  - name: right
    image_topic: /camera/right/color/image/compressed
    camera_info_topic: /camera/right/color/camera_info
    frame: camera_right_frame
```

优点:

- 对仿真 topic 更灵活
- 不依赖命名模板

缺点:

- 需要改 `wildos/nav.py`, `GeoFrontierToImage`, `VisualizeGoalAgnosticGeoFrontierScoring` 的相机配置读取
- 改动面大于方案 A

第一版建议采用方案 A

### TF 要求

三相机必须满足:

```text
map -> camera_front_frame
map -> camera_left_frame
map -> camera_right_frame
```

或通过中间 frame 可查:

```text
map -> odom_fram -> base_link -> camera_*_frame
```

检查命令:

```bash
ros2 run tf2_ros tf2_echo map camera_front_frame
ros2 run tf2_ros tf2_echo map camera_left_frame
ros2 run tf2_ros tf2_echo map camera_right_frame
```

如果 frame 名称不是 `camera_*_frame`, 配置中的 `cam_frame` 必须和 TF 完全一致

### 同步策略

三相机会让 `ApproximateTimeSynchronizer` 输入从:

```text
odom + nav_graph + image + camera_info
```

变成:

```text
odom + nav_graph + front image + front info + left image + left info + right image + right info
```

建议参数:

```yaml
syncsub_queue_size: 120
syncsub_slop: 5.0
tf_lookup_config:
  buffer_size: 8
  cache_time: 10
  wait_for_oldest: false
  clear_buffer_on_process: true
```

风险:

- 三相机时间戳不一致会导致同步队列长时间不触发
- 如果 camera_info 发布频率低, 需要增大 queue 或改成缓存 latest camera_info
- 如果 Unity 使用 sim time, `/clock` 和 use_sim_time 需要一致

## Geometric Map 后端策略

### 默认后端, LiDAR OccupancyGrid baseline

默认联调使用:

```text
/livox/lidar
    -> livox_grid_builder
    -> /spot1/traversability_grid
    -> graph_construction
```

启动:

```bash
./scripts/start_graph_construction_livox.sh
```

兼容旧脚本:

```bash
./scripts/start_graph_construction.sh
```

这个后端用于验证:

- 3 相机 visual scoring
- `/spot1/scored_nav_graph`
- `graphnav_planner`
- RViz graph/frontier 可视化

### 实验后端, Elevation Mapping CuPy

`elevation_mapping_cupy` 后端是可选实验:

```text
/livox/lidar + /tf + /unity/odom
    -> /elevation_mapping_node/elevation_map_raw
    -> grid_map_to_occupancy
    -> /spot1/elevation_traversability_grid
```

启动:

```bash
./scripts/start_graph_construction_elevation.sh
```

这个后端用于验证:

- elevation map 和 traversability layer 是否合理
- GridMap 到 OccupancyGrid 的阈值和后处理
- 后续是否能替换 LiDAR baseline

该后端不作为 3 相机适配的必要条件

## Elevation Traversability Map 实验方案

### 当前安装状态

`elevation_mapping_cupy` 已安装到当前 ROS 2 工作区:

```text
/home/ks-server3/han/wildos_ws/src/elevation_mapping_cupy
```

已 build 的 ROS 包:

```text
elevation_map_msgs
elevation_mapping_cupy
```

安装后 ROS 可发现:

```bash
ros2 pkg prefix elevation_mapping_cupy
ros2 pkg executables elevation_mapping_cupy
```

当前可执行节点:

```text
elevation_mapping_node.py
```

Python 依赖通过 UV 加入 `nebula2-wildos` 的 `.venv`:

```text
cupy-cuda12x
ros2-numpy
simple-parsing
```

GPU 验证结果:

```text
cupy import 正常
cupy.cuda.runtime.getDeviceCount() = 1
device = NVIDIA GeForce RTX 4090
简单 CuPy GPU 计算通过
```

注意: 受限沙箱环境下可能看不到 `/dev/nvidia*`, 会误报 `cudaErrorNoDevice`, 以主机终端或非沙箱 ROS 运行环境为准

### 现状

默认 `livox_grid_builder` 使用点云高度阈值生成 `nav_msgs/OccupancyGrid`:

```text
unknown = -1
free    = 0
occupied = 100
```

这能支持 frontier 和简单建图, 是当前仿真默认 baseline, 但不能表达论文系统需要的地形信息:

- elevation
- slope
- roughness
- traversability cost
- robot footprint clearance
- unknown risk

### 目标输入

实验目标不是一次接完整 Nav2 costmap, 而是先评估 elevation backend 是否能生成 graph construction 可用的 traversability map

建议内部保留两层输出:

```text
1. /spot1/traversability_grid
   nav_msgs/OccupancyGrid
   兼容当前 graph_construction

2. /spot1/traversability_layers
   grid_map_msgs/GridMap or custom debug output
   保存 elevation, slope, roughness, traversability
```

如果当前环境已有 `elevation_mapping_cupy`, 可以直接接它输出的 `grid_map_msgs/GridMap`

如果当前环境没有, 先实现轻量 elevation adapter:

```text
/livox/lidar
/unity/odom
/tf
    -> local elevation accumulator
    -> elevation layer
    -> slope layer
    -> roughness layer
    -> traversability layer
    -> /spot1/traversability_grid
```

### 数据结构

建议使用 `grid_map_msgs/GridMap` 表达多层地图:

```text
layers:
  elevation
  variance
  min_height
  max_height
  slope
  roughness
  traversability
```

如果短期不引入 `grid_map_msgs`, 可以在内部用 numpy layers, 对外仍只发布 `OccupancyGrid`

### 核心计算

#### Elevation

对每个 grid cell 聚合点云高度:

```text
z_mean = mean(z_i)
z_min  = min(z_i)
z_max  = max(z_i)
z_var  = var(z_i)
```

#### Slope

用邻域 elevation 梯度估计坡度:

```text
dz_dx = (z[x+1, y] - z[x-1, y]) / (2 * resolution)
dz_dy = (z[x, y+1] - z[x, y-1]) / (2 * resolution)
slope = atan(sqrt(dz_dx^2 + dz_dy^2))
```

#### Roughness

用局部窗口高度残差估计粗糙度:

```text
roughness = std(z in local window)
```

#### Traversability

第一版用规则融合:

```text
traversability = 1.0
traversability -= w_slope * normalize(slope)
traversability -= w_roughness * normalize(roughness)
traversability -= w_step * normalize(z_max - z_min)
traversability = clamp(traversability, 0.0, 1.0)
```

转为 OccupancyGrid:

```text
if no observation:
    cell = unknown
elif traversability >= free_threshold:
    cell = free
elif traversability <= obstacle_threshold:
    cell = occupied
else:
    cell = unknown or inflated risk
```

### 与 graph_construction 的接口

实验版保持 graph construction 输入不变:

```yaml
grid_topic: /spot1/elevation_traversability_grid
```

也就是说:

- graph construction 仍消费 `nav_msgs/OccupancyGrid`
- elevation/traversability builder 负责把多层地形信息压缩成 free, occupied, unknown
- 后续再扩展 edge cost, node properties, traversability properties

第二版再扩展 graph message:

```text
NodeTraversabilityProperties.properties:
  slope
  roughness
  elevation
  traversability

EdgeTraversability.properties:
  mean_slope
  max_slope
  mean_traversability
```

实验阶段建议只做第一版, 避免同时改 graphnav message 契约

## 输入输出

### 3 相机输入

```text
/camera/front/color/image/compressed
/camera/front/color/camera_info
/camera/left/color/image/compressed
/camera/left/color/camera_info
/camera/right/color/image/compressed
/camera/right/color/camera_info
```

实际 topic 以 `ros2 topic list` 为准, 文档中的名称是建议统一后的目标名称

### 地形输入

```text
/livox/lidar, sensor_msgs/PointCloud2
/livox/imu, sensor_msgs/Imu
/unity/odom, nav_msgs/Odometry
/tf, tf2_msgs/TFMessage
```

### 输出

```text
/spot1/traversability_grid, nav_msgs/OccupancyGrid
/spot1/elevation_map, grid_map_msgs/GridMap, optional
/spot1/nav_graph, graphnav_msgs/NavigationGraph
/spot1/scored_nav_graph, graphnav_msgs/NavigationGraph
/spot1/model_visualization, sensor_msgs/Image
/spot1/score_rings, visualization_msgs/MarkerArray
/spot1/graphnav_planner/path, nav_msgs/Path
```

## 配置建议

新增或更新:

```text
visual_navigation/configs/wildos_nav_sim_conf.yaml
visual_navigation/launch/wildos_sim_launch.py
scripts/start_graph_construction_livox.sh
scripts/start_visual_navigation.sh
```

三相机配置建议:

```yaml
num_cameras: 3
cams_inverted: false
cam_frame: camera_frame
camera_img_topic: /camera/{}/color/image/compressed
camera_info_topic: /camera/{}/color/camera_info
syncsub_queue_size: 120
syncsub_slop: 5.0
```

实验后端已实现的 elevation mapping 参数:

```yaml
map_frame: map
base_frame: odom_fram
corrected_map_frame: map
resolution: 0.2
map_length: 30.0
subscribers:
  livox:
    topic_name: /livox/lidar
    data_type: pointcloud
publishers:
  elevation_map_raw:
    layers: ["elevation", "traversability", "variance"]
```

已实现的 GridMap 转 OccupancyGrid 参数:

```yaml
input_topic: /elevation_mapping_node/elevation_map_raw
output_topic: /spot1/elevation_traversability_grid
traversability_layer: traversability
elevation_layer: elevation
free_threshold: 0.2
occupied_threshold: 0.05
unknown_value: -1
free_value: 0
occupied_value: 100
normalize_traversability: true
normalize_low_quantile: 0.05
normalize_high_quantile: 0.95
enable_postprocess: true
min_free_component_cells: 25
fill_hole_max_cells: 90
fill_hole_min_free_neighbor_ratio: 0.65
majority_fill_iterations: 1
majority_fill_min_neighbors: 6
```

说明:

- `elevation_mapping_cupy` 的 `traversability` 语义是越大越安全, upstream 默认 `safe_thresh=0.7`
- 当前 Unity + Livox 仿真中, 原始 `traversability` 分布明显压缩, 实测首帧 `raw_median=0.004`, `raw_max=0.528`
- 如果直接按 upstream 默认 `0.7/0.4` 二值化, 会得到极少 free cell, graph construction 长期 `nodes=0`
- 当前 adapter 先对有效 cell 做 5%-95% 分位数归一化, 再用 `0.2/0.05` 作为仿真标定阈值
- 这是阶段三仿真适配参数, 不是论文或 upstream 的固定阈值, 后续需要随 elevation mapping 质量继续标定

### GridMap 后处理

当前 `/elevation_mapping_node/elevation_map_raw` 直接二值化后会出现小孔洞, 一格裂缝和 free 孤岛

`grid_map_to_occupancy` 在输出 `/spot1/elevation_traversability_grid` 前执行保守后处理:

```text
1. majority fill
   如果非 free cell 的 8 邻域中 free 数量达到阈值, 将它填为 free

2. enclosed hole fill
   对 unknown 或 occupied 小连通域做 8 邻接检测
   如果连通域不接触地图边界, 尺寸不超过 fill_hole_max_cells, 且周围主要是 free, 将它填为 free

3. small free component removal
   对 free 小连通域做 8 邻接检测
   如果尺寸小于 min_free_component_cells, 将它改回 unknown
```

设计约束:

- 只修复局部小洞和裂缝, 不填大块障碍
- 接触地图边界的 unknown 或 occupied 连通域不填, 避免把未观测区域误判为可通行
- 小 free 孤岛改成 unknown, 避免 graph node 采样到噪声
- 参数以 RViz 和 graph 统计共同标定, 不只看白色区域面积

已新增 elevation 专用 graph 参数:

```yaml
graph_config: graph_construction_elevation.yaml
sample_stride: 6
min_obstacle_clearance: 0.3
edge_radius: 6.0
```

原因:

- 原 `graph_construction.yaml` 面向 Livox 规则栅格, `min_obstacle_clearance=0.5`
- elevation map 初期存在碎片和低分压缩, 直接复用原参数会把少量 free cell 全部过滤掉
- elevation 版本先降低 clearance 并提高采样密度, 后续地图质量稳定后再收紧

## 启动顺序

默认联调启动:

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

./scripts/start_graph_construction_livox.sh
./scripts/start_visual_navigation.sh
ros2 launch graphnav_planner graphnav_planner.launch.yml ns:=spot1 remap_tf_to_ns:=false odom_topic:=odom_for_scoring
```

实验后端启动:

```text
./scripts/start_graph_construction_elevation.sh
```

## 测试步骤

### 1. 检查三相机 topic

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/front/color/image/compressed
ros2 topic hz /camera/left/color/image/compressed
ros2 topic hz /camera/right/color/image/compressed
ros2 topic echo /camera/front/color/camera_info --once
ros2 topic echo /camera/left/color/camera_info --once
ros2 topic echo /camera/right/color/camera_info --once
```

验收:

- 三个 image topic 都有稳定频率
- 三个 camera_info 都能收到
- frame_id 与 TF 中相机 frame 一致

### 2. 检查三相机 TF

```bash
ros2 run tf2_ros tf2_echo map camera_front_frame
ros2 run tf2_ros tf2_echo map camera_left_frame
ros2 run tf2_ros tf2_echo map camera_right_frame
```

验收:

- 三个相机 frame 都可从 map 查询
- 时间戳不会持续报 extrapolation

### 3. 检查 LiDAR baseline traversability map

```bash
ros2 topic echo /spot1/traversability_grid --once --field info
ros2 topic hz /spot1/traversability_grid
```

验收:

- `/spot1/traversability_grid` frame 为 `map`
- grid 中 free, occupied, unknown 都合理存在
- 修改 3 相机配置不应改变该 topic 的生成质量

### 3b. 可选检查 elevation/traversability map 实验后端

```bash
ros2 topic echo /spot1/traversability_grid --once --field info
ros2 topic hz /spot1/traversability_grid
ros2 topic echo /elevation_mapping_node/elevation_map_raw --once --field layers
```

注意:

- `ros2 topic echo /spot1/traversability_grid --once` 会从 `data` 开头输出, 开头连续 `-1` 不代表全图未知
- 必须统计整张 grid 中 `-1`, `0`, `100` 的数量

整图计数命令:

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 - <<'PY'
from collections import Counter

import rclpy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = rclpy.create_node("traversability_grid_counter")
result = {}

def callback(msg):
    result["msg"] = msg

node.create_subscription(OccupancyGrid, "/spot1/traversability_grid", callback, 10)
deadline = node.get_clock().now().nanoseconds + 3_000_000_000
while rclpy.ok() and "msg" not in result and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

msg = result.get("msg")
if msg is None:
    print("no grid received")
else:
    print("frame", msg.header.frame_id)
    print("size", msg.info.width, msg.info.height, "resolution", msg.info.resolution)
    print(Counter(msg.data))

node.destroy_node()
rclpy.shutdown()
PY
```

验收:

- `/spot1/traversability_grid` frame 为 `map`
- grid 中 free, occupied, unknown 都合理存在
- `/elevation_mapping_node/elevation_map_raw` layers 至少包含 `elevation`, `traversability`, `variance`
- free cell 应形成连续区域, 不能只出现零散孤点

### 4. 检查 graph construction

```bash
ros2 topic echo /spot1/nav_graph --once --field header
ros2 topic hz /spot1/nav_graph
```

验收:

- graph 节点和边持续发布
- frontier nodes 数量稳定, 不应长期为 0
- RViz 中节点不应大量出现在明显障碍或不可通行区域

如果 `/spot1/traversability_grid` 中有 free cell 但 `/spot1/nav_graph` 仍为 `nodes=0`, 优先检查:

- 是否有多组 `grid_map_to_occupancy` 或 `graph_construction` 同时运行
- `free` cell 是否过于碎片化, 被 `min_obstacle_clearance` 过滤
- `graph_config` 是否为 `graph_construction_elevation.yaml`
- `/spot1/traversability_grid` 的 `0` 是否只是零散孤点, 而不是连续可通行区域

### 5. 检查三相机 visual scoring

```bash
ros2 topic echo /spot1/scored_nav_graph --once --field header
ros2 topic echo /spot1/model_visualization --once --field header
ros2 topic echo /spot1/score_rings --once
```

验收:

- `/spot1/model_visualization` 应展示三相机输入和对应 heatmap
- `/spot1/scored_nav_graph` 中 frontier nodes 都有 `frontier_scores`
- 单帧中可被真实视觉评分的 frontier 比单相机配置更多
- 后方不可见 frontier 不应被误认为相机可见, 只能使用 default score 或历史分数

### 6. 检查 planner

```bash
ros2 topic pub --once /spot1/imgnav_waypoint geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 4.0, y: -6.0, z: 0.0}, orientation: {w: 1.0}}}"
ros2 topic echo /spot1/graphnav_planner/path --once
```

验收:

- path 非空
- path 应沿 traversability grid 的 free 区域
- path 不应穿过 elevation/traversability 判定为 obstacle 的区域

## 验收标准

### 三相机验收

- `wildos` 可在 `num_cameras: 3` 下稳定启动
- front, left, right 三路 image 和 camera_info 都被同步消费
- 三路相机 TF 都能查到
- `/spot1/model_visualization` 能显示三相机模型输出
- `/spot1/scored_nav_graph` 的非默认评分覆盖率高于单相机配置

### LiDAR baseline traversability map 验收

- `/spot1/traversability_grid` 由 `/livox/lidar` 派生, 与相机数量无直接关系
- 3 相机适配前后, baseline grid 的 free, occupied, unknown 分布不应显著变差
- graph nodes 只采样在可通行区域
- frontier points 位于 free 和 unknown 边界

### Elevation backend 实验验收

- `/elevation_mapping_node/elevation_map_raw` 至少包含 `elevation`, `traversability`, `variance`
- 从 GridMap 压缩出的 `/spot1/elevation_traversability_grid` 质量不低于 LiDAR baseline 后, 才考虑替换默认后端
- grid 中 free, occupied, unknown 与 RViz 中点云和地面形态一致
- graph nodes 只采样在可通行区域
- frontier points 位于 free 和 unknown 边界

### 系统验收

- `/spot1/nav_graph` 正常发布
- `/spot1/scored_nav_graph` 正常发布
- `/spot1/score_rings` 正常显示
- `/spot1/graphnav_planner/path` 可生成非空 path
- 不要求 `/unity/cmdvel`
- 不要求 Nav2 action 成功

## 风险和后续工作

### 三相机同步风险

三相机同步会显著提高 message_filters 队列压力

如果 callback 长时间不触发, 优先检查:

- camera_info 是否持续发布
- 三路 image 时间戳是否接近
- `/clock` 和 use_sim_time 是否一致
- `syncsub_slop` 是否过小

### TF 风险

相机 frame 命名和 optical frame 方向容易出错

表现:

- frontier 投影到图像错误位置
- 左右相机 scoring 方向反了
- 后方节点被误判为可见

需要用 RViz 和 debug image 对每个 camera 单独验证

### Elevation map 风险

点云稀疏或地面反射缺失会导致 elevation 空洞

第一版必须保留 unknown, 不要把未观测区域直接标成 free

### Traversability 阈值风险

坡度, 粗糙度和 step height 阈值会直接影响 frontier 数量和 graph 连通性

建议先保守:

- 宁可 unknown, 不要误判 free
- 宁可少建边, 不要穿障碍

### 与 Nav2 的边界

本阶段不接 Nav2, 因此只验证 high-level path, 不验证底盘实际执行

后续接 Nav2 时, `/spot1/goal_pose` 应作为局部规划目标或 action goal 的输入, `/spot1/traversability_grid` 或 elevation map 应进入 Nav2 costmap

## 阶段三结论

当前阶段三应优先修正论文系统的 3 相机视觉输入, 并保持几何地图 baseline 稳定

完成本阶段后, 系统应具备:

- 三相机 WildOS visual scoring
- 基于 LiDAR 的稳定局部可通行地图
- 可选的 elevation/traversability map 实验后端
- 更接近论文系统的 graph construction 输入
- 仍保留当前 graph planner path 输出能力

Nav2 local planning/control 放到下一阶段处理
