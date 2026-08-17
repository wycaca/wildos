# 高程图范围和偏移问题分析

日期: 2026-07-01

状态: 已完成杂乱高程图回退修正, 待重启仿真复测

## 背景

当前 RViz 对比中, 原始 LiDAR 点云显示范围明显大于 elevation GridMap

高程图只在机器人附近形成较小的彩色局部面, 并且 overlay marker 和高程面存在疑似 XY 偏移

本次先分析文档, 源码, 当前运行 topic 和截图现象, 再按运行时数据完成最小代码修正

新增截图中高程图出现大面积彩色竖向幕布, 说明上一轮放大 `max_ray_length` 和修改轴向后引入了更严重的融合伪影

## 文档规则

后续 Graph Construction 相关变更必须同步写文档

文档分为两类:

- 变更记录文档, 文件名使用 `YYYY-MM-DD-changelog.md`
- 具体说明文档, 文件名使用 `YYYY-MM-DD-主题说明.md`

变更记录文档只记录:

- 已完成
- 实测结果
- 诊断结论摘要
- 文档同步
- 后续待办

具体说明文档负责记录:

- 背景
- 开发目标
- 当前代码链路
- 问题分析
- 可能原因
- 验证步骤
- 修改建议

不要把长篇设计分析塞进 changelog, 也不要只在具体说明文档里记录已完成变更

## 开发目标

目标不是把 2D `/spot1/traversability_grid` 调大

论文 UI 对应的是 experimental elevation backend 输出的 2.5D terrain surface

正确链路应为:

```text
/unitree_go2/lidar/points
    -> optional pointcloud_axis_adapter
    -> /unitree_go2/lidar/points_aligned
    -> elevation_mapping_cupy
    -> /elevation_mapping_node/elevation_map_raw
    -> graph_construction GridMap path
```

所有几何对象最终应统一到同一个 global frame

之前目标文档要求当前仿真统一为 `map`, 但当前源码和运行检查显示仍有 `odom` 链路

## 当前代码链路

### elevation_mapping 配置

`graph_construction/configs/elevation_mapping_sim.yaml` 当前关键参数:

```yaml
map_frame: odom
base_frame: base_link
corrected_map_frame: odom
resolution: 0.2
map_length: 30.0
min_valid_distance: 0.3
max_height_range: 2.0
max_ray_length: 20.0
subscribers:
  livox:
    topic_name: /unitree_go2/lidar/points_aligned
    data_type: pointcloud
```

当前保持 30m rolling local map, 不再用加长 ray tracing 的方式扩大显示范围

### graph construction 配置

`graph_construction/configs/graph_construction_elevation.yaml` 当前关键参数:

```yaml
global_frame: odom
odom_topic: /spot1/odom_for_scoring
grid_input_type: grid_map
grid_map_topic: /elevation_mapping_node/elevation_map_raw
```

这和目标文档中的 `map` frame 要求不一致

### 点云轴转换

`pointcloud_axis_adapter` 当前默认:

```text
input_topic: /unitree_go2/lidar/points
output_topic: /unitree_go2/lidar/points_aligned
output_frame: base_link
axis_mode: isaac_lidar_to_base
```

`isaac_lidar_to_base` 当前默认执行:

```text
(x, y, z) -> (-x, -y, z)
```

上一轮试验中 `isaac_lidar_to_base` 曾改为:

```text
(x, y, z) -> (y, -x, z)
```

该模式曾在上一张截图中触发明显竖向伪影, 因此保留为显式诊断模式

`(y, -x, z)` 保留为显式诊断模式:

```text
isaac_y_forward_to_base
y_forward
```

旧全翻转转换保留为 `negate_xyz`

adapter 仍会将点云声明为 `base_link` 下的点

如果原始点云 frame 已经有正确 TF, 这个处理可能绕过真实 LiDAR 外参

当前运行时确认 `odom -> lidar` TF 不可用, 所以短期仍需要 aligned 点云以 `base_link` frame 输入 elevation mapping

### elevation_mapping_cupy 处理方式

高程后端对点云执行:

```text
lookup transform: map_frame <- msg.header.frame_id
input_pointcloud(points, R, t)
```

同时通过:

```text
lookup transform: map_frame <- base_frame
move_to(base pose)
```

更新 rolling map 中心

所以 `map_frame`, `base_frame`, 点云 `header.frame_id`, TF 树和点云轴转换必须一致

官方文档中的相关约束:

- subscribed point cloud topic 通过 `subscribers` 参数配置, 节点同时使用 `/tf`
- `map_length` 是地图尺寸, 不是 LiDAR 量程
- `max_height_range` 会过滤高于传感器的点, 用于关闭天花板
- `max_ray_length` 是可见性清理的最大 ray tracing 长度
- `base_frame` 是地图中心 frame

因此当点云轴向还没有被可靠验证时, 拉长 `max_ray_length` 会把远处墙面, 悬挂物和错误竖向结构写入高程图

当前 launch 的 `publish_lidar_static_tf` 默认关闭, fallback LiDAR TF 也只有 identity 参数

因此暂时不能直接把 elevation mapping 切到 raw `lidar` frame, 否则会把点云改写问题替换成错误 TF 外参问题

启动脚本中的 `fastdds_profile` 默认值允许为空, launch 文件不能把该空字符串直接传给 `IfCondition`

FastDDS profile 环境变量只应在 `fastdds_profile` 非空时注入, 当前使用 `LaunchConfigurationNotEquals("fastdds_profile", "")` 实现

新增质量问题复查结果:

- raw 点云当前正 Z 包含高处建筑和墙面结构
- 运行中 aligned 点云精确执行 `(-x, -y, -z)`
- 全 XYZ 翻转会把正 Z 高处结构变成负 Z 深坑
- elevation layer 实测最小值约为 `-36.5m`
- `max_height_range` 会过滤高于传感器的点, 但不会过滤被错误翻到传感器下方的点

因此当前高程图质量差的直接原因是 Z 轴翻转

本轮修正将默认 `isaac_lidar_to_base` 改为:

```text
(x, y, z) -> (-x, -y, z)
```

旧全翻转保留为:

```text
negate_xyz
```

## 截图现象

原始点云截图中, 白色点云覆盖远处建筑, 墙面和地形轮廓, 视野明显大于机器人附近区域

高程图截图中, 彩色点和面主要集中在机器人附近小范围, 外围大部分为空

高程图 overlay 中红色机器人点, 黄色轨迹点和绿色区域存在疑似不重合, 需要区分 frame 偏移和 GridMap 有效 cell 不足

## 问题分析

### 范围小的确定原因

当前高程图配置使用 `map_length: 30.0`

这意味着 RViz 里的 GridMap footprint 最大只应是 30m x 30m 级别

即使 RTX LiDAR 配置允许约 70m 远距点云, elevation GridMap 也不会显示完整点云范围

所以高程图比点云小, 本身符合 rolling local map 设计

当前不应直接把 rolling local map 调成完整 LiDAR 量程视图

### 范围过小的可能原因

如果实际有效彩色区域明显小于 30m, 需要继续检查:

- `max_ray_length: 20.0` 会进一步限制 ray integration 距离
- `max_height_range: 2.0` 会过滤与局部高度差过大的点
- `min_valid_distance: 0.3` 会过滤近距离点
- elevation layer 或 traversability layer 中 NaN 过多, graph construction 会把它们视为 unknown
- `traversability` 归一化和阈值可能让大量 cell 无法成为 free
- 点云 frame 或轴向错误会导致地面点被解释成墙面, 高处面或无效点

### 偏移的主要风险

当前源码配置和目标文档存在 frame 分歧:

```text
目标文档: map
elevation_mapping_sim.yaml: odom
graph_construction_elevation.yaml: odom
odom adapter runtime: odom
```

如果 RViz fixed frame, GridMap display, graph markers, odom adapter 和 elevation mapping 使用不同 frame, 会出现整体偏移

这类问题不应通过 z offset 或阈值调参解决

### 点云轴转换风险

当前 adapter 把原始点云固定翻转并重写 frame 为 `base_link`

风险包括:

- 原始点云如果已经在正确 LiDAR frame, 真实外参会被绕过
- `(-x, -y, -z)` 可能导致前后, 左右或上下同时翻转
- elevation mapping 再用 `odom <- base_link` 转换时, 错误会被写入高程图
- 结果表现可能是高程面偏移, 镜像, 只有局部点有效, 或地面被识别成竖直面

新增截图中的竖向彩色幕布更接近“错误轴向加长射线”而不是“地图窗口太小”

因此本轮修复先恢复默认轴向和保守参数, 再继续做真实 TF 外参链路

### GridMap 坐标映射风险

当前 GridMap 直连代码使用 `grid_map_core` 风格:

```text
origin_x = center_x + length_x / 2
origin_y = center_y + length_y / 2
x = origin_x - (row + 0.5) * resolution
y = origin_y - (col + 0.5) * resolution
```

如果 upstream GridMap 实际数组布局或 circular buffer 解码和这里假设不一致, graph marker 会相对 GridMap surface 出现整体转置, 镜像或平移

所以偏移问题需要同时看 GridMap display 和 graph marker, 不能只看 graph 点

## 当前只读运行检查

第一次使用 FastDDS 环境检查 topic 时, 只看到:

```text
/spot1/odom_for_scoring
```

当前 `/spot1/odom_for_scoring` header:

```text
frame_id: odom
```

当前 `/spot1/odom_for_scoring` position:

```text
x: 18.144
y: 41.778
z: 0.042
```

未发现:

```text
/elevation_mapping_node/elevation_map_raw
/unitree_go2/lidar/points
/unitree_go2/lidar/points_aligned
/spot1/nav_graph
/spot1/graph_construction_viz
```

因此本次无法从运行时直接确认截图时刻的 GridMap frame, length, pose 和有效 cell 统计

后续使用 CycloneDDS 复查时, 三条关键运行时数据如下:

```text
/elevation_mapping_node/elevation_map_raw
  header.frame_id: odom
  info.resolution: 0.2
  info.length_x: 30.0
  info.length_y: 30.0
  info.pose.position: (17.987, 41.733, 0.000)
  info.pose.orientation: (0.000, 0.000, 0.000, 1.000)
  layers: elevation, traversability, variance
```

```text
/unitree_go2/lidar/points
  header.frame_id: lidar
  width: 61908
  finite points: 61908
  xyz min: (-7.801, 1.565, -1.552)
  xyz max: (73.890, 154.710, 10.022)
```

```text
/unitree_go2/lidar/points_aligned
  header.frame_id: base_link
  width: 36066
  finite points: 36066
  xyz min: (0.875, -11.211, -4.824)
  xyz max: (27.318, 1.434, 0.544)
```

TF 检查结果:

```text
odom -> base_link: available
odom -> lidar: unavailable
```

这说明当前不能直接把 raw `lidar` frame 点云交给 elevation mapping

同时旧 aligned 点云范围相比 raw 点云明显被压缩, 是高程图过小的主要运行时证据

## 可能原因排序

- 新增杂乱高程图的主因是上一轮错误轴向试验叠加 `max_ray_length: 70.0`, 远处墙面和竖向结构被 ray tracing 和高程融合放大
- 高程图范围小的主因是 `map_length: 30.0` 和 `max_ray_length: 20.0`, 这是 rolling local map 配置, 本身不等于 bug
- 有效区域过小仍可能来自 `max_height_range`, NaN 过滤, traversability 过滤和高程后端地表融合策略
- 偏移或镜像仍可能来自 `odom` / `map` frame 不统一, 以及点云 frame 被 adapter 改写为 `base_link`
- 如果 graph overlay 相对 GridMap surface 错位, 需要继续验证 GridMap row / column 到 world 的转换约定
- install tree 需要在每次配置修改后重新 build 或确认 symlink 生效, 避免运行时继续使用旧参数

## 本次修正

- `pointcloud_axis_adapter` 默认 `isaac_lidar_to_base` 改为 `(-x, -y, z)`
- 新增显式诊断模式 `isaac_y_forward_to_base` 和 `y_forward`, 执行 `(x, y, z) -> (y, -x, z)`
- `elevation_mapping_sim.yaml` 中 `map_length` 恢复为 `30.0`
- `elevation_mapping_sim.yaml` 中 `max_ray_length` 恢复为 `20.0`
- `elevation_mapping_sim.yaml` 中 `max_height_range` 恢复为 `2.0`
- elevation 相关 launch 文件中 `pointcloud_axis_mode` 描述改为通用轴转换说明
- 已重新 build `graph_construction`, install tree 中的 `elevation_mapping_sim.yaml` 已同步为新参数
- 修复 elevation visual navigation 启动脚本中的坏 FastDDS profile 默认值
- `fastdds_profile` launch 参数默认改为空, 避免把不存在的 XML 路径传给所有 ROS 进程

本次暂不把 `odom` 切换为 `map`

原因是当前运行时已确认 `odom -> base_link` 可用, 但没有确认 `map -> base_link` 和 `map -> lidar` 稳定可用

直接切 `map` 可能让 elevation mapping 收不到 TF, 反而导致高程图为空

本次也不把 `max_ray_length` 拉到 LiDAR 量程, 后续范围问题优先通过有效 cell 统计和真实 TF 外参链路排查

长期核心修复应是补齐真实 `base_link -> lidar` 外参, 让 elevation mapping 使用 raw `lidar` frame 和 `/tf` 处理传感器位姿

## 验证步骤

重启 elevation launch 后优先执行:

```bash
ros2 topic echo --once /elevation_mapping_node/elevation_map_raw --field header
ros2 topic echo --once /elevation_mapping_node/elevation_map_raw --field info
ros2 topic echo --once /unitree_go2/lidar/points --field header
ros2 topic echo --once /unitree_go2/lidar/points_aligned --field header
ros2 topic echo --once /spot1/odom_for_scoring --field header
ros2 topic echo --once /spot1/odom_for_scoring --field pose.pose.position
```

然后检查 TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom lidar
ros2 run tf2_ros tf2_echo map lidar
```

再统计点云范围:

```text
/unitree_go2/lidar/points: frame, x/y/z min, x/y/z max
/unitree_go2/lidar/points_aligned: frame, x/y/z min, x/y/z max
```

最后统计 GridMap:

```text
header.frame_id
info.length_x
info.length_y
info.pose.position
layers
finite elevation cell count
finite traversability cell count
```

重启后预期:

```text
/elevation_mapping_node/elevation_map_raw.info.length_x = 30.0
/elevation_mapping_node/elevation_map_raw.info.length_y = 30.0
/unitree_go2/lidar/points_aligned.header.frame_id = base_link
/unitree_go2/lidar/points_aligned should stop producing vertical curtain artifacts
```

## 修改建议

后续继续按运行数据改代码

优先级建议:

- 先统一 global frame, 在 `map` 和 `odom` 中选一个, 并让 elevation mapping, odom adapter, graph construction, RViz fixed frame 一致
- 再验证是否还需要 `pointcloud_axis_adapter`, 如果原始点云 TF 正确, 优先保留原始 frame 并让 TF 处理外参
- 如果确实需要轴转换, 用实测 xyz min / max 和 RViz 方向验证每个 axis mode, 不直接默认 `(-x, -y, -z)`
- 再验证 GridMap cell 坐标映射, 用 GridMap center, footprint marker 和一个已知点做对照
- 最后再按需要小步调整 `map_length`, `max_ray_length`, `max_height_range` 和 traversability 阈值, 不把 ray tracing 长度直接拉到 LiDAR 量程

## 结论

当前不能把问题简单归因于 LiDAR range 不够

原始点云范围大而高程图小, 首先符合 rolling elevation map 的设计

真正需要修的是全链路坐标一致性和点云进入 elevation mapping 前的 frame / axis 语义

本次已先回退会制造竖向伪影的配置, 后续确认运行时 GridMap header 和点云 frame 后, 再做下一步最小修改
