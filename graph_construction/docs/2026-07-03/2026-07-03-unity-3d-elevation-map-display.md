# Unity 3D 高程图不显示诊断

日期: 2026-07-03

## 背景

用户反馈 2D 暂时正常后, 3D 启动中 RViz 不显示高程图

当前运行态确认在:

```text
ROS_DOMAIN_ID=89
RMW_IMPLEMENTATION=rmw_zenoh_cpp
topic_profile=unity
```

## 当前 3D 链路

Unity 3D elevation backend 的链路是:

```text
/livox/lidar
    -> pointcloud_axis_adapter
    -> /livox/lidar_aligned
    -> elevation_mapping_node
    -> /elevation_mapping_node/elevation_map_raw
```

`graph_construction` 直接订阅 `/elevation_mapping_node/elevation_map_raw`

RViz 显示高程图时应使用 `grid_map_rviz_plugin/GridMap`, 不是 `Map`

## 运行态诊断

已确认:

- `/elevation_mapping_node` 正在运行
- `/pointcloud_axis_adapter` 正在运行
- `/livox/lidar` 有 publisher, frame 为 `livox_frame`
- `/livox/lidar_aligned` 有 publisher 和 subscriber, frame 为 `base_link`
- `/elevation_mapping_node/elevation_map_raw` 有 publisher, 但 `echo --once` 等不到新样本

TF 检查结果:

```text
tf2_echo odom base_link
Waiting for transform odom -> base_link
Invalid frame ID "odom"
```

当前 Unity TF 实际发布:

```text
map -> odom_fram
odom_fram -> livox_frame
odom_fram -> camera_frame
```

没有 `map -> base_link` 或 `odom -> base_link`

## 问题分析

Unity profile 中原先配置:

```yaml
odom_parent_frame: map
odom_child_frame: base_link
camera_parent_frame: base_link
lidar_parent_frame: base_link
lidar_frame: livox
```

3D launch 会把 `odom_child_frame` 同时用于:

- `odom_frame_adapter.child_frame`
- `pointcloud_axis_adapter.output_frame`
- `elevation_mapping_node.base_frame`
- `elevation_mapping_node.initialize_frame_id`

因此 elevation mapping 实际等待的是 `map -> base_link`

但 Unity live TF 里的机器人 body frame 是 `odom_fram`, 所以 elevation mapping 收到点云端点, 但缺少需要的 TF, 无法持续发布有效 GridMap

## 修改内容

已将 Unity profile 统一到实际 TF frame:

```yaml
odom_child_frame: odom_fram
camera_parent_frame: odom_fram
lidar_parent_frame: odom_fram
lidar_frame: livox_frame
```

用户后续截图中能看到少量高程面, 但地面大面积缺失, 运行统计显示:

```text
/livox/lidar: frame=livox_frame, points=20000
/livox/lidar_aligned: frame=odom_fram, points=20000
GridMap elevation finite=1598 / 22500
```

这说明 GridMap 只有少量 cell 写入了 elevation, 大部分地面仍是 unknown

进一步看点云适配, 旧 Unity 3D 使用:

```text
axis_mode=isaac_lidar_to_base
output_frame=odom_fram
```

这只对 XYZ 做 `(-x, -y, z)` 并把 header frame 改成 `odom_fram`, 并没有真正执行 `livox_frame -> odom_fram` 的 TF 变换

Unity live TF 已经提供:

```text
map -> odom_fram
odom_fram -> livox_frame
```

因此 Unity 3D 不应把 LiDAR 局部点云重标成 body frame, 应保留点云在 `livox_frame` 下, 让 elevation mapping 自己通过 TF 转到 `map`

已新增 profile 级 3D 点云策略:

```yaml
pointcloud_axis_mode_3d: identity
pointcloud_output_frame_3d: livox_frame
```

Isaac 仍保留:

```yaml
pointcloud_axis_mode_3d: isaac_lidar_to_base
pointcloud_output_frame_3d: base_link
```

重启 3D 后, `pointcloud_axis_adapter` 应发布:

```text
/livox/lidar_aligned.header.frame_id = livox_frame
```

`elevation_mapping_node` 应使用:

```text
map_frame = map
base_frame = odom_fram
```

## 验证步骤

重启 3D:

```bash
export ROS_DOMAIN_ID=89
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export WILDOS_TOPIC_PROFILE=unity
./scripts/start_wildos_3d.sh
```

检查 TF:

```bash
ros2 run tf2_ros tf2_echo map odom_fram
```

检查 aligned 点云:

```bash
ros2 topic echo --once /livox/lidar_aligned --field header --qos-reliability best_effort
```

检查 GridMap:

```bash
ros2 topic echo --once /elevation_mapping_node/elevation_map_raw --field header
ros2 topic echo --once /elevation_mapping_node/elevation_map_raw --field info
ros2 topic echo --once /elevation_mapping_node/elevation_map_raw --field layers
```

RViz 显示:

```text
Class: grid_map_rviz_plugin/GridMap
Topic: /elevation_mapping_node/elevation_map_raw
Height Layer: elevation
Color Layer: elevation
Fixed Frame: map
```

## 后续

如果 GridMap topic 有消息但 RViz 仍不显示, 优先检查 RViz 是否添加的是 `grid_map_rviz_plugin/GridMap`

如果 `/livox/lidar_aligned` 有消息但 GridMap 仍无样本, 继续检查 elevation mapping 日志中的 TF lookup 或 point cloud filter 统计

## 暂缓问题 1: 机器狗近场地面不可观测

当前截图中的地面缺失还有一个独立问题: LiDAR 可能扫不到机器狗附近的地面

这类问题不是 graph construction 能直接补出来的, 因为 elevation mapping 只能融合实际观测到的点云, 传感器盲区内没有地面回波时, GridMap 对应 cell 会保持 unknown

常见原因:

- LiDAR 安装高度, 俯仰角和垂直 FOV 让近场射线从地面上方掠过
- LiDAR 最小测距, 机器人机身遮挡或自过滤半径把脚边地面点滤掉
- Unity / Isaac 的 LiDAR 模型没有正确模拟向下射线, 或 solid-state pattern 在近场覆盖不足
- elevation mapping 的点云过滤, robot footprint clearing 或高度阈值过严, 把近场地面点当成自车或噪声移除

优先解决方向:

1. 从传感器几何先解决, 调整 LiDAR pitch, mount height, vertical FOV 或扫描 pattern, 让射线实际打到机器人前方 0.3m 到 2.0m 的地面
2. 仿真中先用 RViz 直接看 `/livox/lidar` 和 `/livox/lidar_aligned`, 不经过 GridMap, 确认近场地面是否有点
3. 检查 LiDAR JSON 或 Unity sensor 参数里的 `nearRange`, `farRange`, 垂直角度范围和 scan pattern, 不要只改 ROS topic
4. 检查 elevation mapping 的输入过滤半径, 高度阈值和机器人 footprint 清理范围, 避免把真实近场地面点过滤掉
5. 若单个 LiDAR 物理上确实看不到脚边, 需要接受近场 unknown, 并在 graph 层用 robot anchor, 局部安全半径或历史地图做保守补偿

推荐验证指标:

- `/livox/lidar` 中机器人前方 0.3m 到 2.0m, 左右 1.0m 范围内是否存在稳定地面点
- `/livox/lidar_aligned.header.frame_id` 是否仍为 `livox_frame`
- `/elevation_mapping_node/elevation_map_raw` 中机器人附近 1m 半径内 `elevation` finite cell 比例是否明显提高
- RViz 中先显示原始点云, 再显示 GridMap, 区分传感器没有点和 GridMap 过滤掉点

当前建议: 暂缓在 graph construction 中做补洞逻辑, 先确认传感器几何和点云过滤

## 暂缓问题 2: AGX Orin 部署更新效率

如果这一套部署到机器狗上的 AGX Orin, 更新效率不能只靠堆频率, 需要按模块做预算和限频

当前 3D 链路的主要计算压力来自:

- 点云发布和点云坐标适配
- elevation mapping 的点云融合, rolling map 更新和 traversability 计算
- graph construction 的 GridMap 解码, 分类, frontier 检测, edge collision 和 ROS message 发布
- WildOS visual scoring 的图像同步, 模型推理和 graph scoring
- RViz / debug topic 的 marker 和 GridMap 可视化

建议性能策略:

1. 先定义目标频率, 例如 elevation map 5Hz 到 10Hz, graph construction 1Hz 到 3Hz, visual scoring 按相机和模型能力单独限频
2. 让高频传感器链路和低频规划链路解耦, 点云可以高频融合, graph 不需要每帧完整重建
3. graph construction 只处理 rolling map 的有效区域或变化区域, 避免每次全图重建
4. frontier path validation, edge validation 和 visual scoring 做限额, 每周期只处理 top-K 候选
5. debug 可视化默认降频或可关闭, 尤其是大 MarkerArray, GridMap 和点云 echo
6. 保持 stage timing 诊断开启, 用真实耗时决定优化顺序, 不凭感觉调参数
7. 在 AGX Orin 上优先使用 GPU 能加速的 elevation mapping 和模型推理, CPU 侧避免 Python 大循环处理全量点云

建议验收指标:

- elevation mapping 单周期耗时和发布频率
- graph construction `classify`, `builder_update`, `to_msg`, `publish` 分阶段耗时
- `/spot1/nav_graph` 和 `/spot1/scored_nav_graph` 的实际发布频率
- WildOS 推理耗时, 相机同步等待时间和丢帧比例
- AGX Orin 上 CPU, GPU, 内存带宽和温度是否触发降频

当前建议: 暂缓新增优化代码, 先用 stage timing 和真实 AGX Orin runtime 采样确定瓶颈
