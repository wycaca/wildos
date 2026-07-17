# 目标定位偏移修复

日期: 2026-07-16

## 1. 问题现象

Unity 目标搜索运行时，目标 marker 和真实物体位置出现明显偏移，规划曾短暂切向远处错误目标

现场关键数据:

```text
robot odom=(15.68, -9.64, 0.20)
TRACKING position=(44.82, -39.53, -5.58), confidence=0.35, views=2
STABLE_VISION position=(18.35, -12.25, -0.03), confidence=0.81, lidar_support=32
later stable position=(18.75, -12.68, -0.17), confidence=0.69, views=4
```

错误粗目标在水平面上距离机器人约 42m，垂向偏差约 5.8m，但旧 Goal Mux 只检查视角数、置信度和状态，因此仍进入 `TARGET_APPROACH_COARSE`

后续稳定估计已经回到机器人附近，但第二次稳定结果相对首次结果只移动约 0.58m，小于旧的统一更新门槛 0.75m，因此没有继续纠偏

## 2. 原因分析

### 2.1 粗目标缺少物理合理性门控

旧 Goal Mux 没有检查:

- 目标 frame 是否等于当前全局 frame
- 目标与机器人之间的距离
- 目标与机器人之间的垂向偏差
- 粗目标水平协方差

两视角粒子估计刚进入 `TRACKING` 时仍可能处于较大深度不确定区间，仅凭 `confidence=0.35` 不足以接管导航

### 2.2 稳定目标和普通抖动共用更新门槛

旧代码统一使用 `target_update_min_distance=0.75m`

该门槛适合抑制普通粒子抖动，但会阻止稳定融合结果对首次稳定目标进行必要的小幅纠偏

### 2.3 Unity 粒子深度范围过大

目标粒子原先沿相机射线采样到 100m，远大于当前 Unity 目标搜索场景的有效范围，早期粒子容易扩散到射线远端

### 2.4 单帧 LiDAR 和密集背景可能过早影响目标

旧粒子滤波器在第一帧 LiDAR 候选出现时就更新权重，连续两帧约束只控制 `LIDAR_LOCKED` 状态，没有阻止单帧候选改变位置和置信度

旧 LiDAR 表面估计在 mask 投影点中选择最密集高点簇，目标后方墙面比目标表面更密集时，结果可能落到背景

## 3. 代码修复

### 3.1 Goal Mux 物理门控

文件:

```text
visual_navigation/visual_navigation/object_search_goal_mux.py
visual_navigation/configs/object_search_goal_mux.yaml
```

当前粗目标接纳条件:

- 至少两个有效视角
- 置信度不低于 `0.5`
- 与 odom 的平面距离不超过 `30m`
- 与 odom 的垂向偏差不超过 `1.5m`
- 水平标准差不超过 `8m`
- 目标 frame 与当前全局 frame 一致

稳定目标也必须通过 frame 和垂向偏差检查

普通目标继续使用 `target_update_min_distance=0.75m`，稳定目标改用独立的 `stable_target_update_min_distance=0.3m`

### 3.2 Unity 粒子最大深度

文件:

```text
graph_construction/configs/topic_profiles.yaml
graph_construction/graph_construction/topic_profiles.py
graph_construction/launch/elevation_visual_navigation_sim.launch.py
```

新增平台参数 `object_target_max_depth`

- Unity: `30m`
- Isaac: `100m`
- robot: `100m`

launch 将该参数传给 `object_target_fusion.max_depth`

### 3.3 LiDAR 连续性保护

文件:

```text
triangulation3d/triangulation3d/target_particle_filter.py
```

已有视觉轨迹时:

1. 第一帧 LiDAR 候选只记录位置和一致性
2. 不改变粒子、位置、置信度或 source
3. 第二帧候选与上一帧距离不超过 `lidar_consistency_radius` 时才更新粒子
4. 连续帧数达到 `lidar_lock_frames` 后进入 `LIDAR_LOCKED`

LiDAR 独立初始化路径仍可从第一帧候选建立粒子，但不会绕过锁定帧数要求

### 3.4 最近前景簇选择

文件:

```text
visual_navigation/visual_navigation/object_target_fusion.py
```

LiDAR mask 投影点处理流程:

1. 移除局部地面点
2. 查找满足最小支持数的高点簇
3. 使用相机平均位置作为参考点
4. 在有效簇中选择中位距离最近的前景簇
5. 使用该簇中位数作为目标可见表面位置

该规则避免目标后方更密集的墙面覆盖较小的目标表面簇

## 4. 测试覆盖

新增回归测试:

- 异常高度和距离粗目标不能替换初始探索 goal
- 水平不确定度过大的粗目标不能接管导航
- 稳定目标移动约 0.58m 时允许纠正首次稳定结果
- 普通约 0.22m 粒子抖动仍被抑制
- 单帧 LiDAR 不改变已有视觉目标的位置、置信度和 source
- mask 射线中存在更密集背景时仍选择最近前景目标簇

相关功能测试结果:

```text
67 passed
```

## 5. 编译和运行验证

增量构建:

```bash
cd /mnt/hhd/han/wildos_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  triangulation3d visual_navigation graph_construction
```

结果:

```text
3 packages finished
```

Unity 启动验证:

```bash
./scripts/start_wildos_elevation.sh do_object_search:=true
```

确认正常启动:

- `/livox/lidar` 到 `/livox/lidar_aligned`
- odom adapter
- elevation GridMap
- graph construction
- WildOS
- object target fusion
- Object Search Goal Mux
- graphnav planner

运行日志确认 Unity 实际加载:

```text
max_depth=30.0m
```

本次短测时目标最高相似度约为 `0.111`，低于检测峰值阈值 `0.12`，因此没有重新产生真实目标估计，定位保护行为由上述回归测试覆盖

Ctrl-C 停止时 `elevation_mapping_cupy` 仍会输出既有 `KeyboardInterrupt` 退出栈，不属于本次定位修复的运行错误

## 6. 运行边界

Unity 三路相机图像当前仍使用相同的 `camera_frame` header，ROS graph 没有提供三路独立真实相机外参

默认 launch 使用 `base_link -> front_camera/left_camera/right_camera` fallback static TF

本次修复解决异常粗目标接管、首次稳定结果冻结、单帧 LiDAR 覆盖和背景簇偏移问题，但不会猜测或修改 Unity 未发布的真实相机外参

如果后续仍出现方向和距离都近似固定的系统偏差，应先让 Unity 发布三路真实相机 TF，再关闭 `publish_camera_static_tf`
