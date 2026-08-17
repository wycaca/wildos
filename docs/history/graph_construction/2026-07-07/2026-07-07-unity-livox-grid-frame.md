# 2026-07-07 Unity Livox grid frame

## 背景

Unity 2D 运行中, RViz 里点云和 `/spot1/traversability_grid` 明显错位, graph 和 OccupancyGrid 看起来被机器人本体姿态带歪

## 目标

- Unity profile 下, 2D 栅格生成以点云实际坐标为准
- 当前默认订阅 `/mapokk`, 直接投影已经对齐到 `odom_3D` 的点坐标
- 保持 Isaac 和真实狗仍按标准 LiDAR 局部 frame 到 `map` 的 TF 流程

## 问题分析

`livox_grid_builder` 默认假设 `PointCloud2` 的 XYZ 坐标在 `header.frame_id` 指定的 LiDAR 局部坐标系中, 因此会查 `cloud.header.frame_id -> grid_frame` 的 TF 并转换到 `map`

早期运行时检查曾怀疑 `/livox/lidar` 的 XYZ 已经在 map 坐标下, 因此尝试跳过 `livox_frame -> map` TF

后续现场观察到走 TF 时自研地图仍会旋转, 说明当前 fallback identity LiDAR TF 不能解释 Unity raw LiDAR 点云轴向.当前 Unity 2D 使用 `/mapokk` 和 `lidar_assume_input_in_grid_frame=true` 直接投影点云

直接投影点云后, grid 中心也必须使用与点云一致的 `odom_3D` 位置, 避免局部 grid 原点和点云参考系不一致

进一步看原论文和社区实现, WildOS 依赖稀疏导航图保存空间记忆, 局部几何感知不应导致已探索空间和路径目标每帧整体重排.社区 planner 默认 `path_smoothness_period=10.0`, 但当前 launch 之前把它设成 `0.0`, 导致每次 graph 更新都可以立即切换路线

## 修改内容

- `topic_profiles.yaml` 新增 `lidar_assume_input_in_grid_frame`
- `topic_profiles.yaml` 新增 `grid_odom_topic`
- `topic_profiles.yaml` 新增 `grid_origin_mode`
- Unity profile 当前设置 `lidar_topic: /mapokk`
- Unity profile 当前设置 `grid_frame: odom_3D`
- Unity profile 当前设置 `global_frame_2d: odom_3D`
- Unity profile 当前设置 `lidar_assume_input_in_grid_frame: "true"`
- Unity profile 当前设置 `grid_height_diff_mark_rays_free: "true"`
- Unity profile 当前设置 `grid_height_diff_fill_unobserved_as_free: "true"`
- Unity profile 当前设置 `grid_height_diff_unknown_border_width: "1.2"`
- Unity profile 设置 `grid_odom_topic: /unity/odom`
- Unity profile 当前设置 `grid_origin_mode: rolling`
- Unity profile 当前启用 `grid_origin_snap_to_resolution: "true"` 和 `grid_obstacle_detection_mode: height_diff`
- Unity 2D `odom_pose_source_2d` 改为 `tf`, `fallback_to_message` 保持 `true`
- Isaac 和 robot profile 保持 `grid_origin_mode: rolling`, `lidar_assume_input_in_grid_frame: "false"`
- `wildos_2d_sim.launch.py` 将该 profile 键透传给 `livox_grid_builder.assume_input_in_grid_frame`
- `wildos_2d_sim.launch.py` 将 `grid_odom_topic` 透传给 `livox_grid_builder.odom_topic`
- `wildos_2d_sim.launch.py` 将 `grid_origin_mode` 透传给 `livox_grid_builder.origin_mode`
- `livox_grid_builder` 启动日志输出 `grid_frame` 和 `assume_input_in_grid_frame`
- `livox_grid_builder` 支持 `origin_mode=fixed`, 但 Unity 当前自研 builder 调试链路使用 rolling local map
- `livox_grid_builder` 支持 rolling origin 按 resolution 对齐, 奇数 cell 数, 机器人中心清空和 `height_diff` 障碍模式
- `livox_grid_builder` 对字符串形式的 bool 参数做显式转换, 避免 `"false"` 被 Python 当成 True
- 2D launch 将 `graphnav_planner.path_smoothness_period` 调整为 `15.0`, 并增加 path switch hysteresis

## 验证步骤

启动 Unity 2D:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true
```

预期日志:

```text
Livox 栅格构建已启动, lidar=/mapokk, ..., grid_frame=odom_3D, origin_mode=rolling, resolution=0.100, size=14.0x14.0, mode=height_diff, snap=True, force_odd=True, robot_clear=0.20, height_range=(-0.20,1.80), height_rays=True, height_fill_free=True, height_unknown_border=1.20, assume_input_in_grid_frame=True
odom frame adapter 已启动, ..., pose_source=tf
点云坐标按 odom_3D 处理, 跳过 map 到 odom_3D 的 TF 转换
收到第一帧 odom, frame=odom_3D
```

检查 RViz:

- 点云和 `/spot1/traversability_grid` 应在同一 `odom_3D` 坐标下重合
- graph nodes 应落在灰色 free 区域内
- 障碍和 unknown 边界不应随狗本体朝向整体旋转错位
- 机器狗运动时 OccupancyGrid 原点只应按 resolution 离散移动, 不应连续抖动或随朝向旋转
- `/spot1/graphnav_planner/path` 不应在每次 graph 更新时剧烈切换到完全不同的 frontier
