# 2026-07-07 Livox grid builder stabilization

## 背景

Unity 2D 使用本仓库 `livox_grid_builder` 时, 机器人转动会导致 OccupancyGrid 和路线明显跳变。临时切换同事 `/combined_grid` 后, 地图和路线稳定性明显改善

本次将 `/combined_grid` 中可复用的稳定化策略移植到本仓库 `livox_grid_builder`

## 目标

- 自研 `livox_grid_builder` 支持 rolling local map 的 resolution 对齐
- 支持奇数 cell 数, 让机器人中心有唯一中心 cell
- 支持清空机器人中心附近 cell, 避免自身点云污染 anchor
- 支持按 cell 内高度差判定障碍
- 保持 Isaac 和 robot 默认 `ray_height` 旧行为不变

## 问题分析

原 `livox_grid_builder` 主要使用射线清空和单点高度阈值:

- 每个点先按 TF 或输入 frame 转到 grid frame
- 射线经过区域标 free
- 点高度相对机器人超过阈值时标 obstacle

这套逻辑对标准 LiDAR 局部 frame 有效, 但 Unity 点云和 odom frame 混合时更容易受坐标假设, 原点连续移动和自身点云影响

同事 `/combined_grid` 更稳定的关键原因:

- rolling origin 按 resolution 取整
- 机器人锁在 grid 中心 cell
- 中心附近 cell 强制 free
- 用 cell 内 `max_z - min_z` 判断墙体和障碍

## 修改内容

`livox_grid_builder` 新增参数:

- `origin_snap_to_resolution`
- `force_odd_grid_size`
- `robot_clear_radius`
- `obstacle_detection_mode`
- `height_diff_obstacle_threshold`
- `high_obstacle_min_height`

新增两种障碍模式:

- `ray_height`, 原有射线清空和单点高度阈值模式
- `height_diff`, 统计每个 cell 内 `max_z`, `min_z`, 用高度差和高点阈值标障碍

Unity profile 的自研 builder 调试参数:

- `grid_origin_mode: rolling`
- `grid_resolution: "0.1"`
- `grid_local_width: "20.0"`
- `grid_local_height: "20.0"`
- `grid_min_obstacle_height: "0.0"`
- `grid_max_obstacle_height: "1.8"`
- `grid_obstacle_inflation_radius: "0.2"`
- `grid_origin_snap_to_resolution: "true"`
- `grid_force_odd_grid_size: "true"`
- `grid_robot_clear_radius: "0.2"`
- `grid_obstacle_detection_mode: height_diff`
- `grid_height_diff_obstacle_threshold: "0.05"`
- `grid_high_obstacle_min_height: "0.3"`

当前 Unity 默认已切回本仓库自研 `livox_grid_builder`

## 验证步骤

默认 Unity 使用本仓库自研 `livox_grid_builder`:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true
```

临时回切 `/combined_grid` 对照测试:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true launch_livox_grid_builder:=false traversability_grid_topic:=/combined_grid
```

预期 builder 日志:

```text
Livox 栅格构建已启动, ..., origin_mode=rolling, resolution=0.100, size=20.0x20.0, mode=height_diff, snap=True, force_odd=True, robot_clear=0.20, height_range=(0.00,1.80), inflation=0.20
```

检查输出:

```bash
ros2 topic echo --once /spot1/traversability_grid --field header
ros2 topic echo --once /spot1/traversability_grid --field info
```

自研 builder 测试通过的判据:

- 机器人原地转动时, grid 不应整体旋转漂移
- graph nodes 不应大范围重排
- `/multi_planned_path` 不应每帧跳向完全不同方向
- 如果自研 builder 仍不稳定, 用 `/combined_grid` 对照测试确认问题边界
