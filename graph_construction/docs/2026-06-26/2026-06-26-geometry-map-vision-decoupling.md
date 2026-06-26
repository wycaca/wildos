# Graph Construction 修正说明: 几何地图与 3 相机视觉解耦

日期: 2026-06-26

## 背景

阶段三最初把 3 相机适配和 `elevation_mapping_cupy` 接入放在同一份实现文档中, 容易造成误解:

```text
3 相机变更 -> /spot1/traversability_grid 质量变化
```

这个理解不符合 WildOS 论文结构

## 正确链路

`/spot1/traversability_grid` 是几何地图输入, 当前应由 LiDAR 或 elevation backend 生成:

```text
/livox/lidar
    -> geometric map backend
    -> /spot1/traversability_grid
    -> graph_construction
```

3 相机只进入视觉评分链路:

```text
/camera/front
/camera/left
/camera/right
    -> visual_navigation / WildOS
    -> /spot1/scored_nav_graph
```

因此, 3 相机适配不应直接影响 `/spot1/traversability_grid`

## 默认后端

当前仿真默认使用 `livox_grid_builder` 作为稳定 baseline:

```bash
./scripts/start_graph_construction_livox.sh
./scripts/start_visual_navigation.sh
```

旧脚本仍保留为兼容入口:

```bash
./scripts/start_graph_construction.sh
```

## 实验后端

`elevation_mapping_cupy` 后端作为独立实验:

```bash
./scripts/start_graph_construction_elevation.sh
```

该后端用于验证:

- `/elevation_mapping_node/elevation_map_raw`
- `grid_map_to_occupancy`
- `/spot1/elevation_traversability_grid`
- elevation / traversability layer 到 OccupancyGrid 的压缩质量
- 是否能在后续替代 `livox_grid_builder`

实验后端默认不发布 `/spot1/traversability_grid`, 避免和 LiDAR baseline 同名发布造成 RViz Map 闪烁

该后端不作为 3 相机视觉适配的前置条件

## RViz 可视化

`/spot1/traversability_grid` 和 `/spot1/elevation_traversability_grid` 都是 `nav_msgs/OccupancyGrid`, 使用 RViz `Map` 显示

`/elevation_mapping_node/elevation_map_raw` 是 `grid_map_msgs/GridMap`, 不能用 RViz `Map` 显示, 需要使用 `grid_map_rviz_plugin/GridMap`

推荐 GridMap display 配置:

```text
Class: grid_map_rviz_plugin/GridMap
Topic: /elevation_mapping_node/elevation_map_raw
Height Layer: elevation
Color Layer: elevation
Height Transformer: GridMapLayer
Color Transformer: GridMapLayer
Use Rainbow: true
Autocompute Intensity Bounds: true
Fixed Frame: map
```

如果 display 仍是灰色, 优先检查:

```bash
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic echo /elevation_mapping_node/elevation_map_raw --once --field layers
ros2 topic info /elevation_mapping_node/elevation_map_raw -v
```

常见原因:

- 当前只启动了 LiDAR baseline, elevation topic 不存在
- RViz 添加的是 `Map`, 不是 `grid_map_rviz_plugin/GridMap`
- `Height Layer` 或 `Color Layer` 没选 `elevation`
- Fixed Frame 和 GridMap header frame 没有 TF
- elevation layer 数值近似常数或大部分是 NaN, 需要继续调 elevation mapping 参数

## 验收策略

默认阶段三验收:

```text
LiDAR baseline /spot1/traversability_grid 稳定
3 camera visual scoring 正常
/spot1/scored_nav_graph 正常
/spot1/graphnav_planner/path 可生成
```

elevation backend 单独验收:

```text
/elevation_mapping_node/elevation_map_raw layers 正常
/spot1/elevation_traversability_grid 质量不低于 LiDAR baseline
graph nodes, edges, frontiers 稳定
```

只有当 elevation backend 的 grid 质量稳定后, 才考虑把它设为默认 graph construction 后端
