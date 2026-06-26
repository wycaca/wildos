# Graph Construction 修正说明: 几何地图与 3 相机视觉解耦

日期: 2026-06-26

## 背景

阶段三最初把 3 相机适配和 `elevation_mapping_cupy` 接入放在同一份实现文档中, 容易造成误解:

```text
3 相机变更 -> /spot1/traversability_grid 质量变化
```

这个理解不符合 WildOS 论文结构

## 正确链路

`/spot1/traversability_grid` 是默认 LiDAR baseline 的几何地图输入:

```text
/livox/lidar
    -> geometric map backend
    -> /spot1/traversability_grid
    -> graph_construction
```

elevation 实验后端使用独立 GridMap 输入:

```text
/livox/lidar
    -> elevation_mapping_cupy
    -> /elevation_mapping_node/elevation_map_raw
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
- graph_construction 直接消费 `grid_map_msgs/GridMap`
- elevation layer 作为节点和边的 z 高度来源
- traversability layer 作为 free, obstacle, unknown 分类来源
- 是否能在后续替代 `livox_grid_builder`

实验后端默认不发布 `/spot1/traversability_grid`, 也不默认启动 2D projection adapter, 避免 RViz Map 闪烁和误把 2D 投影视为论文主地图

`/spot1/elevation_traversability_grid` 仍可由 `grid_map_to_occupancy` 单独发布, 仅用于 debug 或兼容旧 OccupancyGrid 工具

该后端不作为 3 相机视觉适配的前置条件

## RViz 可视化

`/spot1/traversability_grid` 和 `/spot1/elevation_traversability_grid` 都是 `nav_msgs/OccupancyGrid`, 使用 RViz `Map` 显示

`/elevation_mapping_node/elevation_map_raw` 是 `grid_map_msgs/GridMap`, 不能用 RViz `Map` 显示, 需要使用 `grid_map_rviz_plugin/GridMap`

论文演示中的底图更接近该 GridMap 2.5D 高程面, 不是 2D `OccupancyGrid` 投影

论文界面目标不是把所有元素压到一个 z 平面, 而是在同一 `map` frame 和同一 GridMap elevation convention 下分层显示:

```text
GridMap terrain surface
    + graph node / edge overlay
    + frontier / score marker overlay
    + selected path overlay
    + camera and top-down inset views
```

因此:

- 白色 `robot_position` marker 应表示机器人 XY 投影到 GridMap elevation 的地面点
- 灰色 `robot_odom_position` marker 才表示 raw odom / base 位置
- graph nodes 和 frontier points 应贴近 GridMap 表面
- path, score ring, node glyph 可以有小的可视化 z offset
- raw odom / base marker 不要求贴地
- 如果 graph nodes 大量出现在竖直墙面或高处板面, 优先修 elevation backend 和 graph 采样过滤

详细目标见 `2026-06-26-paper-ui-target-correction.md`

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
graph_construction 可直接消费 /elevation_mapping_node/elevation_map_raw
graph nodes 和 edges 的 z 坐标跟随 elevation layer
graph nodes, edges, frontiers 稳定
robot ground marker 贴近 GridMap elevation surface
raw odom marker 仅作为 base / odom debug
```

只有当 elevation backend 的 grid 质量稳定后, 才考虑把它设为默认 graph construction 后端
