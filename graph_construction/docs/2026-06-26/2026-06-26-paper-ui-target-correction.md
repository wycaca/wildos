# WildOS 论文界面目标修正

日期: 2026-06-26

## 背景

根据论文演示界面截图, 当前开发目标需要从"把所有 marker 强行压到同一个平面"修正为"按论文 UI 分层表达几何地图, 稀疏图, frontier, path 和视觉叠加"

截图中的主视图不是纯 2D map, 也不是所有元素共用一个 z 平面

它更接近下面的结构:

```text
2.5D terrain surface
    + traversability / semantic color
    + sparse navigation graph overlay
    + frontier / node glyph overlay
    + selected path overlay
    + camera and top-down inset panels
```

## 论文界面观察

主 RViz / 3D 视图中可见:

- 底图是 2.5D 地形表面, 有坡度和高低起伏
- 地表颜色表达不同语义或可通行性区域, 不是单色 occupancy grid
- 红色细线是稀疏 graph edges 或可见连接关系, 大体贴近地表, 但为了显示可读性可能有轻微 z offset
- 绿色半球或圆点是 graph nodes / frontier candidates / visited points 一类 overlay glyph, 它们不是地形本体
- 粗绿色曲线是规划路径或轨迹, 明显作为高亮 overlay 显示在地形上方
- 蓝色圆环表示 scored frontier / waypoint / candidate marker, 属于高层导航可视化
- 左侧是相机输入和视觉输出 overlay, 与几何地图不是同一个 display
- 右上角是 top-down / satellite inset, 用于展示全局路线, start, goal, deadend 等高层状态

## 修正后的开发目标

### 目标一, 坐标系统统一

所有几何对象必须处在同一个 global frame, 当前仿真为 `map`

必须确认:

```text
/elevation_mapping_node/elevation_map_raw.header.frame_id = map
/spot1/graph_construction_viz marker header.frame_id = map
/spot1/nav_graph.header.frame_id = map
/unity/odom pose 可解释为 map frame 下位置
```

如果 frame 不统一, 先修 TF / odom adapter / launch remap, 不通过调 z offset 掩盖问题

### 目标二, 地形表面统一

graph node, edge endpoint, frontier point, robot ground point 的 z 值应来自同一套 GridMap `elevation` layer

这表示:

- graph node 的 position.z 使用其所在 GridMap cell 的 elevation
- frontier point 的 z 使用其所在 GridMap cell 的 elevation
- robot white marker 表示机器人 XY 投影到 GridMap elevation 的 ground point
- raw odom / base marker 可以单独显示, 但不能用它判断地面是否对齐

因此, 白色 robot ground marker 应贴近高程图表面

如果白点不贴地, 优先检查:

- robot XY 下方 GridMap cell 是否有有效 elevation
- GridMap display 和 marker 是否同 frame
- `grid_to_world` / `world_to_grid` 是否和 `grid_map_core` 一致
- elevation layer 是否包含竖直面或 NaN 空洞

### 目标三, 可视化 overlay 分层

不是所有 marker 都必须完全贴在同一个 z 值上

合理的显示约定:

```text
terrain surface: raw GridMap elevation
graph nodes: elevation + small marker radius
graph edges: endpoint elevation + small edge z offset if needed
frontier points: elevation + small glyph z offset
path: elevation + larger visual z offset
robot ground point: elevation
raw odom point: raw odom z, debug only
grid footprint: map boundary debug, can use fixed z or elevation-independent line
```

重点是语义一致, 不是视觉元素完全共面

### 目标四, elevation backend 不等同 2D 雷达 baseline

`/spot1/traversability_grid` 由 LiDAR baseline 生成, 是当前稳定默认路径

论文 UI 中更接近 2.5D elevation surface 的部分, 应由 experimental elevation backend 验证:

```text
/livox/lidar
    -> elevation_mapping_cupy
    -> /elevation_mapping_node/elevation_map_raw
    -> graph_construction GridMap path
```

不要把 3 相机适配, 2D occupancy projection, elevation surface alignment 混为同一个调参问题

## 当前问题重新定义

当前截图里的问题应拆成三类:

### A. 高程图本体问题

如果 GridMap surface 出现大片竖直墙面, 悬空板, 锯齿帘状面, 那是 elevation mapping 输出质量问题

graph construction 不应通过阈值把这些面当作地面, 但也不能完全修复 elevation backend 的 surface 本体

### B. graph 采样问题

如果 graph node 出现在墙面或高处 surface 上, 说明采样没有足够约束地面高度或 traversability

当前修正方向:

- 使用 GridMap elevation 给 node / frontier 赋 z
- 使用 robot ground position 做高度门限
- 后续可增加 slope / height discontinuity 过滤

### C. marker 语义问题

如果 robot marker 使用 raw odom z, 它可能代表 base / body center, 不应该要求它贴地

当前修正方向:

- 白色 `robot_position` 表示 GridMap ground projection
- 灰色 `robot_odom_position` 表示 raw odom / base 位置
- 用两者 z 差判断 base 高度和地面高度是否一致

## 新验收标准

### 必须满足

- GridMap 高程面在 RViz 中无整体镜像, 转置, 大幅平移
- 白色 robot ground marker 落在 GridMap 高程面附近
- graph nodes 大部分落在可通行地形表面附近
- graph edges 不再出现大量跨越高处竖直面的长线
- current node 与 robot ground marker 在 XY 上接近
- 高程图, graph marker, path marker 的 frame 均为 `map`

### 可以接受

- graph node 球体中心略高于地表, 因为 marker 有半径
- path / frontier / score ring 略高于地表, 因为它们是可视化 overlay
- raw odom / base marker 高于地表, 因为机器人机体中心本来不在地面上
- 局部 GridMap 仍存在少量噪声, 只要 graph 不采样到明显不可通行面

### 不可接受

- 白色 robot ground marker 长期悬空或埋在地面下
- graph nodes 大量出现在竖直墙面或高处板面
- edges 大量连接地面和墙面, 形成竖直红线
- GridMap surface 与 graph marker 在 XY 上整体错位
- 把 2D `/spot1/traversability_grid` 调参当成 elevation backend 对齐修复

## 后续开发优先级

1. 先确认 frame 和 GridMap cell 坐标约定完全正确
2. 再确认 robot ground marker 能投影到 elevation surface
3. 再确认 graph node / frontier z 值都来自同一 GridMap elevation
4. 再加入 elevation surface 质量过滤, 包括高度突变, slope, 局部法向或 variance
5. 最后恢复视觉评分和 planner 联调, 不在几何未稳定前调 3 相机

