# 2026-07-07 Combined grid implementation reference

## 背景

Unity 2D 曾临时切换到同事发布的 `/combined_grid` 后, 地图和路径稳定性明显好于本仓库旧版 `livox_grid_builder`。当前默认已切回自研 `livox_grid_builder`, `/combined_grid` 只作为对照测试输入

本文件记录 `/combined_grid` 实现中值得参考的部分, 用于修复和对照自研 grid builder

## 目标

- 记录 `/combined_grid` 的坐标和栅格策略
- 说明它为什么能减少机器人转动时的地图旋转和路线跳变
- 为后续自研 grid builder 对齐提供依据

## 代码链路

节点:

- node name: `map_pub`
- input pointcloud: `/mapokk`
- input odom: `/unity/odom`
- output grid: `/combined_grid`
- output type: `nav_msgs/msg/OccupancyGrid`
- output frame: `odom_3D`

运行时检查:

```text
/combined_grid type: nav_msgs/msg/OccupancyGrid
/combined_grid publisher: /map_pub
/combined_grid frame_id: odom_3D
```

## 实现要点

### rolling local map

`/combined_grid` 是机器人附近的 rolling local costmap, 不是固定全局大地图

窗口大小默认 `20m x 20m`, 分辨率 `0.1m`, 用奇数 cell 数保证存在唯一中心像素

### center pixel lock

实现中强制让机器人中心落在 grid 中心像素:

- `center_u = width_cells // 2`
- `center_v = height_cells // 2`
- 最终把中心 `3x3` cell 刷成 free

这个策略避免机器人当前位置附近被自身点云或离散误差污染

### origin snap

grid origin 不直接使用连续机器人坐标, 而是按 resolution 离散对齐:

```python
origin_x = floor((robot_x - center_u * resolution) / resolution) * resolution
origin_y = floor((robot_y - center_v * resolution) / resolution) * resolution
```

这样 origin 只会按 `0.1m` 的整数倍跳动, 避免浮点余数造成 RViz 和 planner 中的微小抖动

### direct world projection

点云 `X, Y, Z` 直接按 `odom_3D` 坐标投影到 grid:

```python
gu = ((X - origin_x) / resolution).astype(np.int32)
gv = ((Y - origin_y) / resolution).astype(np.int32)
```

这说明当前 Unity 侧给 `/combined_grid` 使用的点云已经和 `odom_3D` 对齐, 不需要在该节点内再做 `livox_frame -> map` 旋转

### height based obstacle

障碍不是用射线端点直接标记, 而是统计每个 cell 内的 `max_z` 和 `min_z`

障碍判据:

- cell 内高度差大于 `obstacle_height_threshold`
- 或最高点高于机器人 `robot_z + 0.3`

这个方式对墙体, 台阶, 高障碍更直接, 对平地点云更稳定

### layered dilation

实现使用 OpenCV 形态学膨胀生成多层障碍代价:

- 原始障碍: `100`
- 一层膨胀: `5`
- 二层膨胀: `-8`
- 三层膨胀: `-120`

当前 `nav_msgs/OccupancyGrid` 标准通常只定义 `-1` unknown, `0` free, `100` occupied。这里的负值是同事 A* 节点的自定义代价语义, graph construction 侧如果直接消费, 需要确认当前分类逻辑如何解释这些负值

## 对自研 grid builder 的启发

- Unity 2D 更适合先以 `odom_3D` 局部 rolling map 对齐, 不要在 grid builder 内混用 `map`, `livox_frame`, `base_link` 的旋转假设
- rolling origin 应按 resolution snap, 避免连续原点带来的像素级抖动
- robot anchor 附近应显式清 free, 避免机器人中心落在 unknown 或 obstacle
- 障碍判定可以参考 cell 内高度差, 不只依赖单点高度阈值
- 如果临时回切 `/combined_grid`, graph construction 的 `global_frame` 和 grid frame 需要保持 TF 可达

## 验证步骤

检查 `/combined_grid`:

```bash
ros2 topic info -v /combined_grid
ros2 topic echo --once /combined_grid --field header
```

检查 graph 是否消费该 grid:

```bash
ros2 topic echo --once /spot1/nav_graph
ros2 topic echo --once /spot1/graph_construction_viz
```

检查是否还有本仓库 grid builder 干扰:

```bash
ros2 node list | grep livox_grid_builder
```

Unity `/combined_grid` 对照测试链路下, 不应看到本仓库 `livox_grid_builder` 节点。Unity 默认链路下应看到本仓库 `livox_grid_builder` 节点
