# Graph Builder 分层边界重构说明

日期: 2026-07-02

目标: 先执行 `graphnav_builder` 中最有价值的分层边界思想, 将当前 `graph_construction` 拆成 ROS 适配层, 输入解码层, 纯算法层, 消息转换层和可视化层

## 背景

外部 `graphnav_builder` 的核心工程优点是:

```text
ROS node
    -> GridMap / OccupancyGrid 解码
    -> source-agnostic grid
    -> SparseGraphBuilder
    -> NavigationGraph message conversion
```

当前项目原本已有模块拆分, 但 `graph_builder.py` 仍直接接收 `OccupancyGrid`, `GridMap`, `Odometry`, 并返回 `NavigationGraph`

这会让纯图算法层同时承担 ROS 消息解码和 ROS 消息生成职责, 不利于后续单元测试, GridMap 坐标验证和同步诊断

## 本次开发目标

本次只重构边界, 不改变 topic, 参数语义和 graph 构建算法

目标边界:

```text
node.py
    ROS topic, QoS, timer, 参数, first-message 日志, 发布

grid_adapter.py
    OccupancyGrid / GridMap -> ClassifiedGrid

grid_types.py
    ClassifiedGrid, 坐标转换, elevation 查询, collision line, distance field

graph_builder.py
    SparseGraphBuilder, 只接收 ClassifiedGrid, robot_position, stamp_seconds

msg_utils.py
    GraphState -> graphnav_msgs/NavigationGraph

viz.py
    GraphState + ClassifiedGrid -> MarkerArray
```

## 代码链路

当前 ROS 入口仍是:

```text
GraphConstructionNode._on_timer
```

更新后的调用链:

```text
latest OccupancyGrid / GridMap
    -> GraphConstructionNode._classify_latest_grid
    -> classify_occupancy_grid / classify_grid_map
    -> ClassifiedGrid
    -> GraphBuilder.update
    -> GraphUpdateResult
    -> graph_to_msg
    -> nav_graph publisher
    -> GraphVisualizer.build_markers
    -> marker publisher
```

`GraphBuilder` 现在是 `SparseGraphBuilder` 的兼容别名:

```text
SparseGraphBuilder
GraphBuilder = SparseGraphBuilder
```

这样保留旧 import 的兼容性, 同时代码名表达纯算法层职责

## 主要改动

### 新增 `grid_types.py`

新增纯数据和几何工具:

- `ClassifiedGrid`
- `GridIndex`
- `distance_to_mask`
- `bresenham_line`

`ClassifiedGrid` 不依赖 ROS 消息, 负责:

- `world_to_grid`
- `grid_to_world`
- elevation 查询
- robot ground projection
- free / obstacle / unknown 查询
- world-space collision check

### 收紧 `grid_adapter.py`

`grid_adapter.py` 现在只负责 ROS 地图输入适配:

- `classify_occupancy_grid`
- `classify_grid_map`
- GridMap layer decode
- GridMap circular buffer unwrap
- traversability normalization
- classification postprocess

不再定义 `ClassifiedGrid`, `distance_to_mask`, `bresenham_line`

### 收紧 `graph_builder.py`

`graph_builder.py` 不再导入:

```text
rclpy
nav_msgs
grid_map_msgs
std_msgs
graphnav_msgs
msg_utils
```

`SparseGraphBuilder.update` 新接口:

```text
update(ClassifiedGrid, robot_position, stamp_seconds) -> GraphUpdateResult
```

`GraphUpdateResult` 包含:

- `graph`
- `classified_grid`
- `frontier_cell_count`

ROS header, frame_id 和 `NavigationGraph` 转换都移到 `node.py` 和 `msg_utils.py`

### 更新引用

以下模块改为从 `grid_types.py` 读取纯类型或工具:

- `graph_builder.py`
- `edge_builder.py`
- `frontier_detector.py`
- `viz.py`
- `livox_grid_builder.py`

## 验证步骤

已执行:

```bash
python3 -m py_compile \
  graph_construction/graph_construction/grid_types.py \
  graph_construction/graph_construction/grid_adapter.py \
  graph_construction/graph_construction/graph_builder.py \
  graph_construction/graph_construction/node.py \
  graph_construction/graph_construction/edge_builder.py \
  graph_construction/graph_construction/frontier_detector.py \
  graph_construction/graph_construction/viz.py \
  graph_construction/graph_construction/livox_grid_builder.py
```

结果:

```text
通过
```

已检查纯算法层导入:

```bash
rg -n "rclpy|nav_msgs|grid_map_msgs|std_msgs|graphnav_msgs|msg_utils|OccupancyGrid|Odometry|GridMap|NavigationGraph|Header" \
  graph_construction/graph_construction/graph_builder.py \
  graph_construction/graph_construction/grid_types.py
```

结果说明:

- 没有 ROS 包或 ROS 消息导入
- 只剩 docstring 中提到 `GridMap` / `NavigationGraph` 的文字说明

## 后续建议

下一步可以继续学习 `graphnav_builder` 的测试边界:

- 为 `grid_types.py` 增加坐标 round-trip 单元测试
- 为 `grid_adapter.py` 增加 GridMap `outer_start_index` / `inner_start_index` 解码测试
- 为 `SparseGraphBuilder` 增加不启动 ROS 的纯算法测试
- 为 `node.py` 单独测试参数筛选和消息转换边界

这次没有引入 message_filters, QoS 策略或 TF 缓冲重构

这些仍应放到后续同步诊断工具和 ROS 适配层改造中处理

