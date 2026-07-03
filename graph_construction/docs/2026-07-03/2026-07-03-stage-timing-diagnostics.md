# Graph Construction 诊断日志和 Stage Timing

日期: 2026-07-03

## 背景

`2026-07-02-external-commit-reference-analysis.md` 建议学习外部 `graphnav_builder` 的诊断方向:

- 首帧统计之外增加周期性 stage timing
- 输出 graph 规模, 连通性, frontier 和 degree 统计
- 慢帧 warning 需要打印具体慢在哪个阶段

本次实现只处理不破坏分层边界的部分:

- ROS node 负责日志周期, 慢帧阈值和 ROS message 转换耗时
- `SparseGraphBuilder` 只返回纯算法诊断数据, 不依赖 `rclpy`
- frontier path check 计数暂不加入, 后续等边验证策略确定后再补

## 代码链路

```text
GraphConstructionNode._on_timer
    -> classify_grid_map / classify_occupancy_grid
    -> SparseGraphBuilder.update
        -> prepare_grid
        -> distance_fields
        -> update_nodes
        -> sample_nodes
        -> update_frontiers
        -> current_node
        -> build_edges
        -> GraphUpdateDiagnostics
    -> graph_to_msg
    -> publish nav_graph
    -> publish visualization markers
    -> _maybe_log_diagnostics
```

## 新增诊断数据

`SparseGraphBuilder.update` 返回的 `GraphUpdateResult.diagnostics` 包含:

- `stage_timings_ms`: builder 内部阶段耗时
- `node_count`: 当前 graph node 数
- `edge_count`: 当前 graph edge 数
- `frontier_node_count`: 关联 frontier 的节点数
- `frontier_cell_count`: 当前地图检测到的 frontier cell 数
- `connected_components`: graph 连通分量数量
- `current_component_size`: `current_node_id` 所在分量大小
- `degree_min`, `degree_avg`, `degree_max`: 节点度数统计
- `current_node_id`: 当前机器人绑定节点
- `current_node_status`: `reachable`, `geometry_fallback` 或 `missing`

`GraphConstructionNode` 额外记录:

- `node.classify_grid`
- `node.builder_update`
- `node.convert_graph_msg`
- `node.publish_graph`
- `node.publish_viz`
- `node.total`

日志会同时输出 GridMap / OccupancyGrid 分类统计, 便于定位 elevation backend 是慢在解码, 分类, 建图, 转消息还是发布

## 配置项

两个默认配置都新增:

```yaml
diagnostics_log_period_sec: 5.0
slow_update_warning_ms: 200.0
```

含义:

- `diagnostics_log_period_sec`: 周期性 info 诊断日志间隔, 小于等于 0 时关闭周期日志
- `slow_update_warning_ms`: 单帧总耗时超过阈值时打印 warning, 小于等于 0 时关闭慢帧 warning

## 日志形态

周期日志前缀:

```text
Graph update diagnostics, nodes=..., edges=..., frontier_nodes=..., frontier_cells=..., components=..., current_component=..., current_node=..., current_node_status=..., degree=min/...,avg/...,max/..., stages=...
```

慢帧日志前缀:

```text
Slow graph update, nodes=..., edges=..., stages=node.classify_grid=..., builder.distance_fields=..., builder.build_edges=..., node.total=...
```

慢帧日志保留完整 stage 列表, 便于直接判断下一步优化对象

## 分层边界

- `graph_builder.py` 中的 timing 使用 `perf_counter`, 只生成 Python dataclass, 不调用 ROS logger
- `node.py` 决定日志级别, 日志周期和慢帧阈值
- `GraphUpdateDiagnostics` 是纯数据结构, 后续可以被单元测试或非 ROS benchmark 复用

## 验证

已执行:

```bash
python3 -m py_compile graph_construction/graph_construction/graph_builder.py graph_construction/graph_construction/node.py
PYTHONPATH=graph_construction .venv/bin/python -m pytest -q graph_construction/test/test_graph_builder_diagnostics.py graph_construction/test/test_grid_map_adapter.py
```

结果:

- 语法检查通过
- graph builder diagnostics 和 GridMap adapter 单测共 5 个通过

## 后续

- 若出现慢帧, 优先看 `node.classify_grid`, `builder.distance_fields`, `builder.sample_nodes`, `builder.build_edges`, `node.publish_viz`
- `test_graph_builder_diagnostics.py` 负责守住 diagnostics 字段和 stage 名称
- 后续加入 frontier path validation 后, 再补 `frontier_path_checks`, `reachable_frontier`, `unreachable_frontier`
- 如果 ROS topic 同步成为问题, 需要另写只观察 topic 的同步诊断工具, 不放进 graph builder
