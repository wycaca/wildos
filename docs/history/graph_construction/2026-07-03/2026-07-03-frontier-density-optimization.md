# Frontier 密度优化记录

日期: 2026-07-03

## 背景

运行日志显示 3D graph construction 更新较慢:

```text
节点数=165, 边数=389, frontier节点=53, frontier栅格=1245
node.total=551.7ms
builder.update_frontiers=264.4ms
builder.distance_fields=94.6ms
builder.build_edges=98.4ms
```

这说明紫色 `frontier_points` 不只是 RViz 显示偏密, 已经让 `update_frontiers` 成为最大瓶颈

## 问题原因

旧逻辑会对每个 frontier cell 执行:

1. `grid_to_world`
2. 全图最近节点查找
3. explored radius 检查
4. `is_world_collision_free` 线段检测

当原始 frontier cell 达到 1245 个时, 大量相邻边界点会重复找同一个 owner node, 并重复做非常接近的 line collision check

## 修改内容

新增 `frontier_candidate_spacing` 参数:

```yaml
frontier_candidate_spacing: 0.5
```

含义:

- `0.0`: 关闭候选降采样, 保持旧行为
- `> grid.resolution`: 按该米制间距从密集 frontier cells 中选代表候选
- 原始 `frontier栅格` 仍保留在诊断日志中, 便于判断地图边界密度
- 新增 `frontier候选` 诊断字段, 表示实际进入 owner 分配和 collision check 的候选数量

3D elevation 默认启用:

```yaml
graph_construction_elevation.yaml:
  frontier_candidate_spacing: 0.5
```

2D baseline 也需要启用, 因为 Livox OccupancyGrid 分辨率为 `0.2m`, 未降采样时会出现 `frontier候选=frontier栅格`:

```yaml
graph_construction.yaml:
  frontier_candidate_spacing: 0.8
```

同时 `FrontierDetector` 在 owner 查找前建立临时节点空间索引, 避免每个 frontier 候选都扫描全部 graph node

## 预期效果

原始日志中的 `frontier栅格=1245` 在 0.5m 候选间距下, 应显著减少实际分配候选数

优化后的慢帧日志会类似:

```text
frontier节点=..., frontier栅格=1245, frontier候选=...
builder.update_frontiers=...
```

重点看:

- `frontier候选` 是否明显低于 `frontier栅格`
- `builder.update_frontiers` 是否从 264ms 明显下降
- `frontier节点` 是否仍保留合理数量, 不应直接降为 0
- WildOS `model_visualization` 中仍能看到可投影 frontier node

## 后续调参

如果 `frontier节点` 仍明显偏多:

- 增大 `frontier_min_span`, 例如 `0.8` 或 `1.0`
- 增大 `frontier_min_points`, 过滤短小噪声边界
- 适度增大 `frontier_border_margin`, 避免局部地图边缘被当成 frontier

如果 frontier 被过滤过多:

- 降低 `frontier_candidate_spacing`, 例如 `0.3`
- 降低 `frontier_min_points`
- 检查 GridMap / OccupancyGrid 中 unknown 区域是否本身过碎

## 2D 追加观察

2D 慢帧日志显示:

```text
frontier节点=56, frontier栅格=1306, frontier候选=1306
builder.update_frontiers=141.8ms
node.total=401.0ms
```

这说明 2D 仍按旧配置处理全部边界 cell

`graph_construction.yaml` 已改为:

```yaml
frontier_candidate_spacing: 0.8
```

在 `resolution=0.2m` 的 2D grid 上, 这相当于约每 4 个 cell 选一个代表候选

重启 2D 后应重点看:

```text
frontier栅格=...
frontier候选=...
builder.update_frontiers=...
```

如果 `frontier节点` 仍在 50 个以上, 再考虑把 `frontier_min_span` 提高到 `0.8` 或 `1.0`

## 验证命令

```bash
PYTHONPATH=graph_construction .venv/bin/python -m pytest -q graph_construction/test/test_graph_builder_diagnostics.py
python3 -m py_compile graph_construction/graph_construction/frontier_detector.py graph_construction/graph_construction/graph_builder.py graph_construction/graph_construction/node.py
```

运行 3D 后继续看慢帧日志:

```text
builder.update_frontiers
frontier栅格
frontier候选
frontier节点
```
