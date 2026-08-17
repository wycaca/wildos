# 2026-07-09 robot anchor current node

## 背景

目标已经找到时, 如果机器人脚下点云缺失, 当前 grid 会把脚下 cell 判成 unknown

旧逻辑中, `SparseGraphBuilder._nearest_collision_free_node` 要求机器人位置到候选节点的直线全程 known free

脚下 cell unknown 会让所有直线检查失败, 后续只能 geometry fallback 到最近节点, 但 graph 中没有从机器人当前位置接入近邻点的边, planner 可能从断开的 current node 开始而输出空路径

## 修改

- `InternalNode` 增加 `is_robot_anchor`
- current node 找不到 collision-free 节点时, `SparseGraphBuilder` 创建或更新 robot anchor node
- robot anchor 不参与 frontier assignment, 不会被当成目标搜索 frontier
- robot anchor 只连接 `robot_anchor_edge_radius` 内, 当前可见段没有 obstacle 且满足 obstacle clearance 的近邻节点
- unknown 不会否决 anchor 边, 但可见 obstacle 会否决 anchor 边

## 参数

```yaml
ensure_robot_anchor_node: true
robot_anchor_edge_radius: 0.0
robot_anchor_max_edges: 6
```

`robot_anchor_edge_radius=0.0` 表示复用 `edge_radius`

## 行为

这不是把脚下 unknown 全部改成 free

它只是在 current node 无法安全直连时, 给 planner 一个临时起点锚点, 并按当前可见障碍做保守连边

如果机器人脚下被当前 grid 明确判定为 obstacle, 不会创建 anchor

## 验证

```bash
python3 -m py_compile graph_construction/graph_construction/graph_memory.py graph_construction/graph_construction/frontier_detector.py graph_construction/graph_construction/edge_builder.py graph_construction/graph_construction/graph_builder.py graph_construction/graph_construction/node.py
PYTHONPATH=$PWD/graph_construction PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest graph_construction/test/test_graph_builder_diagnostics.py graph_construction/test/test_grid_map_adapter.py
```
