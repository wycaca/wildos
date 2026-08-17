# Reached Distance Gate And Graph Prune

## 问题

目标在约 10m 外时, WildOS 只要视觉 mask 足够大就可能发布 `object_reached=True`, mux 会进入 `TARGET_REACHED_VIEWPOINT` 并停止继续发目标路线

RViz 中还出现局部孤立点, 以及最左侧旧点和远处非相邻点连边的现象

## 原因

- `object_reached` 之前只检查 mask 面积和连续帧数, 没有要求机器人接近目标 frontier
- `edge_radius=8.0` 对 2D Unity 当前约 1m 的节点间距过大, 会让非相邻点进入边候选
- rolling grid 外或安全走廊检查失败的旧节点可能留在 graph memory 中, 但无法形成当前可验证的安全边

## 修改

- `object_search_goal_mux` 新增 `object_reached_max_target_distance`
- 只有当前目标或目标 latch 距离机器人不超过 2m 时, `object_reached=True` 才能触发 reached latch
- 2D `graph_construction.yaml` 的 `edge_radius` 从 `8.0` 收紧到 `3.0`
- 2D `max_edge_neighbors` 从 `4` 调整到 `6`, 在短边范围内保留更多局部邻接
- graph builder 新增 `prune_disconnected_nodes`, 发布前只保留 current node 所在连通分量
- 当前默认关闭 `prune_disconnected_nodes`, 避免 current node 短时误判时清空大部分图

## 当前到达阈值

- `object_search_target_reached_radius=1.5m`, 主动目标 frontier 到达阈值
- `object_search_object_reached_max_target_distance=2.0m`, 视觉近距离确认触发停止的距离门控
- `graphnav_planner.goal_radius=3.0m`, planner 内部 goal 半径和 hold path 触发半径

## 验证

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
PYTHONPATH=$PWD/graph_construction:$PWD/visual_navigation:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest graph_construction/test/test_graph_builder_diagnostics.py visual_navigation/test/test_stable_frontier_selector.py
```

结果: `7 passed`
