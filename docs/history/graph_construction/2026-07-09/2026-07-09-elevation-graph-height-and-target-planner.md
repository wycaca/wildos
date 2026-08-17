# 2026-07-09 elevation graph height and target planner

## 背景

Unity elevation/2.5D 链路中, RViz 里绿色 graph node 和 edge 看起来落在紫色高程图下方

目标搜索日志中 `目标导航点候选已更新` 高频打印, 影响定位真正的状态切换

目标已被视觉模型看到时, planner 仍可能选择不够直接的 frontier 路线, 甚至到达附近后不能及时停止

## 目标

- graph/path waypoint 不贴在 elevation surface 上
- graph marker 在 RViz 中稳定显示在 GridMap 上方
- 目标候选日志只在候选切换或固定周期打印
- 目标到达和目标连边不受 elevation z 偏差影响
- 目标明确时, planner 更优先接近 goal, 仍然只走 graph 中安全 edge

## 当前代码链路

elevation GridMap 进入 `graph_construction.grid_adapter.classify_grid_map`

`ClassifiedGrid.elevation_at_index` 返回 `elevation + grid_map_z_offset`

`SparseGraphBuilder` 采样 free cell 后生成 node, `EdgeBuilder` 只在 `edge_radius` 内候选, 并沿 line 检查 unknown, obstacle 和 clearance

`GraphVisualizer` 发布 RViz marker, 原先直接使用 node z

`WildOS_Nav.select_object_target_candidate` 每帧检测到候选都会打印日志

`graphnav_planner.Planner` 通过虚拟 goal 连接 goal 半径内 node 或 frontier node, 然后跑 Dijkstra

## 问题分析

绿色点低于高程图有两种含义:

- RViz 显示层面, graph marker 没有抬高, 容易被 GridMap surface 深度遮挡
- planner 数据层面, 目标半径用 3D 距离时会被 elevation z 偏差影响

修正后不再抬高 graph 数据层 z, 只抬高 RViz marker, 避免高程过滤和 current node 判断受影响

点和边的连线不是全连接, 规则是:

- 两个 node 距离小于 `edge_radius`
- 两点间 grid line 不穿过 obstacle 或 unknown
- line 上 clearance 大于 `min_obstacle_clearance`
- 每个 node 只保留最近的 `max_edge_neighbors` 条边

目标存在时路线不一定是几何最短直线, 因为 planner 仍在 graph edge 上求最短路, 并且 frontier score 会参与虚拟 goal 连接代价

`frontier_score_factor=20.0` 当前保留为回退后的默认值, 后续不要在未确认核心因素前继续调权重

## 修改内容

- `graph_construction/configs/graph_construction_elevation.yaml`
  - `grid_map_z_offset` 保持 `0.08`, 只用于 graph/path 数据
  - `prune_disconnected_nodes` 默认关闭, 避免 current node 落入小分量时剪掉大部分图
- `graph_construction/graph_construction/node.py`
  - 默认 `grid_map_z_offset` 保持 `0.08`
  - 默认 `prune_disconnected_nodes` 关闭
- `graph_construction/graph_construction/graph_builder.py`
  - `prune_disconnected_nodes` dataclass 默认关闭
- `graph_construction/graph_construction/viz.py`
  - graph node, edge, frontier point, trajectory 和 radius marker 额外抬高 `0.25m`
- `visual_navigation/visual_navigation/wildos/nav.py`
  - 新增 `object_search_config.target_log_period_sec`
  - 目标候选日志按候选签名和周期节流
- `graph_construction/configs/topic_profiles.yaml`
  - 三个 profile 增加 `object_search_target_log_period_sec: "2.0"`
- `graph_construction/launch/wildos_2d_sim.launch.py`
  - 接入 `object_search_target_log_period_sec`
  - planner 权重已回退为 `goal_dist_cost_factor=1.0`, `frontier_score_factor=20.0`
- `graph_construction/launch/elevation_visual_navigation_sim.launch.py`
  - 接入 `object_search_target_log_period_sec`
  - planner 权重已回退为 `goal_dist_cost_factor=1.0`, `frontier_score_factor=20.0`
- `graphnav_planner`
  - 默认 planner 权重已回退为 `goal_dist_cost_factor=1.0`, `frontier_score_factor=10.0`
  - goal 半径, goal 连边, path 末端停止判断已回退为 3D 距离
  - 已回退目标直连不可达时的额外 fallback 和最近 graph node 兜底
  - 已回退 `PlannerNode` 空 path 复用逻辑

## 验证步骤

```bash
python3 -m py_compile graph_construction/graph_construction/viz.py graph_construction/graph_construction/node.py visual_navigation/visual_navigation/wildos/nav.py
```

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
PYTHONPATH=$PWD/graph_construction:$PWD/visual_navigation:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest visual_navigation/test/test_stable_frontier_selector.py graph_construction/test/test_graph_builder_diagnostics.py
```

```bash
git diff --check
```
