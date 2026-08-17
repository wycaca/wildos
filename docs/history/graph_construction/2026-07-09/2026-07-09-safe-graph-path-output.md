# 2026-07-09 Safe Graph Path Output

## 目标

修复 RViz 中路线或 graph edge 穿过障碍物的问题

## 问题

`graphnav_planner` 会创建 virtual goal 参与 Dijkstra 搜索

此前 virtual goal 会被加入发布的 `nav_msgs/Path`, 当 goal 位于障碍后方或未知边界外侧时, 最后一段路径会从安全 graph node 直线连到 virtual goal, RViz 中表现为路线穿过黑色障碍

同时 `graph_construction` 的 edge 校验只检查 Bresenham 中心线是否经过 obstacle 或 unknown, 没有检查机器人需要的 clearance corridor, 斜线切角或贴墙时可能生成看起来穿障碍的 graph edge

## 修改

- `graphnav_planner` 声明并读取 `append_virtual_goal_to_path` 和 `append_frontier_point_to_path`
- `append_virtual_goal_to_path` 默认 `false`, virtual goal 只参与搜索, 不进入可执行 path
- `EdgeBuilder.build_edges` 接入 obstacle 和 unknown 距离场
- graph edge 现在要求中心线每个 cell 到 obstacle 和 unknown 的距离都不小于 `min_obstacle_clearance`
- `ClassifiedGrid` 新增 `world_line_cells`, 复用 world 坐标到 grid cell 的线段转换
- 新增单测覆盖中心线未踩 obstacle, 但 clearance 不足时拒绝建边

## 验证

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest graph_construction/test/test_graph_builder_diagnostics.py` 通过
- `python3 -m compileall -q graph_construction/graph_construction graph_construction/test` 通过
- `colcon build --packages-select graphnav_planner --symlink-install` 通过
- `git diff --check` 通过

## 注意

需要重启 `wildos_2d_sim.launch.py` 才能让运行中的 planner 使用新编译的 `graphnav_planner`

如果 RViz 仍看到红色 graph edge 贴障碍, 优先增大 `min_obstacle_clearance` 或 `grid_obstacle_inflation_radius`
