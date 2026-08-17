# 2026-07-09 Isolate Unity Graphnav Goal Topic

## 目标

临时隔离 Unity 2D 的高层 goal topic, 避免本仓库 object search 发布的目标被外部 `nav_slam/astar` 当作 `/goal_pose` 消费并影响 `/multi_planned_path`

## 问题

外部 `nav_slam 2dpoints.launch.py` 中的 `astar` 会订阅公共 `/goal_pose`

本仓库 Unity 2D 链路此前也把 `object_search_goal_mux.output_goal_topic` 和 `graphnav_planner ~/goal_pose` remap 到 `/goal_pose`

当两个规划链路同时存在时, 公共 `/goal_pose` 会触发外部 A* 重新规划, 进而干扰当前调试的 `/multi_planned_path`

## 修改

- Unity profile 的 `goal_pose_topic` 从 `/goal_pose` 改为 `/spot1/graphnav_goal_pose`
- Isaac 和 robot profile 仍保留 `/goal_pose`
- `wildos_2d_sim.launch.py` 不需要改代码, 它已经用 `goal_pose_topic` 同时配置 object search 输出和 graphnav planner 输入
- `object_search_goal_mux` 的独立默认参数和 YAML 默认值也改为 `/spot1/graphnav_goal_pose`, 避免绕过 launch 时误发公共 `/goal_pose`
- 更新 `AGENT_README.md`, 明确 Unity 默认高层 goal topic 不再是公共 `/goal_pose`

## 验证

- Unity profile 解析结果应为 `goal_pose_topic=/spot1/graphnav_goal_pose`
- Isaac profile 解析结果应仍为 `goal_pose_topic=/goal_pose`
- Robot profile 解析结果应仍为 `goal_pose_topic=/goal_pose`
- `visual_navigation.object_search_goal_mux` 默认参数应为 `/spot1/graphnav_goal_pose`
- 重启 Unity 2D launch 后, `/goal_pose` 不应再有本仓库 object search publisher
