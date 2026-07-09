# 2026-07-08 Stable object search frontier selector

## 背景

目标未进入视野时, 当前系统容易出现路径终点快速跳变, 机器狗大部分时间在原地转向

根因不是单帧目标识别, 而是目标搜索层给 planner 的 goal 不够稳定

## 实现目标

- 目标搜索层默认从 `/spot1/scored_nav_graph` 中选择稳定 frontier
- 未看到目标时进入 `GEOMETRIC_EXPLORE`, 不再持续追随机器人 yaw 生成远处 goal
- 看到目标时进入 `VISION_GUIDED_FRONTIER`, 目标 pose 只作为 frontier heading score 的方向参考
- 目标短时丢失时进入 `TARGET_MEMORY_GUIDED_FRONTIER`, 使用最近目标记忆继续选择 graph frontier
- 当前 frontier 未失败前至少保持一段时间, 避免 path follower 反复原地转向
- planner 可执行路径停在 graph node 上, 不再无条件追加 unknown 边界上的 frontier_points 均值

## 代码链路

```text
/spot1/scored_nav_graph
    -> object_search_goal_mux
    -> selected frontier state
    -> /goal_pose
    -> graphnav_planner
    -> /spot1/graphnav_planner/path
```

目标相关输入:

```text
/spot1/object_search_target_pose
/spot1/object_search_reached
/spot1/odom_for_scoring
```

新增或明确的诊断输出:

```text
/spot1/object_search_selected_frontier
/spot1/object_search_status
/spot1/object_search_goal_viz
```

## Selector 行为

`object_search_goal_mux` 现在维护一个 `selected_frontier`

每次 timer:

```text
读取 scored nav graph frontier nodes
    -> 读取 node.properties.frontier_scores
    -> 按目标方向取 heading bin, 没有目标时取最高分 bin
    -> 综合 score, distance, switch penalty 计算 utility
    -> 当前 frontier 未失败时优先保持
    -> 满足切换条件后才换到更优 frontier
```

切换保护:

- `frontier_min_dwell_sec`, 当前 frontier 最小保持时间
- `frontier_switch_min_score_margin`, 新 frontier 需要明显更优
- `frontier_progress_timeout_sec`, 长时间没有接近则认为失败
- `frontier_progress_min_delta`, 进展距离阈值
- `frontier_reached_radius`, 到达当前 frontier 后重选
- `frontier_same_position_radius`, frontier UUID 重建但位置接近时继承为同一个 selected frontier
- `deadend_blacklist_timeout_sec`, 失败或已到达 frontier 短时屏蔽

## 默认参数

```yaml
object_search_enable_graph_frontier_selection: "true"
object_search_frontier_min_dwell_sec: "8.0"
object_search_frontier_switch_min_score_margin: "0.15"
object_search_frontier_progress_timeout_sec: "12.0"
object_search_frontier_progress_min_delta: "0.25"
object_search_frontier_reached_radius: "1.5"
object_search_frontier_same_position_radius: "1.2"
object_search_deadend_blacklist_timeout_sec: "20.0"
object_search_frontier_score_weight: "1.0"
object_search_frontier_distance_weight: "0.06"
object_search_frontier_switch_penalty: "0.2"
```

这些默认值已写入 `topic_profiles.yaml`, 适用于 `isaac`, `unity`, `robot`

## Planner 执行路径约束

`graphnav_planner` 仍使用 `frontier_points` 做 frontier 语义和评分

但生成可执行 path 时, 不再把 `frontier_points` 均值追加到路径末端

当 `/goal_pose` 附近已经存在 graph node 时, planner 只连接这些 direct goal nodes 到 virtual goal

只有远处 initial goal 无法直接连接 graph node 时, planner 才会把其他 frontier edges 接到 virtual goal 做探索

原因:

- `frontier_points` 位于 known 和 unknown 边界, 不一定是安全可执行点
- 均值可能落到 unknown, obstacle 后方或墙边
- direct selected frontier goal 如果仍连接所有 frontier, Dijkstra 可能绕到别的 frontier, 造成路径跳变
- path follower 应优先走 graph node 和 graph edge, 这样更符合死路回退和 graph memory 设计

## 启动配置

2D 和 3D launch 都会把以下参数传入 `object_search_goal_mux`:

```text
nav_graph_topic = scored_nav_graph_topic
selected_frontier_topic = object_search_selected_frontier_topic
status_topic = object_search_status_topic
enable_graph_frontier_selection
frontier_min_dwell_sec
frontier_switch_min_score_margin
frontier_progress_timeout_sec
frontier_progress_min_delta
frontier_reached_radius
frontier_same_position_radius
deadend_blacklist_timeout_sec
frontier_score_weight
frontier_distance_weight
frontier_switch_penalty
```

## 预期效果

目标未出现时:

- `/spot1/object_search_status` 应显示 `state=GEOMETRIC_EXPLORE`
- `/spot1/object_search_selected_frontier` 应持续发布同一个 frontier 附近的 pose
- `/goal_pose` 不应随机器人 yaw 每帧旋转
- `/multi_planned_path` 不应在左右两侧高频跳变
- 如果 graph construction 重建 frontier UUID, selector 应通过空间近邻继续保持同一片边界

目标出现时:

- 状态应切换为 `VISION_GUIDED_FRONTIER`
- selected frontier 可以切换, 但需要满足 dwell 和 score margin
- 目标短时丢失后应进入 `TARGET_MEMORY_GUIDED_FRONTIER`, 不应立即回到远处 initial goal

frontier 失败时:

- 当前 frontier 长时间没有接近会短时 blacklist
- 状态日志会打印 `FRONTIER_NO_PROGRESS`
- selector 会从剩余 frontier 中重选

## 验证命令

```bash
ros2 topic echo --once /spot1/object_search_status
ros2 topic echo --once /spot1/object_search_selected_frontier
ros2 topic echo --once /goal_pose
ros2 topic echo --once /multi_planned_path
```

RViz2 建议显示:

- `/spot1/object_search_goal_viz`
- `/spot1/object_search_selected_frontier`
- `/spot1/score_rings`
- `/multi_planned_path`

## 注意事项

- 如果 `/spot1/scored_nav_graph` 暂不可用, `object_search_goal_mux` 才会回退到 `SEARCHING_WITH_INITIAL_GOAL`
- `object_search_target_pose` 仍是目标语义相关 frontier, 不是精确物体坐标
- 精确目标 coarse localization 仍应通过后续 triangulation 接入
- 如果路径仍穿墙, 下一步应检查 graph edge collision 和 grid obstacle inflation, 而不是继续放宽 frontier 切换
