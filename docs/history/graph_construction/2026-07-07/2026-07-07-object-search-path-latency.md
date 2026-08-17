# 2026-07-07 object search path latency

## 背景

Unity 2D 目标搜索中, 低层控制已改为消费 `/corrected_path`, 但运行时出现右相机画面中能看到远处 `blue bucket`, WildOS 仍持续打印 `当前帧未检测到目标`, 导致目标导航点和路径发布不及时

## 目标

- 让 Unity 远距离小目标更早进入 object search mask
- 提高 object search goal 触发 planner 重规划的频率
- 增加可解释日志, 区分视觉未检出, 目标点未发布和路径未发布
- 约束目标搜索路径继续落在 `NavigationGraph` 的可通行安全节点上

## 当前代码链路

```text
三相机图像 + odom + nav_graph
    -> WildOS localize_query
    -> /spot1/object_search_target_pose
    -> object_search_goal_mux
    -> /goal_pose
    -> graphnav_planner
    -> profile.path_topic, Unity 当前为 /corrected_path
```

如果 WildOS 没有生成目标 mask, `/spot1/object_search_target_pose` 不会更新, `object_search_goal_mux` 只能继续发布初始探索 goal 或最近目标 frontier 记忆 goal

## 问题分析

`localize_query` 使用硬阈值 `mask_threshold` 将文本相似度图转成二值 mask, 远处目标面积小且相似度偏低时, 可能肉眼已经能看到目标, 但所有像素仍低于阈值

当前 Unity 默认阈值为 `0.10`, 目标识别更保守, 用于减少远距离误检

`object_search_goal_mux` 原默认 `publish_rate=1.0Hz`, 即使已收到目标点, planner 的 `/goal_pose` 触发频率也偏低, `/multi_planned_path` 会等下一次 goal 或 graph 更新

上一版 `TARGET_MEMORY_GUIDED_SEARCH` 只记忆目标方向向量, 机器人移动后仍沿旧方向延伸 `memory_goal_distance`, 容易变成贴墙或越过安全节点范围的自由空间 goal

`graphnav_planner` 原实现会把虚拟 goal 和 frontier 的平均未知边界点追加到 `nav_msgs/Path`, 这些点不一定是安全可通行 graph node, 低层如果直接跟随路径, 可能在墙边或 unknown 边界上走

## 修改内容

- Unity profile 将 `object_search_mask_threshold` 调整为 `0.10`
- Isaac 和 robot profile 继续保留 `object_search_mask_threshold=0.09`
- 三套 profile 新增 `object_search_goal_publish_rate=5.0`
- 两个完整 launch 入口透传 `object_search_mask_threshold`, `object_search_detection_debug_interval`, `object_search_goal_publish_rate`
- `object_search_goal_mux` 默认发布频率调整为 `5.0Hz`
- WildOS 未检测到目标时输出每路相机最高相似度和近阈值像素数量
- WildOS 对 launch dotlist 传入的 `mask_threshold`, `obj_frontier_score`, `obj_trav_score` 显式转成 `float`
- `object_search_goal_mux` 的目标记忆改为复用最近一次目标 frontier pose, 不再把旧方向外推成自由空间 goal
- `graphnav_planner` 新增 `append_virtual_goal_to_path` 和 `append_frontier_point_to_path`, 2D/3D launch 默认均为 `false`
- `graphnav_planner` 默认执行路径只包含真实 graph node, 避免路径末端落到未知边界点或任意几何 goal

## 验证步骤

启动 Unity 2D 目标搜索:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true
```

检查目标搜索输入是否真的形成目标点:

```bash
ros2 topic echo --once /spot1/object_search_target_pose
```

检查 goal mux 是否以 5Hz 左右触发规划:

```bash
ros2 topic hz /goal_pose
```

检查 planner 输出路径:

```bash
ros2 topic hz /multi_planned_path
ros2 topic echo --once /multi_planned_path
```

检查 `/multi_planned_path` 的末端点应落在绿色 graph node 或其安全边附近, 不应再延伸到紫色 frontier point 或橙色虚拟 goal 线上

如果仍然未检测到目标, 优先看 WildOS 日志中的:

```text
threshold=0.100, 相机最高相似度=front/...,left/...,right/..., 近阈值像素=...
```

右相机最高相似度持续接近但低于 `0.10` 时, 可临时使用:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true object_search_mask_threshold:=0.09
```

如果启动时报 `Unknown format code 'f' for object of type 'str'`, 说明运行的 install tree 还没有包含类型转换修复, 需要重新构建 `visual_navigation`
