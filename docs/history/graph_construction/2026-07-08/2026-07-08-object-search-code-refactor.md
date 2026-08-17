# 2026-07-08 Object search code refactor

## 背景

目标搜索当前存在路径跳变和原地转向问题

前两次分析结论是, 路线一致性不能依赖每帧视觉结果直接刷新 goal, 需要稳定坐标系, 持久 graph memory, frontier 锁定和目标记忆共同约束

## 目标

- 把目标搜索代码按职责拆开
- 保持现有 ROS topic 和 launch 参数兼容
- 让 frontier 选择逻辑脱离 ROS Node, 后续可以单独测试
- 明确 `object_search_target_pose` 当前仍是目标相关 frontier, 不是精确物体坐标
- 为后续接入 triangulation 或 TargetHypothesis 留出入口

## 当前代码链路

```text
visual_navigation/wildos/nav.py
    -> 视觉模型推理, frontier scoring, scored nav graph
    -> /spot1/scored_nav_graph
    -> /spot1/object_search_target_pose

visual_navigation/object_search_goal_mux.py
    -> 订阅 scored graph, object target frontier, odom, reached
    -> ObjectSearchState 状态机
    -> StableFrontierSelector
    -> /goal_pose
    -> /spot1/object_search_selected_frontier
    -> /spot1/object_search_status

graphnav_planner
    -> 按 /goal_pose 在 graph 上规划
```

## 问题分析

原来的 `object_search_goal_mux.py` 同时承担:

- ROS 参数和 topic
- 目标搜索状态机
- 目标记忆
- graph frontier 遍历
- frontier 切换滞回
- deadend blacklist
- goal 可视化

这些职责混在一起后, 后续很难判断路径跳变是目标状态问题, frontier 选择问题, 还是 planner 问题

另外, `object_search_target_pose` 的语义容易误解, 它当前来自最高分目标相关 frontier node, 并不是真实物体坐标

## 修改内容

新增 `visual_navigation/visual_navigation/object_search_types.py`:

- `ObjectSearchState`, 集中定义目标搜索状态名
- `TargetHypothesis`, 表达目标证据快照

新增 `visual_navigation/visual_navigation/stable_frontier_selector.py`:

- `StableFrontierSelectorConfig`, 集中保存 frontier 选择参数
- `FrontierCandidate`, 表达 scored graph 中可选 frontier
- `StableFrontierSelector`, 只负责从 scored graph 中选择稳定 frontier

重构 `visual_navigation/visual_navigation/object_search_goal_mux.py`:

- mux 保留 ROS Node 职责和状态机职责
- graph frontier 遍历和切换保护下沉到 `StableFrontierSelector`
- `selected_frontier` 变为 selector 的只读属性
- `blacklisted_frontier_count` 从 selector 读取
- 收到 `object_search_target_pose` 时构造 `TargetHypothesis`
- 状态名改为 `ObjectSearchState` 常量

新增 `visual_navigation/test/test_stable_frontier_selector.py`:

- dwell 时间内不切换 frontier
- dwell 后新候选明显更优才切换
- frontier UUID 重建但位置接近时继承当前 frontier

## 验证步骤

基础静态检查:

```bash
python3 -m py_compile \
  visual_navigation/visual_navigation/object_search_goal_mux.py \
  visual_navigation/visual_navigation/stable_frontier_selector.py \
  visual_navigation/visual_navigation/object_search_types.py
```

ROS 环境下运行单测:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
pytest visual_navigation/test/test_stable_frontier_selector.py
```

运行时检查:

```bash
ros2 topic echo --once /spot1/object_search_status
ros2 topic echo --once /spot1/object_search_selected_frontier
ros2 topic echo --once /goal_pose
ros2 topic echo --once /multi_planned_path
```

预期:

- 未看到目标时, `state=GEOMETRIC_EXPLORE`
- `/goal_pose` 不随机器人原地转向频繁跳变
- selected frontier 在 dwell 时间内保持稳定
- 目标短时丢失时, 状态进入 `TARGET_MEMORY_GUIDED_FRONTIER` 或 `TARGET_MEMORY_GUIDED_SEARCH`
