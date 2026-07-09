# 2026-07-08 Object search principle and stability plan

## 背景

当前目标识别已经可以在目标进入视野后输出 object mask, object target pose, score rings 和目标可视化

现场新问题是目标未进入视野时, 机器人探索效率很低:

```text
未看到目标
    -> goal 或 path 终点快速变化
    -> graph planner 左右切换 frontier
    -> path follower 主要在原地转向
    -> 实际位移很小, 探索效率低
```

截图中的红色 odom 箭头呈放射状, 橙色目标线和红色路径频繁换向, 这更像 goal selection contract 不稳定, 不是单纯模型识别问题

## 参考来源

- WildOS paper: https://arxiv.org/abs/2602.19308
- WildOS project page: https://leggedrobotics.github.io/wildos/
- Community reference: `external_references/nebula2-wildos-main_ws`
- Community planner: `external_references/nebula2-wildos-main_ws/graphnav_planner`
- Community visual scoring: `external_references/nebula2-wildos-main_ws/visual_navigation/visual_navigation/wildos`
- Community triangulation: `external_references/nebula2-wildos-main_ws/visual_navigation/visual_navigation/explorfm_triangulation`

## WildOS 原理论证

WildOS 的目标搜索不是每帧把视觉检测方向直接变成控制目标

论文和项目页描述的是五层闭环:

```text
local geometry
    -> sparse navigation graph
    -> geometric frontier nodes
    -> ExploRFM visual traversability, visual frontiers, object similarity
    -> scored navigation graph
    -> hierarchical planner executes stable intermediate goals
```

关键原则:

- sparse navigation graph 是空间记忆, 用来保存已探索区域和待探索 frontier
- ExploRFM 只负责给 frontier node 增加语义偏好, 不替代几何可达性
- geometric frontier 会投影到相机图像中, 用 visual frontier, traversability 和 object similarity 评分
- 目标在深度可靠范围外时, 原论文使用 particle filter 或 coarse localization 估计目标位置
- planner 最终在 scored graph 上规划, 执行的是图上的中间目标, 不是每帧漂移的图像像素或机器人 yaw 方向

因此, 目标搜索需要稳定的 graph frontier 选择, 而不是高频重发一个随当前朝向变化的远处 goal

## 社区参考代码结论

### Visual scoring

社区 `wildos/nav.py` 的核心流程:

```text
navgraph + odom + camera images
    -> extract_geofrontiers
    -> model.forward
    -> object mask enhances image frontiers and traversability
    -> score_geofrontiers
    -> write frontier_scores into scored_nav_graph
    -> publish score_rings
```

可学习点:

- object mask 是用来增强 frontier score, 不是直接替代 planner goal
- 未被当前相机评分的 frontier 仍会获得 default heading scores
- 评分结果写入 node property `frontier_scores`, planner 按 goal 方向选择对应 heading bin
- removed frontier 会记录位置, 新 frontier 靠近已移除位置时分数清零, 用来减少重复回头

### Planner smoothing

社区 `graphnav_planner` 有两个稳定机制:

```text
latest_frontier
local_frontier_radius
path_smoothness_period
```

planner 插入 virtual goal 后, 会遍历 frontier node:

- 根据 frontier_points 到 unexplored space 的距离计算几何代价
- 根据 node 到 goal 的方向取 `frontier_scores[best_bin]`
- 如果当前还在 `path_smoothness_period` 内, 优先保留 latest_frontier 附近的 local frontier
- 只有超过平滑周期, 或当前 frontier 不可用, 才允许更大范围切换

这说明原始实现并不希望每次图更新都全局重选 frontier

### Initial goal mux

社区 `initial_goal_mux.py` 的行为很保守:

```text
等待 planner 订阅
    -> 只发布一次 initial coarse goal
收到第一条有效 triangulated goal
    -> 永久切换到真实目标
```

可学习点:

- 初始粗目标不是循环追踪目标, 只是启动 graph planner 探索的方向提示
- 初始粗目标不应跟随机器人 yaw 每帧重算
- 收到有效目标后不应因为短时丢帧立即回退到初始搜索
- 输入输出 topic 必须隔离, 避免 goal 回环

### Triangulation and target memory

社区 `obj_mask_triangulation.py` 的对象定位链路:

```text
ObjectMaskWithTf + PointCloud2
    -> project lidar points into camera
    -> if enough points fall in mask, use lidar based target estimate
    -> otherwise accumulate multi-view ray particles
    -> publish triangulated object marker, navigation goal, particle hypotheses
```

这部分对应论文中的 coarse goal localization, 作用是把短时视觉检测变成更稳定的目标假设

## 当前自研链路问题

当前本仓库已经具备:

- `visual_navigation/wildos/nav.py` 可以增强 frontier score, 发布 object target pose 和 scored navgraph
- `object_search_goal_mux.py` 可以在 initial goal, target frontier, memory target 之间切换
- `graphnav_planner` 已有 latest frontier smoothing 和 path switch hysteresis

但现场仍然转圈, 说明问题集中在 frontier 和 path 的选择契约:

```text
每帧输入变化
    -> target frontier 或 initial goal 改变
    -> planner 看到新的 virtual goal
    -> best frontier 重新排序
    -> /multi_planned_path 终点跳变
    -> path follower 优先转向新路径
```

具体风险:

- initial goal 即使被短时缓存, 超时后仍可能按当前 yaw 重建, 造成新一轮旋转
- object target pose 当前是视觉评分最高 frontier, 不是真实物体坐标, 所以它会随相机视角和 frontier 列表变化
- 多个 frontier 分数接近时, 少量评分噪声就能让目标从左侧切到右侧
- path hysteresis 只比较 path endpoint 和 goal delta, 不能理解 frontier uuid 是否已经被锁定
- 若 graph edge 或 frontier point 穿过 unknown 或低矮障碍, 即使路径稳定也可能执行到不安全区域

## 稳定搜索状态机

后续实现应把目标搜索拆成明确状态机:

```text
GEOMETRIC_EXPLORE
    -> no object evidence, use stable selected frontier
VISION_GUIDED_FRONTIER
    -> object weakly visible, latch best scored reachable frontier
TARGET_MEMORY_GUIDED
    -> object lost, continue last target hypothesis or last target frontier
TARGET_APPROACH
    -> target evidence strong and nearby viewpoint reachable
REACHED
    -> reachable viewpoint reached and object confidence held for several frames
FAILED_OR_REPLAN
    -> selected frontier unreachable, no progress, or blocked
```

状态切换原则:

- 没看到目标时, 不发布随 yaw 变化的 far goal, 而是选择一个稳定 frontier
- 看到目标时, 不立即用当前帧覆盖已选目标, 先进入 latch 窗口
- 目标短时丢失时, 使用 target memory, 不马上回到 initial search
- 到达判定不是物体坐标可达, 而是到达一个可通行 viewpoint 且目标连续可见

## Frontier 选择策略

后续应新增或扩展一个 stable frontier selector, 输入 scored navgraph, odom, object target evidence, 输出单一稳定 goal 或 selected frontier id

推荐综合评分:

```text
utility =
    visual_score_weight * visual_score
    - path_cost_weight * graph_path_cost
    - heading_change_weight * heading_change
    - switch_penalty_weight * recent_switch_penalty
    - deadend_penalty_weight * deadend_penalty
```

必要约束:

- frontier 必须在 current component 可达, 或 planner 能给出非空 graph path
- frontier 所在 edge 必须通过当前 grid collision check
- 新 frontier 分数必须明显优于当前 frontier 才允许切换
- 当前 frontier 未失败时, 至少保持 `frontier_min_dwell_sec`
- 如果机器人已有有效位移进展, 延长当前 frontier 的保持时间
- 如果连续 `frontier_progress_timeout_sec` 没有接近, 标记该 frontier 短时不可选

推荐参数:

```yaml
frontier_min_dwell_sec: 8.0
frontier_switch_min_score_margin: 0.15
frontier_switch_min_path_delta: 3.0
frontier_progress_timeout_sec: 12.0
frontier_reached_radius: 1.5
target_memory_timeout_sec: 20.0
deadend_blacklist_timeout_sec: 20.0
heading_change_cost: 0.2
```

## Planner contract

目标搜索层和 planner 的接口需要收紧:

- planner 不应该直接消费高频变化的视觉候选点
- planner 应消费 stable selected frontier 或 stable goal pose
- `/multi_planned_path` 应以 graph path 为准, 终点应落在安全 node 或安全 viewpoint
- frontier_points 可以作为探索方向参考, 不应无条件追加成可执行终点
- path follower 执行中的 path 不能被同等质量的新 path 高频覆盖

建议输出:

```text
/spot1/object_search_selected_frontier
/spot1/object_search_selected_goal
/spot1/object_search_status
/multi_planned_path
```

其中 selected frontier/status 用于诊断和 RViz, `/multi_planned_path` 继续作为实际执行路径

## 诊断字段

为了定位继续转圈的问题, 后续日志和可视化应包含:

```text
state
selected_frontier_uuid
selected_frontier_score
selected_frontier_path_cost
selected_frontier_age
switch_reason
path_endpoint_delta
goal_delta
robot_progress_to_selected_frontier
object_detected
object_memory_age
blacklisted_frontier_count
```

RViz 中至少显示:

- 当前 selected frontier
- latched target frontier
- blacklisted deadend frontier
- 当前执行 path endpoint
- 每次切换的 reason

## 不应继续做的事情

- 不应每个 timer 都按当前 yaw 重新生成远处 initial goal
- 不应让每帧最高分 frontier 直接覆盖当前执行目标
- 不应把 object target pose 理解成真实物体坐标, 它当前只是目标相关 frontier
- 不应把 unknown 或障碍后的 frontier point 直接追加为可执行路径终点
- 不应只靠 path endpoint hysteresis 解决所有跳变, 因为它不知道 frontier 语义和状态

## 实施方案

### Step 1, 增加选择诊断

先不改变行为, 补充 selected frontier, switch reason, path endpoint delta 和 progress 日志, 确认路径跳变来自 goal, frontier score, graph 更新还是 follower 侧

### Step 2, 引入 stable frontier selector

在 `object_search_goal_mux` 内扩展, 或新增 `object_search_frontier_selector`

输入:

```text
/spot1/scored_nav_graph
/spot1/odom_for_scoring
/spot1/object_search_target_pose
/spot1/object_search_reached
```

输出:

```text
/goal_pose
/spot1/object_search_selected_frontier
/spot1/object_search_status
```

核心行为:

- no target 时从 scored navgraph 中选择稳定 frontier
- target visible 时 latch 目标相关 frontier
- target lost 时保持 memory target 或 memory frontier
- failed 时短时 blacklist 当前 frontier, 再换下一个候选

### Step 3, 收紧 planner 可执行路径

planner 只执行 graph reachable path

如果 path 最后一段是 frontier_points 均值, 需要检查该点是否仍在 safe free cell 中, 否则终点回退到 frontier node 或最近安全 viewpoint

### Step 4, 接入 triangulation

当 object mask 稳定后, 再接入 `obj_mask_triangulation.py`

目标:

- 近距离或有 lidar mask 点时, 发布 triangulated object
- 远距离时, 累计多视角 particle hypotheses
- selected frontier 使用 target hypothesis 作为长期方向, 不再被单帧检测完全覆盖

### Step 5, 到达判定

目标完成不应要求走到物体坐标

推荐条件:

```text
robot near selected reachable viewpoint
    and object visible for N frames
    and mask fraction or score above threshold
```

如果物体附近不可达, 则停在最近可达 viewpoint, 状态进入 `REACHED_VIEWPOINT_OBJECT_VISIBLE`

## 当前截图对应判断

截图里的路径和 odom 箭头说明, 机器人尚未形成一个持续执行的 selected frontier

后续优先解决:

1. 让目标搜索层只输出一个稳定 frontier 或 goal
2. 让 planner 在该 frontier 未失败前保持图路径
3. 让 path follower 有足够时间执行位移, 而不是不断响应新方向

这条路线更接近 WildOS 的 graph memory 设计, 也更适合后续 dead end recovery
