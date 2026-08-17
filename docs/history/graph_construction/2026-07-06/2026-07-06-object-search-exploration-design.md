# 目标不在视野时的搜索探索设计

日期: 2026-07-06

## 背景

当前目标识别已经验证通过, `WildOS` 可以在目标进入相机视野时生成 object mask, 增强 visual frontier scoring, 并发布目标相关可视化

下一步要解决的是更完整的 object search:

```text
目标不在当前视野
    -> 机器人需要探索
    -> 目标进入视野后识别
    -> 切换到目标引导
    -> 继续规划并接近目标
```

这不是单纯的检测问题, 而是目标缺失状态下的搜索策略, 目标出现后的目标估计, 以及 planner goal 管理问题

## 参考资料

- WildOS 论文: https://arxiv.org/abs/2602.19308
- WildOS 项目页: https://leggedrobotics.github.io/wildos/
- 社区实现仓库: https://github.com/TIKTOKDAD/nebula2-wildos-main_ws
- 社区 `initial_goal_mux_conf.yaml`: https://github.com/TIKTOKDAD/nebula2-wildos-main_ws/blob/main/visual_navigation/configs/initial_goal_mux_conf.yaml
- 社区 `triangulation3d_objsearch_conf.yaml`: https://github.com/TIKTOKDAD/nebula2-wildos-main_ws/blob/main/visual_navigation/configs/triangulation3d_objsearch_conf.yaml
- 社区 `obj_mask_triangulation.py`: https://github.com/TIKTOKDAD/nebula2-wildos-main_ws/blob/main/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py
- 社区 `graphnav_planner`: https://github.com/TIKTOKDAD/nebula2-wildos-main_ws/tree/main/graphnav_planner

## 论文思路摘要

WildOS 的 object search 不是直接把检测框当导航点

论文中的完整链路可以拆成五层:

```text
几何局部地图
    -> sparse navigation graph
    -> geometric frontier nodes
    -> ExploRFM 预测 visual traversability, visual frontier, object similarity
    -> 将 geometric frontier 投影到图像中评分
    -> 生成 scored navigation graph
    -> graph planner 选择 frontier 或 coarse goal path
```

关键点:

- 目标不在视野时, 系统仍然需要用 visual frontier 和 geometric frontier 做安全探索
- 图像语义只给方向偏好, 不替代几何可达性
- 目标进入视野后, object similarity 会增强目标相关区域的评分
- 目标距离超出深度可靠范围时, 论文使用 particle-filter-based coarse localization 估计目标位置
- planner 最终仍然在 scored graph 上做高层路径选择, 而不是直接追逐图像像素

## 当前本地能力

### 已具备

`visual_navigation/wildos/nav.py` 已经具备:

- 三相机同步输入
- ExploRFM inference
- 目标 query mask
- object mask 发布
- geometric frontier 投影
- `frontier_scores` 写入 `/spot1/scored_nav_graph`
- `/spot1/object_search_target_pose`
- `/spot1/object_search_target_viz`
- `/spot1/score_rings`

`graphnav_planner` 已经具备:

- 订阅 scored graph
- 订阅 `~/goal_pose`
- 把 goal 转到 graph frame
- 计算每个 frontier 到 goal 的方向 bin
- 用对应 `frontier_scores[best_bin]` 调整 frontier cost
- 在没有直接到达 goal 的 graph edge 时, 通过 frontier edge 继续探索
- 用 `latest_frontier`, `local_frontier_radius`, `path_smoothness_period` 减少频繁跳目标

这说明目标不在视野时, 本地缺的不是 planner 核心算法, 而是稳定的 search goal 输入和目标状态切换

### 当前缺口

当前缺口:

- 没有一个节点负责在目标未出现时发布初始搜索 goal
- 没有一个节点负责在目标出现后从初始 goal 切换到真实目标或目标 frontier
- `obj_mask_triangulation` 已存在, 但还未接入 2D / 3D 启动链路和 topic profile
- `/spot1/object_search_target_pose` 当前是目标语义最高分 frontier, 不是三角定位物体坐标
- 目标搜索状态没有明确诊断, 例如 searching, detected, localized, reached

## 社区实现可借鉴点

### initial goal mux

社区实现中的 `initial_goal_mux` 用于目标缺失阶段:

```text
初始粗目标
    -> 输出给 planner
三角定位目标首次有效
    -> 永久切换到真实目标
```

可借鉴点:

- 在目标未定位前, 先给 planner 一个远处粗 goal
- 等 output topic 有订阅者后再发布一次初始 goal, 避免一次性消息丢失
- 第一条有效三角定位目标到来后永久切换到真实目标
- 拒绝 NaN / infinite 目标
- 检查输入 topic 和输出 topic 不能相同, 避免消息回环

本项目应实现同类节点, 但命名和 topic 应适配现有 profile

建议节点名:

```text
object_search_goal_mux
```

### triangulation3d object search

社区实现和本地已有 `obj_mask_triangulation.py` 的思路一致:

```text
ObjectMaskWithTf + LiDAR PointCloud2
    -> 将点云投影到相机
    -> 取落在 object mask 内的 LiDAR 点
    -> 点数足够时用 mask 内点的距离中位数估计目标
    -> 点数不足时累计多视角 bbox / mask ray particles
    -> triangulated goal pose
    -> particle hypotheses visualization
```

可借鉴参数:

```yaml
max_views: 350
min_lidar_points: 150
min_view_distance: 1.0
particle_generator_config:
  num_particles: 1000
  depth_range: [1.0, 100.0]
```

本项目已有:

```text
visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py
triangulation3d/
visual_navigation/configs/triangulation3d_objsearch_conf.yaml
```

因此二阶段不是搬运代码, 而是接入启动, profile, topic, frame 和可视化

### graphnav_planner

社区 planner 的核心行为和本地一致:

- graph 中插入 virtual goal
- 如果 goal 附近已有 graph node, 连接 goal
- 对 frontier node, 查询 unexplored space distance
- 根据 `frontier_scores` 中朝向 goal 的 bin 修改 frontier cost
- 如果一段时间内已有局部 frontier, 优先保持附近 frontier, 减少抖动

这部分当前不需要大改, 需要先保证 goal 输入合理

## 推荐实现路线

### 阶段一: 目标不在视野时能探索

目标:

```text
do_object_search=true
    -> 没看到目标
    -> 自动发布 initial search goal
    -> planner 基于 scored frontier 探索
    -> 目标进入视野后切换到 object target frontier
```

新增节点:

```text
visual_navigation/object_search_goal_mux.py
```

输入:

```text
/spot1/object_search_target_pose
/spot1/odom_for_scoring
```

输出:

```text
/goal_pose
```

配置:

```yaml
object_search_goal_mux:
  output_goal_topic: /goal_pose
  object_target_pose_topic: /spot1/object_search_target_pose
  odom_topic: /spot1/odom_for_scoring
  initial_goal_mode: heading
  initial_goal_distance: 30.0
  initial_goal_heading_deg: 0.0
  frame_id: map
  publish_rate: 1.0
  target_timeout_sec: 3.0
  latch_target_after_first_detection: false
  latch_target_timeout_sec: 3.0
  memory_timeout_sec: 10.0
  memory_goal_distance: 10.0
  target_reached_radius: 1.5
  object_reached_timeout_sec: 2.0
  reached_mask_fraction: 0.01
  reached_min_pixel_count: 1200
  reached_confirm_frames: 2
```

状态机:

```text
WAIT_FOR_SUBSCRIBER
    -> SEARCHING_WITH_INITIAL_GOAL
    -> TARGET_FRONTIER_ACTIVE
    -> TARGET_MEMORY_GUIDED_SEARCH
    -> TARGET_REACHED_VIEWPOINT
    -> TARGET_LOST_RECOVERY
```

行为:

- 等待 `/goal_pose` 有订阅者后发布初始 goal
- 初始 goal 可以按当前 odom 朝向加 heading 生成, 默认前方 30 m
- 如果收到 `/spot1/object_search_target_pose`, 切换到目标 frontier goal
- 如果目标短暂丢失, 保持最近一次目标 frontier 一段时间, 不立即退回初始 goal
- 如果目标长时间丢失, 回到 searching, 继续用当前 heading 或最后目标方向探索

这样能先形成闭环, 不依赖三角定位

### 阶段二: 接入三角定位目标

目标:

```text
object mask + lidar
    -> /spot1/triangulated_object
    -> /spot1/imgnav_waypoint
    -> object_search_goal_mux
    -> /goal_pose
```

需要做:

- 把 `obj_mask_triangulation` 加入 2D / 3D launch 可选启动
- 将 `triangulation3d_objsearch_conf.yaml` 改为 profile override, 支持 unity, isaac, robot
- 输出 topic 统一进 `topic_profiles.yaml`
- `object_search_goal_mux` 的真实目标输入改为优先 `/spot1/imgnav_waypoint`
- `/spot1/object_search_target_pose` 继续作为目标 frontier 可视化和 fallback
- RViz 显示 `/spot1/object_hypotheses` 和 `/spot1/triangulated_object`

建议优先级:

```text
triangulated goal > object target frontier > initial search goal
```

### 阶段三: 搜索状态诊断和安全策略

新增诊断 topic:

```text
/spot1/object_search_status
```

建议字段可以先用 `diagnostic_msgs/DiagnosticArray`, 避免新增 msg:

```text
state
query
initial_goal_active
target_frontier_active
triangulated_goal_active
last_detection_age
last_goal_frame
last_goal_distance
planner_path_available
```

安全策略:

- 只给 planner 发布 goal, 不直接发 cmd_vel
- planner path 为空时不更新 goal 到更远点
- object target frontier 必须来自当前 connected component 或 planner 可达路径
- 三角定位目标如果落在 obstacle / unknown 内, 不直接作为最终控制目标, 只作为 graph planner 的 virtual goal
- 目标切换要有 timeout 和 hysteresis, 避免目标闪烁导致路径抖动

## 具体文件改动建议

阶段一:

```text
visual_navigation/visual_navigation/object_search_goal_mux.py
visual_navigation/configs/object_search_goal_mux.yaml
visual_navigation/setup.py
visual_navigation/package.xml
graph_construction/configs/topic_profiles.yaml
graph_construction/launch/wildos_2d_sim.launch.py
graph_construction/launch/elevation_visual_navigation_sim.launch.py
scripts/start_wildos_2d.sh
scripts/start_wildos_3d.sh
```

阶段二:

```text
visual_navigation/configs/triangulation3d_objsearch_conf.yaml
visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py
graph_construction/configs/topic_profiles.yaml
graph_construction/launch/wildos_2d_sim.launch.py
graph_construction/launch/elevation_visual_navigation_sim.launch.py
```

可选增强:

```text
graphnav_planner/src/planner_node.cpp
graphnav_planner/src/planner.cpp
```

只有当 planner 对 unreachable target frontier 的处理不够稳定时, 再改 planner

## 验证计划

### 阶段一验证

启动:

```bash
bash scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true
```

目标不在视野时检查:

```bash
ros2 topic echo --once /goal_pose
ros2 topic echo --once /path2
ros2 topic info /spot1/scored_nav_graph --no-daemon
ros2 topic info /spot1/score_rings --no-daemon
```

期望:

- `/goal_pose` 有 initial search goal
- `/path2` 有路径
- `/spot1/score_rings` 有方向评分
- RViz 中机器人沿 scored frontier 探索, 不是原地等待

目标进入视野后检查:

```bash
ros2 topic echo --once /spot1/object_search_target_pose
ros2 topic echo --once /goal_pose
ros2 topic echo --once /spot1/scored_nav_graph
```

期望:

- `/spot1/object_search_target_pose` 发布目标 frontier
- `/goal_pose` 切换到目标 frontier 或保持最近目标方向
- `scored_nav_graph` 中目标节点带 `object_target_score`

### 阶段二验证

检查:

```bash
ros2 topic info /spot1/object_mask --no-daemon
ros2 topic info /spot1/object_hypotheses --no-daemon
ros2 topic info /spot1/triangulated_object --no-daemon
ros2 topic info /spot1/imgnav_waypoint --no-daemon
```

期望:

- 目标进入视野后有 particle hypotheses
- LiDAR 点落入 mask 时发布 triangulated object
- `object_search_goal_mux` 优先使用 `/spot1/imgnav_waypoint`

## 风险和注意事项

- 初始 goal 不是目标位置, 只是让 graph planner 产生搜索方向
- 如果初始 goal 太远, planner 仍然只会通过 frontier 逐步接近, 这是预期行为
- 如果 frontier_scores 全部接近 0, planner 会倾向几何距离, 需要检查 ExploRFM 输出和 score ring
- 三角定位依赖点云落入 mask, 目标远于 LiDAR 稠密范围时会退化为粒子假设
- Unity, Isaac 和真实狗的 frame / camera TF 不同, 所有目标位姿必须走 TF 转换
- 不要让 `real_goal_topic` 和 `output_goal_topic` 相同, 否则会形成回环

## 结论

最小可用路线是先补 `object_search_goal_mux`

它能在目标不在视野时给 `graphnav_planner` 一个初始搜索方向, 让已有 scored frontier 机制真正开始探索, 目标进入视野后再切换到目标 frontier

三角定位和 particle hypotheses 是第二阶段, 负责把目标 frontier 升级为更接近论文的 coarse object goal

## 2026-07-06 实现状态

阶段一已经实现:

- 新增 `visual_navigation/visual_navigation/object_search_goal_mux.py`
- 新增 `visual_navigation/configs/object_search_goal_mux.yaml`
- `do_object_search=true` 时, 2D / 3D launch 自动启动 `object_search_goal_mux`
- profile 中新增 `object_search_initial_goal_distance`, `object_search_initial_goal_heading_deg`, `object_search_target_timeout_sec`, `object_search_latch_target_after_first_detection`, `object_search_latch_target_timeout_sec`, `object_search_memory_timeout_sec`, `object_search_memory_goal_distance`, `object_search_target_reached_radius`, `object_search_object_reached_timeout_sec`, `object_search_reached_mask_fraction`, `object_search_reached_min_pixel_count`, `object_search_reached_confirm_frames`
- 目标未出现时发布 initial search goal 到 `/goal_pose`
- `/spot1/object_search_target_pose` 出现后切换为目标 frontier goal

详细实现见 `2026-07-06-object-search-goal-mux-implementation.md`
