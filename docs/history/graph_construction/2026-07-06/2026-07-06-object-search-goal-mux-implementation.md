# Object Search Goal Mux 实现说明

日期: 2026-07-06

## 背景

目标识别已经能在目标进入视野后发布 `/spot1/object_search_target_pose`

缺少的是目标不在视野时的搜索 goal 来源, 导致 `graphnav_planner` 没有高层目标时只能等待

## 目标

阶段一目标:

```text
do_object_search=true
    -> 未看到目标时发布 initial search goal
    -> planner 根据 scored frontier 进行探索
    -> 看到目标 frontier 后切换到目标 frontier goal
```

本阶段不接入三角定位和 particle hypotheses, 那是后续阶段

## 当前代码链路

新增节点:

```text
visual_navigation/visual_navigation/object_search_goal_mux.py
```

默认配置:

```text
visual_navigation/configs/object_search_goal_mux.yaml
```

启动链路:

```text
WildOS object target frontier
    -> /spot1/object_search_target_pose
    -> object_search_goal_mux
    -> /goal_pose
    -> graphnav_planner
    -> /path2
    -> graphnav_path_follower
    -> /spot1/tracking_goal_pose
```

当 `do_object_search=true` 时, 2D 和 3D launch 都会启动 `object_search_goal_mux`

## 修改内容

- `object_search_goal_mux` 订阅 `/spot1/odom_for_scoring`, 用当前 odom 朝向生成初始搜索 goal
- `object_search_goal_mux` 订阅 `/spot1/object_search_target_pose`, 目标 frontier 出现后优先发布该目标
- 默认初始目标为当前朝向前方 `30.0m`, 可通过 profile 覆盖
- 默认 `target_timeout_sec=3.0`, `latch_target_after_first_detection=false`
- 目标丢失后优先使用最近目标方向进入 `TARGET_MEMORY_GUIDED_SEARCH`
- `memory_timeout_sec=10.0`, `memory_goal_distance=10.0`, 记忆过期后才回到初始搜索 goal
- WildOS 发布 `/spot1/object_search_reached`, 目标 mask 足够大且连续确认后进入 `TARGET_REACHED_VIEWPOINT`
- `target_reached_radius=1.5`, active target frontier 到达后也可以作为兜底进入 `TARGET_REACHED_VIEWPOINT`
- 如需短暂遮挡容忍, 可以手动开启 latch, 并通过 `latch_target_timeout_sec` 限制最长保持时间
- 启动时检查 `output_goal_topic` 和 `object_target_pose_topic` 不能相同, 避免 topic 回环
- 2D / 3D launch 将 mux 的输出接到 profile 中的 `goal_pose_topic`
- `topic_profiles.yaml` 新增三套 profile 的目标搜索参数
- `object_search_goal_mux` 额外发布 `/spot1/object_search_goal_viz`, 用 MarkerArray 显示高层搜索 goal
- `path_follower_node` 输出改到 `/spot1/tracking_goal_pose`, 不再和 planner 输入 `/goal_pose` 混用

## 参数

profile 参数:

```yaml
object_search_initial_goal_distance: "30.0"
object_search_initial_goal_heading_deg: "0.0"
object_search_target_timeout_sec: "3.0"
object_search_latch_target_after_first_detection: "false"
object_search_latch_target_timeout_sec: "3.0"
object_search_memory_timeout_sec: "10.0"
object_search_memory_goal_distance: "10.0"
object_search_target_reached_radius: "1.5"
object_search_object_reached_timeout_sec: "2.0"
object_search_reached_mask_fraction: "0.01"
object_search_reached_min_pixel_count: "1200"
object_search_reached_confirm_frames: "2"
object_search_goal_viz_topic: /spot1/object_search_goal_viz
object_reached_topic: /spot1/object_search_reached
tracking_goal_pose_topic: /spot1/tracking_goal_pose
```

launch 覆盖示例:

```bash
./scripts/start_wildos_2d.sh do_object_search:=true object_search_initial_goal_distance:=20.0
./scripts/start_wildos_3d.sh do_object_search:=true object_search_initial_goal_heading_deg:=30.0
```

## 验证步骤

首次拉到本改动后需要重新构建, 因为 `object_search_goal_mux` 是新的 console script:

```bash
colcon build --packages-select visual_navigation graph_construction --symlink-install
source install/setup.bash
```

启动:

```bash
./scripts/start_wildos_2d.sh do_object_search:=true
```

检查目标缺失阶段:

```bash
ros2 topic echo --once /goal_pose
ros2 topic info -v /goal_pose
ros2 topic echo --once /spot1/object_search_goal_viz
ros2 topic echo --once /path2
ros2 topic info /spot1/scored_nav_graph --no-daemon
```

期望:

- `object_search_goal_mux` 日志出现 `SEARCHING_WITH_INITIAL_GOAL` 或 `TARGET_MEMORY_GUIDED_SEARCH`
- `/goal_pose` 有当前朝向前方的初始目标
- `/goal_pose` 只有 `object_search_goal_mux` 一个 publisher
- `/spot1/object_search_goal_viz` 在 RViz2 中显示高层搜索目标
- `/path2` 有 graph planner 输出路径

目标进入视野后检查:

```bash
ros2 topic echo --once /spot1/object_search_target_pose
ros2 topic echo --once /goal_pose
```

期望:

- `object_search_goal_mux` 日志切换到 `TARGET_FRONTIER_ACTIVE`
- `/goal_pose` 切换为目标 frontier pose

## 后续建议

- 第二阶段接入 `obj_mask_triangulation`, 让 `triangulated goal > object target frontier > initial search goal`
- 增加 `/spot1/object_search_status`, 输出 searching, detected, localized 等诊断状态
- 评估 `path_follower_node` 和 planner 是否应分离高层 `/goal_pose` 与低层 waypoint topic

## 2026-07-06 启动失败修复

现象:

```text
executable 'object_search_goal_mux' not found on the libexec directory
```

原因:

- 新增 console script 后未重新构建 `visual_navigation`, install 目录中还没有 `object_search_goal_mux`
- `external_references/nebula2-wildos-main_ws` 位于当前 ROS2 workspace 的 `src` 下, 如果不加 `COLCON_IGNORE`, colcon 会扫描到参考仓库里的同名 `visual_navigation`

处理:

- 已新增 `external_references/COLCON_IGNORE`, 外部参考仓库不参与当前 workspace 构建
- 已调整 `.gitignore`, 保留 `external_references/COLCON_IGNORE`
- 已在 2D / 3D 启动脚本中增加 `do_object_search=true` 预检, 缺少 mux 时会先输出中文构建提示
- 已重新构建 `visual_navigation` 和 `graph_construction`, install 中已生成 `object_search_goal_mux`

## 2026-07-06 参数类型修复

现象:

```text
rclpy.exceptions.InvalidParameterTypeException:
Trying to set parameter 'initial_goal_distance' to '30.0' of type 'STRING', expecting type 'DOUBLE'
```

原因:

- `topic_profiles.yaml` 中 profile 值以字符串形式保存
- launch 直接把字符串传给 `object_search_goal_mux`
- rclpy 在 `declare_parameter("initial_goal_distance", 30.0)` 阶段已经按 double 校验, 节点内部的字符串解析逻辑还没有机会执行

处理:

- 2D / 3D launch 对 `initial_goal_distance`, `initial_goal_heading_deg`, `target_timeout_sec`, `latch_target_timeout_sec`, `memory_timeout_sec`, `memory_goal_distance`, `target_reached_radius`, `object_reached_timeout_sec`, `reached_mask_fraction` 使用 `_float_value`
- 2D / 3D launch 对 `reached_min_pixel_count`, `reached_confirm_frames` 传入 WildOS, WildOS 内部转换为整数
- 2D / 3D launch 对 `latch_target_after_first_detection` 使用 `_bool_value`
- 保留 profile 中的字符串形式, 方便 launch 参数覆盖和 YAML profile 统一管理

## 2026-07-06 Goal Topic 拆分和 RViz 可视化

现象:

```text
ros2 topic info -v /goal_pose
Publisher count: 2
object_search_goal_mux
graphnav_path_follower
```

原因:

- `object_search_goal_mux` 把高层搜索 goal 发布到 `/goal_pose`
- `path_follower_node` 也通过 launch remap 把低层跟踪 waypoint 发布到 `/goal_pose`
- RViz2 中 `/goal_pose` 是 `PoseStamped`, 不是 MarkerArray, 不会出现在已有 graph marker 图层中

处理:

- `/goal_pose` 只保留为 planner 高层输入, publisher 应只有 `object_search_goal_mux`
- `path_follower_node` 输出改为 `/spot1/tracking_goal_pose`
- 新增 `/spot1/object_search_goal_viz`, 类型为 `visualization_msgs/MarkerArray`, 用于 RViz2 直接显示搜索目标
- RViz2 中查看高层搜索目标优先添加 `/spot1/object_search_goal_viz`
