# 阶段三实现方案: Path Execution 与 CmdVel 闭环

日期: 2026-06-25

状态: 暂缓

说明: 经过对论文实现和当前仿真能力的重新确认, 阶段三优先级调整为 3 相机视觉适配, 几何地图默认继续使用 LiDAR baseline, elevation/traversability map 后端作为独立实验验证, Nav2 暂不接, 也暂不实现轻量 cmd_vel controller, 当前文档仅作为后续 Path Execution 方案备选记录

目标: 在阶段一几何图构建和阶段二视觉评分, graph planner 已经跑通的基础上, 增加路径执行层, 让系统从 `/spot1/graphnav_planner/path` 进一步输出可驱动仿真机器狗的 `/unity/cmdvel`

## 设计边界

本阶段不重写阶段一 graph construction, 不重写阶段二 WildOS visual scoring, 不重写 `graphnav_planner` 的 Dijkstra 主逻辑

本阶段只补齐当前缺失的执行闭环:

```text
/livox/lidar
    -> /spot1/traversability_grid
    -> /spot1/nav_graph
    -> /spot1/scored_nav_graph
    -> /spot1/graphnav_planner/path
    -> cmd_vel controller
    -> /unity/cmdvel
```

阶段三优先面向当前 Unity 仿真环境, 不直接引入完整 Nav2 stack

原因:

- 当前仿真已经提供 `/unity/cmdvel` 订阅入口
- 当前已有 `/unity/odom`, `/tf`, `/spot1/traversability_grid`, `/spot1/graphnav_planner/path`
- Nav2 需要全局 costmap, local costmap, controller server, behavior tree, lifecycle 等额外配置, 当前阶段会扩大问题面
- 当前目标是先验证 WildOS graph planner 的路径能否驱动机器狗移动

Nav2 可以作为后续阶段的增强方案, 用于替换轻量 controller 或承接真实机器人底盘控制

## 当前阶段二状态

已完成链路:

```text
livox_grid_builder
    publishes /spot1/traversability_grid

graph_construction
    subscribes /spot1/traversability_grid
    publishes /spot1/nav_graph

visual_navigation/wildos
    subscribes /spot1/nav_graph
    subscribes /camera/color/image/compressed
    publishes /spot1/scored_nav_graph
    publishes /spot1/model_visualization
    publishes /spot1/score_rings
    publishes /spot1/nav_graph_viz

graphnav_planner
    subscribes /spot1/scored_nav_graph
    subscribes /spot1/imgnav_waypoint
    publishes /spot1/graphnav_planner/path

graphnav_path_follower
    subscribes /spot1/graphnav_planner/path
    publishes /spot1/goal_pose
```

实测结论:

- `/spot1/scored_nav_graph` 有输出
- frontier nodes 带有 `frontier_scores`
- `/spot1/model_visualization` 有图像输出
- `/spot1/score_rings` 有 marker 输出
- `/spot1/graphnav_planner/path` 可生成非空 path

当前缺口:

- repo 内没有节点向 `/unity/cmdvel` 发布 `geometry_msgs/Twist`
- `/spot1/goal_pose` 只是局部目标点, 不是底盘速度指令
- path 可以显示, 但机器狗不会自动沿 path 运动

## 当前仿真 topic

阶段三需要适配当前已有 topic:

| Topic | 类型 | 当前作用 | 阶段三用途 |
|---|---|---|---|
| `/unity/odom` | `nav_msgs/Odometry` | Unity 仿真里机器狗位姿和速度 | controller 反馈输入 |
| `/tf` | `tf2_msgs/TFMessage` | `map -> odom_fram -> livox_frame/camera_frame` | 坐标转换 |
| `/spot1/graphnav_planner/path` | `nav_msgs/Path` | graph planner 输出路径 | controller 主输入 |
| `/spot1/goal_pose` | `geometry_msgs/PoseStamped` | 当前 path follower 输出的跟踪目标 | 可作为调试目标 |
| `/spot1/traversability_grid` | `nav_msgs/OccupancyGrid` | Livox 构建的局部可通行栅格 | controller 安全检查 |
| `/unity/cmdvel` | `geometry_msgs/Twist` | Unity 仿真速度指令入口 | controller 输出 |

启动环境仍保持:

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 模块建议

建议在 `graphnav_planner` 中新增执行节点:

```text
graphnav_planner
    src/cmd_vel_follower_node.cpp
    launch/graphnav_execution.launch.yml
```

理由:

- `graphnav_planner` 已经拥有 planner node 和 path follower node
- path execution 和 planner 输出契约关系最紧
- C++ 实现和现有 `path_follower_node.cpp` 保持一致
- 后续可以复用已有 path, odom, tf 参数

保留现有 `path_follower_node`:

- `/spot1/goal_pose` 继续作为 RViz 调试点
- 新增 controller 不依赖 RViz
- 如果 controller 出问题, 仍可以先检查 path follower 的目标点是否合理

## 输入

### 必需输入

```text
/spot1/graphnav_planner/path, nav_msgs/Path
/unity/odom, nav_msgs/Odometry
/tf, tf2_msgs/TFMessage
```

### 建议输入

```text
/spot1/traversability_grid, nav_msgs/OccupancyGrid
```

`traversability_grid` 用于安全停止:

- 当前目标点落在 occupied cell 时停止
- 前方短距离出现 obstacle 时停止
- grid 超时未更新时停止

## 输出

### 必需输出

```text
/unity/cmdvel, geometry_msgs/Twist
```

约定:

```text
linear.x  > 0, 前进
linear.x  = 0, 停止
angular.z > 0, 左转
angular.z < 0, 右转
```

### 建议调试输出

```text
/spot1/cmd_vel_controller/target_pose, geometry_msgs/PoseStamped
/spot1/cmd_vel_controller/status, diagnostic_msgs/DiagnosticArray
/spot1/cmd_vel_controller/viz, visualization_msgs/MarkerArray
```

RViz 显示含义:

| Topic | 显示内容 | 用途 |
|---|---|---|
| `/spot1/cmd_vel_controller/target_pose` | 当前局部跟踪点 | 检查 controller 是否选中了 path 前方点 |
| `/spot1/cmd_vel_controller/viz` | lookahead 点, 停止半径, 当前速度方向 | 检查控制方向 |
| `/unity/cmdvel` | 无直接 RViz 图形 | 用 `ros2 topic echo` 检查速度指令 |

## 控制原理

阶段三先使用纯追踪式轻量 controller, 不引入复杂轨迹优化

### Lookahead 目标选择

从 `/spot1/graphnav_planner/path` 中选择距离机器人当前位置大于 `lookahead_distance` 的第一个 pose:

```text
p_robot = current odom position in map frame
p_i     = path pose i in map frame

target = first p_i where ||p_i - p_robot|| >= lookahead_distance
```

如果所有点都小于 lookahead distance, 使用 path 最后一个点

### 误差计算

将 target 转到机器人当前 heading 下:

```text
dx = target.x - robot.x
dy = target.y - robot.y

rho   = sqrt(dx^2 + dy^2)
alpha = atan2(dy, dx) - yaw_robot
alpha = normalize_angle(alpha)
```

### 速度控制

```text
v = clamp(k_v * rho, 0, max_linear_speed)
w = clamp(k_w * alpha, -max_angular_speed, max_angular_speed)
```

当目标角度过大时, 降低线速度:

```text
if abs(alpha) > slow_turn_angle:
    v = min(v, turn_linear_speed)
```

当接近 path 终点时停止:

```text
if distance_to_final_goal < goal_tolerance:
    v = 0
    w = 0
```

### 安全停止

controller 必须在以下情况发布零速度:

- path 超过 `path_timeout_sec` 未更新
- odom 超过 `odom_timeout_sec` 未更新
- 当前 path 为空
- TF 不可用
- 当前目标点落在 occupied cell
- 前方 `safety_check_distance` 内存在 occupied cell
- 收到手动停止参数或 emergency stop topic

## 参数建议

初始参数建议:

```yaml
cmd_vel_topic: /unity/cmdvel
path_topic: /spot1/graphnav_planner/path
odom_topic: /unity/odom
grid_topic: /spot1/traversability_grid
global_frame: map
robot_frame: odom_fram

control_rate_hz: 10.0
lookahead_distance: 0.8
goal_tolerance: 0.4
path_timeout_sec: 2.0
odom_timeout_sec: 1.0
grid_timeout_sec: 1.0

max_linear_speed: 0.5
max_angular_speed: 0.8
k_v: 0.6
k_w: 1.2
slow_turn_angle: 0.7
turn_linear_speed: 0.15

obstacle_threshold: 65
safety_check_distance: 0.8
publish_zero_on_stop: true
```

速度参数需要从小到大调, 第一版不要追求快速移动

## 启动方案

阶段三建议新增统一启动脚本:

```text
scripts/start_wildos_graphnav_execution.sh
```

内部顺序:

```text
1. livox_grid_builder
2. graph_construction
3. odom_frame_adapter
4. wildos visual scoring
5. graphnav_planner
6. cmd_vel_follower
```

也建议新增 launch:

```text
graph_construction/launch/wildos_graphnav_execution.launch.py
```

或在 `graphnav_planner` 内新增:

```text
graphnav_planner/launch/graphnav_execution.launch.yml
```

最小测试时可分终端启动:

```bash
ros2 launch graph_construction graph_construction_sim.launch.py
ros2 launch visual_navigation wildos_sim_launch.py
ros2 launch graphnav_planner graphnav_planner.launch.yml ns:=spot1 remap_tf_to_ns:=false odom_topic:=odom_for_scoring
ros2 launch graphnav_planner graphnav_execution.launch.yml ns:=spot1 cmd_vel_topic:=/unity/cmdvel odom_topic:=/unity/odom
```

## 测试步骤

### 1. 环境检查

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic list
ros2 topic info /unity/cmdvel
ros2 topic info /unity/odom
```

预期:

- `/unity/cmdvel` 存在订阅者
- `/unity/odom` 有发布者
- `/tf` 有发布者

### 2. 启动阶段一和阶段二

```bash
ros2 launch graph_construction graph_construction_sim.launch.py
ros2 launch visual_navigation wildos_sim_launch.py
ros2 launch graphnav_planner graphnav_planner.launch.yml ns:=spot1 remap_tf_to_ns:=false odom_topic:=odom_for_scoring
```

检查:

```bash
ros2 topic echo /spot1/scored_nav_graph --once --field header
ros2 topic echo /spot1/graphnav_planner/path --once --field header
```

### 3. 发布测试目标

第一版可以手动发 waypoint:

```bash
ros2 topic pub --once /spot1/imgnav_waypoint geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 4.0, y: -6.0, z: 0.0}, orientation: {w: 1.0}}}"
```

更可靠的测试方式是从当前 `/spot1/scored_nav_graph` 中选择同连通分量内的 frontier node 作为目标

### 4. 启动 cmd_vel controller

```bash
ros2 launch graphnav_planner graphnav_execution.launch.yml ns:=spot1 cmd_vel_topic:=/unity/cmdvel odom_topic:=/unity/odom
```

检查:

```bash
ros2 topic echo /unity/cmdvel
ros2 topic echo /spot1/cmd_vel_controller/target_pose --once
```

预期:

- path 非空时 `/unity/cmdvel` 持续输出非零速度
- 接近终点后 `/unity/cmdvel` 输出零速度
- path 超时或 odom 超时时输出零速度

### 5. RViz 检查

RViz 建议显示:

- `/spot1/traversability_grid`
- `/spot1/nav_graph_viz`
- `/spot1/score_rings`
- `/spot1/graphnav_planner/path`
- `/spot1/cmd_vel_controller/target_pose`
- `/spot1/cmd_vel_controller/viz`
- `/unity/odom`

检查点:

- 目标点应该在 path 前方
- 机器狗 odom 应该沿 path 移动
- path 更新后 controller target 应该跟着更新
- grid 中 obstacle 前方 controller 应该停止或减速

## 验收标准

### 功能验收

- `/spot1/graphnav_planner/path` 非空时, controller 能发布 `/unity/cmdvel`
- `/unity/cmdvel` 的 `linear.x` 和 `angular.z` 符合目标点方向
- 机器狗 odom 会随 cmd_vel 改变
- 接近 path 终点后自动停止
- path 丢失或 odom 丢失时自动停止

### 安全验收

- 启动 controller 时默认不暴冲
- 没有 path 时输出零速度
- 没有 odom 时输出零速度
- grid 超时或目标落障碍时输出零速度
- Ctrl-C 停止时最后发布一次零速度

### 可视化验收

- RViz 中能看到 planner path
- RViz 中能看到 controller 当前 target
- RViz 中能看到机器狗 odom 移动
- `/unity/cmdvel` 可用 `ros2 topic echo` 看到

## 风险和后续工作

### 坐标系风险

当前 TF 中存在 `map -> odom_fram -> livox_frame/camera_frame`

controller 必须确认 path 和 odom 都能落到 `map` frame, 否则角度误差会错误

### cmd_vel 语义风险

Unity 的 `/unity/cmdvel` 需要确认是否严格使用 ROS 标准:

```text
linear.x, angular.z
```

如果 Unity 端有自定义坐标系或速度缩放, controller 参数需要重新标定

### path 抖动风险

graph 和 scored graph 会随局部窗口更新, planner path 可能跳变

第一版 controller 需要:

- path timeout
- lookahead smoothing
- 限速
- 限角速度

后续可以增加 path hysteresis 或目标点保持时间

### 局部障碍风险

graph path 由栅格和 frontier 生成, 但机器狗实际运动时可能靠近 obstacle

第一版只做简单 grid 安全检查, 不做完整局部避障

真实机器人上不能直接使用第一版 controller, 必须接入底层安全层或 Nav2 local controller

### 视觉评分风险

当前只有前向相机, 很多 frontier 会使用默认分数

这不影响阶段三验证路径执行, 但会影响自主探索选择质量

## 阶段三结论

阶段三的核心不是继续优化图或视觉模型, 而是补齐从 graph planner path 到仿真速度控制的最后一段

只要系统能在 `/spot1/graphnav_planner/path` 非空时稳定发布 `/unity/cmdvel`, 并让机器狗沿 path 移动且能安全停止, 就可以认为 Path Execution 闭环基础跑通

完成阶段三后, 系统链路将从感知, 建图, 视觉评分, 图规划推进到仿真运动执行, 后续再决定是否接入 Nav2 或真实机器狗控制接口
