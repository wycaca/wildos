# 2026-07-08 Self grid rotation and planner guard

## 背景

Unity 2D 切回本仓库自研 `livox_grid_builder` 后, 现场仍观察到两个问题:

- 机器人转向时, 自研 OccupancyGrid 仍跟着机器人旋转
- `planner_node` 在收到异常 graph 后触发 Eigen 矩阵尺寸断言, 进程退出

## 目标

- 明确地图旋转的优先排查方向
- 避免 planner 因上游空图或非法 graph 直接崩溃
- 保持 `/combined_grid` 作为对照测试输入, 默认链路继续使用自研地图

## 当前代码链路

Unity 2D 默认链路:

```text
/mapokk
    -> livox_grid_builder
    -> /spot1/traversability_grid
    -> graph_construction
    -> /spot1/scored_nav_graph
    -> graphnav_planner
```

当前 Unity profile 中:

- `launch_livox_grid_builder: "true"`
- `traversability_grid_topic: /spot1/traversability_grid`
- `global_frame_2d: odom_3D`
- `grid_frame: odom_3D`
- `lidar_topic: /mapokk`
- `lidar_assume_input_in_grid_frame: "true"`
- `grid_odom_topic: /unity/odom`
- `grid_local_width: "14.0"`
- `grid_local_height: "14.0"`
- `grid_min_obstacle_height: "-0.2"`
- `grid_obstacle_inflation_radius: "0.25"`
- `grid_obstacle_detection_mode: height_diff`
- `grid_height_diff_mark_rays_free: "true"`
- `grid_height_diff_fill_unobserved_as_free: "true"`
- `grid_height_diff_unknown_border_width: "1.2"`
- `grid_height_diff_obstacle_threshold: "0.04"`
- `grid_high_obstacle_min_height: "0.12"`

## 问题分析

地图跟着机器狗转向旋转, 通常不是 planner 本身造成的.OccupancyGrid 的 origin 没有 yaw, 如果画面中的障碍随狗转向, 更可能是点云坐标仍被当成本体 frame 处理, 或 `livox_frame -> map` TF 链路不正确

现场验证 `/mapokk` 点云点数约 6k, 是 Unity 侧对齐后的局部点云, 比 `/livox/lidar` 更适合作为 2D graph 输入.自研 builder 之前仍订阅 `/livox/lidar`, 该 raw 点云范围更大且随传感器姿态变化, 因此地图会继续旋转或发黑.当前自研 builder 改为订阅 `/mapokk`, 使用 `assume_input_in_grid_frame=true`, 并统一 2D graph, WildOS, planner 到 `odom_3D` frame

planner 崩溃的直接原因是 `NavigationGraph` 可能短暂为空或 `current_node_idx` 越界.旧 planner 在空图下仍构造 `UnexploredSpaceMap`, 导致 Eigen 收到非法矩阵尺寸并 abort

现场统计显示, 如果 `height_diff` 只把点云命中的 cell 标为 free, 10m x 10m grid 中只有约 1k cell 有观测, 其余约 9k cell 会保持 unknown.这会让局部图大面积发黑, graph construction 无法采样连续 free nodes, planner 也会持续收到空图

旋转问题解决后, 如果没有视觉边界点, 主要原因是 `height_diff_fill_unobserved_as_free=true` 会让局部 grid 内没有 unknown.而 WildOS frontier detector 依赖 free 邻接 unknown, 没有 unknown 就不会产生 frontier node 和视觉边界点

低矮障碍漏检时, 不能只依赖 `max_z > robot_z + 0.3`.桶, 台阶边缘和低矮墙体可能低于该阈值, 导致 graph edge 穿过障碍.当前 Unity profile 降低高度门槛并稍微增加 obstacle inflation

未识别目标时, 如果初始搜索 goal 每次按机器人当前 yaw 重新生成, 机器狗转身会让 goal 跟着旋转, planner 也会在不同 frontier 之间跳变.当前 goal mux 锁定初始搜索 goal, planner 对路径终点大幅跳变增加短时间滞回

## 修改内容

- `graphnav_planner` 收到空导航图时跳过 planner 更新
- `current_node_idx` 越界时跳过 planner 更新
- edge 的 `from_idx` 或 `to_idx` 越界时跳过该 edge
- unexplored map 构建前检查 graph 顶点和边界是否有效
- goal 与 node 重合时使用默认 heading, 避免 NaN
- 路径相邻点重合时保留默认 orientation, 避免零向量归一化
- `AGENT_README.md` 更新 Unity LiDAR 点云输入和 TF 诊断结论
- Unity 自研 builder 输入从 `/livox/lidar` 改为 `/mapokk`
- Unity 2D graph, WildOS 和 planner 统一使用 `odom_3D` 作为稳定参考 frame
- Unity 自研 builder odom 输入改为 `/unity/odom`, 用原始里程计位置稳定滚动局部窗口
- Unity 自研 builder 局部窗口调整为 `14m x 14m`, 给视觉 frontier 和目标搜索更大的局部上下文
- Unity `grid_min_obstacle_height` 保持 `0.0`, 避免把明显低于机身的噪声纳入障碍统计
- `height_diff` 模式新增 `height_diff_mark_rays_free`, Unity 默认开启, 让观测射线经过区域标为 free
- `height_diff` 模式新增 `height_diff_fill_unobserved_as_free`, Unity 默认开启, 让局部窗口有连续 free 区域供 WildOS graph 采样
- `height_diff` 模式新增 `height_diff_unknown_border_width`, Unity 默认 `1.2m`, 在局部窗口内部保留 unknown 边界供 WildOS frontier detector 使用
- Unity `visual_frontiers_range` 调整为 `11.0`, 覆盖放大后的局部地图边界
- Unity `grid_min_obstacle_height` 调整为 `-0.2`, `grid_high_obstacle_min_height` 调整为 `0.12`, `grid_height_diff_obstacle_threshold` 调整为 `0.04`
- Unity `grid_obstacle_inflation_radius` 调整为 `0.25`, 减少 graph edge 贴近或穿过低矮障碍
- `object_search_goal_mux` 新增初始搜索 goal 锁定, Unity 默认 `initial_goal_latch_timeout_sec=20.0`, `initial_goal_reached_radius=3.0`
- `graphnav_planner` 新增 path switch hysteresis, Unity 默认 `path_switch_hysteresis_sec=6.0`, `path_switch_min_endpoint_delta=3.0`, `path_switch_min_goal_delta=5.0`
- Unity 2D planner `path_smoothness_period` 调整为 `15.0`, `local_frontier_radius` 调整为 `5.0`
- planner 无有效导航图时改为 DEBUG 日志, 空图 WARN 只提示一次

## 验证步骤

构建 planner:

```bash
cd /mnt/hhd/han/wildos_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select graphnav_planner --symlink-install
```

启动 Unity 2D:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true
```

重点检查:

```bash
ros2 topic echo --once /spot1/traversability_grid --field header
ros2 topic echo --once /spot1/nav_graph --field nodes
ros2 topic echo --once /spot1/scored_nav_graph --field current_node_idx
ros2 run tf2_ros tf2_echo odom_3D base_link
```

预期:

- `livox_grid_builder` 启动日志中 `assume_input_in_grid_frame=True`
- `livox_grid_builder` 启动日志中 `lidar=/mapokk`
- `livox_grid_builder` 启动日志中 `grid_frame=odom_3D`
- `livox_grid_builder` 启动日志中 `size=14.0x14.0`
- `livox_grid_builder` 启动日志中 `height_rays=True`
- `livox_grid_builder` 启动日志中 `height_fill_free=True`
- `livox_grid_builder` 启动日志中 `height_unknown_border=1.20`
- `livox_grid_builder` 启动日志中 `height_range=(-0.20,1.80)`
- `livox_grid_builder` 启动日志中 `inflation=0.25`
- `livox_grid_builder` 启动日志中 `height_unknown_border=1.20`
- 机器人原地转向时, 障碍物不应围绕机器人一起旋转
- `/spot1/nav_graph` 应持续有 nodes 和 edges, 不应长期为空
- `/spot1/nav_graph` 和 `/spot1/graph_construction_viz` 应恢复 frontier nodes / frontier_points
- 未看到目标搜索时, `/multi_planned_path` 不应在数秒内左右大幅跳变
- object_search_goal_mux 在 `SEARCHING_WITH_INITIAL_GOAL` 状态下不应每次机器人转身都改变远处 goal
- graph 为空或 current node 越界时, planner 只打印中文 WARN, 不再 abort
- graph 为空时不会持续刷 `planner 暂无有效导航图`
- 如果地图仍随狗旋转, 继续检查 Unity 发布的 `/mapokk` 点坐标是否随机器人 yaw 改变, 以及 `/mapokk` 发布端是否仍混入本体局部坐标
