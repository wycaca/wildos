# Agent README

目的: 帮助开发人员和 agent 快速理解 WildOS 当前项目目标, 运行架构, 模块职责和文件夹结构

## 项目目标

本仓库当前目标是把 WildOS 的开词汇目标搜索系统接入 Isaac, Unity 和真实机器人候选环境

系统核心能力:

- 从局部几何地图生成稀疏 `NavigationGraph`
- 用 graph memory 保留历史节点, frontier, edge 和机器人当前位置
- 用三相机视觉模型给 graph frontier 做语义评分
- 在没有目标检测时选择稳定的前向探索 frontier
- 在看到目标后选择安全的目标导航点, 并通过 graph planner 输出路径
- 在目标到达或视觉近距离确认后锁存停止状态, 避免继续规划旧路径

系统不负责:

- 训练 ExploRFM 或 RADIO backbone
- 直接发布底盘 `cmd_vel`
- 替代局部控制器或 Nav2
- 把语义目标点当成精确物体坐标

## 当前主链路

默认主线是 elevation/2.5D GridMap 后端, 这和论文及社区参考实现一致:

```text
PointCloud2
  -> pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> /elevation_mapping_node/elevation_map_raw
  -> graph_construction, GridMap path
  -> /spot1/nav_graph
  -> visual_navigation WildOS scoring
  -> /spot1/scored_nav_graph
  -> object_search_goal_mux
  -> graphnav_planner
  -> path output
```

2D OccupancyGrid 只是 fallback, 用于隔离 `livox_grid_builder`, 对照 `/combined_grid`, 或在 elevation 后端不可用时调试:

```text
PointCloud2 or aligned PointCloud2
  -> livox_grid_builder
  -> /spot1/traversability_grid
  -> graph_construction, OccupancyGrid path
  -> /spot1/nav_graph
```

视觉链路只影响 semantic scoring 和 object search goal selection, 不直接修改 elevation GridMap 或 2D fallback grid

## 当前启动入口

当前仓库里的唯一脚本入口是:

```bash
./scripts/start_wildos_elevation.sh
```

常用启动方式:

```bash
./scripts/start_wildos_elevation.sh
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh do_object_search:=true
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_elevation.sh do_object_search:=true
```

脚本会处理:

- source `/opt/ros/humble/setup.bash`, 可用 `ROS_SETUP` 覆盖
- source workspace `install/setup.bash`, 可用 `INSTALL_SETUP` 覆盖
- 自动激活 `.venv` 或 `wildos_venv`, 可用 `VENV_ACTIVATE` 覆盖
- 设置 `PYTHONNOUSERSITE=1`
- 将仓库根目录加入 `PYTHONPATH`
- 检查 `elevation_mapping_cupy`
- 修正部分已安装 Python entrypoint 的 shebang
- 调用 `graph_construction/launch/elevation_visual_navigation_sim.launch.py`

2D fallback 仍保留 launch 文件, 但当前没有对应启动脚本:

```bash
ros2 launch graph_construction wildos_2d_sim.launch.py topic_profile:=unity
```

不要引用旧的 `scripts/start_wildos_2d.sh` 或 `scripts/start_wildos_3d.sh`, 当前 source tree 中不存在这两个脚本

## Topic Profile

平台差异集中在:

```text
graph_construction/configs/topic_profiles.yaml
```

内置 profile:

- `isaac`, Isaac Sim 5.1 Go2, 默认 `ROS_DOMAIN_ID=3`, raw LiDAR `/unitree_go2/lidar/points`, path `/path2`, goal `/goal_pose`
- `unity`, Unity 仿真, 默认 `ROS_DOMAIN_ID=89`, RMW `rmw_zenoh_cpp`, elevation 输入 `/livox/lidar`, 2D fallback 输入 `/mapokk`, path `/multi_planned_path`, goal `/spot1/graphnav_goal_pose`
- `robot`, 真实机器人占位 profile, 需要按现场 topic 和 TF 更新

Unity 的重要约束:

- elevation 主线使用 raw `/livox/lidar`
- 2D fallback 使用已经对齐到 `odom_3D` 的 `/mapokk`
- Unity 2D fallback 必须保持 `lidar_assume_input_in_grid_frame=true`
- 不要把 raw `/livox/lidar` 直接替换成 2D fallback 输入, 否则会重新引入地图随机器人朝向旋转的问题
- Unity 高层 goal topic 使用 `/spot1/graphnav_goal_pose`, 避免触发外部 `nav_slam/astar` 的公共 `/goal_pose`

## Graph Construction 架构

`graph_construction` 的职责是把局部几何地图变成稳定稀疏 graph:

```text
node.py
  ROS 参数, topic, QoS, timer, TF, 发布和诊断

grid_adapter.py
  OccupancyGrid / GridMap 解码为 ClassifiedGrid

grid_types.py
  ClassifiedGrid, 坐标转换, elevation 查询, collision line, distance field

graph_builder.py
  SparseGraphBuilder, 纯算法入口, 不直接依赖 ROS

graph_memory.py
  GraphState, InternalNode, InternalEdge, UUID 和历史记忆

frontier_detector.py
  free / unknown 边界检测和 frontier owner 分配

edge_builder.py
  当前边生成, historical edge validation, robot anchor 连边

msg_utils.py
  GraphState 转 graphnav_msgs/NavigationGraph

viz.py
  GraphState 和 ClassifiedGrid 转 MarkerArray

livox_grid_builder.py
  2D fallback 点云转 OccupancyGrid

pointcloud_axis_adapter.py
  elevation 主线点云轴向和 frame 适配

grid_map_to_occupancy.py
  GridMap debug projection, 不在主链路中使用
```

关键实现原则:

- 纯算法层不导入 `rclpy` 或 ROS message package
- ROS message 解码集中在 `grid_adapter.py`
- ROS message 生成集中在 `msg_utils.py`
- topic, frame, 参数和发布集中在 `node.py`
- GridMap rolling buffer, row / column 映射和 frame 约定必须用测试或运行数据确认

## Graph 行为

当前 graph 默认行为:

- elevation path 直接消费 `/elevation_mapping_node/elevation_map_raw`
- 小型 elevation NaN 洞会在 graph adapter 内为 free cell 补 elevation, 不修改原始 GridMap topic
- 历史 edge 默认保留, 只用当前可见障碍证伪
- `prune_disconnected_nodes` 默认关闭, 避免 current node 短时误判时清空大部分 graph
- 脚下点云缺失时启用 robot anchor current node, 让 planner 能从机器人当前位置接回近邻 graph
- graph 数据 z 保持贴近 elevation surface, RViz marker 额外抬高显示
- graph edge 要求 line 和 clearance 都安全, 避免路径贴墙或穿障碍

## Object Search 架构

目标搜索链路:

```text
front / left / right camera
  -> visual_navigation.wildos.nav
  -> /spot1/scored_nav_graph
  -> /spot1/object_search_target_pose
  -> visual_navigation.object_search_goal_mux
  -> graphnav_planner goal topic
```

当前策略:

- 没有目标时, `StableFrontierSelector` 从 scored graph 选择稳定 frontier
- 无目标搜索默认优先 odom 前方 frontier, 前方没有候选时才回退到任意 frontier
- graph frontier UUID 抖动时, selector 用位置半径继承近邻 frontier
- 目标出现时, 优先发布目标方向上的安全 graph frontier
- Unity 已启用 target latch, 支持短时遮挡后继续朝目标方向规划
- 目标到达或近距离视觉确认后, `object_search_goal_mux` 进入 reached latch 并持续发布当前位置 hold goal
- `graphnav_planner` 到达 goal 半径内时发布当前位置单点 path, 让下游停止

## Planner 架构

`graphnav_planner` 是 C++ graph planner:

- 输入 `graphnav_msgs/NavigationGraph`
- 输入 goal pose topic, 不同 profile 可不同
- 使用 graph edge 做搜索, virtual goal 只参与搜索
- 默认不把 virtual goal 或 unknown frontier point 追加进可执行 path
- 输出 profile 配置的 path topic
- `path_follower_node` 输出 `/spot1/tracking_goal_pose`

## 文件夹结构

```text
graph_construction/
  configs/        graph, grid, elevation, topic profile 配置
  launch/         elevation 主线和 2D fallback launch
  graph_construction/
                  geometry map 到 navigation graph 的 Python 实现
  test/           graph builder, grid adapter 和诊断测试
  docs/           dated implementation notes 和本说明

visual_navigation/
  configs/        WildOS, object search goal mux 和 baseline 配置
  launch/         WildOS 视觉节点和 baseline launch
  visual_navigation/wildos/
                  ExploRFM 推理, frontier scoring, object target selection
  visual_navigation/object_search_goal_mux.py
                  object search goal 状态机和 planner goal 输出
  visual_navigation/stable_frontier_selector.py
                  稳定 frontier 选择器
  visual_navigation/utils/
                  TF, odom adapter, goal navigator, scoring 和 buffer 工具

graphnav_planner/
  src/            planner node 和 path follower node
  include/        planner C++ 接口
  launch/         planner launch 配置

graphnav_msgs/
  msg/            NavigationGraph, Node, Edge 和 traversability 消息

object_search_msgs/
  msg/            object mask 与 TF 相关消息

explorfm/
  ExploRFM inference model

nvidia_radio/
  RADIO / NACLIP / SigLIP2 backbone 相关代码

explorfm_trainer/
  ExploRFM head 训练代码

triangulation3d/
  旧 3D object triangulation 和可视化工具

external_references/
  本地社区参考代码, 需要保留 COLCON_IGNORE
```

## 主要配置文件

```text
graph_construction/configs/topic_profiles.yaml
  profile 级 topic, frame, RMW, goal, path 和 object search 默认参数

graph_construction/configs/graph_construction_elevation.yaml
  elevation/2.5D GridMap graph construction 默认参数

graph_construction/configs/graph_construction.yaml
  2D OccupancyGrid fallback graph construction 默认参数

graph_construction/configs/elevation_mapping_sim.yaml
  elevation_mapping_cupy 仿真配置

graph_construction/configs/livox_grid_builder.yaml
  2D fallback 点云转 OccupancyGrid 配置

visual_navigation/configs/wildos_nav_sim_conf.yaml
  WildOS 视觉评分仿真配置

visual_navigation/configs/object_search_goal_mux.yaml
  object search goal mux 独立默认配置

graphnav_planner/launch/graphnav_planner.launch.yml
  graph planner 和 path follower 参数
```
