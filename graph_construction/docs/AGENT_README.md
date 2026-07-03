# Agent README

目的: 给后续 agent 一个优先阅读入口, 快速理解 `graph_construction` 的项目目标, 当前实现, 文档规则和近期上下文

## 先读顺序

优先阅读:

1. `graph_construction/docs/AGENT_README.md`
2. 当天 `graph_construction/docs/YYYY-MM-DD/YYYY-MM-DD-changelog.md`
3. 和当前任务相关的具体说明文档
4. 相关源码

如果运行现象和源码判断冲突, 以实际 ROS graph, install tree, launch 参数和运行中 topic 为准

## 文档规则

每次代码或配置变动都必须同步写文档

文档分两类:

- 变更记录文档: `graph_construction/docs/YYYY-MM-DD/YYYY-MM-DD-changelog.md`
- 具体说明文档: `graph_construction/docs/YYYY-MM-DD/YYYY-MM-DD-topic-name.md`

变更记录文档只写:

- 已完成
- 实测结果
- 诊断结论摘要
- 后续待办

从 2026-07-03 开始, changelog 不再需要单独写 `文档同步` 小节

具体说明文档写:

- 背景
- 目标
- 当前代码链路
- 问题分析
- 修改内容
- 验证步骤
- 修改建议

不要把长篇分析塞进 changelog, 也不要只在具体说明文档里记录已完成变更

如果同一天有多次变更, 优先更新当天同一个 changelog

## 代码注释规则

代码注释使用中文, 保持简洁

注释和文档中的中文句子优先使用英文标点, 不使用中文句号

复杂逻辑函数需要用简短 docstring 或注释说明意图, 不要逐行解释显而易见的赋值

新增代码时, 不要引入英文注释, 除非是外部 API 名称, 参数名或原文错误信息

## 项目目标

当前目标是复现和适配 WildOS 未开源的 Graph Construction 几何记忆层

Graph Construction 的职责:

- 从局部几何地图生成稀疏 `NavigationGraph`
- 维护 free nodes, frontier nodes, edges, current node
- 保持节点 UUID 稳定, 支持视觉评分缓存
- 给 WildOS visual scoring 提供几何 frontier
- 给 `graphnav_planner` 提供可规划的高层图

Graph Construction 不负责:

- 训练 ExploRFM
- 直接生成语义 `frontier_scores`
- 替代 `graphnav_planner`
- 直接控制底盘或发布 cmd_vel

## 当前默认系统链路

默认 LiDAR baseline:

```text
/unitree_go2/lidar/points or /livox/lidar
    -> livox_grid_builder
    -> /spot1/traversability_grid
    -> graph_construction
    -> /spot1/nav_graph
    -> visual_navigation / WildOS
    -> /spot1/scored_nav_graph
    -> graphnav_planner
    -> /spot1/graphnav_planner/path
```

实验 elevation backend:

```text
/unitree_go2/lidar/points
    -> pointcloud_axis_adapter
    -> /unitree_go2/lidar/points_aligned
    -> elevation_mapping_cupy
    -> /elevation_mapping_node/elevation_map_raw
    -> graph_construction GridMap path
    -> /spot1/nav_graph
```

三相机视觉链路只影响 visual scoring, 不应直接改变 `/spot1/traversability_grid`

2D baseline 的 `/spot1/traversability_grid` 是 LiDAR 观测生成的局部 OccupancyGrid, 不是仿真器全局真值地图, 未被射线观测过的区域必须保持 unknown

Isaac LiDAR 单帧可能只包含旋转扫描中的一个扇区, `livox_grid_builder` 默认累计最近扫描帧后再生成局部 grid, 避免 RViz 中只看到随雷达转动的扇形地图

## 当前实现边界

`graph_construction` 目前按以下边界组织:

```text
node.py
    ROS topic, QoS, timer, 参数, first-message 日志, 发布

grid_adapter.py
    OccupancyGrid / GridMap -> ClassifiedGrid

grid_types.py
    ClassifiedGrid, 坐标转换, elevation 查询, collision line, distance field

graph_builder.py
    SparseGraphBuilder, 只接收 ClassifiedGrid, robot_position, stamp_seconds

graph_memory.py
    GraphState, InternalNode, InternalEdge, UUID 和图记忆

frontier_detector.py
    free / unknown 边界检测和 frontier owner 分配

edge_builder.py
    collision-free sparse edges

msg_utils.py
    GraphState -> graphnav_msgs/NavigationGraph

viz.py
    GraphState + ClassifiedGrid -> MarkerArray
```

关键约束:

- 纯算法层不要导入 `rclpy` 或 ROS 消息包
- ROS message 解码放在 `grid_adapter.py`
- ROS message 生成放在 `msg_utils.py`
- ROS topic, header, frame, 参数读取和发布放在 `node.py`
- GridMap 坐标, rolling buffer, row / column 映射必须用测试和运行数据确认

## 当前启动入口

只使用两个完整启动脚本:

```bash
./scripts/start_wildos_2d.sh
./scripts/start_wildos_3d.sh
```

`start_wildos_2d.sh` 启动 LiDAR GridMap baseline, graph construction, odom adapter, 三相机 static TF fallback, WildOS visual scoring, graphnav planner 和 path follower

`start_wildos_3d.sh` 启动 elevation mapping, pointcloud axis adapter, graph construction, odom adapter, 三相机 static TF fallback, WildOS visual scoring, graphnav planner 和 path follower

脚本会自动执行:

- source `/opt/ros/humble/setup.bash`, 可用 `ROS_SETUP=...` 覆盖
- source workspace `install/setup.bash`, 可用 `INSTALL_SETUP=...` 覆盖
- 如果存在 `.venv` 或 `wildos_venv`, 自动激活 Python venv, 可用 `VENV_ACTIVATE=...` 覆盖
- 设置 `PYTHONNOUSERSITE=1`
- 将仓库根目录加入 `PYTHONPATH`, 让 ROS 子进程可以导入本仓库内的 `explorfm`
- 默认使用 `WILDOS_TOPIC_PROFILE=isaac`

安装到 ROS share 目录的 launch 只应保留:

- `wildos_2d_sim.launch.py`
- `elevation_visual_navigation_sim.launch.py`

不要恢复 graph-only, visual-only, adapter-only 的旧 launch 入口

topic 和 frame 不应继续散落硬编码, 当前统一入口是:

```bash
graph_construction/configs/topic_profiles.yaml
```

内置 profile:

- `isaac`, Isaac Sim 5.1, `/unitree_go2/...`
- `unity`, Unity 仿真, `/livox/lidar`, `/unity/odom`, `/camera/...`
- `robot`, 真实狗占位, `/spot1/ouster/...`, `/spot1/realsense/...`

### 默认配置

当前默认配置是 Isaac Sim 5.1 profile:

- 默认 profile: `isaac`
- 默认 namespace: `spot1`
- 默认 `ROS_DOMAIN_ID`: `3`
- 2D 默认 RMW: `rmw_cyclonedds_cpp`
- 3D 默认 RMW: `rmw_fastrtps_cpp`
- 默认 raw LiDAR: `/unitree_go2/lidar/points`
- 默认 aligned LiDAR: `/unitree_go2/lidar/points_aligned`
- 默认 raw odom: `/odom`
- 默认 adapted odom: `/spot1/odom_for_scoring`
- Isaac 2D 默认 odom pose source: `tf`, `fallback_to_message=false`
- Unity 2D 默认 odom pose source: `message`, `fallback_to_message=true`
- Unity 当前实际 body frame: `odom_fram`, 不要假设是 `base_link`
- Unity 3D 点云输出 frame: `livox_frame`, axis mode: `identity`, 让 elevation mapping 通过 TF 转换点云
- 默认 2D traversability grid: `/spot1/traversability_grid`
- 默认 3D GridMap: `/elevation_mapping_node/elevation_map_raw`
- 默认 graph output: `/spot1/nav_graph`
- 默认 scored graph: `/spot1/scored_nav_graph`
- 默认 path output: `/path2`
- 默认 goal input: `/goal_pose`
- 默认三相机图像: `/unitree_go2/{}_cam/color_image`, 其中 `{}` 为 `front`, `left`, `right`
- 默认三相机 camera info: `/unitree_go2/{}_cam/info`

2D 默认配置文件:

- grid backend: `livox_grid_builder.yaml`
- graph construction: `graph_construction.yaml`
- visual scoring: `wildos_nav_sim_conf.yaml`
- `graph_start_delay=1.0`
- `visual_start_delay=4.0`
- `planner_start_delay=5.0`
- `grid_use_sim_time=false`
- `use_sim_time=true`
- `odom_pose_source=tf`
- `odom_fallback_to_message=false`
- `publish_lidar_static_tf=true`
- `publish_camera_static_tf=true`
- `wildos_python_executable` 由 `start_wildos_2d.sh` 自动传入当前 venv Python
- WildOS 子进程默认继承仓库根目录 `PYTHONPATH`, 用于导入本地 `explorfm`
- `scan_accumulation_time_sec=4.0`
- `max_accumulated_scans=40`

3D 默认配置文件:

- elevation backend: `elevation_mapping_sim.yaml`
- graph construction: `graph_construction_elevation.yaml`
- visual scoring: `wildos_nav_sim_conf.yaml`
- `graph_start_delay=3.0`
- `visual_start_delay=6.0`
- `planner_start_delay=7.0`
- `use_sim_time=true`
- `launch_pointcloud_axis_adapter=true`
- `pointcloud_axis_mode=isaac_lidar_to_base`
- `odom_pose_source=tf`
- `odom_fallback_to_message=false`
- `publish_lidar_static_tf=false`
- `publish_camera_static_tf=true`
- `wildos_python_executable` 由 `start_wildos_3d.sh` 自动传入当前 venv Python
- WildOS 子进程默认继承仓库根目录 `PYTHONPATH`, 用于导入本地 `explorfm`

### 常用启动命令

默认 Isaac 2D:

```bash
./scripts/start_wildos_2d.sh
```

默认 Isaac 3D:

```bash
./scripts/start_wildos_3d.sh
```

切换 Unity:

```bash
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_2d.sh
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_3d.sh
```

切换真实狗占位 profile:

```bash
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_2d.sh
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_3d.sh
```

也可以直接用 launch 参数覆盖 profile:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity
./scripts/start_wildos_3d.sh topic_profile:=robot
```

指定自定义 profile 文件:

```bash
./scripts/start_wildos_2d.sh topic_profile:=lab topic_profile_file:=/abs/path/topic_profiles.yaml
./scripts/start_wildos_3d.sh topic_profile:=lab topic_profile_file:=/abs/path/topic_profiles.yaml
```

单项 topic 或 frame 覆盖, launch 参数优先级高于 profile:

```bash
./scripts/start_wildos_2d.sh lidar_topic:=/custom/lidar odom_input_topic:=/custom/odom
./scripts/start_wildos_3d.sh pointcloud_input_topic:=/custom/lidar pointcloud_output_frame:=base_link
```

调试日志和启动延迟:

```bash
./scripts/start_wildos_2d.sh log_level:=DEBUG graph_start_delay:=2.0 visual_start_delay:=5.0 planner_start_delay:=6.0
./scripts/start_wildos_3d.sh log_level:=DEBUG graph_start_delay:=4.0 visual_start_delay:=8.0 planner_start_delay:=9.0
```

覆盖 DDS:

```bash
ROS_DOMAIN_ID=3 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./scripts/start_wildos_2d.sh
ROS_DOMAIN_ID=3 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ./scripts/start_wildos_3d.sh
```

3D 脚本如果找到 `configs/fastdds_shm_profile.xml`, 会自动传入 `fastdds_profile` 并设置 FastDDS profile 环境变量

Isaac 2D 默认发布 `base_link -> lidar` fallback static TF, 如果仿真器已经发布同名 TF, 用下面命令关闭:

```bash
./scripts/start_wildos_2d.sh publish_lidar_static_tf:=false
```

2D 脚本会检查 `.venv` 中的 `omegaconf`, 并自动修正 install tree 中 `wildos` 和 `odom_frame_adapter` 的 Python shebang

### Debug Adapter

`grid_map_to_occupancy` 仅用于 debug / 兼容旧 OccupancyGrid 工具, 默认 3D graph construction 直接消费 `/elevation_mapping_node/elevation_map_raw`

手动启动 debug projection:

```bash
ros2 run graph_construction grid_map_to_occupancy --config grid_map_to_occupancy.yaml
```

默认 ROS 环境:

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

注意: `/home/ks-server3/han/wildos_ws` 通常解析到 `/mnt/hhd/han/wildos_ws`

## 近期状态记忆

已完成:

- LiDAR baseline 可生成 `/spot1/traversability_grid`
- `graph_construction` 可发布 `/spot1/nav_graph`
- visual scoring 可发布 `/spot1/scored_nav_graph`, `/spot1/model_visualization`, `/spot1/score_rings`
- `graphnav_planner` 可输出 `/spot1/graphnav_planner/path`
- `graph_builder.py` 已重构为纯算法层, 不再直接处理 ROS 消息
- 启动脚本已收敛为完整 2D 和 3D 两个入口
- topic 和 frame 已集中到 `topic_profiles.yaml`, launch 会按 profile 直接传递节点 config override
- `grid_map_to_occupancy` 仅作为 debug / 兼容 adapter 保留, 默认 graph 输入不经过它
- 2D LiDAR baseline 默认启用 `frontier_candidate_spacing: 0.8`, 用于降低密集 frontier_points 的 owner 分配成本
- 3D elevation 默认启用 `frontier_candidate_spacing: 0.5`, 用于降低密集 frontier_points 的 owner 分配成本

正在关注:

- elevation backend 的 frame, point cloud axis, GridMap 坐标映射和 surface 质量
- frontier 密度和 `builder.update_frontiers` 耗时, 慢帧日志应同时看 `frontier栅格`, `frontier候选` 和 `frontier节点`
- 3D LiDAR 近场地面不可观测问题, 优先从传感器安装几何, FOV, 近距过滤和 elevation mapping 过滤链路排查
- AGX Orin 部署效率问题, 优先用 stage timing 和真实 runtime 采样确定瓶颈
- 目标搜索启用后必须先确认 `/wildos` 节点存在, 再检查 `/spot1/score_rings`, `/spot1/scored_nav_graph` 和 `/spot1/model_visualization` 的 publisher
- 三相机视觉输入和 `front`, `left`, `right` 语义保持
- 运行时同步, QoS, TF buffer 和 stamp 差异诊断

暂缓:

- Nav2 local planning/control
- 轻量 cmd_vel controller
- 直接把 elevation backend 设为默认地图后端
- 在 graph construction 中直接为 3D 近场地面做补洞逻辑
- 未经过 AGX Orin 实测瓶颈定位的性能优化代码

## 外部参考结论

`TIKTOKDAD/nebula2-wildos-main_ws` 中值得学习:

- `graphnav_builder` 的 ROS 适配和纯算法层分离
- GridMap rolling buffer 和坐标约定测试
- frontier reachability 和 historical edge validation 的诊断化
- `wildos_sync_tuner.py` 和 `objmask_sync_tuner.py` 的同步调参思路
- `goal_pose_to_nav2.py` 的 look-ahead goal 限频和 stale cancel 策略

不建议直接照搬:

- A300 专用 Nav2 配置
- `/a300_0000/...` topic
- 数字 camera mapping 替换 `front`, `left`, `right`
- 平台专用 footprint, LaserScan 和 cmd_vel 参数

## 修改代码前检查

改代码前先确认:

- 当前路径是不是 `/mnt/hhd/han/wildos_ws/src/nebula2-wildos`
- 修改的是 source tree 还是 install tree
- runtime topic 是否和配置一致
- 是否需要同步改 launch, config, script, docs
- 新 topic 应先进入 `topic_profiles.yaml`, 不要直接散写到多个 launch 默认值
- 是否会影响 LiDAR baseline 和 elevation backend 的边界

改完后至少执行:

```bash
python3 -m compileall -q graph_construction/graph_construction
```

如果改了 ROS launch 或配置, 还要补充 `--show-args`, `ros2 topic info`, `ros2 topic echo --once` 或 build 结果
