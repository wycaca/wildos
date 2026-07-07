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
    -> /spot1/object_search_target_pose
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
- Unity 2D 默认 odom pose source: `tf`, `fallback_to_message=true`
- Unity 当前 TF 主链路: `map -> odom_3D -> base_link`
- Unity 的 `odom_3D` 是里程计中间层, `base_link` 是机器人本体层, 相机和 LiDAR fallback TF 应挂在 `base_link`
- Unity 2D grid builder 默认 odom 输入: `/spot1/odom_for_scoring`
- Unity 2D 当前已切回本仓库自研 `livox_grid_builder`, `launch_livox_grid_builder=true`
- `/combined_grid` 仅作为对照测试输入保留, 参考实现见 `2026-07-07-combined-grid-implementation-reference.md`, 其负值 cell 是同事 A* 的自定义代价语义
- Unity 2D 自研 `livox_grid_builder` 调试链路保留, `/livox/lidar` 按 Unity 已输出的世界坐标处理, `lidar_assume_input_in_grid_frame=true`
- Unity 2D 自研 `livox_grid_builder` 调试链路使用 rolling local map, `grid_origin_mode=rolling`
- Unity 2D 自研 `livox_grid_builder` 当前调试参数: `grid_resolution=0.1`, `grid_local_width=20.0`, `grid_local_height=20.0`, `grid_min_obstacle_height=0.0`, `grid_obstacle_inflation_radius=0.2`, `grid_origin_snap_to_resolution=true`, `grid_force_odd_grid_size=true`, `grid_robot_clear_radius=0.2`, `grid_obstacle_detection_mode=height_diff`
- Unity 3D 点云输出 frame: `livox_frame`, axis mode: `identity`, 让 elevation mapping 通过 TF 转换点云
- 默认 2D traversability grid: `/spot1/traversability_grid`
- 默认 3D GridMap: `/elevation_mapping_node/elevation_map_raw`
- 默认 graph output: `/spot1/nav_graph`
- 默认 scored graph: `/spot1/scored_nav_graph`
- 默认 path output: Isaac/robot 为 `/path2`, Unity 为 `/multi_planned_path`
- 默认 goal input: `/goal_pose`
- object search 默认初始 goal: 当前 odom 朝向前方 `30.0m`
- object search 默认目标 frontier 输入: `/spot1/object_search_target_pose`
- object search 默认目标可视化: `/spot1/object_search_goal_viz`
- object search 默认到达确认: `/spot1/object_search_reached`
- object search 默认 goal mux 发布频率: `5.0Hz`
- object search 默认目标 mask 阈值: Isaac/robot 为 `0.09`, Unity 当前为 `0.10`
- WildOS visual frontier 默认: Isaac/robot 为 `frontiers_range=9.0`, `frontier_threshold=0.60`, Unity 当前为 `frontiers_range=11.0`, `frontier_threshold=0.55`
- object search 默认未检测诊断间隔: `20` 个 WildOS 同步处理帧
- object search 默认 `target_timeout_sec=3.0`, `latch_target_after_first_detection=false`, `latch_target_timeout_sec=3.0`
- object search 默认目标记忆: `memory_timeout_sec=10.0`, `memory_goal_distance=10.0`, `target_reached_radius=1.5`
- object search 默认近距离确认: `reached_mask_fraction=0.01`, `reached_min_pixel_count=1200`, `reached_confirm_frames=2`
- 默认 path follower 跟踪点输出: `/spot1/tracking_goal_pose`
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
- `base_frame` 来自 profile, 默认 Isaac 为 `base_link`, Unity 为 `base_link`, robot 为 `spot1/base_link`
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

启用 2D 目标搜索:

```bash
./scripts/start_wildos_2d.sh do_object_search:=true
```

默认 Isaac 3D:

```bash
./scripts/start_wildos_3d.sh
```

启用 3D 目标搜索:

```bash
./scripts/start_wildos_3d.sh do_object_search:=true
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

目标搜索参数覆盖:

```bash
./scripts/start_wildos_2d.sh do_object_search:=true object_search_initial_goal_distance:=20.0
./scripts/start_wildos_3d.sh do_object_search:=true object_search_initial_goal_heading_deg:=30.0
./scripts/start_wildos_2d.sh do_object_search:=true object_search_mask_threshold:=0.05 object_search_goal_publish_rate:=8.0
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
- WildOS 目标搜索默认应加载本地 `ckpts/c-radio_v3-b_half.pth.tar` 和完整 SigLIP2 cache `ckpts/siglip2`
- `/spot1/object_search_target_pose` 是目标语义引导下的最高分 frontier, 不是精确物体坐标
- `/spot1/object_search_target_viz` 显示目标 frontier 和相机检测射线
- `/spot1/score_rings` 默认只显示 frontier heading score 的方向彩色圆环
- Unity profile 默认 `camera_static_tf_convention: x_forward_y_left`, 对应 ROS `base_link` 的 `x 前, y 左, z 上`
- `negative_y_forward_x_right` 只适用于旧 `odom_fram` 坐标习惯, 不要用于当前 `base_link`
- Unity profile 默认 `camera_parent_frame=base_link`, `lidar_parent_frame=base_link`, `odom_child_frame=odom_3D`
- `camera_image_flip_x` 仅作为图像水平轴补偿开关保留, Unity 当前默认关闭
- 目标不在视野时的 object search 设计见 `2026-07-06-object-search-exploration-design.md`
- `object_search_goal_mux` 已实现, 负责给 graphnav planner 提供 initial search goal, 三角定位仍是第二阶段
- 目标丢失后优先进入 `TARGET_MEMORY_GUIDED_SEARCH`, 复用最近目标 frontier 安全节点, 不把固定方向外推成自由空间 goal
- `TARGET_REACHED_VIEWPOINT` 表示目标近距离可见或已到达目标 frontier 对应观察点, 不等于物体精确抵达
- `TARGET_FRONTIER_LATCHED` 默认关闭, 只在需要短暂遮挡容忍时手动开启
- `/goal_pose` 只作为 planner 高层 goal 输入, 不要让 path follower 或底层控制节点继续发布到同一个 topic
- `graphnav_planner` 默认不把虚拟 goal 和未知 `frontier_points` 追加到执行路径, `/corrected_path` 应优先由真实 graph node 组成
- RViz2 查看高层搜索目标优先订阅 `/spot1/object_search_goal_viz`, 类型为 `visualization_msgs/MarkerArray`
- 社区参考仓库本地副本位于 `external_references/nebula2-wildos-main_ws`, 当前记录 commit `225be74`, 该目录被 `.gitignore` 忽略
- `external_references/COLCON_IGNORE` 必须保留, 否则 colcon 会扫描参考仓库并报同名 package 冲突
- 新增 console script 后必须重新构建对应 ROS2 package, 否则 launch 会报 `executable ... not found on the libexec directory`

正在关注:

- elevation backend 的 frame, point cloud axis, GridMap 坐标映射和 surface 质量
- frontier 密度和 `builder.update_frontiers` 耗时, 慢帧日志应同时看 `frontier栅格`, `frontier候选` 和 `frontier节点`
- 3D LiDAR 近场地面不可观测问题, 优先从传感器安装几何, FOV, 近距过滤和 elevation mapping 过滤链路排查
- AGX Orin 部署效率问题, 优先用 stage timing 和真实 runtime 采样确定瓶颈
- 目标搜索启用后必须先确认 `/wildos` 节点存在, 再检查 `/spot1/score_rings`, `/spot1/scored_nav_graph` 和 `/spot1/model_visualization` 的 publisher
- 如果目标搜索日志停在 `WildOS 加载视觉模型`, 优先检查是否误用根 `ckpts/models--google--...` 的不完整 HuggingFace cache
- SigLIP2 离线加载应使用 `ckpts/siglip2/.../snapshots/<revision>`, 不要让启动路径依赖在线 repo id 解析
- Unity 启动期可能出现图像或点云时间略早于 TF buffer 的过去外推, 当前代码会用 latest TF 或上一帧 TF 兜底
- 如果 TF 过去外推持续出现, 优先检查 `/clock`, `/tf`, 点云 stamp 和 static TF publisher 是否一致
- 三相机视觉输入和 `front`, `left`, `right` 语义保持
- 目标搜索路径慢时先看 WildOS 未检测日志中的每路相机最高相似度, 再看 `/spot1/object_search_target_pose`, `/goal_pose`, `/corrected_path` 或 profile 配置的 `path_topic`
- 路径贴墙或离开绿色 graph node 区域时, 优先检查是否有节点重新打开了 `append_virtual_goal_to_path` 或 `append_frontier_point_to_path`
- 点云和 2D grid 错位时, 优先检查 `livox_grid_builder` 日志中的 `assume_input_in_grid_frame` 和 odom frame, Unity 应为 True 且 odom frame 应为 `map`
- Unity 2D 默认使用本仓库 `livox_grid_builder`, 应看到本仓库 `livox_grid_builder` 进程启动
- Unity 2D 临时回切 `/combined_grid` 对照测试时, 启动命令需要显式加 `launch_livox_grid_builder:=false traversability_grid_topic:=/combined_grid`
- Unity 2D 自研 builder 地图随狗旋转或路线乱跳时, 优先检查 `origin_mode=rolling`, `snap=True`, `mode=height_diff`, `height_range=(0.00,1.80)`, `inflation=0.20` 是否生效
- 路线每帧剧烈变化时, 优先检查 `graphnav_planner.path_smoothness_period`, 当前 2D/3D launch 应为 `10.0`
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
