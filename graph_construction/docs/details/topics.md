# WildOS Topic 梳理

本文维护默认 elevation 集成链路的 ROS2 topic 契约，范围以 `elevation_visual_navigation_sim.launch.py` 实际启动的节点为准

文档依据源码、launch、配置和 RViz 配置静态核对，topic 名称以默认 `unity` profile 展示，实际部署可由 `topic_profiles.yaml` 覆盖

## 1. 范围和判定规则

默认入口:

```bash
./scripts/start_wildos_elevation.sh do_object_search:=true
```

纳入范围:

- 点云和 odom 适配
- elevation mapping
- graph construction
- WildOS 视觉评分
- 目标位置融合和 Goal Mux
- graph planner
- 默认 launch 创建的静态 TF

不纳入业务 topic 数量:

- ROS2 自动生成的 `/parameter_events` 和 `/rosout`
- 默认 launch 不启动的研究 baseline 和 `path_follower_node`

状态定义:

- `必须保留`, 当前主链路存在直接消费者，删除会中断功能
- `条件保留`, 只在目标搜索或特定适配开关启用时生效
- `诊断保留`, 不参与控制，但提供当前仍有明确用途的可视化或调试信息
- `建议清理`, 默认发布但当前主链路和 RViz 均无消费者，或与保留 topic 重复

## 2. Topic 数据流

```text
/livox/lidar
  -> /livox/lidar_aligned
  -> /elevation_mapping_node/elevation_map_raw
  -> /spot1/nav_graph
  -> /spot1/scored_nav_graph
  -> /spot1/graphnav_planner/path

/unity/odom
  -> /spot1/odom_for_scoring
     -> graph construction, WildOS, Goal Mux, planner

DLIO mode:

/livox/lidar + /livox/imu
  -> /spot1/dlio/odom_node/odom
  -> /spot1/odom_for_scoring

/livox/lidar + /livox/imu
  -> /spot1/dlio/odom_node/pointcloud/deskewed
  -> elevation_mapping_cupy

camera image + camera info
  -> /spot1/object_mask
  -> /spot1/object_target_estimate
  -> /spot1/graphnav_goal_pose
  -> planner

/spot1/object_search_reached
  -> Goal Mux
  -> /spot1/object_search_completed
```

## 3. 外部输入 Topic

### 3.1 Unity 默认输入

| Topic | 类型 | 外部发布者 | WildOS 消费者 | 状态 | 说明 |
|---|---|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Unity LiDAR bridge | pointcloud adapter 或 DLIO | 必须提供 | 原始雷达点云，当前 Unity 包含 FLOAT32 XYZ、intensity 和 line，但没有逐点时间字段 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Unity Livox bridge | DLIO | DLIO 模式必须提供 | 雷达对应 IMU，必须包含角速度、线加速度，并与点云使用同一仿真时钟 |
| `/unity/odom` | `nav_msgs/msg/Odometry` | Unity robot bridge | `odom_frame_adapter` | 必须提供 | 原始机器人里程计，adapter 默认使用 TF 位姿覆盖消息 pose |
| `/camera/front/color/image/compressed` | `sensor_msgs/msg/CompressedImage` | Unity camera bridge | WildOS | 必须提供 | 前相机压缩彩色图像 |
| `/camera/left/color/image/compressed` | `sensor_msgs/msg/CompressedImage` | Unity camera bridge | WildOS | 必须提供 | 左相机压缩彩色图像 |
| `/camera/right/color/image/compressed` | `sensor_msgs/msg/CompressedImage` | Unity camera bridge | WildOS | 必须提供 | 右相机压缩彩色图像 |
| `/camera/front/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Unity camera bridge | WildOS | 必须提供 | 前相机内参，header 应与对应图像使用同一 frame 和时间基准 |
| `/camera/left/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Unity camera bridge | WildOS | 必须提供 | 左相机内参 |
| `/camera/right/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Unity camera bridge | WildOS | 必须提供 | 右相机内参 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Unity robot bridge | odom adapter、elevation mapping、WildOS、目标融合和 planner | 必须提供 | 至少需要 `odom_3D -> base_link` 动态 TF，odom adapter 默认 `pose_source=tf` 且不回退到 odom 消息 pose |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Unity bridge 或默认 launch fallback | 所有依赖 TF 的节点 | 必须提供 | 需要 `base_link` 到三相机 frame，以及 `livox_frame` 可达 `odom_3D` 的静态变换 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Unity ROS bridge | 所有启用 `use_sim_time` 的节点 | 必须提供 | 默认 launch 使用仿真时间，无 `/clock` 时 timer、消息新鲜度和 TF 查询不会正常工作 |
| `/spot1/graphnav_goal_pose` | `geometry_msgs/msg/PoseStamped` | 外部目标发布者 | `graphnav_planner` | 条件提供 | `do_object_search:=false` 时由外部系统提供高层目标，启用目标搜索时改由 Goal Mux 内部发布 |

### 3.2 输入语义和约束

#### 点云

- 外部系统只需发布 `/livox/lidar`，不要同时发布 `/livox/lidar_aligned`
- `pointcloud_axis_adapter` 使用 sensor-data QoS，可靠性为 `BEST_EFFORT`，队列深度为 5
- adapter 会把输出 `header.frame_id` 固定改为 profile 的 `pointcloud_output_frame`，Unity 默认是 `livox_frame`
- TF 树必须能把 `livox_frame` 转换到全局 `odom_3D`，否则 elevation mapping 和目标 LiDAR 细化无法使用点云
- adapter 只保留 XYZ 字段，强度、ring 和其他雷达字段不会进入高程图链路
- DLIO 模式中 `/livox/lidar` 直接进入 DLIO，不经过 `pointcloud_axis_adapter`
- 官方 DLIO 使用 `timestamp` 字段识别 Livox 并执行 deskew，当前 Unity 点云没有该字段，因此 Unity DLIO 配置显式关闭 deskew
- DLIO deskewed cloud 位于独立的 `dlio_odom`，通过 `/spot1/tf` 转换到 `odom_3D` 后进入 elevation mapping
- Unity 每帧约 40000 点中有约 20000 个无效零点, DLIO crop 会移除这些近场点
- Unity 配置固定 `adaptive: false`, 避免 adaptive GICP 在当前稀疏点云上缩小对应距离并导致里程计发散

#### DLIO 模式

Unity DLIO 模式使用:

```text
localization_backend:=dlio
launch_dlio:=true
```

主 launch 托管 DLIO 时，输入和输出为:

| Topic | 角色 |
|---|---|
| `/livox/lidar` | DLIO 原始 PointCloud2 输入 |
| `/livox/imu` | DLIO IMU 输入 |
| `/spot1/dlio/odom_node/odom` | DLIO odom 输出 |
| `/spot1/dlio/odom_node/aligned_odom` | 启动锚定到 `odom_3D` 后的 DLIO odom |
| `/spot1/dlio/odom_node/healthy` | DLIO 健康状态, false 时暂停 canonical odom、TF 和下游点云 |
| `/spot1/dlio/odom_node/pointcloud/deskewed_raw` | DLIO 原始输出, 仅供健康门控 |
| `/spot1/dlio/odom_node/pointcloud/deskewed` | 健康门控后的 elevation mapping 和目标 LiDAR 细化输入 |
| `/spot1/dlio/odom_node/tf_raw` | 官方 DLIO scan-rate TF，仅用于诊断 |
| `/spot1/tf` | `dlio_tf_adapter` 从 DLIO odom 重建的统一动态 TF |
| `/spot1/tf_static` | WildOS 专用相机静态 TF |

DLIO 直接订阅原始 LiDAR 和 IMU, 不再使用 Python 去重转发节点, 避免大点云回调压低 IMU 输入频率

DLIO 模式会绕过 XYZ-only pointcloud axis adapter，并保留 DLIO odom 的原始 timestamp

Unity 模式等待 DLIO 的 3 秒 IMU 标定稳定后，使用同时间戳 `/unity/odom` 一次性确定 `odom_3D -> dlio_odom`，后续运动只由 DLIO 更新

WildOS 节点统一读取 `/spot1/tf` 和 `/spot1/tf_static`，TF 链为 `odom_3D -> dlio_odom -> base_link -> livox_frame`，避免 Unity 和 DLIO 发布同一个 child frame

Unity 原始相机 header stamp 与主 `/clock` 不同源, `camera_stamp_adapter` 将三路 image 和 camera info 重打到当前 `/clock`, WildOS 实际订阅 `/spot1/camera_synced/{front,left,right}/...`

普通 RViz 可以用 Unity `/tf` 显示已经对齐到 `odom_3D` 的 GridMap，检查 DLIO 局部 frame 时需要 remap `/tf:=/spot1/tf` 和 `/tf_static:=/spot1/tf_static`，或使用 `launch_paper_rviz:=true`

Zenoh 干净启动时 WildOS 可能先打印一次 `暂时未找到 TF`, 当前实测约 18 秒后会打印 `首次找到相机 TF` 并开始发布 scored graph。只有后续始终没有恢复日志和 scored graph 消息时才判定为故障

`launch_dlio:=false` 时 DLIO 由外部进程提供，外部集成必须保证 canonical TF 与 odom pose 同源且时间戳一致

2026-07-17 Unity 实测中，LiDAR 约为 `10 Hz`，IMU 约为 `100 Hz`，DLIO odom、cloud、隔离 TF、GridMap、navigation graph 和 WildOS 首帧均已连通。关闭 adaptive 后静止运行约 98 秒，位置漂移保持在 3 mm 内。当前验证仍不覆盖运动 deskew 与完整轨迹精度，因为 Unity 点云缺少逐点时间字段

#### Odom 和 TF

- platform backend 默认使用 TF pose，DLIO backend 默认直接保留 DLIO odom pose
- 官方 DLIO 的 odom 为 IMU-rate，原始 TF 为 scan-rate，主链禁止直接使用原始 TF pose
- `dlio_tf_adapter` 发布固定启动锚定 `odom_3D -> dlio_odom`、每帧 `dlio_odom -> base_link` 和对齐后的 odom，并转发传感器外参
- TF、odom、点云和相机必须使用同一个仿真时钟，不能混用系统时间和 `/clock`
- 平台已提供相机静态 TF 时应设置 `publish_camera_static_tf:=false`，避免同一 child frame 存在两个 authority
- 平台没有相机静态 TF 时，可保留默认 launch 的三个 fallback static publisher
- LiDAR fallback static TF 默认关闭，只有平台没有对应变换时才启用 `publish_lidar_static_tf:=true`

#### 三相机同步

WildOS 使用一个近似时间同步器联合等待以下 8 路消息:

- odom
- base navigation graph
- front、left、right 三路图像
- front、left、right 三路 CameraInfo

默认同步参数:

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `qos_history_depth` | 1 | 每个订阅者只保留最新消息 |
| `syncsub_queue_size` | 120 | 仿真配置的同步器候选队列长度 |
| `syncsub_slop` | 5.0s | 仿真配置中 8 路消息允许的最大时间差 |

任一路相机或 CameraInfo 缺失、时间戳不更新、时间差长期超过 `5.0s`，都会让整组视觉推理停止触发。排障时应先检查三路图像频率和 header stamp，再检查模型性能

`wildos_nav_conf.yaml` 的真实机器人配置使用 `queue=2, slop=0.2s`，不要把该阈值误认为默认仿真参数

### 3.3 Profile 输入差异

| Profile | 原始点云 | 原始 odom | 相机图像模板 | CameraInfo 模板 | 外部高层目标 |
|---|---|---|---|---|---|
| `unity` | `/livox/lidar` | `/unity/odom` | `/camera/{}/color/image/compressed` | `/camera/{}/color/camera_info` | `/spot1/graphnav_goal_pose` |
| `isaac` | `/livox/lidar` | `/odom` | `/unitree_go2/{}_cam/color_image` | `/unitree_go2/{}_cam/info` | `/goal_pose` |
| `robot` | `/spot1/ouster/front/points_filtered` | `/spot1/odom` | `/spot1/realsense/{}/color/image_raw/compressed` | `/spot1/realsense/{}/color/camera_info` | `/goal_pose` |

`{}` 固定展开为 `front`、`left`、`right`。profile 只改变平台 topic 和 frame 契约，不改变三相机、点云、odom、TF 的逻辑角色

### 3.4 输入检查命令

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /unity/odom
ros2 topic hz /camera/front/color/image/compressed
ros2 topic hz /camera/left/color/image/compressed
ros2 topic hz /camera/right/color/image/compressed
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom_3D base_link
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/imu
```

## 4. 默认主链路输出

### 4.1 输入适配和高程地图

| Topic | 类型 | 发布者 | 主要消费者 | 状态 | 说明 |
|---|---|---|---|---|---|
| `/livox/lidar_aligned` | `sensor_msgs/msg/PointCloud2` | `pointcloud_axis_adapter` | `elevation_mapping_node`, `object_target_fusion` | 必须保留 | 统一轴向和 frame 后的点云，既用于建图也用于目标 LiDAR 细化 |
| `/spot1/odom_for_scoring` | `nav_msgs/msg/Odometry` | `odom_frame_adapter` | `graph_construction`, WildOS, Goal Mux, planner | 必须保留 | 统一 frame、时间戳和 TF 位姿后的唯一内部 odom |
| `/elevation_mapping_node/elevation_map_raw` | `grid_map_msgs/msg/GridMap` | `elevation_mapping_node` | `graph_construction`, RViz | 必须保留 | 当前唯一几何地图，包含 `elevation`、`traversability` 和 `variance` |

结论:

- `/livox/lidar` 是外部原始输入，`/livox/lidar_aligned` 是 WildOS 内部规范化输出，两者不重复
- `/elevation_mapping_node/elevation_map_recordable` 已从默认配置删除
- 如未来需要低频录包，应由 rosbag 侧降采样或显式启用专用配置，不应让默认运行持续发布第二份大体积 GridMap

### 4.2 图构建和视觉评分

| Topic | 类型 | 发布者 | 主要消费者 | 状态 | 说明 |
|---|---|---|---|---|---|
| `/spot1/nav_graph` | `graphnav_msgs/msg/NavigationGraph` | `graph_construction` | WildOS | 必须保留 | 未加入视觉分数的基础持久图 |
| `/spot1/graph_construction_viz` | `visualization_msgs/msg/MarkerArray` | `graph_construction` | RViz | 诊断保留 | 统一显示历史节点、当前节点、边、frontier 和轨迹，不参与规划 |
| `/spot1/scored_nav_graph` | `graphnav_msgs/msg/NavigationGraph` | WildOS | `graphnav_planner` | 必须保留 | 在基础图上写入当前视觉 frontier 分数后的 planner 输入 |
| `/spot1/model_visualization` | `sensor_msgs/msg/Image` | WildOS | 人工调试工具 | 诊断保留 | 合成显示模型分割、frontier 投影和评分结果，不是相机原图副本 |
| `/spot1/within_range_geofrontiers` | `visualization_msgs/msg/MarkerArray` | WildOS | 人工调试工具 | 诊断保留 | 只显示当前相机可投影且进入评分范围的几何 frontier |
| `/spot1/score_rings` | `visualization_msgs/msg/MarkerArray` | WildOS | RViz | 诊断保留 | 显示视觉模型对 frontier heading 的分数 |

这些 topic 不重复:

- `nav_graph` 是几何图，`scored_nav_graph` 是视觉评分后的控制输入，不能合并
- `graph_construction_viz` 显示图结构，`within_range_geofrontiers` 显示当前相机可评分子集，语义不同
- `score_rings` 表达视觉 heading 分数，planner 的 `frontier_scores` 表达融合路线代价，计算阶段不同
- `model_visualization` 是二维模型调试图，其他三个是三维 marker

性能说明:

- `model_visualization` 每帧生成合成图，即使没有订阅者也会执行图像构造和发布
- `within_range_geofrontiers` 和 `score_rings` 同样持续构造 marker
- 三者不是重复 topic，但可进一步改为存在订阅者时才生成，或通过统一的 debug 参数关闭

### 4.3 目标搜索

以下 topic 仅在 `do_object_search:=true` 时构成完整链路

| Topic | 类型 | 发布者 | 主要消费者 | 状态 | 说明 |
|---|---|---|---|---|---|
| `/spot1/object_mask` | `object_search_msgs/msg/ObjectMaskWithTf` | WildOS | `object_target_fusion` | 条件保留 | 连续帧视觉确认后的 mask、相机内参和观测 TF |
| `/spot1/object_search_reached` | `std_msgs/msg/Bool` | WildOS | `object_search_goal_mux` | 条件保留 | 近距离视觉到达证据，不等于最终完成状态 |
| `/spot1/object_target_estimate` | `object_search_msgs/msg/TargetEstimate` | `object_target_fusion` | `object_search_goal_mux` | 条件保留 | 多视角粒子融合后的唯一目标位置和稳定性状态 |
| `/spot1/object_target_estimate_viz` | `visualization_msgs/msg/Marker` | `object_target_fusion` | RViz | 诊断保留 | 目标位置球体和射线 marker，仅用于显示 |
| `/spot1/object_target_particles` | `sensor_msgs/msg/PointCloud2` | `object_target_fusion` | RViz | 诊断保留 | 当前目标粒子分布，仅用于定位质量诊断 |
| `/spot1/graphnav_goal_pose` | `geometry_msgs/msg/PoseStamped` | `object_search_goal_mux` | `graphnav_planner` | 条件保留 | Unity profile 的唯一高层探索、追踪和停止目标 |
| `/spot1/object_search_status` | `std_msgs/msg/String` | `object_search_goal_mux` | `graphnav_planner`, 人工日志诊断 | 条件保留 | 传递状态机阶段和目标来源，planner 用它切换探索策略 |
| `/spot1/object_search_completed` | `std_msgs/msg/Bool` | `object_search_goal_mux` | WildOS, `object_target_fusion` | 条件保留 | 最终完成状态的唯一 owner 输出，完成后停止新增视觉证据 |

结论:

- `object_search_reached` 是输入证据，`object_search_completed` 是 Mux 门控后的最终状态，不能合并
- `object_target_estimate` 是机器可消费的数据，`object_target_estimate_viz` 是 RViz marker，不能互相替代
- 已删除的 `/spot1/object_search_goal_viz` 不应恢复，最终目标直接显示 `/spot1/graphnav_goal_pose` 即可
- 已删除的 `/spot1/object_target_pose` 和旧视觉粗定位 marker 不应恢复

### 4.4 Planner 输出

| Topic | 类型 | 发布者 | 主要消费者 | 状态 | 说明 |
|---|---|---|---|---|---|
| `/spot1/graphnav_planner/path` | `nav_msgs/msg/Path` | `graphnav_planner` | 外部运动执行器, RViz | 必须保留 | 当前规划结果，默认系统的最终执行输出 |
| `/spot1/graphnav_planner/unexplored_space_map` | `grid_map_msgs/msg/GridMap` | `graphnav_planner` | 按需调试订阅者 | 诊断保留 | planner 内部未探索空间距离图，仅在存在订阅者时构造和发布 |
| `/spot1/graphnav_planner/frontier_scores` | `visualization_msgs/msg/MarkerArray` | `graphnav_planner` | 按需调试订阅者 | 诊断保留 | 显示 planner 最终 frontier score 和 cost，仅在存在订阅者时构造和发布 |

Planner 两个 debug topic 已通过 `get_subscription_count()` 避免无人订阅时构造消息，保留成本较低

`/spot1/score_rings` 与 `/spot1/graphnav_planner/frontier_scores` 的区别:

- `score_rings` 来自 WildOS，展示视觉模型对方向的评分
- `frontier_scores` 来自 planner，展示加入图代价、分支状态和探索策略后的最终评分
- 二者可用于定位“视觉判断错误”还是“规划策略压制”，不属于重复发布

## 5. TF 输出

| Topic | 发布者 | 状态 | 说明 |
|---|---|---|---|
| `/tf_static` | 三个 camera `static_transform_publisher` | 条件保留 | `publish_camera_static_tf:=true` 时发布 front、left、right 相机静态 TF |
| `/tf_static` | LiDAR `static_transform_publisher` | 条件保留 | 仅 `publish_lidar_static_tf:=true` 时发布，默认关闭 |

`graph_construction` 在 namespace 中将 TF 订阅重映射到 `/spot1/tf` 和 `/spot1/tf_static`，但静态 TF publisher 默认仍发布全局 `/tf_static`。平台已提供 TF 时应关闭对应 fallback，避免重复 frame authority

## 6. 默认链路之外的输出

### 6.1 `path_follower_node`

该节点保留但不由默认集成 launch 启动

| 默认 topic | 类型 | 说明 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 从 path 选择 lookahead waypoint，供需要单点目标的实验底盘使用 |

它不是 `/spot1/graphnav_goal_pose` 的重复发布者。前者是 path 的下游适配输出，后者是 planner 的高层输入，两者方向相反

### 6.2 研究 baseline

LRN、ImgFrontier 和 GeoFrontier 的 `goal_waypoints`、`goal_direction`、`predicted_path`、`lrn_scores_debug` 等 topic 只属于各自独立 launch。研究仓库继续保留这些实现，但不得加入默认 elevation topic profile，也不应计入默认系统 topic 清理范围

## 7. 清理结论

### 7.1 已清理

| Topic | 处理结果 | 原因 |
|---|---|---|
| `/elevation_mapping_node/elevation_map_recordable` | 已从默认 `elevation_mapping_sim.yaml` 删除 | 无消费者，与 `elevation_map_raw` 内容重复，持续发布大体积 GridMap |

### 7.2 暂不删除但可优化

| Topic | 优化方向 |
|---|---|
| `/spot1/model_visualization` | 增加 debug 开关或仅有订阅者时生成合成图 |
| `/spot1/within_range_geofrontiers` | 增加 debug 开关或仅有订阅者时构造 marker |
| `/spot1/score_rings` | 增加 debug 开关或仅有订阅者时构造 marker |
| `/spot1/object_target_particles` | 可按定位调试开关控制，默认论文 RViz 仍需要 |

### 7.3 不应按重复删除

- `/livox/lidar` 和 `/livox/lidar_aligned`
- `/spot1/nav_graph` 和 `/spot1/scored_nav_graph`
- `/spot1/object_search_reached` 和 `/spot1/object_search_completed`
- `/spot1/object_target_estimate` 和 `/spot1/object_target_estimate_viz`
- `/spot1/score_rings` 和 `/spot1/graphnav_planner/frontier_scores`
- `/spot1/graph_construction_viz` 和 `/spot1/within_range_geofrontiers`

## 8. 维护检查

修改发布者、launch remapping 或 `topic_profiles.yaml` 后必须同步本文，并执行:

```bash
ros2 topic list -t
ros2 topic info -v <topic>
ros2 node info <node>
```

运行期重点确认:

- 每个必须保留 topic 至少有一个预期消费者
- 同一语义只有一个状态 owner 或数据 owner
- 大体积 GridMap、PointCloud2 和 Image 无无人消费的持续副本
- debug topic 无订阅者时不做高成本消息构造
- profile 改名后不存在旧 topic 的残留发布者或订阅者
