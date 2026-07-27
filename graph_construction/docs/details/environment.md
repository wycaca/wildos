# 环境配置

> 本文只列出更换仿真器、机器人、LiDAR、IMU 或相机时需要检查的配置

## 1. 配置文件职责

| 文件 | 修改时机 | 内容 |
|---|---|---|
| `graph_construction/configs/topic_profiles.yaml` | 更换平台或通信环境 | 外部 topic、frame、RMW、内部 topic 契约 |
| `graph_construction/configs/dlio/<profile>.yaml` | 使用 DLIO 或更换 LiDAR、IMU | 传感器 frame、外参、去畸变、IMU 和 GICP 参数 |
| `graph_construction/configs/elevation_mapping_sim.yaml` | 更换地图范围或传感器量程 | GridMap 尺寸、更新率、启动安全先验 |
| `graph_construction/configs/graph_construction_elevation.yaml` | 更换机器人尺寸或图密度要求 | 地图分类、净空、节点和边参数 |
| `visual_navigation/configs/wildos_nav_sim_conf.yaml` | 更换视觉模型或相机性能要求 | 模型、同步、评分和目标检测参数 |
| `visual_navigation/configs/object_search_goal_mux.yaml` | 更换搜索任务策略 | 观察距离、超时、粗目标和到达条件 |
| `graphnav_planner/config/planner.yaml` | 更换运动执行特性 | 路径稳定、进展超时和目标半径 |

内部 topic 的完整发布者和消费者见 [Topic 契约](topics.md)

## 2. 新环境最少修改项

建议复制一个现有 profile 并改名, 不要直接把 `unity` 改成另一套平台

### 2.1 通信

在 `topic_profiles.yaml` 中检查:

- `ros_domain_id`
- `rmw_implementation`
- `namespace`
- launch 参数 `use_sim_time`
- 使用 FastDDS 时的 `fastdds_profile`

仿真通常需要 `/clock` 和 `use_sim_time:=true`

真机通常不订阅 `/clock`, 使用 `use_sim_time:=false`

### 2.2 Frame

必须形成一条无冲突的 TF 链:

```text
global_frame
  -> odom or dlio_local_frame
  -> base_frame
  -> lidar_frame
  -> imu_frame
  -> camera frames
```

需要检查的 profile 字段:

- `global_frame`
- `parent_frame`
- `odom_parent_frame`
- `odom_child_frame`
- `base_frame`
- `lidar_parent_frame`
- `lidar_frame`
- `imu_frame`
- `camera_parent_frame`
- `cam_frame`
- `pointcloud_output_frame`

传感器驱动已经发布静态 TF 时, 关闭对应 fallback:

```text
publish_lidar_static_tf:=false
publish_camera_static_tf:=false
```

同一个 child frame 只能有一个 TF owner

### 2.3 外部输入 Topic

所有定位模式必须配置:

- `pointcloud_input_topic`
- `camera_img_topic`
- `camera_info_topic`
- `goal_pose_topic`

platform 模式还必须配置:

- `aligned_lidar_topic`
- `odom_input_topic`

DLIO 模式还必须配置:

- `dlio_imu_input_topic`
- `dlio_reference_odom_topic`
- `dlio_topic_root`
- `dlio_local_frame`
- `dlio_alignment_delay`

`pointcloud_input_topic` 是唯一原始点云入口, platform adapter 和 DLIO 共用

DLIO odom、aligned odom、健康状态、deskewed 点云和隔离 TF 都从 `dlio_topic_root` 自动生成, 不需要逐个配置

`dlio_reference_odom_topic` 在仿真中用于启动对齐和误差诊断

真机没有外部全局参考时可以留空, 但必须确认系统如何确定 DLIO 到全局 frame 的初始变换

### 2.4 相机

检查:

- `camera_img_topic`
- `camera_info_topic`
- `cam_frame`
- `camera_parent_frame`
- `camera_static_tf_convention`
- `camera_image_flip_x`
- `camera_stamp_mode`

`camera_stamp_mode=preserve` 表示驱动时间已经与 LiDAR、odom 和 TF 同步

`camera_stamp_mode=now` 只用于仿真相机时间不在当前 ROS 时钟域的情况

更换分辨率或镜头后还要重新检查:

- `object_search_detection_min_component_pixels`
- `object_search_detection_min_component_fraction`
- `object_search_reached_min_pixel_count`
- `object_search_reached_mask_fraction`
- `visual_frontiers_range`
- `visual_frontier_threshold`

## 3. DLIO 环境参数

### 3.1 必须按硬件修改

在 `configs/dlio/<profile>.yaml` 中检查:

| 参数 | 含义 |
|---|---|
| `frames/odom` | DLIO 局部 odom frame |
| `frames/baselink` | 机器人本体 frame |
| `frames/lidar` | 点云 header frame |
| `frames/imu` | IMU header frame |
| `extrinsics/baselink2lidar/t` | base 到 LiDAR 平移, 单位 m |
| `extrinsics/baselink2lidar/R` | base 到 LiDAR 旋转矩阵 |
| `extrinsics/baselink2imu/t` | base 到 IMU 平移, 单位 m |
| `extrinsics/baselink2imu/R` | base 到 IMU 旋转矩阵 |
| `odom/gravity` | 当前环境重力加速度 |
| `pointcloud/deskew` | 是否使用逐点时间去畸变 |

启用 `pointcloud/deskew` 前必须确认点云包含 DLIO 支持的逐点时间字段, 单位正确且单帧内单调

### 3.2 根据数据质量调整

以下参数不是 topic 接入参数, 只有日志或轨迹表明存在问题时才调整:

- `odom/preprocessing/voxelFilter/res`
- `odom/preprocessing/cropBoxFilter/size`
- `odom/keyframe/threshD`
- `odom/keyframe/threshR`
- `odom/submap/keyframe/knn`
- `odom/gicp/minNumPoints`
- `odom/gicp/kCorrespondences`
- `odom/gicp/maxCorrespondenceDistance`
- `odom/gicp/maxIterations`
- `odom/geo/*`

不要用调 GICP 或 observer 增益掩盖错误外参、错误 frame、时间不同步或点云轴向错误

## 4. 高程图和图构建

更换场景或 LiDAR 后检查高程图:

| 参数 | 何时修改 |
|---|---|
| `resolution` | 地图精度和计算量需要变化 |
| `map_length` | 局部可见范围变化 |
| `min_valid_distance` | LiDAR 近场盲区变化 |
| `max_height_range` | 场景垂直范围变化 |
| `max_ray_length` | LiDAR 有效距离变化 |
| `initialize_tf_offset` | base_link 到地面高度变化 |
| `initialize_tf_grid_size` | 启动盲区尺寸变化 |

更换机器人尺寸后检查图构建:

| 参数 | 何时修改 |
|---|---|
| `min_obstacle_clearance` | 机器人安全净空变化 |
| `robot_ground_height_offset` | odom 原点到地面高度变化 |
| `robot_ground_elevation_tolerance` | 可接受地面高度误差变化 |
| `max_free_radius` | 稀疏节点覆盖尺度变化 |
| `edge_radius` | 最大局部连边距离变化 |

启动先验搜索半径必须大于膨胀后人工区域的半宽

当前 Unity 参数中, 12 m 初始化方形经过两轮 5-cell 膨胀后约为 16 m 宽, 因此图层搜索半径使用 12 m

## 5. 内部输出 Topic

`common_contract` 中的内部 topic 通常不随环境改变:

- `/spot1/odom_for_scoring`
- `/elevation_mapping_node/elevation_map_raw`
- `/spot1/nav_graph`
- `/spot1/scored_nav_graph`
- `/spot1/graphnav_planner/path`
- `/spot1/object_mask`
- `/spot1/object_target_estimate`

`goal_pose_topic` 既可以由目标搜索 Goal Mux 输出, 也可以由外部任务系统输入, 因此保留在各环境 profile 中

如果必须修改 namespace 或内部 topic, 应在 `common_contract` 中统一修改, 并同步检查:

- graph construction
- WildOS
- object target fusion
- object search goal mux
- Planner
- performance monitor
- RViz
- 仓库外运动执行适配层

不要只修改某一个节点的 fallback YAML

## 6. 启动方式

使用内置 profile:

```bash
./scripts/start_wildos_elevation.sh \
  topic_profile:=unity \
  localization_backend:=dlio \
  launch_dlio:=true \
  do_object_search:=true
```

使用自定义环境文件:

```bash
./scripts/start_wildos_elevation.sh \
  topic_profile:=my_robot \
  topic_profile_file:=/absolute/path/topic_profiles.yaml \
  localization_backend:=dlio \
  dlio_config_file:=/absolute/path/dlio.yaml \
  elevation_config:=/absolute/path/elevation.yaml \
  graph_config:=/absolute/path/graph.yaml
```

相对配置名从对应 ROS package 的安装目录解析, 自定义文件建议使用绝对路径

## 7. 上线前检查

```bash
ros2 topic list -t
ros2 topic hz <pointcloud_topic>
ros2 topic hz <imu_topic>
ros2 topic hz <odom_topic>
ros2 topic echo <pointcloud_topic> --once
ros2 topic echo <imu_topic> --once
ros2 run tf2_ros tf2_echo <global_frame> <base_frame>
ros2 node info /spot1/graph_construction
```

检查顺序:

1. 时钟和 ROS domain
2. 原始传感器 topic
3. frame 和静态外参
4. odom 与 canonical TF
5. 高程图
6. 导航图和 scored graph
7. Planner Path
8. 运动执行适配层输出

只有 topic、时间和 TF 契约正确后, 才开始调整算法阈值
