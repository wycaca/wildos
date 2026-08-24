# WildOS 实机 Topic 契约

> 具体名称以 `graph_construction/configs/topic_profiles.yaml` 的 `robot` profile 为准

## 1. 主数据流

```text
x86:
/livox/lidar + /livox/imu
  -> D-LIO
  -> /cloud_registered + /odom + /tf

相机 AGX:
/cloud_registered
  -> /spot1/cloud_registered_local
  -> elevation GridMap
  -> NavigationGraph

三路相机 + NavigationGraph
  -> WildOS
  -> Scored NavigationGraph
  -> Graph Planner
  -> Path
```

## 2. x86 输入和输出

| Topic | 类型 | 发布者 | 消费者 | 频率或要求 |
|---|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Livox 驱动 | D-LIO | 约 10 Hz, 含逐点 `timestamp` |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Livox 驱动 | D-LIO | 时间连续 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | D-LIO guard | 相机 AGX | 约 10 Hz, frame=`dlio_odom` |
| `/odom` | `nav_msgs/msg/Odometry` | D-LIO TF adapter | 相机 AGX | parent=`odom`, child=`base_link` |
| `/tf` | `tf2_msgs/msg/TFMessage` | D-LIO TF adapter | 两台主机 | 不能有重复 child owner |

`/cloud_registered` 静止时也必须以配置频率持续发布

## 3. 相机 AGX 外部输入

| Topic 模板 | 类型 | 说明 |
|---|---|---|
| `/spot1/realsense/{}/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `{}` 为 front、left、right |
| `/spot1/realsense/{}/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 对应相机内参 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 外部目标输入或 Goal Mux 输出 |

前相机为 D435if, 左右相机为 D435i

## 4. 相机 AGX 适配层

| Topic | 类型 | 发布者 | 消费者 |
|---|---|---|---|
| `/spot1/cloud_registered_local` | `sensor_msgs/msg/PointCloud2` | pointcloud relay | elevation mapping、目标融合 |
| `/spot1/odom_for_scoring` | `nav_msgs/msg/Odometry` | odom frame adapter | graph、WildOS、Goal Mux、Planner |
| `/elevation_mapping_node/elevation_map_raw` | `grid_map_msgs/msg/GridMap` | elevation mapping | graph construction |

点云 relay 零拷贝转发 x86 已限频的输入, 只跨机订阅一次原始注册点云
输入 frame 必须为 `dlio_odom`, 不匹配时直接丢弃, 不允许只改 header 冒充坐标变换

## 5. 主链输出

| Topic | 类型 | 发布者 | 消费者 |
|---|---|---|---|
| `/spot1/nav_graph` | `graphnav_msgs/msg/NavigationGraph` | graph construction | WildOS、Goal Mux |
| `/spot1/scored_nav_graph` | `graphnav_msgs/msg/NavigationGraph` | WildOS | Planner、Goal Mux |
| `/spot1/graphnav_planner/path` | `nav_msgs/msg/Path` | Planner | 仓库外运动执行模块 |
| `/spot1/graph_construction_viz` | `visualization_msgs/msg/MarkerArray` | graph construction | RViz |
| `/spot1/model_visualization` | `sensor_msgs/msg/Image` | WildOS | RViz |
| `/spot1/within_range_geofrontiers` | `visualization_msgs/msg/MarkerArray` | WildOS | RViz |
| `/spot1/score_rings` | `visualization_msgs/msg/MarkerArray` | WildOS | RViz |

`nav_graph` 是纯几何图, `scored_nav_graph` 是加入视觉评分后的 Planner 输入
评分内容变化时立即发布, 内容不变时以 1 Hz 心跳维持 freshness；Planner 会跳过心跳图的重建和 Dijkstra

## 6. 目标搜索

| Topic | 类型 | 发布者 | 消费者 |
|---|---|---|---|
| `/spot1/object_search_target` | `std_msgs/msg/String` | 操作端或任务系统 | WildOS、target fusion、Goal Mux |
| `/spot1/object_mask` | `ObjectMaskWithTf` | WildOS | target fusion |
| `/spot1/object_target_estimate` | `TargetEstimate` | target fusion | Goal Mux |
| `/spot1/object_search_reached` | `std_msgs/msg/Bool` | WildOS | Goal Mux |
| `/spot1/object_search_completed` | `std_msgs/msg/Bool` | Goal Mux | WildOS、target fusion |
| `/spot1/object_search_status` | `object_search_msgs/msg/ObjectSearchStatus` | Goal Mux | Planner |
| `/spot1/object_target_estimate_viz` | `visualization_msgs/msg/Marker` | target fusion | RViz |
| `/spot1/object_target_particles` | `sensor_msgs/msg/PointCloud2` | target fusion | RViz |

`object_search_reached` 是视觉证据, `object_search_completed` 才是最终完成状态

`object_search_status` 的 `state` 使用固定 enum, `pending_protection` 明确控制 Planner 的探索失败计时冻结

运行时切换目标:

```bash
ros2 topic pub --once /spot1/object_search_target \
  std_msgs/msg/String "{data: 'red fire extinguisher'}"
```

配置文件中的 `object_search_config.text_queries` 仅作为启动默认值

## 7. 三相机同步

当前实机视觉配置:

- 相机发布 15 Hz
- 视觉处理限制为 10 Hz
- 三相机同步队列为 2
- 三相机最大时间差为 0.2 s
- odom 匹配最大时间差为 3.0 s
- NavigationGraph 最大年龄为 6.0 s
- CameraInfo 使用每路最新有效帧
- TF 使用三路图像中间时间查询

这些较宽的 odom 和 graph 容差用于当前跨机联调, 网络稳定后应根据实测消息年龄继续收紧

## 8. 常用检查

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic hz /spot1/cloud_registered_local
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /spot1/nav_graph
ros2 topic hz /spot1/scored_nav_graph
ros2 topic info -v /spot1/object_search_target
ros2 topic info -v /cloud_registered
ros2 run tf2_ros tf2_echo odom base_link
```

容器健康不等于 topic 数据正确, 必须单独检查类型、频率、时间戳、frame 和发布者数量

## 9. 维护规则

- 同一语义只能有一个发布 owner
- 大点云避免重复跨机订阅
- debug topic 无订阅者时不构造高成本消息
- topic 或发布者变化后同步更新本文和 `robot` profile
