# WildOS Topic 契约

> 本文说明 topic 的功能角色, 具体名称以 `graph_construction/configs/topic_profiles.yaml` 为准

## 1. 默认入口

```bash
./scripts/start_wildos_elevation.sh do_object_search:=true
```

默认使用 `unity` profile

本文不包含 ROS 自动 topic、研究 baseline 和默认不启动的 `path_follower_node`

## 2. 主数据流

### 定位和建图

```text
/livox/lidar + /livox/imu
  -> DLIO
  -> odom + TF + 对齐点云
  -> elevation GridMap
  -> NavigationGraph
```

platform backend 可以直接使用平台 odom, 但下游仍统一消费内部 canonical odom

### 视觉评分和规划

```text
NavigationGraph + 三路相机
  -> WildOS
  -> Scored NavigationGraph
  -> Graph Planner
  -> Path
```

### 目标搜索

```text
三路相机
  -> ObjectMaskWithTf
  -> object_target_fusion
  -> TargetEstimate
  -> ObjectSearchGoalMux
  -> goal pose
  -> Graph Planner
```

## 3. Unity 外部输入

| Topic | 类型 | 用途 | 要求 |
|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | DLIO 和建图原始点云 | frame、header 时间和逐点 `timestamp` 有效 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | DLIO IMU | 与 LiDAR 使用同一时钟, 目标频率约 200 Hz |
| `/unity/odom` | `nav_msgs/msg/Odometry` | platform 定位或 DLIO 启动锚定 | pose、frame 和时间连续 |
| 三路 image | `sensor_msgs/msg/CompressedImage` | WildOS 输入 | front、left、right 都必须持续发布 |
| 三路 CameraInfo | `sensor_msgs/msg/CameraInfo` | 相机内参 | 与对应图像 frame 和时间一致 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间 | 所有仿真节点共用 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | 平台 TF | 不能与 DLIO canonical TF 冲突 |

Unity 相机典型输入:

```text
/camera/front/color/image/compressed
/camera/left/color/image/compressed
/camera/right/color/image/compressed
/camera/front/color/camera_info
/camera/left/color/camera_info
/camera/right/color/camera_info
```

## 4. 传感器时间要求

### 点云 header 时间

表示整帧点云的参考时刻, 用于 ROS 同步和 TF 查询

### 点云逐点时间

表示每个点在一次扫描中的实际采样时刻, DLIO 使用它做运动去畸变

header 时间不能替代逐点时间

当前 Unity 发布端已经修复重复时间戳并提供逐点 `timestamp`, 运行前仍需检查字段类型、单位和单帧内是否单调

### 相机时间

三路图像、CameraInfo、LiDAR、odom 和 TF 必须使用同一时间基准

如果 Unity 原始相机时间与 `/clock` 不一致, 保留 `camera_stamp_adapter` 统一时间后再进入 WildOS

## 5. DLIO 模式

启动方式:

```text
localization_backend:=dlio
launch_dlio:=true
```

| Topic | 角色 |
|---|---|
| `/livox/lidar` | DLIO 原始点云输入 |
| `/livox/imu` | DLIO IMU 输入 |
| `/spot1/dlio/odom_node/odom` | DLIO 原始 odom |
| `/spot1/dlio/odom_node/aligned_odom` | 对齐到系统全局 frame 的 odom |
| `/spot1/dlio/odom_node/healthy` | DLIO 健康状态 |
| `/spot1/dlio/odom_node/pointcloud/deskewed_raw` | DLIO 原始输出点云 |
| `/spot1/dlio/odom_node/pointcloud/deskewed` | 健康门控后的下游点云 |
| `/spot1/dlio/odom_node/tf_raw` | DLIO 原始 TF, 只用于诊断 |
| `/spot1/tf` | WildOS 主链使用的统一动态 TF |
| `/spot1/tf_static` | WildOS 主链使用的静态 TF |

关键约束:

- DLIO 直接订阅原始 LiDAR 和 IMU
- 不恢复 Python 高频传感器转发或去重节点
- `odom/publishRate` 只降低输出频率, 不降低 IMU 输入或内部传播频率
- DLIO odom、统一 TF 和对齐点云必须使用同一位姿源
- Unity 真值 TF 与 DLIO TF 使用不同 topic, 避免 frame authority 冲突
- DLIO 健康失败时暂停 canonical odom、TF 和下游点云
- 修复错误定位后重启高程图, 防止旧错误位姿继续留在地图中

坐标对齐、外参方向、TF 发布权和方向差诊断见 [DLIO 坐标与 TF](dlio.md)

## 6. 内部 canonical 输入

| Topic | 类型 | 发布者 | 消费者 | 说明 |
|---|---|---|---|---|
| `/spot1/odom_for_scoring` | `nav_msgs/msg/Odometry` | odom adapter | graph、WildOS、Goal Mux、Planner | 下游唯一 odom |
| `/elevation_mapping_node/elevation_map_raw` | `grid_map_msgs/msg/GridMap` | elevation mapping | graph construction | 唯一几何地图 |
| `/livox/lidar_aligned` | `sensor_msgs/msg/PointCloud2` | pointcloud adapter | platform backend 建图 | 非 DLIO 模式的规范化点云 |
| `/spot1/dlio/odom_node/pointcloud/deskewed` | `sensor_msgs/msg/PointCloud2` | DLIO guard | DLIO 模式建图和目标精修 | DLIO 模式使用 |

`/livox/lidar_aligned` 和 DLIO deskewed cloud 属于两种定位 backend, 同一次运行只应选择正确的一条

## 7. 导航图和视觉评分

| Topic | 类型 | 发布者 | 消费者 | 状态 |
|---|---|---|---|---|
| `/spot1/nav_graph` | `graphnav_msgs/msg/NavigationGraph` | graph construction | WildOS | 必须 |
| `/spot1/scored_nav_graph` | `graphnav_msgs/msg/NavigationGraph` | WildOS | Planner | 必须 |
| `/spot1/graph_construction_viz` | `visualization_msgs/msg/MarkerArray` | graph construction | RViz | 诊断 |
| `/spot1/model_visualization` | `sensor_msgs/msg/Image` | WildOS | 调试工具 | 诊断 |
| `/spot1/within_range_geofrontiers` | `visualization_msgs/msg/MarkerArray` | WildOS | RViz | 诊断 |
| `/spot1/score_rings` | `visualization_msgs/msg/MarkerArray` | WildOS | RViz | 诊断 |

`nav_graph` 是纯几何图, `scored_nav_graph` 是加入视觉分数后的 Planner 输入, 两者不能合并

## 8. 目标搜索 topic

仅在 `do_object_search:=true` 时使用完整链路

| Topic | 类型 | 发布者 | 消费者 | 说明 |
|---|---|---|---|---|
| `/spot1/object_mask` | `ObjectMaskWithTf` | WildOS | target fusion | 确认后的目标 Mask 和测量 TF |
| `/spot1/object_target_estimate` | `TargetEstimate` | target fusion | Goal Mux | 唯一目标位置估计 |
| `/spot1/object_search_reached` | `std_msgs/msg/Bool` | WildOS | Goal Mux | 近距离视觉证据, 不是最终完成 |
| `/spot1/object_search_completed` | `std_msgs/msg/Bool` | Goal Mux | WildOS、target fusion | 最终完成状态 |
| `/spot1/graphnav_goal_pose` | `geometry_msgs/msg/PoseStamped` | Goal Mux | Planner | 当前高层目标 |
| `/spot1/object_search_status` | `std_msgs/msg/String` | Goal Mux | Planner | 当前搜索状态 |
| `/spot1/object_target_estimate_viz` | `visualization_msgs/msg/Marker` | target fusion | RViz | 目标位置诊断 |
| `/spot1/object_target_particles` | `sensor_msgs/msg/PointCloud2` | target fusion | RViz | 粒子分布诊断 |

不能混淆:

- `object_search_reached` 是视觉证据
- `object_search_completed` 是 Goal Mux 门控后的最终状态
- `object_target_estimate` 是算法数据
- `object_target_estimate_viz` 是 RViz 显示

## 9. Planner 输出

| Topic | 类型 | 用途 |
|---|---|---|
| profile 中的 `planner_path_topic` | `nav_msgs/msg/Path` | Planner 原始 graph 路径 |
| `/spot1/graphnav_planner/unexplored_space_map` | `grid_map_msgs/msg/GridMap` | Planner 调试 |
| `/spot1/graphnav_planner/frontier_scores` | `visualization_msgs/msg/MarkerArray` | 最终候选分数调试 |

Planner 在所有 profile 中发布 `/spot1/graphnav_planner/path`, Unity 自研导航消费该路径并输出 `/corrected_path` 给底层控制

Planner、性能监控和 RViz 必须读取同一个 profile 配置, 不能在 launch 中根据 namespace 推导另一个 Path topic

两个 Planner 调试 topic 只在存在订阅者时构造

Planner Path 只在路线实际变化时发布, 晚启动的 `topic echo` 没看到消息不能直接说明 Planner 从未发布

## 10. 三相机同步

WildOS 只对三路相机图像做逐帧同步:

- front、left、right 三路图像

其余输入独立缓存, 不要求与每组图像同时到达:

- CameraInfo 保存每路相机最新一帧, 相机标定不变时可以重复使用
- odom 保存最近 100 帧, 按三路图像的中间时间选择最近一帧, 最大时间差为 0.2 s
- NavigationGraph 保存最新有效帧, 图像时间相对导航图最多允许约 1 s 延迟

任一缓存输入不满足条件时, 只丢弃当前相机组, 后续新图像仍会继续尝试匹配

三相机同步队列为 5, 最大时间差为 0.2 s, 这些参数只作用于图像

TF 使用三路图像的中间时间查询, 避免机器人运动时使用当前位姿替代拍摄时位姿

WildOS 每 30 s 输出一次 DEBUG 诊断, 包括:

- 三路图像、三路 CameraInfo、odom 和 NavigationGraph 的输入频率
- 三相机时间差、相机与最近 odom 时间差、NavigationGraph 年龄
- 相机同步频率和完整输入匹配频率
- 连续多久没有形成相机同步组或完整输入组
- 当前缓存状态和各类拒绝次数

排查顺序:

1. 检查三路图像频率
2. 查看相机同步频率和三相机时间差
3. 检查三路 CameraInfo 是否都进入缓存
4. 检查最近 odom 时间差是否超过 0.2 s
5. 检查 NavigationGraph 是否有效、年龄是否超过 1 s
6. 最后检查 TF 等待和模型处理性能

## 11. Profile 差异

| Profile | 原始点云 | 原始 odom | 相机模板 |
|---|---|---|---|
| `unity` | `/livox/lidar` | `/unity/odom` | `/camera/{}/color/...` |
| `isaac` | `/livox/lidar` | `/odom` | `/unitree_go2/{}_cam/...` |
| `robot` | Ouster profile topic | 机器人 odom | RealSense profile topic |

`{}` 展开为 `front`、`left`、`right`

`robot` profile 仍是占位配置, 真机部署前必须根据实际驱动确认 topic、frame、QoS 和时间同步

## 12. 常用检查命令

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --once
ros2 topic echo /clock --once
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/imu
ros2 run tf2_ros tf2_echo odom_3D base_link
```

修改发布者、remapping 或 profile 后还应检查:

```bash
ros2 topic list -t
ros2 node info <node>
```

## 13. 维护规则

- 每个必须 topic 应有预期消费者
- 同一语义只能有一个 owner
- 不持续发布无人消费的大体积副本
- debug topic 无订阅者时不做高成本构造
- profile 改名后删除旧发布者和订阅者残留
- topic、类型、发布者或消费者变化后同步更新本文
