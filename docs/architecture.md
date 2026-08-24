# WildOS 架构

## 目标

WildOS 在未知环境中完成定位、局部高程建图、持久拓扑记忆、开放词汇目标搜索和路径规划

```text
x86: MID360 + IMU -> D-LIO -> /cloud_registered + /odom + /tf
AGX: 点云 -> elevation GridMap -> NavigationGraph -> 视觉评分 -> Planner -> Path
AGX: 三路相机 -> 目标 Mask -> 多视角融合 -> Goal Mux -> Planner
```

Planner 只发布 `nav_msgs/msg/Path`, 仓库外控制器负责局部避障、速度限制、急停和运动执行

## 主机和模块

| 位置 | 模块 | 职责 |
| --- | --- | --- |
| x86 | Livox、D-LIO | 原始传感器、连续位姿、注册点云和动态 TF |
| AGX | RealSense | 前、左、右三路图像、内参和静态 TF |
| AGX | elevation mapping | 注册点云到局部 `GridMap` |
| AGX | `graph_construction` | 持久安全拓扑和 Frontier |
| AGX | `visual_navigation` | ExploRFM 评分、目标 Mask 和高层目标状态 |
| AGX | `triangulation3d` | 多视角目标粒子滤波 |
| AGX | `graphnav_planner` | 图路径、探索记忆和 Path |

## 主数据契约

Topic 名称以 `graph_construction/configs/topic_profiles.yaml` 的 `robot` profile 为准

| 数据 | Topic | Owner |
| --- | --- | --- |
| 注册点云 | `/cloud_registered` | x86 D-LIO guard |
| canonical odom | `/spot1/odom_for_scoring` | AGX odom adapter |
| 高程图 | `/elevation_mapping_node/elevation_map_raw` | elevation mapping |
| 几何导航图 | `/spot1/nav_graph` | graph construction |
| 视觉评分图 | `/spot1/scored_nav_graph` | WildOS |
| 目标 Mask | `/spot1/object_mask` | WildOS |
| 目标估计 | `/spot1/object_target_estimate` | target fusion |
| 搜索状态 | `/spot1/object_search_status` | Goal Mux |
| 执行路径 | `/spot1/graphnav_planner/path` | Planner |

目标运行时入口是 `/spot1/object_search_target`. `object_search_reached` 是视觉证据, `object_search_completed` 才是最终任务状态

## Frame 和时间

```text
odom -> dlio_odom -> base_link -> lidar_link
                             -> imu_link
                             -> camera links -> optical frames
```

- x86 D-LIO 是 odom、动态 TF 和注册点云的统一位姿源
- 同一 TF child frame 只能有一个 owner
- 不能只改 `frame_id` 冒充坐标变换
- 图像、点云、odom 和 TF 保留测量时间
- WildOS 只使用测量时刻 TF, 实机不回退到最新 TF

## 不可破坏的安全边界

- `free` 是当前确认可通行, `obstacle` 优先于历史和人工先验
- `unknown` 不允许生成新边, 但不直接否定已确认的历史安全边
- 图节点只生成在机器人可达的 free 分量
- 新障碍必须删除冲突节点和边
- 定位异常时停止不可信输出, 污染地图后重启 elevation mapping
- 单视角目标只有方向证据, 不能直接作为导航坐标
- Goal Mux 是高层目标和最终完成状态的唯一 owner
- debug topic 无订阅者时不构造高成本消息
- 不新增职责重复的 launch、Compose 或 adapter

## 配置入口

| 内容 | 文件 |
| --- | --- |
| Topic、frame 和 DDS | `graph_construction/configs/topic_profiles.yaml` |
| D-LIO | `graph_construction/configs/dlio/mid360.yaml` |
| 高程图 | `graph_construction/configs/elevation_mapping.yaml` |
| 导航图 | `graph_construction/configs/graph_construction_elevation.yaml` |
| 视觉与检测 | `visual_navigation/configs/wildos_nav_conf.yaml` |
| Goal Mux | `visual_navigation/configs/object_search_goal_mux.yaml` |
| Planner | `graphnav_planner/config/planner.yaml` |

参数默认值和当前数值直接查看这些配置, 文档不复制完整参数表
