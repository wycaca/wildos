# Agent README

本文说明 `sim2real` 分支当前实机架构、运行入口、约束和未完成事项

历史过程保存在日期目录, 当前行为以 `docs/details/` 和代码为准

## 1. 文档入口

| 内容 | 文档 |
|---|---|
| 系统总览 | `docs/details/overview.md` |
| 实现原则 | `docs/details/principles.md` |
| 导航图更新 | `docs/details/graph_update.md` |
| D-LIO 和 TF | `docs/details/dlio.md` |
| Topic 契约 | `docs/details/topics.md` |
| 环境配置 | `docs/details/environment.md` |
| Docker 部署 | `docs/details/docker_deployment.md` |
| 当前问题 | `docs/details/issues.md` |
| 目标探索 | `docs/details/target_exploration.md` |
| 目标定位 | `docs/details/target_localization.md` |

## 2. 当前实机目标

1. x86 主机接入 MID360 和内置 IMU, 运行 D-LIO
2. 相机 AGX 接入三台 RealSense, 运行高程图、图构建、视觉和 Planner
3. 从局部 elevation `GridMap` 构建持久稀疏导航图
4. 使用三路相机为 Frontier 评分并检测目标
5. 使用多视角 Mask 和 LiDAR 估计目标位置
6. 输出 `/spot1/graphnav_planner/path` 给仓库外运动执行模块

当前主线已经移除:

- 旧 2D `OccupancyGrid` 后端
- 默认链路中的 `cmd_vel` 控制
- 旧视觉射线粗定位和 batch triangulation 演示

## 3. 当前部署架构

```text
MID360
  -> x86 lidar 容器
  -> /livox/lidar + /livox/imu
  -> x86 localization 容器
  -> D-LIO
  -> /cloud_registered + /odom + /tf

2 台 D435i + 1 台 D435if
  -> 相机 AGX cameras 容器
  -> 三路图像 + CameraInfo + 相机 TF

x86 定位输出 + 三路相机
  -> 相机 AGX wildos 容器
  -> 高程图
  -> NavigationGraph
  -> 视觉评分和目标融合
  -> graphnav_planner
  -> Path
```

两台主机统一使用:

```dotenv
ROS_DOMAIN_ID=2
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 4. 运行入口

x86:

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  up -d
```

相机 AGX:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  up -d
```

容器外调试入口仍为 `scripts/start_wildos_elevation.sh`, 实机必须显式使用 `robot` profile 和 `use_sim_time:=false`

`elevation_visual_navigation_sim.launch.py`、`elevation_mapping_sim.yaml` 等名称是历史兼容命名, 当前 Docker 实机链路仍会加载它们

## 5. 模块职责

- `lidar` 容器只运行 Livox 驱动
- `localization` 容器只运行 D-LIO 和 canonical 输出适配
- `cameras` 容器只运行三台 RealSense 和相机静态 TF
- `wildos` 容器运行点云适配、高程图、导航图、视觉、目标融合和 Planner
- `graph_construction` 只消费 elevation `GridMap`
- `ObjectSearchGoalMux` 是高层目标和最终完成状态的唯一 owner
- `graphnav_planner` 只生成路径, 不控制底盘

## 6. 当前实机契约

### 6.1 定位和点云

- x86 D-LIO 订阅 `/livox/lidar` 和 `/livox/imu`
- x86 发布 `/cloud_registered`、`/odom` 和 `/tf`
- `/cloud_registered` 的 frame 为 `dlio_odom`
- 相机 AGX 只跨机订阅一次 `/cloud_registered`
- `pointcloud_axis_adapter` 以 2 Hz 发布 `/spot1/cloud_registered_local`
- elevation mapping 消费 `/spot1/cloud_registered_local`
- canonical odom 为 `/spot1/odom_for_scoring`

### 6.2 三相机

- 前相机为 D435if
- 左右相机为 D435i
- 相机驱动发布 15 Hz 彩色图像
- WildOS 视觉处理限制为 10 Hz
- 相机序列号由 `.env.orin.wildos-cameras` 固定
- 当前外参是近似值, 精确标定仍是 TODO

### 6.3 高程图和启动盲区

当前配置服务于小推车实机:

| 参数 | 当前值 |
|---|---:|
| `initialize_tf_offset` | -0.90 m |
| `initialize_tf_grid_size` | 1.0 m |
| `dilation_size_initialize` | 2 cell |
| `robot_blind_zone_radius` | 0.8 m |
| `robot_blind_zone_elevation_search_radius` | 2.0 m |
| `max_height_range` | 1.0 m |

启动先验只修补脚下到近场真实地面的连通 unknown, 不能越过墙体填充外围区域

实机验证中人工修补约从 9258 格降至 296 格, 并连接到约 1.8 m 外的真实地面

## 7. 必须保持的约束

- 唯一几何输入是 elevation `GridMap`
- unknown 不能直接删除历史安全路线
- obstacle 必须删除冲突节点和边
- 新节点只在机器人可达的 free 区域生成
- 启动人工区域固定在初始世界坐标, 不能跟随机器人移动
- 人工区域不能覆盖 obstacle 或穿过墙体
- 节点 UUID 必须稳定
- 内部增量更新, 对外发布完整 `NavigationGraph`
- odom、TF 和注册点云必须来自同一 D-LIO 位姿源
- 同一个 TF child frame 只能有一个 owner
- 定位发散并污染高程图后必须重启地图

## 8. 当前未完成事项

- 标定 MID360 倾斜 7 度和三相机精确外参
- 验证小推车转弯、急停和震动时 D-LIO 不发散
- 验证启动局部先验在墙边、桌边和狭窄通道不会越界
- 完成三路视觉评分、目标融合和最终 `REACHED` 的实机闭环
- 记录跨机点云带宽、消息年龄和持续负载
- 完成不少于 10 分钟的实机稳定性测试

## 9. 配置来源

| 配置 | 文件 |
|---|---|
| 实机 topic、frame 和 DDS | `graph_construction/configs/topic_profiles.yaml` 的 `robot` profile |
| x86 D-LIO | `graph_construction/configs/dlio/mid360.yaml` |
| 高程图 | `graph_construction/configs/elevation_mapping_sim.yaml` |
| 图构建 | `graph_construction/configs/graph_construction_elevation.yaml` |
| 实机视觉 | `visual_navigation/configs/wildos_nav_conf.yaml` |
| x86 Compose 环境 | `.env.x86_64.lidar-dlio` |
| 相机 AGX Compose 环境 | `.env.orin.wildos-cameras` |

不要在实机分支新增第二套功能重复的 launch 或 Compose

## 10. 修改后验证

1. 运行受影响模块的最小测试
2. 执行 `git diff --check`
3. 验证 Compose 能正常展开
4. 重建受影响镜像
5. 检查源 topic、时间戳和 TF
6. 检查高程图、导航图、视觉输出和 Path
7. 性能或状态修改至少运行 10 分钟
8. 保持本地、x86 和相机 AGX 处于同一提交

## 11. 文档维护

- `docs/details/` 只描述当前实机实现
- 日期目录保留历史过程, 不作为部署依据
- topic、参数、容器职责或启动顺序变化后同步更新对应 details 文档
- 不再把仿真参数复制回当前实机说明
