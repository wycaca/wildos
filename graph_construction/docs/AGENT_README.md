# Agent README

本文帮助开发人员和 agent 快速了解 WildOS 当前主线、启动方式、模块边界和高风险事项

历史修改过程放在日期目录, 当前实现说明放在 `docs/details/`

## 1. 文档入口

| 内容 | 文档 |
|---|---|
| 系统总览 | `docs/details/overview.md` |
| 实现原则 | `docs/details/principles.md` |
| 导航图更新 | `docs/details/graph_update.md` |
| 目标搜索和探索路线 | `docs/details/target_exploration.md` |
| 目标定位 | `docs/details/target_localization.md` |
| Topic 契约 | `docs/details/topics.md` |
| 当前 TODO | `docs/2026-07-23/2026-07-23-todo.md` |

## 2. 文档维护规则

- 当前架构和长期规则写入 `docs/details/`
- 当天问题、方案、修改和测试写入日期目录
- 同一个问题在同一天只维护一份文档
- 同一天多次修改继续更新原文档, 不新建同主题文件
- 跨天继续处理时新建当天记录并引用前一天文档
- 历史日期文档保持当时状态, 不改写成当前实现
- topic、参数、消息或模块职责变化后同步更新对应 details 文档
- 综合 TODO 已覆盖的问题继续更新同一 TODO

## 3. 项目目标

WildOS 当前主线完成以下工作:

1. 使用 LiDAR 和 IMU 或平台 odom 获取机器人位姿
2. 从 elevation `GridMap` 构建持久稀疏导航图
3. 使用三路相机给 Frontier 增加视觉分数
4. 在未知环境中选择探索路线
5. 使用多视角 Mask 和 LiDAR 估计目标三维位置
6. 在探索、目标观察、目标接近和停止之间切换
7. 通过 graph planner 输出 Path

当前不再支持:

- 旧 2D `OccupancyGrid` 建图后端
- 旧视觉射线粗目标 pose
- 旧 batch triangulation 演示
- 默认链路中的底盘 `cmd_vel` 控制

## 4. 当前主链路

```text
LiDAR + IMU
  -> DLIO 或平台定位
  -> elevation GridMap
  -> graph_construction
  -> NavigationGraph
  -> WildOS 三相机评分
  -> Scored NavigationGraph
  -> graphnav_planner
  -> Path

三路相机
  -> ObjectMaskWithTf
  -> object_target_fusion
  -> TargetEstimate
  -> ObjectSearchGoalMux
  -> graphnav goal
```

职责边界:

- `graph_construction` 只消费 GridMap
- WildOS 负责视觉评分、目标 Mask 和近距离视觉证据
- `object_target_fusion` 是唯一目标位置融合 ROS 节点
- `TargetParticleFilter` 是唯一目标粒子算法
- `ObjectSearchGoalMux` 是高层 goal 和最终完成状态的唯一 owner
- `graphnav_planner` 负责图路径, 不负责底盘控制
- `path_follower_node` 只作为可选实验组件

## 5. 默认启动

```bash
./scripts/start_wildos_elevation.sh
```

目标搜索:

```bash
WILDOS_TOPIC_PROFILE=unity \
  ./scripts/start_wildos_elevation.sh do_object_search:=true
```

可用 profile:

- `unity`
- `isaac`
- `robot`, 当前仍是实机占位配置

统一集成 launch:

```text
graph_construction/launch/elevation_visual_navigation_sim.launch.py
```

启动脚本会阻止重复启动第二套同名主链路, 避免旧 DLIO、TF 或 WildOS publisher 残留

## 6. 当前模块状态

| 模块 | 当前结果 | 仍需完成 |
|---|---|---|
| 导航图 | free radius 稀疏节点和局部全 pair 更新完成 | Unity 10 分钟性能回归 |
| 启动观察 | 4.0 m 初始化阶段保守修补和条件扫描完成 | Unity 验证修补安全性和扫描执行 |
| 探索路线 | 能持续探索和死路恢复 | 减少普通路线回头和频繁切换 |
| 视觉检测 | 0.120/0.110 双门槛完成 | 无目标 10 分钟误检测试 |
| 目标定位 | 重复帧过滤和视角软权重完成 | 排查 Mask 延迟和 LiDAR 精修率 |
| 最终完成 | 最终观察、换位和 `REACHED` 门控代码完成 | Unity 完整闭环验收 |
| DLIO | 仿真主链路已接通 | 排查相对参考 odom 的累计方向差 |

文档记录的最新分模块测试:

- graph 137 项通过
- 粒子滤波 9 项通过
- 目标链路 54 项通过, 1 项环境相关测试跳过
- Goal Mux 29 项通过
- Planner 路线策略 28 项和分支记忆 3 项通过
- DLIO 相关 37 项通过

这些结果是自动化和构建验证, 不能替代本轮修改后的 Unity 完整运行

## 7. 导航图关键约束

- 唯一几何输入是 elevation `GridMap`
- unknown 不能直接删除历史安全路线
- 新障碍必须删除冲突节点和边
- 新节点只在机器人可达的新 free 区域生成
- 当前节点使用附近安全普通节点
- unknown 或 obstacle 中不能创建兜底节点
- 旧的移动 anchor 和 breadcrumb 逻辑不得恢复
- 脚下 unknown 修补半径当前为 4.0 m, 只在第一帧有效地图初始化时执行
- 节点 UUID 必须稳定
- 内部增量更新, 对外仍发布完整 `NavigationGraph`

## 8. 目标搜索关键约束

- 启动时先等待地图和评分图, 不默认旋转 360 度
- 单视角 `PENDING` 不能接管导航
- 粗目标先引导机器人到安全观察位置
- 目标丢失时只做小角度重捕获
- 定位不稳定时通过横向移动获得新视差
- 0.3 m 横向基线和 3 度夹角是满质量参考, 不是双重硬门槛
- 单帧 LiDAR 不能覆盖已有视觉目标
- 到达目标坐标附近不等于完成
- `REACHED` 必须经过 Goal Mux 的稳定目标、距离和视觉证据门控

## 9. DLIO 和时间同步

- 真机默认使用 DLIO 或经过同等验证的 6DoF LiDAR-inertial odometry
- Unity 和 Isaac Sim 可以使用 DLIO
- Unity 真值 odom 只用于启动锚定和评测
- Unity DLIO 输入为 `/livox/lidar` 和 `/livox/imu`
- DLIO 直接订阅原始传感器, 不使用 Python 高频转发节点
- Unity 发布端已经修复重复时间戳并提供逐点 `timestamp`
- `/livox/imu` 目标频率为 200 Hz, 启动后仍需用 topic 实测确认
- 点云 header 时间用于帧同步, 逐点时间用于运动去畸变
- DLIO 输出频率可以降低, 不能降低 IMU 输入和内部传播频率
- Unity 真值 TF 和 DLIO TF 必须隔离
- canonical odom、TF 和点云必须来自同一 DLIO 位姿源
- DLIO 健康失败时暂停下游输出
- 错误位姿污染高程图后必须重启地图

当前仍需排查 DLIO 与 Unity 参考 odom 约 10 至 16.6 度的累计方向差

## 10. Sim-to-Real 风险

论文实机使用 Spot、Ouster OS0-128、VectorNav VN-100 和三台 RealSense D455

论文计算分工:

- Intel NUC i7 运行 DLIO 和 Nav2
- Jetson AGX Orin 运行高程图和 WildOS
- 论文没有注明 NUC 具体型号和 Orin 显存版本

真机部署必须重点验证:

- 三相机、LiDAR 和 IMU 使用统一硬件时间
- LiDAR 每个点有真实采样时间
- LiDAR、IMU 和三相机外参完成现场标定
- 三路图像同步不会长期等待最慢相机
- USB、网口和 DDS 不产生持续积压
- CPU、GPU、显存、温度和功耗满足持续运行
- DLIO 转向、快速行走和震动场景不发散
- 最终目标搜索可以进入 `REACHED`

`robot` profile 在完成时间同步、外参、动态定位和完整任务验收前不能视为可部署状态

## 11. 参数来源

| 配置范围 | 文件 |
|---|---|
| topic、frame、ROS domain | `graph_construction/configs/topic_profiles.yaml` |
| 高程图解码和 graph ROS 参数 | `graph_construction/configs/graph_construction_elevation.yaml` |
| graph 算法 | `GraphBuilderConfig` |
| WildOS 模型和视觉阈值 | `visual_navigation/configs/wildos_nav_*.yaml` |
| Goal Mux | `visual_navigation/configs/object_search_goal_mux.yaml` |
| Planner | `graphnav_planner/config/planner.yaml` |

不要把平台差异复制到新的 launch

## 12. 构建和测试

必须从 ROS workspace 根目录构建, 不要在仓库目录内生成 `build/`、`install/` 和 `log/`

当前 checkout:

```bash
cd /mnt/hhd/han/wildos_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to \
  object_search_msgs triangulation3d visual_navigation \
  graph_construction graphnav_planner --symlink-install
```

修改后按影响范围执行:

1. 对应单元测试
2. flake8 和 pydocstyle 项目兼容检查
3. ROS 包构建
4. Unity 场景测试
5. 性能修改运行至少 10 分钟
6. 退出流程检查残留进程

## 13. 研究基线

以下内容保留用于论文和算法对照, 不属于默认部署主线:

- `visual_navigation/lrn/`
- `visual_navigation/imgfrontier_nav/`
- `visual_navigation/geofrontier_nav/`
- `explorfm_trainer/`
- GPS 记录和可视化工具

公共消息变化时仍需检查这些调用方能否构建
