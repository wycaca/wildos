# Agent README

本文帮助开发人员和 agent 快速确认 WildOS 的当前目标、有效主链路、参数边界和研究基线

本轮清理的逐文件执行结果见 `docs/2026-07-15/2026-07-15-current-code-cleanup-audit.md`

## 项目定位

本仓库按研究仓库维护，包含当前 WildOS 主线、可复现实验基线和少量独立调试工具

当前主线能力:

- 从 elevation/2.5D GridMap 构建带高度的稀疏 `NavigationGraph`
- 持久化 graph 节点、边、探索覆盖、frontier 和机器人锚点
- 使用三相机 ExploRFM 结果给 graph frontier 评分
- 使用多视角粒子滤波融合目标位置，可选使用 LiDAR 近距离细化
- 由 `ObjectSearchGoalMux` 统一选择探索目标、融合目标和停止目标
- 由 `graphnav_planner` 输出可执行路径

当前主线不包含:

- 自研 2D `OccupancyGrid` 建图 fallback
- 旧视觉射线粗定位和 `/spot1/object_search_target_pose`
- 旧 batch triangulation 演示实现
- 底盘 `cmd_vel` 控制

## 唯一集成主链路

```text
PointCloud2
  -> pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> GridMap
  -> graph_construction
  -> /spot1/nav_graph
  -> WildOS visual scoring
     -> /spot1/scored_nav_graph -> graphnav_planner -> path
     -> /spot1/object_mask -> object_target_fusion
        -> /spot1/object_target_estimate -> object_search_goal_mux
  -> goal_pose
```

职责边界:

- `graph_construction` 只消费 GridMap，不再支持 `OccupancyGrid`
- `wildos` 负责视觉评分、目标 mask 和到达证据，不发布独立粗目标 pose
- `object_target_fusion` 是唯一目标位置融合 ROS 节点
- `TargetParticleFilter` 是唯一保留的纯目标融合算法
- `ObjectSearchGoalMux` 是最终完成状态的唯一 owner
- `path_follower_node` 保留为可选实验组件，不在默认集成 launch 中启动

## 启动入口

默认入口:

```bash
./scripts/start_wildos_elevation.sh
```

常用方式:

```bash
./scripts/start_wildos_elevation.sh
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh do_object_search:=true
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_elevation.sh do_object_search:=true
```

脚本最终启动:

```text
graph_construction/launch/elevation_visual_navigation_sim.launch.py
```

单组件调试入口:

```bash
ros2 launch visual_navigation wildos_component.launch.py do_object_search:=true
ros2 launch graphnav_planner graphnav_planner.launch.py
ros2 launch graphnav_planner path_follower.launch.py
```

`path_follower.launch.py` 只能在明确需要 `path -> goal_pose` 适配时单独启动，默认集成链路不使用它

## Topic Profile

平台差异统一维护在:

```text
graph_construction/configs/topic_profiles.yaml
```

内置 profile:

- `isaac`, Isaac Sim 5.1 Go2，默认 domain 3
- `unity`, Unity 仿真，默认 domain 89
- `robot`, 真实机器人占位配置

profile 只保存平台相关内容:

- ROS domain 和 RMW
- namespace、frame 和 TF 约定
- 点云、odom、相机、graph、目标搜索和 planner topic
- 平台相关的 object search 策略覆盖

不要向 profile 重新加入 2D grid、旧粗目标 pose 或 planner 内部算法参数

## 参数单一来源

| 范围 | 单一来源 |
|---|---|
| 平台 topic、frame、通信环境 | `graph_construction/configs/topic_profiles.yaml` |
| GridMap 解码和 graph ROS 适配 | `graph_construction/configs/graph_construction_elevation.yaml` |
| 稀疏图算法默认值 | `GraphBuilderConfig` |
| WildOS 视觉算法 | `visual_navigation/configs/wildos_nav_*.yaml` |
| Goal Mux 策略 | `visual_navigation/configs/object_search_goal_mux.yaml` |
| Planner 算法 | `graphnav_planner/config/planner.yaml` |

已固化、不再暴露的旧参数:

- `grid_input_type`
- `free_threshold` 和 `obstacle_threshold`
- GridMap transpose 和 flip 开关
- `initial_goal_mode`
- `object_reached_require_target_distance`
- `append_virtual_goal_to_path`
- `trav_class`

## Graph Construction 结构

```text
graph_construction/graph_construction/
  node.py                 ROS 参数、订阅、发布和 timer
  grid_adapter.py         GridMap 解码和后处理
  grid_types.py           ClassifiedGrid 和空间查询
  graph_builder.py        稀疏图纯算法入口
  graph_memory.py         持久 graph 状态
  frontier_detector.py    frontier 检测和 owner 分配
  edge_builder.py         当前边生成和历史边校验
  msg_utils.py            GraphState 到 ROS message
  viz.py                  graph 和地图 marker
  pointcloud_axis_adapter.py
```

实现原则:

- 纯算法层不导入 `rclpy` 或 ROS message
- ROS message 解码集中在 `grid_adapter.py`
- ROS message 生成集中在 `msg_utils.py`
- rolling GridMap 的坐标、层布局和 frame 约定必须有测试或运行证据
- 运行期诊断使用 ROS 日志和 topic 工具，不把内部 stage timing 混入核心返回类型

## 目标搜索结构

```text
wildos/nav.py
  -> ObjectMaskWithTf
  -> object_target_fusion.py
     -> triangulation3d/target_particle_filter.py
     -> TargetEstimate
  -> object_search_goal_mux.py
     -> exploration goal or stable target goal
     -> completion latch
```

`ObjectMaskWithTf` 只携带融合实际需要的数据:

- mask image
- measurement header
- camera transform
- camera intrinsics
- per-camera score

目标融合状态语义:

- 单视角只建立粒子，不替换初始探索目标
- 两个有效视角后可发布视觉跟踪目标
- 稳定多视角结果可成为导航目标
- LiDAR 只做可选细化，不是视觉链路成立的前提
- 完成事件必须同时满足稳定目标和距离约束

## 研究基线

以下目录保留，但不属于默认部署主线:

- `visual_navigation/lrn/`
- `visual_navigation/imgfrontier_nav/`
- `visual_navigation/geofrontier_nav/`
- `explorfm_trainer/`
- `test_explorfm_folder.py`
- GPS 记录和可视化工具

维护要求:

- 基线必须保持可导入、可构建
- 基线不得依赖固定机器路径
- 基线入口不得混入默认 elevation 集成 launch
- 公共消息变更时必须同步基线调用方

## 构建和验证

从工作区构建:

```bash
cd /mnt/hhd/han/wildos_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to \
  object_search_msgs triangulation3d visual_navigation \
  graph_construction graphnav_planner --symlink-install
```

核心单元测试覆盖:

- GridMap adapter
- sparse graph、persistent graph 和 frontier
- target particle filter
- object detection filter 和 reached evidence
- goal mux
- object target fusion

## 文档规则

- 当前架构以本文件、两个 `2026-06-22` 原理文档和清理审计文档为准
- 其他日期目录是历史实现记录，允许描述已经删除的 2D 或旧 triangulation 方案
- 不要为了匹配当前代码回写历史 changelog
- 修改代码后优先更新当前入口、参数单一来源和逐文件清单
