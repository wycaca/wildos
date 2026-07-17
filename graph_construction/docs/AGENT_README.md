# Agent README

本文帮助开发人员和 agent 快速确认 WildOS 的当前目标、有效主链路、参数边界和研究基线

代码清理记录见 `docs/2026-07-15/2026-07-15-current-code-cleanup-audit.md`

目标定位偏移修复见 `docs/2026-07-16/2026-07-16-object-target-localization-hardening.md`

目标搜索日志整理见 `docs/2026-07-16/2026-07-16-object-search-log-clarification.md`

论文风格可视化和 RViz 性能优化见 `docs/2026-07-16/2026-07-16-paper-style-visualization-and-rviz-performance.md`

双架构 Docker 和 Compose 部署见 `docs/2026-07-16/2026-07-16-wildos-docker-deployment.md`

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

未显式指定时默认使用 `unity` profile

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

## 容器部署

- x86_64 使用 `compose.x86_64.yaml` 和 `docker/Dockerfile.x86_64`
- AGX Orin 使用 `compose.orin.yaml` 和 `docker/Dockerfile.orin`
- Orin 默认基线是 JetPack 6.2 和 L4T 36.4，不得跨 JetPack 复用 iGPU 基础镜像
- Compose 必须使用 host network，跨容器或跨主机 ROS2 通信不得改成普通 Docker bridge
- 完整镜像必须包含 RADIO、Frontier、Traversability 和 SigLIP2 权重
- Unity、Isaac Sim、传感器驱动和底盘控制不属于 WildOS 容器，由外部 ROS2 系统提供 topic

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

## 重复问题：初始探索路线和脚下点云盲区

这两个问题相互影响，是 Unity 目标搜索的长期高风险项。其中“初始时先向机器人后方探索，走一段后又回到前方”已多次修改，不得仅根据单次编译或短时仿真宣布彻底解决

已确认的因果链:

1. Livox 近场盲区会让初始 odom 投影点周围出现 unknown 空洞
2. current node 若被近邻采样点代替，路径首段可能从侧后方开始
3. 旧 planner 只用路径第一个超过 0.25m 的分段判定初始方向，会把“短暂绕行后整体向前”误判为后向分支
4. 所有候选被误判后，旧逻辑会从 `initial_heading_blocked` 无条件落到 `global_best`，从而立即选择后方
5. 分支创建后 12 秒就开始判定无进展，原地转向和初始建图时间会被误计为死路

当前必须保持的修复不变量:

- `SparseGraphBuilder` 只修补 `robot_blind_zone_radius` 内的 unknown，高程优先取与 odom 预期地面一致的最近地面中位数，无可信样本时才使用 odom 高度偏移，明确 obstacle 绝不得覆盖
- 脚下修补区是孤立 free 小岛时，可跨越无明确障碍的 unknown 盲区引导采样外围 free 分量，但不得将整个盲区改成 free
- `NavigationGraph.current_node_idx` 优先对应跟随机器人的 anchor，不得恢复为普通近邻采样点
- 初始前向候选按整条路线的 frontier 前向进展和最大回退量判定，方向使用 frontier points 质心而不是 owner 节点，不得只看路径首段
- 前向分支未确认阻塞时，不得无条件 fallback 到后方 `global_best`
- 只有连续 `directional_block_confirm_timeout` 没有可用前向路线，或已提交的前向分支确认失效/无进展，才允许尝试其他方向
- 新分支必须先经过 `frontier_progress_start_grace`，再使用 `frontier_progress_timeout` 判定无进展
- 历史分支恢复必须使用当前位置重新计算前进量、回退量和路线代价，旧 `discovery_direction` 只能作为无实时方向时的补充，不能覆盖实时指标
- 分支确认失效或无进展后必须写入多条失败走廊记录，同一位置附近且轴线一致的新 UUID 继承冷却，不得只记住最近一次失败
- 活动分支延伸必须用当前节点到新 Frontier 的简单路径替换执行路线，不得向历史缓存路线继续追加并形成回头路径
- 单帧 edge 消失不得立即释放活动分支，必须持续超过 `path_invalid_confirm_duration`

相关参数:

| 参数 | 默认值 | 语义 |
|---|---:|---|
| `robot_blind_zone_radius` | 0.8m | 只允许修补的机器人脚下半径 |
| `robot_blind_zone_elevation_search_radius` | 6.0m | 周边地面高程搜索半径，只取最近边缘样本 |
| `robot_ground_height_offset` | 0.22m | base odom 高度到预期脚下地面的偏移 |
| `robot_ground_elevation_tolerance` | 0.5m | 周边高程相对预期地面的可信偏差 |
| `directional_min_forward_progress` | 0.5m | 前向候选相对当前位置必须具有的最小进展 |
| `directional_max_initial_backtrack` | 2.0m | 整条路线允许的短暂最大回退 |
| `directional_block_confirm_timeout` | 5.0s | 没有前向候选时的持续确认时间 |
| `frontier_progress_start_grace` | 20.0s | 转向和初始起步的宽限时间 |
| `frontier_progress_timeout` | 12.0s | 宽限后路径弧长无增长的超时时间 |
| `frontier_failure_cooldown` | 60.0s | 失败走廊首次冷却时间，重复失败按次数延长 |
| `frontier_failure_merge_radius` | 2.5m | 新 UUID 继承附近失败走廊记录的空间半径 |
| `path_invalid_confirm_duration` | 1.5s | 路径边持续缺失后才确认失效的时间 |

回归测试不得删除:

- `test_graph_builder_repairs_robot_blind_zone_from_nearby_ground`
- `test_graph_builder_uses_robot_anchor_as_current_node_on_known_ground`
- `test_graph_builder_rejects_implausible_high_surface_near_blind_zone`
- `test_graph_builder_samples_outer_free_component_after_blind_zone_repair`
- `DirectionalSelection.UsesWholeRouteWhenFirstSegmentPointsBackward`
- `DirectionalSelection.ConfirmsForwardBlockBeforeSelectingRearBranch`
- `DirectionalSelection.UsesFrontierPointsWhenOwnerNodeIsBehind`
- `CommittedBranch.StartGracePreventsPrematureNoProgressRecovery`
- `CommittedBranch.ReleasesBranchWhenCommittedEdgeDisappears`
- `CommittedBranch.FailedCorridorsSuppressNearbyUuidAliases`
- `CommittedBranch.DeferredHandoffRebuildsPathWithoutReturningToOldTail`

验收时必须在 Unity 完整跑行 `./scripts/start_wildos_elevation.sh do_object_search:=true`，至少检查首个 graph 的脚下修补统计和地面高度、首次分支选择为 `reason=initial_forward_route`，以及启动宽限期内不出现 `reason=no_path_progress`。2026-07-16 的 Unity 验证首帧为 34 节点、45 边、19 frontier，修补 49 个 cell，地面投影正常，首次规划直接进入 `initial_forward_route`。同日长时间运行又确认旧恢复逻辑会在 `(-3.50,-9.50)` 和 `(-3.50,-8.30)` 附近循环，实时 Path 出现先到 `-9.50` 再返回 `-8.30` 的回头路线，现已增加失败走廊继承、实时恢复门控、简单路径替换和路径失效去抖，但仍需重启后的长时间闭环验证，因此该问题继续按“未完全解决”管理

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
- Unity 粒子最大射线深度为 30m，避免近场任务扩散到无效远端
- LiDAR mask 内优先选择最近有效前景簇，避免背景墙覆盖目标表面
- 已有视觉轨迹时单帧 LiDAR 不改变位置、置信度或 source，连续两帧一致后才更新和锁定
- Goal Mux 会拒绝 frame、距离、高度、协方差或置信度异常的粗目标
- 稳定目标使用独立的 0.3m 门槛继续纠偏，不会冻结在首次稳定位置
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

## 可视化和 RViz 性能

- 论文风格显示配置统一维护在 `graph_construction/rviz/wildos_paper.rviz`
- GridMap 使用固定浅灰色显示, `elevation` 仍负责真实表面高度
- `/spot1/graph_construction_viz` 的红线是导航图边, 默认保留
- `free_radius` 和 `explored_radius` 圆只用于专项调试, 默认关闭
- `/spot1/object_target_estimate_viz` 同时显示目标球和绿色观测射线, 不增加重复作用的目标可视化话题
- 论文风格 RViz 默认不显示 LiDAR PointCloud2, 也不发布专用降采样点云 topic
- `/livox/lidar` 和 `/livox/lidar_aligned` 继续供算法使用, 不得为了显示性能降低建图输入质量
- 临时排查原始点云时可以手动添加 PointCloud2 Display, 排查完成后应关闭

## 构建和验证

`colcon` 默认在当前工作目录生成 `build/`、`install/` 和 `log/`。不得在 `src/nebula2-wildos` 仓库目录内直接运行 `colcon build`，否则会错误生成 `src/nebula2-wildos/build`、`src/nebula2-wildos/install` 和 `src/nebula2-wildos/log`，且这些产物不得用于启动或部署

构建前必须先切换到 ROS workspace 根目录并检查当前路径。`/home/ks-server3/han` checkout 使用:

```bash
cd /home/ks-server3/han/wildos_ws
test -d src/nebula2-wildos
pwd
source /opt/ros/humble/setup.bash
colcon build --packages-up-to \
  object_search_msgs triangulation3d visual_navigation \
  graph_construction graphnav_planner --symlink-install
```

`/mnt/hhd/han` checkout 使用:

```bash
cd /mnt/hhd/han/wildos_ws
test -d src/nebula2-wildos
pwd
source /opt/ros/humble/setup.bash
colcon build --packages-up-to \
  object_search_msgs triangulation3d visual_navigation \
  graph_construction graphnav_planner --symlink-install
```

正确产物只能位于对应 workspace 根目录:

```text
wildos_ws/build
wildos_ws/install
wildos_ws/log
```

如果仓库目录下已经出现错误的 `build/install/log`，先确认当前运行进程没有引用这些目录，再单独清理。禁止为了省事 source 仓库目录内的错误 `install/setup.bash`

核心单元测试覆盖:

- GridMap adapter
- sparse graph、persistent graph 和 frontier
- target particle filter
- object detection filter 和 reached evidence
- goal mux
- object target fusion

## 文档规则

- 当前代码是实现行为的最终依据, 本文件维护当前架构、有效主链路和长期约束
- 两个 `2026-06-22` 原理文档维护当前核心原理, 只有原理变化或内容错误时才修改
- 所有日期目录文档都是对应修改当时的专项记录, 清理审计与其他日期文档地位相同
- 每个独立代码修改都必须在当天日期目录下建立独立主题文档, 记录问题、原因、修改文件、行为边界和验证结果
- 同一修改从方案分析进入代码实现时可以继续完善同一份文档, 实现完成后补充实际结果
- 后续无关修改不得追加到已有专项文档, 即使涉及同一模块也应建立新的日期文档
- 历史文档允许描述已经删除的 2D 或旧 triangulation 方案, 不要为了匹配当前代码回写历史记录
- 修改形成新的长期架构约束、排障规则或高风险不变量时, 同步更新本文件
