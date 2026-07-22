# WildOS 当前系统概览

> 本文随当前实现长期维护，代码行为变化时必须同步更新

## 1. 项目目标

当前系统把局部 elevation/2.5D 几何地图、稀疏持久图、三相机视觉评分、目标位置融合和 graph planner 组合成开词汇目标搜索链路

核心目标:

- 使用 GridMap 构建可持久化的稀疏导航图
- 在 rolling map 移动时保留已探索拓扑
- 给当前 frontier 附加视觉语义分数
- 从多个有效相机视角递归融合目标位置
- 在探索目标、融合目标和停止目标之间保持唯一控制 owner

## 2. 当前主链路

```text
raw PointCloud2
  -> pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> elevation GridMap
  -> graph_construction
  -> NavigationGraph
  -> WildOS scoring
  -> Scored NavigationGraph
  -> graphnav_planner
  -> Path
```

启用目标搜索时增加:

```text
camera images + camera info + TF
  -> WildOS object detection
  -> ObjectMaskWithTf
  -> object_target_fusion
  -> TargetEstimate
  -> ObjectSearchGoalMux
  -> goal_pose and completion latch
```

当前不存在 2D `OccupancyGrid` fallback，`graph_construction` 只订阅 elevation `GridMap`

## 3. 默认启动

```bash
./scripts/start_wildos_elevation.sh
```

未设置 `WILDOS_TOPIC_PROFILE` 时默认使用 `unity`

平台选择:

```bash
WILDOS_TOPIC_PROFILE=isaac ./scripts/start_wildos_elevation.sh
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_elevation.sh
```

目标搜索:

```bash
WILDOS_TOPIC_PROFILE=unity \
  ./scripts/start_wildos_elevation.sh do_object_search:=true
```

唯一集成 launch:

```text
graph_construction/launch/elevation_visual_navigation_sim.launch.py
```

## 4. 模块职责

### 4.1 `graph_construction`

- 解码 elevation GridMap
- 分类 free、obstacle 和 unknown
- 生成世界坐标对齐的稀疏节点
- 校验并持久化边
- 检测 frontier 并分配给稳定 graph owner
- 发布 `NavigationGraph` 和调试 marker

### 4.2 `visual_navigation`

- 运行 ExploRFM 推理
- 将视觉 frontier 分数写入 graph
- 过滤目标 mask 并生成到达证据
- 运行目标融合 ROS adapter
- 统一管理 object search goal 和完成状态

### 4.3 `triangulation3d`

只保留 `TargetParticleFilter`，提供固定规模粒子的递归目标融合算法

### 4.4 `graphnav_planner`

- 消费 scored graph、odom 和高层 goal
- 在 graph 上计算路径
- 发布路径和 planner marker
- 可选保留 `path_follower_node`，但默认链路不启动

### 4.5 `object_search_msgs`

定义目标 mask 和融合状态消息

`ObjectMaskWithTf` 不再携带 odom 副本和 query 字符串，测量时间与 frame 由 header 表达

## 5. 当前文件结构

```text
graph_construction/
  configs/
    graph_construction_elevation.yaml
    topic_profiles.yaml
  graph_construction/
    node.py
    grid_adapter.py
    grid_types.py
    graph_builder.py
    graph_memory.py
    frontier_detector.py
    edge_builder.py
    msg_utils.py
    viz.py
    pointcloud_axis_adapter.py
  launch/
    elevation_visual_navigation_sim.launch.py

visual_navigation/
  configs/
    wildos_nav_conf.yaml
    wildos_nav_sim_conf.yaml
    object_search_goal_mux.yaml
  launch/
    wildos_component.launch.py
  visual_navigation/
    wildos/
    object_target_fusion.py
    object_search_goal_mux.py
    object_detection_filter.py
    object_reached_evidence.py

triangulation3d/
  triangulation3d/
    target_particle_filter.py

graphnav_planner/
  config/planner.yaml
  launch/graphnav_planner.launch.py
  launch/path_follower.launch.py
```

核心业务模块详细说明:

- `graph_update.md`: 局部高程图如何增量更新持久导航图
- `target_exploration.md`: 尚未发现目标时如何选路, 发现目标后如何切换状态
- `target_localization.md`: 多视角 Mask 和 LiDAR 如何形成三维目标位置

## 6. 参数边界

| 内容 | 位置 |
|---|---|
| 平台 topic、frame、RMW、domain | `graph_construction/configs/topic_profiles.yaml` |
| GridMap 解码和 graph ROS 配置 | `graph_construction/configs/graph_construction_elevation.yaml` |
| graph 算法默认值 | `GraphBuilderConfig` |
| WildOS 模型和评分 | `visual_navigation/configs/wildos_nav_*.yaml` |
| Goal Mux | `visual_navigation/configs/object_search_goal_mux.yaml` |
| Planner | `graphnav_planner/config/planner.yaml` |

## 7. Topic 契约

主要 topic 名称由 profile 提供，逻辑角色保持一致:

| 角色 | 典型 topic |
|---|---|
| elevation map | `/elevation_mapping_node/elevation_map_raw` |
| base graph | `/spot1/nav_graph` |
| scored graph | `/spot1/scored_nav_graph` |
| object mask | `/spot1/object_mask` |
| target estimate | `/spot1/object_target_estimate` |
| completion | `/spot1/object_search_completed` |
| planner path | `/spot1/graphnav_planner/path` |

## 8. 研究基线边界

LRN、ImgFrontier、GeoFrontier、训练脚本和 GPS 工具保留用于研究对照

它们必须可构建且不依赖固定机器路径，但不会由默认 elevation launch 启动

## 9. 已删除的旧实现

- 自研 PointCloud2 到 2D OccupancyGrid 后端
- GridMap 到 OccupancyGrid debug adapter
- 2D 集成 launch 和 2D 参数
- 视觉射线粗目标 pose 和 marker
- standalone ExploRFM triangulation
- legacy triangulation demos、teleop 和 batch helper

历史日期文档仍可保留这些实现的实验记录，但不能作为当前运行说明
