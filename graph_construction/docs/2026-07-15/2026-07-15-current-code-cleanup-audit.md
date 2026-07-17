# 当前自研代码清理审计与执行结果

日期: 2026-07-15

执行状态: 已按 2026-07-16 人工确认完成代码清理

仓库定位: 研究仓库，保留可复现实验 baseline，默认部署主线保持唯一

文档边界: 本文只记录本轮代码清理计划和实际清理结果，清理完成后的功能修改与问题修复使用独立日期文档

## 1. 已确认的清理决定

- elevation/2.5D GridMap 是唯一当前几何主线
- 完全删除自研 2D `OccupancyGrid` fallback
- 完全删除无控制作用的旧视觉射线粗定位
- 目标融合统一命名为 `object_target_fusion`
- 只保留 `TargetParticleFilter`，删除旧 batch triangulation 和 demo
- 保留 LRN、ImgFrontier、GeoFrontier 等研究 baseline
- 保留 `path_follower_node`，但从默认链路隔离
- 不修改历史 changelog 和历史实验记录

## 2. 清理后的有效主链路

```text
PointCloud2
  -> pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> GridMap
  -> graph_construction
  -> NavigationGraph
  -> WildOS visual scoring
     -> Scored NavigationGraph -> graphnav_planner -> Path
     -> ObjectMaskWithTf -> object_target_fusion
        -> TargetEstimate -> ObjectSearchGoalMux -> goal_pose
```

默认启动:

```bash
./scripts/start_wildos_elevation.sh
```

默认集成 launch:

```text
graph_construction/launch/elevation_visual_navigation_sim.launch.py
```

## 3. 已删除文件

### 3.1 2D fallback

| 文件 | 删除原因 |
|---|---|
| `graph_construction/launch/wildos_2d_sim.launch.py` | 旧 2D 集成入口 |
| `graph_construction/graph_construction/livox_grid_builder.py` | 自研 PointCloud2 到 OccupancyGrid 后端 |
| `graph_construction/graph_construction/grid_map_to_occupancy.py` | 只服务旧 2D 调试链路 |
| `graph_construction/configs/graph_construction.yaml` | 旧 OccupancyGrid graph 配置 |
| `graph_construction/configs/livox_grid_builder.yaml` | 旧 2D builder 配置 |
| `graph_construction/configs/grid_map_to_occupancy.yaml` | 旧 debug adapter 配置 |

### 3.2 旧视觉粗定位

旧实现位于 `wildos/nav.py` 和 `wildos/viz.py` 内部，没有独立文件

已删除:

- object ray 构造
- 候选射线筛选
- graph node 粗目标标注
- `PoseStamped` 粗目标发布
- 粗目标 marker 发布
- 粗目标日志和全部相关参数

### 3.3 旧 standalone 视觉 triangulation

| 文件 | 删除原因 |
|---|---|
| `visual_navigation/visual_navigation/explorfm_triangulation/explorfm_triangulator.py` | 旧 standalone 实现 |
| `visual_navigation/visual_navigation/explorfm_triangulation/triangulator_viz.py` | 只服务旧 standalone 实现 |
| `visual_navigation/launch/explorfm_triangulation_launch.py` | 旧启动入口 |
| `visual_navigation/configs/explorfm_triangulator_conf.yaml` | 旧配置 |
| `visual_navigation/visual_navigation/explorfm_triangulation/__init__.py` | 目录整体清空后删除 |

原 `obj_mask_triangulation.py` 的当前逻辑已移动到 `visual_navigation/object_target_fusion.py`

### 3.4 旧 `triangulation3d` demo 和 batch 实现

已删除:

```text
triangulation3d/triangulation3d/bbox_generator.py
triangulation3d/triangulation3d/camera_data.py
triangulation3d/triangulation3d/multicam_visualizer.py
triangulation3d/triangulation3d/particle_generator.py
triangulation3d/triangulation3d/pcl_utils.py
triangulation3d/triangulation3d/quantitative_metrics.py
triangulation3d/triangulation3d/random_cam_initializer.py
triangulation3d/triangulation3d/teleop_triangulation.py
triangulation3d/triangulation3d/teleop_twist_keyboard.py
triangulation3d/triangulation3d/triangulator.py
```

### 3.5 重复或含混 launch

| 文件 | 处理 |
|---|---|
| `visual_navigation/launch/wildos_sim_launch.py` | 删除，避免复制 elevation 集成适配逻辑 |
| `visual_navigation/launch/wildos_launch.py` | 删除并替换为职责明确的 `wildos_component.launch.py` |
| `graphnav_planner/launch/graphnav_planner.launch.yml` | 删除并替换为类型安全的 Python launch |

## 4. 逐文件修改结果

### 4.1 `graph_construction`

| 文件 | 已执行修改 |
|---|---|
| `setup.py` | 删除 2D launch、builder 和 adapter 安装入口 |
| `graph_construction/node.py` | 固定 GridMap 输入，删除 `OccupancyGrid` 分支和 2D 阈值 |
| `graph_construction/grid_adapter.py` | 删除 OccupancyGrid 解码、方向兼容开关和可关闭后处理分支 |
| `graph_construction/graph_builder.py` | 删除 stage timing、诊断结构和 `GraphBuilder` 别名 |
| `graph_construction/frontier_detector.py` | 删除只为诊断返回的计数结构 |
| `graph_construction/graph_memory.py` | 删除未使用查询函数 |
| `configs/graph_construction_elevation.yaml` | 删除 `grid_input_type` |
| `configs/topic_profiles.yaml` | 删除全部 2D、旧粗目标和固定策略参数 |
| `graph_construction/topic_profiles.py` | 同步当前 profile key 说明 |
| `launch/elevation_visual_navigation_sim.launch.py` | 删除旧参数，加载 Goal Mux 和 planner 的单一配置文件 |
| `test/test_graph_builder.py` | 保留 graph 行为测试，移除诊断对象契约 |
| `test/test_grid_map_adapter.py` | 删除 OccupancyGrid 和旧兼容参数测试 |
| `test/test_node_config.py` | 验证 `grid_input_type` 已成为未知参数 |

### 4.2 `visual_navigation`

| 文件 | 已执行修改 |
|---|---|
| `wildos/nav.py` | 删除旧视觉粗目标定位、topic 和参数 |
| `wildos/viz.py` | 删除旧粗目标 marker 和未使用变量 |
| `object_target_fusion.py` | 作为唯一目标融合 ROS 节点 |
| `object_search_goal_mux.py` | 固化当前策略并精简参数和状态 |
| `utils/object_search_utils.py` | 生成精简后的 `ObjectMaskWithTf` |
| `utils/paths.py` | 新增仓库根目录解析和环境变量覆盖 |
| `configs/wildos_nav_conf.yaml` | 删除粗目标配置 |
| `configs/wildos_nav_sim_conf.yaml` | 删除粗目标配置 |
| `configs/object_search_goal_mux.yaml` | 成为 Goal Mux 策略单一来源 |
| `launch/wildos_component.launch.py` | 提供独立 WildOS 和目标融合调试入口 |
| `launch/lrn_launch.py` | 同步当前融合 executable |
| `launch/imgfrontier_nav_launch.py` | 同步当前融合 executable |
| `setup.py` | 删除旧 triangulation entrypoint，注册 `object_target_fusion` |
| `test/test_object_target_fusion.py` | 导入当前模块名 |

### 4.3 `object_search_msgs`

| 文件 | 已执行修改 |
|---|---|
| `msg/ObjectMaskWithTf.msg` | 删除未消费的 odom 和 query 字段 |
| `CMakeLists.txt` | 删除 `nav_msgs` 依赖 |
| `package.xml` | 删除 `nav_msgs` 依赖 |

### 4.4 `graphnav_planner`

| 文件 | 已执行修改 |
|---|---|
| `config/planner.yaml` | 新增 planner 算法参数单一来源 |
| `CMakeLists.txt` | 安装 config 和当前 launch |
| `src/planner.cpp` | 删除 virtual goal 追加分支 |
| `src/planner_node.cpp` | 删除 `trav_class` 和 `append_virtual_goal_to_path` 参数 |
| `include/graphnav_planner/planner.hpp` | 删除对应状态和废弃声明 |
| `launch/graphnav_planner.launch.py` | planner-only 启动入口 |
| `launch/path_follower.launch.py` | 独立可选 path follower 入口 |

### 4.5 路径和调试工具

| 文件 | 已执行修改 |
|---|---|
| `scripts/start_wildos_elevation.sh` | 导出 `WILDOS_REPO_ROOT`，检查当前融合入口 |
| `visual_navigation/*/nav.py` | 当前主线和 baseline 统一使用仓库根目录解析 |
| `test_explorfm_folder.py` | 删除固定仓库路径 |
| `explorfm_trainer/test_sidewalk.py` | 删除固定输入和输出仓库路径 |
| `nvidia_radio/hubconf.py` | checkpoint 目录改为仓库根目录或环境变量覆盖 |
| `visual_navigation/launch/save_gps_launch.py` | 修复未声明的 launch 参数 |

## 5. 参数清理结果

### 5.1 已删除 2D 参数

```text
grid_input_type
grid_topic
free_threshold
obstacle_threshold
launch_livox_grid_builder
traversability_grid_topic
grid_origin_mode
lidar_assume_input_in_grid_frame
```

以及旧 builder 的尺寸、射线、累计帧、height diff 和 inflation 参数

### 5.2 已删除视觉粗目标参数

```text
object_target_pose_topic
object_target_viz_topic
target_log_period_sec
target_min_score
target_ray_length
target_ray_match_radius
```

### 5.3 已收敛 Goal Mux 参数

删除:

```text
initial_goal_mode
object_reached_require_target_distance
```

重命名:

```text
metric_target_update_distance -> target_update_min_distance
```

### 5.4 已收敛 planner 参数

删除:

```text
trav_class
append_virtual_goal_to_path
```

当前算法参数只在 `graphnav_planner/config/planner.yaml` 定义

## 6. 保留项及理由

### 6.1 `UnexploredSpaceMap`

保留。它是 planner 根据 graph 覆盖生成的内部代价结构，不订阅 `OccupancyGrid`，不属于已删除的 2D 建图 fallback

### 6.2 `path_follower_node`

保留。它是研究和特定部署可选的 path-to-goal 适配器

隔离方式:

- 继续构建组件和 executable
- 默认 elevation launch 不启动
- 使用 `graphnav_planner/launch/path_follower.launch.py` 显式启动

### 6.3 Baseline

保留:

- LRN
- ImgFrontier
- GeoFrontier
- ExploRFM trainer
- GPS 和离线可视化工具

基线已同步公共消息变更和资源路径规则，不进入默认主线

## 7. 文档边界

已同步当前文档:

- `graph_construction/docs/AGENT_README.md`
- `graph_construction/docs/2026-06-22/2026-06-22-overview.md`
- `graph_construction/docs/2026-06-22/2026-06-22-principles.md`
- 本清理审计文档
- 当前 package README 和根 README 的入口说明

未修改其他日期目录的历史 changelog、实验记录和实施记录

历史文档中的 2D fallback、旧 triangulation 和旧参数只代表当时状态

## 8. 验证结果

### 8.1 静态检查

- 所有本轮修改的 Python 和 launch 文件通过 `py_compile`
- 当前 YAML 文件通过解析
- `scripts/start_wildos_elevation.sh` 通过 `bash -n`
- 当前代码和非历史文档中不再引用旧 2D entrypoint、旧粗目标 topic 和旧 triangulation executable

### 8.2 单元测试

核心测试结果:

```text
62 passed
```

覆盖 GridMap、稀疏 graph、持久 graph、粒子滤波、目标检测、到达证据、Goal Mux 和目标融合

### 8.3 干净构建

在独立 `/tmp` build/install 目录中执行 overlay 构建，结果:

```text
7 packages finished
```

构建包包含:

- `graphnav_msgs`
- `gps_visualization`
- `object_search_msgs`
- `triangulation3d`
- `graph_construction`
- `visual_navigation`
- `graphnav_planner`

只有既有 CMake policy 和最低版本兼容性警告，没有编译错误

2026-07-16 已清理相关包的旧 `build/install` 产物，并在主工作区完成重新构建，旧 setuptools manifest 已消失

## 9. 最终验收结论

- 默认几何主线唯一且只使用 elevation GridMap
- 2D fallback 的代码、配置、launch 和 entrypoint 已彻底删除
- 旧视觉粗定位的控制、显示、topic 和参数已删除
- 当前目标融合命名和实现边界唯一
- 研究 baseline 保留且同步公共契约
- `path_follower_node` 保留但不进入默认链路
- 参数来源、启动入口和文档边界明确
- 核心测试和干净构建通过
