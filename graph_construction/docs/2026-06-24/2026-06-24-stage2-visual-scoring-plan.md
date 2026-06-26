# 阶段二实现方案: Visual Scoring 与 Graph Planner 闭环

日期: 2026-06-24

目标: 在阶段一几何图构建已经跑通的基础上, 接入 WildOS 视觉评分模块, 将 `/spot1/nav_graph` 转换为 `/spot1/scored_nav_graph`, 并让 `graphnav_planner` 基于 scored graph 输出路径

核心链路:

```text
/livox/lidar
    -> livox_grid_builder
    -> /spot1/traversability_grid
    -> graph_construction
    -> /spot1/nav_graph
    -> visual_navigation/wildos
    -> /spot1/scored_nav_graph
    -> graphnav_planner
    -> /spot1/graphnav_planner/path
```

## 设计边界

本阶段不重写 ExploRFM, 不重写 WildOS 视觉评分算法, 只做当前仿真环境下的 topic, frame, 配置和闭环联调

需要完成:

- 适配 `visual_navigation/wildos` 的相机, odom, TF 和 nav graph topic
- 让视觉模块订阅 `/spot1/nav_graph`
- 让视觉模块读取当前相机图像和 camera info
- 将 geometric frontier 投影到图像
- 使用 ExploRFM 输出 visual traversability, visual frontier, object similarity
- 给 frontier node 写入 `frontier_scores` 和 `is_default_scored`
- 发布 `/spot1/scored_nav_graph`
- 启动 `graphnav_planner`, 消费 `/spot1/scored_nav_graph`
- 验证 planner 输出 path
- 提供 RViz 可视化检查 score rings, scored nav graph, path

暂不完成:

- 不训练或微调 ExploRFM
- 不改变 `graphnav_msgs`
- 不改变 `graphnav_planner` 的核心 Dijkstra 逻辑
- 不实现完整 Web UI
- 不优化物体搜索效果, object search 可先关闭

## 当前阶段一状态

阶段一几何链路已经跑通:

```text
/livox/lidar
    -> /spot1/traversability_grid
    -> /spot1/nav_graph
```

实测结果:

```text
frame_id: map
nodes: 约 146
edges: 约 330-340
```

当前 Graph Construction 已提供视觉评分所需的基础字段:

```text
NavigationGraph
  header.frame_id = map
  trav_classes = ["default"]
  nodes
  edges
  current_node_idx

Node
  uuid
  pose
  trav_properties[0].is_frontier
  trav_properties[0].frontier_points
  trav_properties[0].free_radius
  trav_properties[0].explored_radius
```

## 当前仿真环境差异

WildOS 默认配置是 Spot/Realsense 三相机:

```text
parent_frame: spot1/odom
cam_frame: spot1/realsense/{}_color_optical_frame
camera_img_topic: /spot1/realsense/{}/color/image_raw/compressed
camera_info_topic: /spot1/realsense/{}/color/camera_info
odometry_topic: /spot1/odom
navigation_graph_topic: /spot1/nav_graph
```

当前仿真环境是:

```text
parent_frame: map
camera frame: camera_frame
camera image: /camera/color/image 或 /camera/color/image/compressed
camera info: /camera/color/camera_info
odometry topic: /unity/odom
navigation graph topic: /spot1/nav_graph
```

当前 TF 树:

```text
map
  -> odom_fram
      -> livox_frame
      -> camera_frame
      -> imu_link
```

注意:

- `/unity/odom` 的 `header.frame_id` 当前为空, 但 pose 与 `map -> odom_fram` 一致
- `visual_navigation/wildos/nav.py` 当前会检查 `odom_msg.header.frame_id == parent_frame`
- 因此阶段二需要先处理 odom header frame 不匹配问题

## 模块和配置建议

建议新增当前仿真专用配置:

```text
visual_navigation/configs/wildos_nav_sim_conf.yaml
```

建议配置:

```text
num_cameras: 1
cams_inverted: false
num_angular_bins: 16
reach_in_2D: true
frontier_ckpt: frontier_ckpt_new.ckpt
traversability_ckpt: traversability_ckpt.ckpt

frontiers_range: 9.0
traversability_class: default
heading_sim_thresh: 0.0
default_max_score: 0.3
std_for_default_scores: 80.0
min_frontier_separation: 0.75

parent_frame: map
cam_frame: camera_frame
camera_img_topic: /camera/color/image/compressed
camera_info_topic: /camera/color/camera_info
odometry_topic: /spot1/odom_for_scoring
navigation_graph_topic: /spot1/nav_graph
syncsub_queue_size: 80
syncsub_slop: 5.0
tf_lookup_config.buffer_size: 8
tf_lookup_config.wait_for_oldest: false

scored_navgraph_topic: /spot1/scored_nav_graph
model_viz_topic: /spot1/model_visualization
valid_geofrontiers_topic: /spot1/within_range_geofrontiers
score_ring_topic: /spot1/score_rings
graph_viz_topic: /spot1/nav_graph_viz
```

建议新增一个轻量 odom adapter:

```text
/unity/odom
    -> odom_frame_adapter
    -> /spot1/odom_for_scoring
```

作用:

- 复制 `/unity/odom`
- 设置 `header.frame_id = map`
- 设置 `child_frame_id = odom_fram`
- 默认设置 `header.stamp = now`, 避免 `/unity/odom` 原始时间戳和相机, graph 的仿真时间不一致
- 保持 pose 和 twist 不变

这样可以不修改 `visual_navigation/wildos/nav.py` 的 frame assert

当前实现:

```text
visual_navigation/configs/wildos_nav_sim_conf.yaml
visual_navigation/visual_navigation/utils/odom_frame_adapter.py
visual_navigation/launch/wildos_sim_launch.py
```

## 输入

视觉评分节点输入:

```text
/spot1/nav_graph, graphnav_msgs/NavigationGraph
/spot1/odom_for_scoring, nav_msgs/Odometry
/camera/color/image/compressed, sensor_msgs/CompressedImage
/camera/color/camera_info, sensor_msgs/CameraInfo
/tf, tf2_msgs/TFMessage
```

如果使用 raw image, 则:

```text
/camera/color/image, sensor_msgs/Image
```

`camera_img_topic` 中包含 `compressed` 时, `wildos/nav.py` 会使用 `sensor_msgs/CompressedImage`

## 输出

主要输出:

```text
/spot1/scored_nav_graph, graphnav_msgs/NavigationGraph
```

调试输出:

```text
/spot1/model_visualization, sensor_msgs/Image
/spot1/within_range_geofrontiers, visualization_msgs/MarkerArray
/spot1/score_rings, visualization_msgs/MarkerArray
/spot1/nav_graph_viz, visualization_msgs/MarkerArray
```

Planner 输出:

```text
/spot1/graphnav_planner/path, nav_msgs/Path
```

## Topic 和 RViz 可视化说明

本节记录当前仿真链路中各 topic 的含义, 作用, 以及 RViz 中建议如何查看

### 上游输入 topic

| Topic | 类型 | 来源 | 作用 | RViz 显示 |
| --- | --- | --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 仿真 Livox | 原始 3D 雷达点云, `livox_grid_builder` 用它生成局部可通行栅格 | `PointCloud2`, 显示雷达扫描点, 用于检查点云是否覆盖地面和障碍物 |
| `/unity/odom` | `nav_msgs/Odometry` | 仿真 | 原始机器人位姿, 用于局部 grid 中心和 odom adapter | `Odometry`, 显示机器人姿态箭头, 但原始 header 可能不满足视觉评分节点要求 |
| `/camera/color/image/compressed` | `sensor_msgs/CompressedImage` | 仿真相机 | WildOS 视觉模型输入图像 | `Image`, 显示原始前视相机画面 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 仿真相机 | 相机内参, 用于将 geometric frontier 投影到图像 | RViz 通常不直接显示, 用 `ros2 topic echo --once` 检查 |
| `/tf` | `tf2_msgs/TFMessage` | 仿真 TF | 提供 `map -> odom_fram -> camera_frame/livox_frame` 等坐标变换 | `TF`, 显示坐标轴树, 用于检查 frame 是否连通 |

### 阶段一输出 topic

| Topic | 类型 | 发布模块 | 作用 | RViz 显示 |
| --- | --- | --- | --- | --- |
| `/spot1/traversability_grid` | `nav_msgs/OccupancyGrid` | `livox_grid_builder` | 将 `/livox/lidar` 投影成机器人附近局部栅格, 供 `graph_construction` 采样节点和检测 frontier | `Map`, free 区域通常为浅色, obstacle 为深色, unknown 为灰色或透明, 具体颜色取决于 RViz map scheme |
| `/spot1/nav_graph` | `graphnav_msgs/NavigationGraph` | `graph_construction` | 稀疏几何导航图, 包含 nodes, edges, current_node_idx, frontier_points, free_radius, explored_radius | RViz 不能直接显示自定义 graph 消息, 需要看 `/spot1/graph_construction_viz` 或 `/spot1/nav_graph_viz` |
| `/spot1/graph_construction_viz` | `visualization_msgs/MarkerArray` | `graph_construction` | 阶段一几何图调试可视化, 用于检查图构建是否合理 | `MarkerArray`, 见下方 Graph Construction 图例 |

`/spot1/graph_construction_viz` 图例:

| 图形 | namespace | 含义 | 判断标准 |
| --- | --- | --- | --- |
| 亮绿色小球 | `free_nodes` | 当前局部 grid 内新鲜观测到的可通行节点 | 应落在 free space 内, 不应压在 obstacle 上 |
| 深绿色小球 | `memory_free_nodes` | 历史保留的 graph memory 节点 | 可以出现在当前 LiDAR 局部范围外, 这是正常现象 |
| 蓝色小球 | `frontier_nodes` | geometric frontier node, 表示靠近 free 和 unknown 边界的可探索节点 | 应靠近未知区域边界, 不应大量出现在已知 free 区域内部 |
| 紫色小方块 | `frontier_points` | frontier node 关联的 unknown 边界点 | 应贴近局部 grid 的 free/unknown 交界 |
| 红色细线 | `edges` | graph 中可直线通行的边 | 不应明显穿过 obstacle 或未知大块区域 |
| 红色半透明圆盘 | `free_radius` | 节点周围估计的安全可通行半径 | 仅用于调试, 大面积开启会遮挡地图 |
| 青色半透明圆盘 | `explored_radius` | 节点周围已经观测过的范围 | 仅用于调试, 用于检查 explored area 是否合理 |
| 橙色大球 | `current_node` | 当前机器人所在或最近 graph node | 应跟随机器人移动, 不应在旧位置残留 |
| 黄色小点 | `trajectory` | 机器人走过的位置轨迹 | 只表示历史路径, 不参与 graph node 采样 |
| 青色矩形边框 | `grid_footprint` | 当前局部 OccupancyGrid 覆盖范围 | 用于区分当前观测范围和历史 graph memory |

### 阶段二输出 topic

| Topic | 类型 | 发布模块 | 作用 | RViz 显示 |
| --- | --- | --- | --- | --- |
| `/spot1/odom_for_scoring` | `nav_msgs/Odometry` | `odom_frame_adapter` | 将 `/unity/odom` 适配为视觉评分可用 odom, `header.frame_id=map`, `child_frame_id=odom_fram`, stamp 默认使用当前 ROS time | `Odometry`, 显示机器人位姿, 用于检查 scoring 输入 frame 是否正确 |
| `/spot1/scored_nav_graph` | `graphnav_msgs/NavigationGraph` | `visual_navigation/wildos` | 在 `/spot1/nav_graph` 基础上给 frontier node 添加 `frontier_scores` 和 `is_default_scored` | RViz 不能直接显示自定义 graph 消息, 用 topic echo 检查属性, 或看 `/spot1/nav_graph_viz` 和 `/spot1/score_rings` |
| `/spot1/model_visualization` | `sensor_msgs/Image` | `visual_navigation/wildos` | 视觉模型调试拼图, 显示图像, visual frontier, traversability, projected frontier/path | `Image`, 见下方 Model Visualization 图例 |
| `/spot1/within_range_geofrontiers` | `visualization_msgs/MarkerArray` | `visual_navigation/wildos` | 显示当前相机视野和距离范围内可被视觉评分的 geometric frontier | `MarkerArray`, 黄色箭头为 camera heading, 彩色箭头为 frontier heading |
| `/spot1/score_rings` | `visualization_msgs/MarkerArray` | `visual_navigation/wildos` | 显示每个 frontier node 周围 16 个方向 bin 的评分 | `MarkerArray`, 见下方 Score Rings 图例 |
| `/spot1/nav_graph_viz` | `visualization_msgs/MarkerArray` | `visual_navigation/wildos` | 从视觉评分节点视角显示 nav graph, 便于和 score rings 一起检查 | `MarkerArray`, 见下方 Visual Navigation 图例 |

`/spot1/model_visualization` 图例:

| 行 | 标题 | 含义 | 颜色 |
| --- | --- | --- | --- |
| 第 1 行 | `Image front` 或 `Image front + graph` | 原始前视相机图, 如果当前有可投影 frontier, 会叠加绿色 projected graph 点和线 | 绿色点/线表示投影到图像上的 frontier/path 调试线 |
| 第 2 行 | `Frontier Conf.` | ExploRFM 输出的 visual frontier 置信度 | 色条 0 到 1, 蓝色低, 红色高 |
| 第 3 行 | `Traversability Conf.` | ExploRFM 输出的 visual traversability 置信度 | 色条 0 到 1, 蓝色低, 红色高 |
| 第 4 行 | `Frontier Nodes` 或 `Frontier Nodes: none` | 几何 frontier 投影和当前 heading bin 的 score map, 没有有效投影时仍显示原图 | 热力图表示 score map, 绿色点/线表示 projected frontier/path |
| 右侧色条 | `0.00` 到 `1.00` | 模型输出或 score map 的归一化数值范围 | 0 为低置信度, 1 为高置信度 |

`/spot1/nav_graph_viz` 图例:

| 图形 | namespace | 含义 | 当前默认显示 |
| --- | --- | --- | --- |
| 绿色球 | `nodes` | 非 frontier graph nodes | 显示 |
| 蓝色球 | `frontier_nodes_default` 或对应 class | frontier nodes | 显示 |
| 紫色方块 | `frontier_points` | frontier node 关联的 unknown 边界点 | 显示 |
| 蓝色细线 | `frontier_point_to_node` | frontier point 到所属 frontier node 的关联线 | 显示, 半透明 |
| 红色细线 | `edges` | graph edges | 显示, 半透明 |
| 红色半透明圆盘 | `free_radius_*` | free radius 调试层 | 默认隐藏并清理旧 marker |
| 蓝色半透明圆盘 | `explored_radius` | explored radius 调试层 | 默认隐藏并清理旧 marker |
| 白色长文本 | `ids` | node UUID 调试文本 | 默认隐藏并清理旧 marker |

`/spot1/score_rings` 图例:

| 图形 | 含义 | 判断标准 |
| --- | --- | --- |
| frontier 周围的小圆弧 | 该 frontier 在一个离散朝向 bin 上的视觉评分 | 每个 frontier 周围最多 16 段圆弧, 对应 `num_angular_bins=16` |
| 颜色 | 归一化 score, 范围 `[0,1]` | 蓝色低, 绿色/黄色中等, 红色高, 高分方向更值得 planner 探索 |
| 圆弧位置 | 方向 bin 的空间朝向 | 用于判断视觉模型偏向哪个探索方向 |

注意:

- `/spot1/score_rings` 的圆弧不是 obstacle, 也不是安全半径, 它只表示方向评分
- 论文视频中的主视图通常不显示大面积 free_radius/explored_radius 调试圆盘, 当前默认也隐藏这些半径层
- 绿色历史节点在当前雷达扫描范围外通常是 graph memory 正常现象, 不代表当前雷达正在看到那些点

### Planner 输出 topic

| Topic | 类型 | 发布模块 | 作用 | RViz 显示 |
| --- | --- | --- | --- | --- |
| `/spot1/graphnav_planner/path` | `nav_msgs/Path` | `graphnav_planner` | 基于 `/spot1/scored_nav_graph` 和 goal 计算出的导航路径 | `Path`, 显示规划线, 应沿 graph 可通行区域走 |
| `/spot1/graphnav_planner/frontier_scores` | `visualization_msgs/MarkerArray` | `graphnav_planner` | planner 侧 frontier score/cost 调试输出, 只有有订阅者时发布 | `MarkerArray`, 彩色 cube 表示 frontier cost, 文本显示归一化 score |
| `/spot1/graphnav_planner/unexplored_space_map` | `grid_map_msgs/GridMap` | `graphnav_planner` | planner 内部 unexplored space debug map, 只有有订阅者时发布 | `GridMap`, 用于检查 frontier 到 unknown 的距离代价 |
| `/spot1/graphnav_planner/goal_pose` | `geometry_msgs/PoseStamped` | `path_follower_node` | path follower 给底层控制或目标跟踪发布的下一目标点 | `PoseStamped`, 显示当前跟踪目标点 |

## Scored Graph 契约

阶段二输出的 `/spot1/scored_nav_graph` 应保留原始 graph 的:

```text
nodes
edges
current_node_idx
trav_classes
trav_properties
```

并在 frontier node 的 `properties` 中追加:

```text
key = frontier_scores
value = num_angular_bins 个方向评分

key = is_default_scored
value = [0.0] 或 [1.0]
```

解释:

- `is_default_scored = [0.0]` 表示该 frontier 被视觉模型实际评分
- `is_default_scored = [1.0]` 表示该 frontier 没有被相机看到, 使用默认方向评分
- `frontier_scores` 会被 `graphnav_planner` 用于计算 frontier 到虚拟目标的代价

## 核心评分模型

视觉模块会将几何 frontier 投影到图像:

```text
frontier_points_3d
    -> camera extrinsics
    -> camera intrinsics
    -> frontier_pixels
```

ExploRFM 对图像输出:

```text
visual_traversability
visual_frontier
object_similarity
```

每个 frontier 对应一组方向 bin:

```text
S_i = [s_0, s_1, ..., s_15]
```

planner 根据目标方向选择 bin:

```text
best_bin = BinIndex(goal_heading)
frontier_score = frontier_scores[best_bin]
```

再计算 frontier 代价:

```text
frontier_cost = frontier_path_distance * (1 - frontier_score_factor * log(frontier_score))
```

解释:

- score 越高, frontier_cost 越低
- graphnav_planner 越倾向选择视觉模型认为更有希望的 frontier
- 如果没有视觉评分, default score 会让 planner 仍可运行, 但语义选择能力较弱

## 主流程伪代码

```text
function VisualScoringStep(nav_graph, odom, image, camera_info, tf):
    assert nav_graph.header.frame_id == parent_frame
    assert odom.header.frame_id == parent_frame

    camera_pose = LookupTransform(parent_frame, camera_frame)
    frontier_nodes = ExtractFrontierNodes(nav_graph)
    visible_frontiers = ProjectFrontiersToImage(frontier_nodes, camera_pose, camera_info)

    traversability, visual_frontiers, object_similarity = ExploRFM(image)

    for each visible frontier:
        scores = ScoreFrontier(
            visual_frontiers,
            traversability,
            object_similarity,
            frontier_pixels
        )
        WriteNodeProperty(frontier_node, "frontier_scores", scores)
        WriteNodeProperty(frontier_node, "is_default_scored", [0.0])

    for each frontier not visible:
        scores = DefaultScores(frontier_heading)
        WriteNodeProperty(frontier_node, "frontier_scores", scores)
        WriteNodeProperty(frontier_node, "is_default_scored", [1.0])

    publish scored_nav_graph
```

Planner 流程:

```text
function PlannerStep(scored_nav_graph, odom, goal):
    graph = ConvertNavigationGraph(scored_nav_graph)
    for each frontier node:
        score = SelectFrontierScoreByGoalHeading(frontier_scores)
        add virtual goal edge with score-modulated cost
    path = Dijkstra(graph, current_node_idx, virtual_goal)
    publish path
```

## 启动顺序

阶段一基础链路:

```bash
cd /home/ks-server3/han/wildos_ws/src/nebula2-wildos
scripts/start_graph_construction.sh
```

阶段二视觉评分:

```bash
cd /home/ks-server3/han/wildos_ws/src/nebula2-wildos
scripts/start_visual_navigation.sh
```

推荐使用脚本启动视觉评分, 因为 ROS 生成的 `visual_navigation` console script 可能固定到 `/usr/bin/python3`, 直接 `ros2 run visual_navigation wildos ...` 会绕过 UV venv, 导致 `skimage`, `torch`, `explorfm` 等依赖不可见

等价手动启动方式, 仅用于排查, 两个 `ros2 run` 需要分终端执行:

```bash
source /opt/ros/humble/setup.bash
source /home/ks-server3/han/wildos_ws/src/nebula2-wildos/.venv/bin/activate
export PYTHONNOUSERSITE=1
export PYTHONPATH=/home/ks-server3/han/wildos_ws/src/nebula2-wildos:${PYTHONPATH:-}
source /home/ks-server3/han/wildos_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
sed -i '1c#!/usr/bin/env python3' /home/ks-server3/han/wildos_ws/install/visual_navigation/lib/visual_navigation/*

# 终端 1
ros2 run visual_navigation odom_frame_adapter --ros-args \
  -p input_topic:=/unity/odom \
  -p output_topic:=/spot1/odom_for_scoring \
  -p parent_frame:=map \
  -p child_frame:=odom_fram \
  -p stamp_mode:=now

# 终端 2
ros2 run visual_navigation wildos --config wildos_nav_sim_conf.yaml --do_object_search false
```

默认 `wildos_launch.py` 已支持 `config` 参数, 但当前仿真推荐使用 `wildos_sim_launch.py`, 因为它默认不 remap `/tf`, 并会同时启动 odom adapter

阶段二 planner:

```bash
ros2 launch graphnav_planner graphnav_planner.launch.yml ns:=spot1 remap_tf_to_ns:=false odom_topic:=odom_for_scoring
```

注意:

- `graphnav_planner` 默认将 `~/nav_graph` remap 到 `scored_nav_graph`
- 在 namespace `spot1` 下, 它应消费 `/spot1/scored_nav_graph`
- `odom_topic:=odom_for_scoring` 会让 planner 和 path follower 使用 `/spot1/odom_for_scoring`
- 当前 TF 是全局 `/tf`, 不是 `/spot1/tf`, 因此可能需要 `remap_tf_to_ns:=false`

## 验收标准

### 环境验收

检查 checkpoint 文件:

```bash
ls -l \
  /home/ks-server3/han/wildos_ws/src/nebula2-wildos/ckpts/frontier_ckpt_new.ckpt \
  /home/ks-server3/han/wildos_ws/src/nebula2-wildos/ckpts/traversability_ckpt.ckpt
```

期望两个文件都存在

检查 UV venv 依赖:

```bash
cd /home/ks-server3/han/wildos_ws/src/nebula2-wildos
source .venv/bin/activate
python - <<'PY'
import sys
import skimage
import torch

print(sys.executable)
print(f"skimage={skimage.__version__}")
print(f"torch={torch.__version__}")
PY
```

期望:

```text
/home/ks-server3/han/wildos_ws/src/nebula2-wildos/.venv/bin/python
skimage=<nonempty>
torch=<nonempty>
```

检查 ROS console script 是否会使用当前 venv:

```bash
head -1 /home/ks-server3/han/wildos_ws/install/visual_navigation/lib/visual_navigation/wildos
```

期望:

```text
#!/usr/bin/env python3
```

如果输出是 `#!/usr/bin/python3`, 需要执行:

```bash
cd /home/ks-server3/han/wildos_ws/src/nebula2-wildos
scripts/start_visual_navigation.sh --show-args
```

该脚本会修正 `visual_navigation` 安装入口的 shebang

### Visual Scoring 验收

检查输入:

```bash
ros2 topic echo /spot1/nav_graph --once --field header
ros2 topic echo /spot1/odom_for_scoring --once --field header
ros2 topic echo /camera/color/image/compressed --once --field header
ros2 topic echo /camera/color/camera_info --once --field header
```

期望:

```text
/spot1/nav_graph.header.frame_id = map
/spot1/odom_for_scoring.header.frame_id = map
/camera/color/image/compressed.header.frame_id = camera_frame
/camera/color/camera_info.header.frame_id = camera_frame
```

四路输入的 `stamp.sec` 应处于同一时间系, 当前仿真中应都接近 `/clock` 时间, 不能出现 `/unity/odom` 的 epoch 时间戳

当前实测 `/spot1/nav_graph` 的 stamp 约比相机和 odom 慢 `3.6s`, 原始 `syncsub_slop=0.2` 会导致 `ApproximateTimeSynchronizer` 永远不触发

仿真配置使用:

```text
syncsub_queue_size: 80
syncsub_slop: 5.0
```

如果日志持续出现 `Message buffer is empty`, 需要再次检查四路输入 stamp 差距是否超过 `syncsub_slop`

如果日志出现 `TF not found ... extrapolation into the past`, 说明视觉节点启动后收到的早期同步消息早于 TF listener 缓存的最早 transform, 当前实现会丢弃这类 stale message 并等待新同步消息

检查 scored graph:

```bash
ros2 topic info /spot1/scored_nav_graph
ros2 topic echo /spot1/scored_nav_graph --once --field header
```

期望:

```text
Type: graphnav_msgs/msg/NavigationGraph
Publisher count: 1
frame_id: map
```

检查 frontier score:

```bash
python3 - <<'PY'
import rclpy
from graphnav_msgs.msg import NavigationGraph

rclpy.init()
node = rclpy.create_node("scored_graph_check")

def callback(msg):
    scored = 0
    default_scored = 0
    frontier_nodes = 0
    for graph_node in msg.nodes:
        is_frontier = graph_node.trav_properties and graph_node.trav_properties[0].is_frontier
        if not is_frontier:
            continue
        frontier_nodes += 1
        props = {kv.key: kv.value for kv in graph_node.properties}
        if "frontier_scores" in props:
            scored += 1
        if props.get("is_default_scored", [0.0])[0] > 0.5:
            default_scored += 1
    print(f"frontier_nodes={frontier_nodes}")
    print(f"frontier_nodes_with_scores={scored}")
    print(f"default_scored={default_scored}")
    node.destroy_node()
    rclpy.shutdown()

node.create_subscription(NavigationGraph, "/spot1/scored_nav_graph", callback, 10)
rclpy.spin(node)
PY
```

基础通过条件:

- frontier node 数量非零
- 至少一部分 frontier node 带 `frontier_scores`
- `/spot1/model_visualization` 有图像输出
- `/spot1/score_rings` 有 marker 输出

### Planner 验收

检查 planner 输入:

```bash
ros2 topic info /spot1/scored_nav_graph
ros2 topic info /spot1/goal_pose
```

发布测试目标:

```bash
ros2 topic pub --once /spot1/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

检查 path:

```bash
ros2 topic info /spot1/graphnav_planner/path
ros2 topic echo /spot1/graphnav_planner/path --once --field header
```

基础通过条件:

- planner 能收到 `/spot1/scored_nav_graph`
- planner 不报缺少 `frontier_scores`
- 发布 goal 后能输出 path
- path frame 为 `map`
- RViz 中 path 不明显穿过 obstacle

## 风险和后续工作

### topic 和 frame 不匹配

当前最大风险是默认 WildOS 配置与当前仿真 topic 不一致

必须优先处理:

- `parent_frame: map`
- `cam_frame: camera_frame`
- 单相机 `num_cameras: 1`
- `/unity/odom` header 为空的问题

### 模型依赖和 GPU

WildOS visual scoring 会加载 ExploRFM 和 RADIO backbone

风险:

- checkpoint 路径不对
- CUDA 不可用
- Python venv 和 ROS Python 环境冲突
- 模型加载慢或显存不足

### 同步问题

`wildos/nav.py` 使用 ApproximateTimeSynchronizer 同步 odom, nav graph, image, camera info

风险:

- 图像和 nav graph 频率差过大
- `syncsub_slop` 太小
- `use_sim_time` 不一致
- camera stamp 和 odom stamp 不在同一时钟体系

### frontier 投影问题

风险:

- frontier 不在相机视野内
- camera intrinsics 或 TF 错误
- `cams_inverted` 设置不对
- `heading_sim_thresh` 太严格

### planner score 代价问题

风险:

- score 为 0 时 `log(score)` 可能导致代价异常
- frontier score factor 过大导致路径选择过激
- default scored frontier 太多, 语义评分效果不明显

## 阶段二结论

阶段二的核心不是重新做图构建, 而是验证几何 frontier 能否被相机看到, 被 ExploRFM 评分, 并以 `frontier_scores` 的形式进入 `graphnav_planner`

只要 `/spot1/scored_nav_graph` 中的 frontier nodes 稳定带有 `frontier_scores`, 且 `graphnav_planner` 能输出 path, 就可以认为 Visual Scoring 与 Planner 闭环基础跑通
