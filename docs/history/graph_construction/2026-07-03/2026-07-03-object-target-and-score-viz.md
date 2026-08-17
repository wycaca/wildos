# 目标导航点和 score ring 可视化

日期: 2026-07-03

## 背景

目标搜索模型已经可以在图像中找到目标, 但运行中仍有两个可观测性缺口:

- 找到目标后没有明确的目标导航点 topic
- `/spot1/score_rings` 在 RViz2 中不容易看出每个 frontier 的评分大小

WildOS 论文中的导航决策不是直接把图像检测框当成底盘目标, 而是把目标语义注入 frontier scoring, 再让 graph planner 选择更可能通向目标的 frontier

本项目当前先实现这个层级的可视化和发布, 精确物体三角定位仍由 `obj_mask_triangulation` 链路负责

## 当前代码链路

目标搜索打开后:

```text
三相机图像 + nav_graph + odom
    -> WildOS / ExploRFM
    -> object mask
    -> 增强 img_frontiers 和 traversability
    -> score_geofrontiers
    -> scored_nav_graph
    -> graphnav_planner
```

本次新增:

```text
score_geofrontiers
    -> 选择最高分目标 frontier
    -> /spot1/object_search_target_pose
    -> /spot1/object_search_target_viz
```

## 目标导航点语义

`/spot1/object_search_target_pose` 发布的是当前最高分目标 frontier, 不是精确物体坐标

选择逻辑:

1. 只有 `do_object_search=true` 且图像 mask 有目标像素时才选择
2. 遍历可投影到相机的 geometric frontier
3. 使用目标增强后的 heading score
4. 选择最大分数超过 `object_search_config.target_min_score` 的 frontier
5. 在 `scored_nav_graph` 对应 node 上写入:

```text
object_target_score
object_target_heading_bin
```

这样后续可以通过 `ros2 topic echo /spot1/scored_nav_graph` 确认当前目标导航点落在哪个 node 上

## 目标可视化

新增 topic:

```text
/spot1/object_search_target_viz
```

类型:

```text
visualization_msgs/MarkerArray
```

显示内容:

- `object_search_target`: 当前最高分目标 frontier, 用按分数着色的 sphere 显示
- `object_search_detection_ray`: 每个检测到目标的相机 ray, 从相机位置指向 mask 质心方向
- `object_search_detection_text`: 只显示 `front detected`, `left detected`, `right detected`

检测射线只是视觉观测方向, 没有深度约束, 不代表物体精确位置

RViz 主视图不显示 mask 像素数量, 该数值只适合模型调试, 不适合作为长期导航可视化信息

### Unity 图像方向适配

Unity 2D 仿真中前向相机能正确检测到目标, 但 RViz 中目标射线可能和 map 方向左右相反

当前处理方式:

- `topic_profiles.yaml` 新增 `camera_static_tf_convention`
- Isaac 和真实狗默认 `x_forward_y_left`, 对应 ROS base_link 的 `x 前, y 左, z 上`
- Unity 默认 `negative_y_forward_x_right`, 对应当前 `odom_fram` 的 `-y 前, x 右, z 上`
- `camera_image_flip_x` 保留为图像水平轴补偿开关, Unity 当前默认关闭
- `camera_image_flip_x` 同时作用于 frontier 投影评分和目标 mask 质心反投影射线

这次修正的核心不是改模型识别结果, 而是让 `front_camera` 的 optical z 轴和 Unity 机器狗实际前方一致

实测中 `y_forward_x_right` 会让 `front_camera` 朝 map `+y`, 目标射线落在机器狗后方, 因此 Unity profile 使用 `negative_y_forward_x_right`

## score ring 可视化

`/spot1/score_rings` 仍然表示每个 frontier 的离散 heading score

本次增强:

- 每帧重画全部 score ring, 避免旧 marker 残留
- 每个 heading bin 用弧线显示, 不再只是两点线段
- 默认只显示方向彩色圆环, 不显示 `max_score`, `best_bin` 或其它文字
- 低分使用蓝 / 青色, 高分使用黄色 / 红色, 让推荐方向更醒目
- `DELETEALL` marker 使用独立 clear namespace, 避免同一个 `MarkerArray` 中出现相同 `ns/id`

RViz2 中建议同时打开:

```text
/spot1/score_rings
/spot1/object_search_target_viz
/spot1/scored_nav_graph
/spot1/model_visualization
```

## Topic 配置

新增 topic 已写入:

```text
graph_construction/configs/topic_profiles.yaml
visual_navigation/configs/wildos_nav_sim_conf.yaml
visual_navigation/configs/wildos_nav_conf.yaml
```

默认值:

```yaml
object_mask_topic: /spot1/object_mask
object_target_pose_topic: /spot1/object_search_target_pose
object_target_viz_topic: /spot1/object_search_target_viz
camera_static_tf_convention: negative_y_forward_x_right
camera_image_flip_x: false
```

调参:

```yaml
object_search_config:
  target_min_score: 0.2
  target_ray_length: 8.0
```

## 验证方式

启动:

```bash
bash scripts/start_wildos_2d.sh do_object_search:=true
```

检查 publisher:

```bash
ros2 topic info /spot1/object_search_target_pose --no-daemon
ros2 topic info /spot1/object_search_target_viz --no-daemon
ros2 topic info /spot1/score_rings --no-daemon
```

检查目标导航点:

```bash
ros2 topic echo --once /spot1/object_search_target_pose
```

如果模型图像能检测到目标, 但没有目标导航点:

- 检查 `/spot1/model_visualization` 中目标 mask 是否覆盖目标
- 检查是否存在可投影 geometric frontier
- 检查 `target_min_score` 是否过高
- 检查 `/spot1/scored_nav_graph` 中 frontier node 是否带有 `frontier_scores`

如果 RViz2 报:

```text
Multiple Markers in the same MarkerArray message had the same ns and id
```

优先检查同一个 MarkerArray 中清理 marker 和新增 marker 是否复用了相同 namespace 和 id, 当前实现用 `score_rings_clear` 和 `object_search_clear` 避免这个问题
