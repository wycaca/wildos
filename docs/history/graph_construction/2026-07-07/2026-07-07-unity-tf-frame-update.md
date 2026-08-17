# 2026-07-07 Unity TF frame update

## 背景

Unity 仿真 TF 已从早期 `map -> odom_fram -> sensor` 结构调整为:

```text
map
  -> odom_3D
      -> base_link
          -> front_camera / left_camera / right_camera / livox_frame
```

当前运行时已确认 `/tf` 中存在 `odom_3D -> base_link`, `/tf_static` 中存在 `map -> odom_3D`

## 问题

旧 profile 把 `camera_parent_frame` 和 `lidar_parent_frame` 直接配置为 `odom_3D`

这会把 `odom_3D` 同时当作里程计层和机器人本体层使用, 在新 TF 下会导致以下问题:

- WildOS 查询 `map -> front_camera` 时可能找不到同一棵 TF 树
- LiDAR grid builder 查询 `livox_frame -> map` 时依赖错误的父 frame
- 3D elevation mapping 把 `odom_child_frame` 当 `base_frame`, 导致高程图基准不是机器人本体

## 本次调整

- `topic_profiles.yaml` 新增 `base_frame`
- Unity profile 保持 `odom_child_frame: odom_3D`
- Unity profile 改为 `camera_parent_frame: base_link`
- Unity profile 改为 `lidar_parent_frame: base_link`
- 3D launch 的 elevation mapping `base_frame` 改为读取 profile 的 `base_frame`
- `AGENT_README.md` 更新 Unity 当前 TF 约定

## 目标检测射线方向修正

后续实测发现目标在机器人前方时, `/spot1/object_search_target_viz` 的检测射线指向右侧

原因是 Unity profile 仍沿用旧 `odom_fram` 的 `negative_y_forward_x_right` 相机约定:

- 旧约定下 front camera optical z 指向父 frame 的 `-Y`
- 传感器父 frame 改为 ROS `base_link` 后, `-Y` 表示机器人右侧
- WildOS 反投影检测射线使用 `R_wc @ [x, y, 1]`, 因此 TF 旋转错误会直接表现为检测射线方向错误

当前修正:

- Unity profile 的 `camera_static_tf_convention` 改为 `x_forward_y_left`
- front camera optical z 对齐 `base_link +X`, 即机器人前方
- image right 对齐 `base_link -Y`, image down 对齐 `base_link -Z`

## 设计约定

- `odom_parent_frame` / `odom_child_frame` 描述 adapted odom 的里程计层
- `base_frame` 描述机器人本体层, 给 elevation mapping 使用
- `camera_parent_frame` 和 `lidar_parent_frame` 描述 fallback static TF 的父 frame
- 传感器已有真实 TF 时, 可以通过 `publish_camera_static_tf:=false` 或 `publish_lidar_static_tf:=false` 关闭 fallback TF
- `negative_y_forward_x_right` 只适用于旧 `odom_fram` 坐标习惯, 当前 Unity `base_link` 不应使用

## 验证建议

启动 Unity 2D 后检查:

```bash
ros2 run tf2_ros tf2_echo map front_camera
ros2 run tf2_ros tf2_echo map livox_frame
ros2 topic echo --once /spot1/odom_for_scoring
```

启动 Unity 3D 后检查:

```bash
ros2 param get /elevation_mapping_node base_frame
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map livox_frame
```

如果仍出现 `Requested time ... earliest data ...`，优先确认当前运行的是重新构建后的 install tree, 因为源码中的 `livox_grid_builder` 已使用 latest TF 查询

## TF 时间外推保护

Unity 启动或重启仿真时, 图像和点云时间戳可能略早于 TF listener 缓存中的最早 TF

典型日志:

```text
Lookup would require extrapolation into the past
Requested time 2291.115479 but the earliest data is at time 2291.215088
```

本次增加保护:

- `TFLookupSubscriber` 遇到过去外推时, 会优先回退查询 latest TF
- `livox_grid_builder` 遇到启动期旧点云时, 会在已有成功 TF 后使用上一帧 TF 兜底
- `goalagnostic_scoring` 遇到零长度 frontier heading 时返回零分, 避免 `invalid value encountered in divide`

这些保护只处理启动期和仿真时间轻微抖动, 不替代真实 TF 链路检查
