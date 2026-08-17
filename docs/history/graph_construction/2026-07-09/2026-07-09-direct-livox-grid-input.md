# 2026-07-09 Direct Livox Grid Input

## 当前结论

本方案已回滚, Unity 2D 当前默认不再直接使用 `/livox/lidar`

后续实测确认, 直接 raw `/livox/lidar + TF` 会重新引入地图随机器人朝向旋转的问题。Unity 2D 当前应使用已经对齐到 `odom_3D` 的 `/mapokk`, 并设置 `lidar_assume_input_in_grid_frame=true`

## 原目标

修复 Unity 2D 地图生成链路, 让本仓库 `livox_grid_builder` 直接使用 `/livox/lidar` 原始点云生成 `/spot1/traversability_grid`, 不再依赖外部 `nav_slam/points_pub_map` 发布的 `/mapokk`

## 问题

运行图中 `/livox/lidar` 有 publisher, 但 `/mapokk` 没有 publisher

此前 Unity profile 将 `lidar_topic` 设为 `/mapokk`, 并启用 `lidar_assume_input_in_grid_frame=true`, 这会让地图生成依赖外部 topic

当只保留本仓库 WildOS/graphnav 链路, 不启动整套 `nav_slam 2dpoints.launch.py` 时, `/mapokk` 不存在, `livox_grid_builder` 收不到点云, 地图不会实际更新

## 已回滚的修改

- Unity profile 曾将 `lidar_topic` 改为 `/livox/lidar`
- Unity profile 曾将 `lidar_assume_input_in_grid_frame` 改为 `false`
- 2D grid builder 默认链路变为 `/livox/lidar + TF + /unity/odom -> /spot1/traversability_grid`
- 更新 `AGENT_README.md`, 明确 `/mapokk` 不再是默认地图输入

## 回滚修改

- Unity profile 的 `lidar_topic` 恢复为 `/mapokk`
- Unity profile 的 `lidar_assume_input_in_grid_frame` 恢复为 `true`
- 2D grid builder 默认链路恢复为 `/mapokk + /unity/odom -> /spot1/traversability_grid`
- 如果要完全去掉 `/mapokk` 外部依赖, 需要把 `/mapokk` 发布端的 odom_3D 对齐逻辑移植到本仓库, 不能直接消费 raw `/livox/lidar`

## 验证

- `topic_profiles.yaml` 可解析
- Unity profile 解析结果为 `lidar_topic=/livox/lidar`
- Unity profile 解析结果为 `lidar_assume_input_in_grid_frame=false`
- `python3 -m compileall -q graph_construction/graph_construction graph_construction/launch` 通过

## 注意

当前运行中的 launch 进程已持有旧参数, 需要重启 `wildos_2d_sim.launch.py` 才会切换到 `/livox/lidar`
