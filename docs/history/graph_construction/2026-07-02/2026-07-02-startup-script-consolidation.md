# 启动脚本整理说明

日期: 2026-07-02

## 目标

将仓库根目录 `scripts/` 下的启动入口收敛为 2D 和 3D 两类, 并确保每个启动脚本都拉起完整程序链路

## 变更范围

保留并新增的脚本:

- `scripts/start_wildos_2d.sh`
- `scripts/start_wildos_3d.sh`

删除的旧脚本:

- `scripts/start_graph_construction.sh`
- `scripts/start_graph_construction_livox.sh`
- `scripts/start_graph_construction_elevation.sh`
- `scripts/start_visual_navigation.sh`
- `scripts/start_elevation_visual_navigation.sh`

## 2D 完整启动链路

入口:

```bash
./scripts/start_wildos_2d.sh
```

对应 launch:

```bash
ros2 launch graph_construction wildos_2d_sim.launch.py
```

启动内容:

- `livox_grid_builder`, 从 LiDAR 点云生成 2D traversability grid
- `graph_construction`, 从 2D grid 构建 sparse navigation graph
- `odom_frame_adapter`, 输出 `/spot1/odom_for_scoring`
- 三相机 static TF fallback, 对应 front, left, right camera
- `wildos`, 对 graph frontier 做视觉评分
- `graphnav_planner`, 基于 `/spot1/scored_nav_graph` 规划路径
- `graphnav_path_follower`, 跟踪 planner 输出路径

默认环境:

- `ROS_DOMAIN_ID=3`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

## 3D 完整启动链路

入口:

```bash
./scripts/start_wildos_3d.sh
```

对应 launch:

```bash
ros2 launch graph_construction elevation_visual_navigation_sim.launch.py
```

启动内容:

- `pointcloud_axis_adapter`, 将 Isaac LiDAR 点云转换到 elevation mapping 期望坐标
- `elevation_mapping_node.py`, 生成 elevation GridMap
- `graph_construction`, 从 elevation traversability grid 构建 sparse navigation graph
- `odom_frame_adapter`, 输出 `/spot1/odom_for_scoring`
- 三相机 static TF fallback, 对应 front, left, right camera
- `wildos`, 对 graph frontier 做视觉评分
- `graphnav_planner`, 基于 `/spot1/scored_nav_graph` 规划路径
- `graphnav_path_follower`, 跟踪 planner 输出路径

默认环境:

- `ROS_DOMAIN_ID=3`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- 如果 `configs/fastdds_shm_profile.xml` 存在, 自动导出 FastDDS profile

## 参数约定

两个脚本都支持透传 ROS launch 参数, 例如:

```bash
./scripts/start_wildos_2d.sh log_level:=DEBUG
./scripts/start_wildos_3d.sh planner_start_delay:=10.0
```

两个脚本都支持按环境选择 topic profile:

```bash
WILDOS_TOPIC_PROFILE=isaac ./scripts/start_wildos_2d.sh
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_2d.sh
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_2d.sh
```

profile 定义在 `graph_construction/configs/topic_profiles.yaml`, 单个 topic 仍可用 launch 参数覆盖:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity lidar_topic:=/custom/lidar
```

planner 默认使用 namespace 内的 `odom_for_scoring`:

```text
~/odom -> odom_for_scoring
```

在默认 `ns=spot1` 下解析为 `/spot1/odom_for_scoring`

脚本默认优先激活仓库内 `.venv`, 如果不存在则尝试 `wildos_venv`, 也可以通过 `VENV_ACTIVATE=/path/to/venv/bin/activate` 显式覆盖

## 验证

已执行:

```bash
python3 -m py_compile graph_construction/launch/wildos_2d_sim.launch.py graph_construction/launch/elevation_visual_navigation_sim.launch.py
bash -n scripts/start_wildos_2d.sh scripts/start_wildos_3d.sh
source /opt/ros/humble/setup.bash && colcon build --packages-select graph_construction --symlink-install
./scripts/start_wildos_2d.sh --show-args
./scripts/start_wildos_3d.sh --show-args
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_2d.sh --show-args
```

本次只验证 launch 参数解析, 未启动完整运行节点
