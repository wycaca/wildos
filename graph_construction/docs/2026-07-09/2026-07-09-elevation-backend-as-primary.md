# 2026-07-09 elevation/2.5D 后端主线化

## 背景

社区 `external_references/nebula2-wildos-main_ws` 不在 WildOS 仓库内实现点云转地图

论文和社区 README 都把 `Elevation Mapping CuPy` 作为外部几何建图后端, `graphnav_builder` 只消费 `grid_map_msgs/GridMap`

## 目标

把当前自研 2D `livox_grid_builder` 从默认主线降级为 fallback

将默认推荐链路改为:

```text
PointCloud2 + odom / TF
  -> pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> /elevation_mapping_node/elevation_map_raw
  -> graph_construction GridMap path
```

## 当前代码链路

- `scripts/start_wildos_elevation.sh` 是新的 elevation/2.5D 主启动入口
- `scripts/start_wildos_3d.sh` 只保留为兼容 wrapper, 内部转发到 `start_wildos_elevation.sh`
- `scripts/start_wildos_2d.sh` 保留为 2D OccupancyGrid fallback
- `graph_construction/launch/elevation_visual_navigation_sim.launch.py` 启动 `elevation_mapping_cupy`
- `graph_construction/configs/graph_construction_elevation.yaml` 使用 `grid_input_type: grid_map`
- `graph_construction/configs/topic_profiles.yaml` 新增 `pointcloud_input_topic`, 让 elevation 原始点云输入与 2D fallback 点云输入分离

## 问题分析

Unity 2D fallback 当前为了稳定使用 `/mapokk`, 这个 topic 已经过外部节点按 `/unity/odom` 转到 `odom_3D`

如果 elevation 后端也复用 `lidar_topic=/mapokk`, 就会把已经对齐过的点云再次作为 LiDAR frame 输入处理, 职责混乱, 也容易重新引入地图旋转或 TF 错配

因此 profile 中保留:

- `lidar_topic`, 供 2D fallback 使用
- `pointcloud_input_topic`, 供 elevation/2.5D 后端使用

Unity profile 下:

- `lidar_topic: /mapokk`
- `pointcloud_input_topic: /livox/lidar`
- `global_frame_3d: odom_3D`

## 修改内容

- 新增 `scripts/start_wildos_elevation.sh`
- 将 `scripts/start_wildos_3d.sh` 改为兼容 wrapper
- 更新 `README.md`, elevation/2.5D 成为推荐默认链路
- 更新 `graph_construction/docs/AGENT_README.md`, 明确 2D 只是 fallback
- 更新 topic profile 文案和 Unity elevation 输入 topic
- 更新 `elevation_visual_navigation_sim.launch.py`, `pointcloud_input_topic` 默认读取独立 profile key

## 验证步骤

语法和配置检查:

```bash
python3 -m py_compile graph_construction/launch/elevation_visual_navigation_sim.launch.py graph_construction/launch/wildos_2d_sim.launch.py
bash -n scripts/start_wildos_elevation.sh scripts/start_wildos_3d.sh scripts/start_wildos_2d.sh
```

运行时检查:

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
ros2 pkg prefix elevation_mapping_cupy
ros2 pkg executables elevation_mapping_cupy
```

Unity elevation 主线启动:

```bash
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh do_object_search:=true
```

如果 Unity 缺少 `base_link -> livox_frame` TF, 临时加:

```bash
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh do_object_search:=true publish_lidar_static_tf:=true
```
