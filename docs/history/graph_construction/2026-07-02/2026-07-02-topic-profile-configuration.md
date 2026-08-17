# Topic Profile 配置说明

日期: 2026-07-02

## 背景

当前项目需要同时支持 Isaac Sim, Unity 仿真和后续真实机器狗

三类环境的 ROS topic 和 frame 不一致, 如果继续把 topic 分散写在 launch 默认值, node YAML 和脚本中, 后续切换环境会反复漏改

## 目标

将环境差异集中到一份 topic profile 配置:

```text
graph_construction/configs/topic_profiles.yaml
```

完整启动入口保持不变:

```bash
./scripts/start_wildos_2d.sh
./scripts/start_wildos_3d.sh
```

## 当前 profile

`isaac`:

- LiDAR: `/unitree_go2/lidar/points`
- aligned LiDAR: `/unitree_go2/lidar/points_aligned`
- odom input: `/odom`
- camera image: `/unitree_go2/{}_cam/color_image`
- camera info: `/unitree_go2/{}_cam/info`

`unity`:

- LiDAR: `/livox/lidar`
- aligned LiDAR: `/livox/lidar_aligned`
- odom input: `/unity/odom`
- camera image: `/camera/{}/color/image/compressed`
- camera info: `/camera/{}/color/camera_info`

`robot`:

- LiDAR: `/spot1/ouster/front/points_filtered`
- aligned LiDAR: `/spot1/ouster/front/points_aligned`
- odom input: `/spot1/odom`
- camera image: `/spot1/realsense/{}/color/image_raw/compressed`
- camera info: `/spot1/realsense/{}/color/camera_info`

`robot` 目前是接真实狗前的占位 profile, 接入时应以实机 `ros2 topic list` 和 TF tree 为准更新

## 启动方式

默认使用 `isaac`:

```bash
./scripts/start_wildos_2d.sh
./scripts/start_wildos_3d.sh
```

切换环境:

```bash
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_2d.sh
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_3d.sh
```

也可以直接传 launch 参数:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity
./scripts/start_wildos_3d.sh topic_profile:=robot
```

单个 topic 可覆盖 profile:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity lidar_topic:=/custom/lidar
```

优先级:

1. launch 单项参数, 例如 `lidar_topic:=...`
2. `topic_profile_file` 中同名 profile
3. 内置 `topic_profiles.yaml`

## 当前代码链路

2D launch:

```text
topic_profiles.yaml
    -> wildos_2d_sim.launch.py
    -> livox_grid_builder --config livox_grid_builder.yaml --config-override key=value
    -> graph_construction --config graph_construction.yaml --config-override key=value
    -> wildos --config wildos_nav_sim_conf.yaml --config-override key=value
```

3D launch:

```text
topic_profiles.yaml
    -> elevation_visual_navigation_sim.launch.py
    -> elevation_mapping_node.py, 通过 launch parameters 覆盖 ROS 参数
    -> graph_construction --config graph_construction_elevation.yaml --config-override key=value
    -> wildos --config wildos_nav_sim_conf.yaml --config-override key=value
```

节点仍通过原有 `--config` 读取基础 YAML, launch 只负责把环境 topic / frame 作为覆盖项传入

当前不再生成 `/tmp/wildos_topic_profiles` 中间配置文件, 避免运行状态和源码配置脱节

## 修改建议

新增环境时:

1. 在 `topic_profiles.yaml` 添加一个 profile
2. 保持 key 名和现有 profile 一致
3. 用 `topic_profile:=new_profile --show-args` 检查 launch 参数
4. 用实际 ROS graph 验证 topic 是否存在

不要为不同环境复制新的 launch 文件, 除非节点链路本身不同

## 验证

已执行:

```bash
python3 -m py_compile graph_construction/launch/wildos_2d_sim.launch.py graph_construction/launch/elevation_visual_navigation_sim.launch.py graph_construction/graph_construction/topic_profiles.py
bash -n scripts/start_wildos_2d.sh scripts/start_wildos_3d.sh
source /opt/ros/humble/setup.bash && colcon build --packages-select graph_construction --symlink-install
./scripts/start_wildos_2d.sh --show-args
./scripts/start_wildos_3d.sh --show-args
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_2d.sh --show-args
```

并验证三套 profile 都能生成 2D 和 3D launch 节点 Action

2026-07-02 更新: 已移除临时 YAML materialize 方案, profile 覆盖项改为直接通过节点命令行参数和 ROS launch parameters 传递
