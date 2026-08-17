# 实机 Docker 部署说明

本文是 `sim2real` 分支当前部署入口

## 1. 架构

```text
x86 主机, 192.168.50.1
  lidar
    -> MID360 驱动
    -> /livox/lidar + /livox/imu
  localization
    -> D-LIO
    -> /cloud_registered + /odom + /tf

相机 AGX, 192.168.50.2
  cameras
    -> D435if 前相机
    -> D435i 左右相机
  wildos
    -> 点云适配、高程图、图构建
    -> 视觉、目标融合和 Planner
```

两台主机使用有线局域网、Domain 2 和 FastDDS

## 2. Compose 文件

### `compose.x86_64.lidar-dlio.yaml`

- 平台 `linux/amd64`
- 镜像 `wildos-localization:x86_64`
- `lidar` 和 `localization` 使用同一镜像、不同入口
- `localization` 等待 `lidar` 健康后启动
- MID360 JSON 和 D-LIO YAML 以只读文件挂载
- host network 和 host IPC

### `compose.orin.wildos-cameras.yaml`

- 平台 `linux/arm64`
- `cameras` 镜像只包含 RealSense 和 TF
- `wildos` 镜像包含高程图、图、视觉和 Planner
- `wildos` 使用 NVIDIA runtime
- 模型目录只读挂载
- 两个服务可以单独重建和重启

## 3. Docker 文件

| 文件 | 作用 |
|---|---|
| `docker/Dockerfile.lidar-dlio` | 构建 Livox、D-LIO 和定位适配 |
| `docker/Dockerfile.camera.orin` | 构建三相机运行环境 |
| `docker/Dockerfile.orin` | 构建 WildOS 主镜像 |
| `docker/entrypoint.lidar.mid360.sh` | 校验配置并启动 MID360 |
| `docker/entrypoint.localization.sh` | 启动 x86 D-LIO 主链 |
| `docker/entrypoint.camera.orin.sh` | 启动三相机和静态 TF |
| `docker/entrypoint.orin.sh` | 固定使用 `robot` profile 启动 WildOS |
| `docker/healthcheck.*` | 进程和阻塞状态健康检查 |
| `docker/configure_apt_mirrors.sh` | 配置 Ubuntu 和 ROS APT 镜像 |
| `docker/install_ros_humble.sh` | 安装 ROS 2 Humble |
| `docker/fastdds.wildos.xml` | 两台主机共用 FastDDS 配置 |

健康检查当前不完整验证 topic 内容, `healthy` 不能替代 ROS topic 验收

## 4. x86 配置

```bash
cp .env.x86_64.lidar-dlio.example .env.x86_64.lidar-dlio
```

至少填写:

```dotenv
ROS_DOMAIN_ID=2
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
MID360_CONFIG_FILE=/absolute/path/to/MID360_config.json
DLIO_CONFIG_FILE=/absolute/path/to/graph_construction/configs/dlio/mid360.yaml
POINTCLOUD_INPUT_TOPIC=/livox/lidar
IMU_INPUT_TOPIC=/livox/imu
POINTCLOUD_OUTPUT_TOPIC=/cloud_registered
ODOM_OUTPUT_TOPIC=/odom
GLOBAL_FRAME=odom
BASE_FRAME=base_link
LIDAR_FRAME=lidar_link
IMU_FRAME=imu_link
```

MID360 配置中主机网卡地址和雷达 IP 必须与现场一致

验证、构建和启动:

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  config --quiet

docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  build

docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  up -d
```

x86 验收:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic echo /cloud_registered --field header --once
ros2 run tf2_ros tf2_echo odom base_link
```

## 5. 相机 AGX 配置

```bash
cp .env.orin.wildos-cameras.example .env.orin.wildos-cameras
```

至少填写:

```dotenv
ROS_DOMAIN_ID=2
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FRONT_CAMERA_SERIAL=...
LEFT_CAMERA_SERIAL=...
RIGHT_CAMERA_SERIAL=...
FRONT_CAMERA_TRANSFORM="x y z qx qy qz qw"
LEFT_CAMERA_TRANSFORM="x y z qx qy qz qw"
RIGHT_CAMERA_TRANSFORM="x y z qx qy qz qw"
WILDOS_CKPT_DIR=/absolute/path/to/ckpts
```

三台相机已经按前、左、右接入, 新安装时可逐台登记:

```bash
bash scripts/deploy_orin_cameras.sh --assign front
bash scripts/deploy_orin_cameras.sh --assign right
bash scripts/deploy_orin_cameras.sh --assign left
```

当前角色:

- front: D435if
- left: D435i
- right: D435i

当前外参仍是近似值, 精确视觉评分、LiDAR 投影和目标定位前必须完成标定

验证、构建和启动:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  config --quiet

docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  build

docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  up -d
```

相机验收:

```bash
ros2 topic hz /spot1/realsense/front/color/image_raw/compressed
ros2 topic hz /spot1/realsense/left/color/image_raw/compressed
ros2 topic hz /spot1/realsense/right/color/image_raw/compressed
ros2 run tf2_ros tf2_echo base_link front_color_optical_frame
ros2 run tf2_ros tf2_echo base_link left_color_optical_frame
ros2 run tf2_ros tf2_echo base_link right_color_optical_frame
```

WildOS 验收:

```bash
ros2 topic hz /spot1/cloud_registered_local
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /spot1/nav_graph
ros2 topic hz /spot1/scored_nav_graph
ros2 topic hz /spot1/model_visualization
```

## 6. 推荐启动顺序

1. 启动 x86 `lidar`
2. 验证原始点云和 IMU
3. 启动 x86 `localization`
4. 验证 `/cloud_registered`、`/odom` 和 TF
5. 启动相机 AGX `cameras`
6. 验证三路图像、CameraInfo 和相机 TF
7. 启动相机 AGX `wildos`
8. 验证高程图、NavigationGraph、视觉输出和 Path

## 7. 重建范围

| 修改 | 操作 |
|---|---|
| `.env.*` | 重新创建对应服务 |
| MID360 JSON | 重启 `lidar` |
| D-LIO YAML | 重启 `localization` |
| x86 定位代码或 Dockerfile | 重建 x86 镜像 |
| 相机脚本或 Dockerfile | 重建 `cameras` |
| WildOS 代码或配置 | 重建 `wildos` |
| 模型 | 重启 `wildos` |

## 8. 常见故障

### 没有雷达 topic

- 检查 MID360 和主机网卡 IP
- 检查 JSON 挂载路径
- 检查雷达供电和网线
- 查看 `lidar` 日志

### D-LIO 无输出或发散

- 检查点云逐点时间和 IMU 时间
- 检查 LiDAR/IMU frame 和外参
- 检查 7 度安装倾角
- 检查近场 crop 是否误删有效结构
- 修复后重启高程图, 不复用已污染地图

### 跨机 topic 延迟

- 确认两台主机使用 Domain 2 和 FastDDS
- 确认 `ROS_LOCALHOST_ONLY=0`
- 优先使用 `192.168.50.0/24` 有线网络
- 避免多个工具重复订阅 `/cloud_registered`
- 检查系统时间同步

### 高程图人工区域过大

确认当前镜像加载:

```text
initialize_tf_grid_size=1.0
dilation_size_initialize=2
robot_blind_zone_radius=0.8
robot_blind_zone_elevation_search_radius=2.0
```

### 容器 healthy 但没有数据

当前健康检查主要检查进程和 DDS 发送线程, 必须继续执行 topic 频率、类型和单帧内容检查

## 9. 日常状态

```bash
docker ps --filter name=wildos-x86
docker ps --filter name=wildos-orin
```

测试完成后保持服务运行, 除非明确要求停止
