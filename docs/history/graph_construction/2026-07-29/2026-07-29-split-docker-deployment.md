# WildOS 双主机拆分 Docker 部署

日期: 2026-07-29

## 1. 部署目标

当前实机计算分工:

```text
x86 主机
  Livox MID360 驱动
  -> /livox/lidar + /livox/imu
  -> DLIO
  -> /cloud_registered
  -> /odom
  -> /tf 和 /tf_static

AGX Orin 相机容器
  2 台 D435i + 1 台 D435if
  -> front, left, right 彩色图像
  -> CameraInfo
  -> 相机内部 TF

AGX Orin WildOS 容器
  -> elevation mapping
  -> graph construction
  -> visual navigation
  -> object target fusion
  -> goal mux
  -> graph planner
  -> Path
```

AGX Orin 上的点云、高程图、运动轨迹和高程图生成已经完成实机验证

本轮只拆分运行职责, 不修改已经验证的 Orin canonical 输入:

- `/cloud_registered`
- `/odom`
- `/tf`
- `/tf_static`

## 2. 文件职责

| 文件 | 职责 |
|---|---|
| `docker/Dockerfile.x86_64` | x86 MID360 和 D-LIO 共用镜像 |
| `dependencies/x86_localization.repos` | 固定 Livox SDK2、Livox ROS Driver 2 和 D-LIO 版本 |
| `docker/build_x86_localization.sh` | 编译 Livox SDK2、驱动、D-LIO 和定位适配 |
| `compose.x86_64.lidar-dlio.yaml` | 独立运行 x86 `lidar` 和 `localization` 服务 |
| `docker/entrypoint.lidar.mid360.sh` | 校验 MID360 配置并启动 Livox 驱动 |
| `docker/healthcheck.lidar.mid360.sh` | 检查原始点云、IMU 和逐点时间字段 |
| `docker/entrypoint.localization.sh` | 校验定位配置并启动 DLIO |
| `docker/healthcheck.localization.sh` | 检查 DLIO 进程和 canonical 输出 |
| `graph_construction/launch/dlio_localization.launch.py` | DLIO、健康门控和 canonical TF |
| `.env.x86_64.lidar-dlio.example` | x86 雷达和 DLIO 环境模板 |
| `docker/Dockerfile.camera.orin` | Orin 三相机镜像 |
| `docker/entrypoint.camera.orin.sh` | 按序列号启动 front、left、right |
| `docker/healthcheck.camera.orin.sh` | 检查三路 CameraInfo |
| `compose.orin.wildos-cameras.yaml` | Orin WildOS 和三相机服务 |
| `.env.orin.wildos-cameras.example` | Orin WildOS 和三相机环境模板 |

`compose.orin.wildos.yaml` 用于不启动相机容器的单 WildOS 部署

完整逐文件说明和部署步骤见 `docs/details/docker_deployment.md`

## 3. x86 定位部署

### 3.1 准备 DLIO 配置

DLIO 配置必须使用现场标定结果, 不能直接使用 Unity 配置

至少核对:

- LiDAR 和 IMU topic
- LiDAR 和 IMU frame
- `base_link` 到 LiDAR 外参
- `base_link` 到 IMU 外参
- IMU 内参
- 点云逐点时间字段和单位
- `pointcloud/deskew`
- 重力和 GICP 参数

复制环境模板:

```bash
cp .env.x86_64.lidar-dlio.example .env.x86_64.lidar-dlio
cp docker/config/MID360_config.example.json /absolute/path/MID360_config.json
```

编辑:

```dotenv
MID360_CONFIG_FILE=/absolute/path/MID360_config.json
POINTCLOUD_INPUT_TOPIC=/livox/lidar
IMU_INPUT_TOPIC=/livox/imu
POINTCLOUD_OUTPUT_TOPIC=/cloud_registered
ODOM_OUTPUT_TOPIC=/odom
DLIO_CONFIG_FILE=/absolute/path/to/robot_dlio.yaml
```

`MID360_CONFIG_FILE` 和 `DLIO_CONFIG_FILE` 是 x86 主机路径, Compose 会分别只读挂载到两个服务

MID360 JSON 中所有 `host_net_info` IP 必须填写 x86 雷达网卡地址，`lidar_configs.ip` 填写雷达地址

### 3.2 构建和启动

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  build lidar localization

docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  up -d
```

查看日志:

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  logs -f lidar localization
```

### 3.3 输出验收

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --field fields --once
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic echo /cloud_registered --field header --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

验收要求:

- `/cloud_registered` 和 `/odom` 时间戳持续递增
- 点云包含 DLIO 去畸变需要的真实逐点时间
- `odom -> base_link` 只有一个 publisher
- DLIO 不健康时 `/cloud_registered` 暂停输出
- DLIO 恢复后 canonical 输出自动恢复
- Orin 不再启动第二套 DLIO 或 canonical odom publisher

## 4. Orin 三相机部署

### 4.1 相机角色

三台设备必须按序列号固定角色:

| 角色 | 型号 | 序列号 |
|---|---|---|
| front | D435i 或 D435if | 现场填写 |
| left | D435i 或 D435if | 现场填写 |
| right | D435i 或 D435if | 现场填写 |

不能依赖 `/dev/videoN`, 设备编号可能在重启后变化

当前相机容器只启用彩色流:

- 深度关闭
- 红外关闭
- 相机 IMU 关闭
- CameraInfo 保留
- 相机内部 TF 保留

DLIO 继续使用雷达惯导 IMU, 不消费三台 RealSense IMU

### 4.2 准备配置

```bash
cp .env.orin.wildos-cameras.example .env.orin.wildos-cameras
```

填写三个序列号:

```dotenv
FRONT_CAMERA_SERIAL=...
LEFT_CAMERA_SERIAL=...
RIGHT_CAMERA_SERIAL=...
```

安装和标定完成后填写外参:

```dotenv
FRONT_CAMERA_TRANSFORM="x y z qx qy qz qw"
LEFT_CAMERA_TRANSFORM="x y z qx qy qz qw"
RIGHT_CAMERA_TRANSFORM="x y z qx qy qz qw"
```

外参表达 `base_link -> <name>_link`

外参留空时相机图像仍可启动, 但相机 TF 不连接 `base_link`, 不能进行视觉评分和目标三维定位验收

### 4.3 构建和启动

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  build cameras wildos

docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  up -d
```

两个服务相互独立, 可以分别启动和重启

单独重启相机:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  restart cameras
```

单独重启 WildOS:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  restart wildos
```

### 4.4 相机接口

每个 `{}` 展开为 `front`、`left`、`right`:

```text
/spot1/realsense/{}/color/image_raw
/spot1/realsense/{}/color/image_raw/compressed
/spot1/realsense/{}/color/camera_info
{}_color_optical_frame
```

检查:

```bash
ros2 topic hz /spot1/realsense/front/color/image_raw
ros2 topic hz /spot1/realsense/left/color/image_raw
ros2 topic hz /spot1/realsense/right/color/image_raw
ros2 topic echo /spot1/realsense/front/color/camera_info --once
ros2 run tf2_ros tf2_echo base_link front_color_optical_frame
ros2 run tf2_ros tf2_echo base_link left_color_optical_frame
ros2 run tf2_ros tf2_echo base_link right_color_optical_frame
```

如果 D435if 不能被当前 `realsense2_camera` 驱动识别, 只替换相机镜像和相机服务, 不修改 WildOS topic 契约

## 5. 双主机通信

x86 和 Orin 必须保持相同环境:

```bash
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

必须确认:

- 两台主机系统时间同步
- 防火墙允许 DDS 通信
- 点云跨机传输没有持续丢包
- `/cloud_registered` 到 Orin 后消息年龄稳定
- x86 和 Orin 不重复发布相同 TF child frame

建议先只启动 x86 定位和 Orin WildOS, 对比拆分前后的轨迹和高程图

确认几何链路不变后再启动相机容器和目标搜索

## 6. 启动顺序

```text
1. 启动 LiDAR 和 IMU 驱动
2. 启动 x86 localization
3. 验证 /cloud_registered, /odom 和 TF
4. 启动 Orin cameras
5. 验证三路图像, CameraInfo 和相机 TF
6. 启动 Orin WildOS
7. 验证 elevation map, NavigationGraph 和 Path
8. 启用目标搜索
```

## 7. 当前待现场确认

- 三台相机序列号和 front、left、right 对应关系
- D435if 在 Orin 当前 JetPack 上的驱动识别
- 三相机实际分辨率和稳定帧率
- 三相机硬件或系统时间同步
- 三组 `base_link -> camera_link` 标定外参
- 真机 DLIO 配置和逐点时间字段
- x86 到 Orin 的点云带宽、延迟和 DDS 稳定性
