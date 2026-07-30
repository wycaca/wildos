# Docker 部署说明

本文档是当前实机 Docker 部署的操作入口，覆盖 x86 雷达定位主机和 AGX Orin 计算主机

日期目录下的 Docker 文档作为历史记录保留，当前部署以本文档和当前文件名为准

## 1. 部署架构

```text
x86_64 主机
  lidar 服务
    -> Livox MID360 驱动
    -> /livox/lidar
    -> /livox/imu
  localization 服务
    -> D-LIO
    -> /cloud_registered
    -> /odom
    -> /tf

AGX Orin
  cameras 容器
    -> 2 台 D435i + 1 台 D435if
    -> 三路彩色图像和 CameraInfo
    -> 相机 TF
  wildos 容器
    -> 高程图
    -> 图构建和路径规划
    -> 视觉评分和目标搜索
    -> 运动输出
```

x86 和 Orin 使用 ROS 2 DDS 通信，必须使用相同的 `ROS_DOMAIN_ID` 和 `RMW_IMPLEMENTATION`

## 2. Compose 配置文件

### 2.1 `compose.x86_64.lidar-dlio.yaml`

平台和模块:

- 平台: `linux/amd64`
- 服务: `lidar`、`localization`
- 镜像: `wildos-localization`
- Dockerfile: `docker/Dockerfile.x86_64`

职责:

- 使用同一个镜像分别运行 MID360 驱动和 D-LIO，避免重复保存 ROS、PCL 和编译依赖
- `lidar` 挂载 MID360 JSON 配置并输出 `/livox/lidar` 和 `/livox/imu`
- `localization` 挂载现场 DLIO 参数文件到 `/config/dlio.yaml`
- `localization` 等待 `lidar` 健康后启动
- 两个服务均使用 host network
- 输出 `/cloud_registered`、`/odom` 和 canonical TF
- 分别保存 ROS 日志到 `lidar_ros_logs` 和 `localization_ros_logs`

配套环境模板:

- 模板: `.env.x86_64.lidar-dlio.example`
- 部署文件: `.env.x86_64.lidar-dlio`
- Compose 只向 `lidar` 和 `localization` 注入各自需要的变量

### 2.2 `compose.orin.wildos.yaml`

平台和模块:

- 平台: `linux/arm64`
- 服务: `wildos`
- 镜像: `wildos`
- Dockerfile: `docker/Dockerfile.orin`

职责:

- 单独部署已经验证过的 WildOS 主链
- 使用 NVIDIA runtime 运行 PyTorch 和 CuPy
- 只读挂载模型目录
- 接收外部 `/cloud_registered`、`/odom`、TF 和相机 topic
- 固定使用 robot profile、platform localization 和无界面模式

此配置不启动相机驱动，适合相机服务尚未准备好或需要独立调试 WildOS 的情况

配套环境模板:

- 模板: `.env.orin.wildos.example`
- 部署文件: `.env.orin.wildos`

### 2.3 `compose.orin.wildos-cameras.yaml`

平台和模块:

- 平台: `linux/arm64`
- 服务: `cameras`、`wildos`
- 相机镜像: `wildos-cameras`
- WildOS 镜像: `wildos`
- Dockerfile: `docker/Dockerfile.camera.orin`、`docker/Dockerfile.orin`

职责:

- `cameras` 独立管理三台 RealSense
- `wildos` 运行高程图、图构建、视觉和运动主链
- 两个服务可单独构建、启动、停止和重启
- 相机服务挂载 `/dev/bus/usb` 并使用 host network
- WildOS 服务使用 NVIDIA runtime
- Compose 按服务过滤环境变量，相机容器不接收模型和导航配置，WildOS 容器不接收相机序列号

配套环境模板:

- 模板: `.env.orin.wildos-cameras.example`
- 部署文件: `.env.orin.wildos-cameras`

### 2.4 配置文件旧名称

| 旧名称 | 当前名称 |
|---|---|
| `compose.x86_64.yaml` | `compose.x86_64.lidar-dlio.yaml` |
| `compose.orin.yaml` | `compose.orin.wildos.yaml` |
| `compose.orin.split.yaml` | `compose.orin.wildos-cameras.yaml` |
| `.env.localization.example` | `.env.x86_64.lidar-dlio.example` |
| `.env.docker.example` | `.env.orin.wildos.example` |
| `.env.orin.split.example` | `.env.orin.wildos-cameras.example` |

旧名称不再作为部署入口

## 3. `docker` 目录逐文件说明

### 3.1 镜像定义

#### `docker/Dockerfile.x86_64`

x86 雷达定位镜像，只包含 Livox MID360、D-LIO 和定位输出适配所需依赖

主要操作:

- 使用 ROS 2 Humble 基础镜像
- 安装 Livox SDK2、PCL、Eigen、OpenMP、CycloneDDS 和 colcon 的构建依赖
- 根据 `dependencies/x86_localization.repos` 导入固定版本 Livox SDK2、Livox ROS Driver 2 和 D-LIO
- 编译 `livox_ros_driver2`、`direct_lidar_inertial_odometry` 和 `graph_construction`
- 安装 MID360 与定位各自的入口脚本和健康检查

该镜像不包含 WildOS 模型、PyTorch、CuPy、高程图和路径规划模块

#### `docker/Dockerfile.orin`

AGX Orin WildOS 主镜像

主要操作:

- 基于 JetPack 对应的 NVIDIA PyTorch arm64 镜像
- 安装 ROS 2 Humble 和 WildOS Python 运行依赖
- 编译高程图消息、图构建、规划和视觉导航包
- 创建系统 Python 可见的项目虚拟环境
- 校验 PyTorch、CuPy 和 CUDA
- 使用 `entrypoint.orin.sh` 启动实机主链

模型不写入镜像，通过 Compose 只读挂载到项目 `ckpts` 目录

#### `docker/Dockerfile.camera.orin`

AGX Orin 三相机镜像

主要操作:

- 使用 ROS 2 Humble arm64 基础镜像
- 安装 `realsense2_camera`
- 安装图像传输、CycloneDDS 和 TF 工具
- 使用 `entrypoint.camera.orin.sh` 启动三台相机
- 使用 `healthcheck.camera.orin.sh` 检查三路 CameraInfo

相机镜像不包含 WildOS、模型和 CUDA 推理依赖

### 3.2 Docker 构建上下文过滤

#### `docker/Dockerfile.x86_64.dockerignore`

只允许 x86 依赖描述、`graph_construction`、换源脚本、构建脚本和运行脚本进入 x86 构建上下文，避免传输模型和其他模块

#### `docker/Dockerfile.orin.dockerignore`

允许 WildOS、`elevation_mapping_cupy` 和 `graaf_vendor` 进入 Orin 构建上下文，排除 Git 数据、测试、构建产物、文档和模型

#### `docker/Dockerfile.camera.orin.dockerignore`

只允许换源脚本、相机入口脚本和相机健康检查进入构建上下文

### 3.3 容器入口脚本

#### `docker/entrypoint.lidar.mid360.sh`

用于 x86 `lidar` 服务

启动前检查:

- MID360 JSON 配置存在且语法有效
- 原始点云、IMU topic 和 LiDAR frame 已配置
- 驱动固定使用 `xfer_format=0`，确保输出包含逐点 `timestamp` 字段的 `PointCloud2`
- 驱动固定使用 `multi_topic=0`，确保 topic remap 不受设备 IP 后缀影响

检查通过后启动 `livox_ros_driver2_node`

#### `docker/entrypoint.localization.sh`

用于 x86 `localization` 服务

启动前检查:

- DLIO 参数文件存在
- 雷达和 IMU 输入 topic 已配置
- canonical 输出 topic 已配置
- `global`、`base`、LiDAR 和 IMU frame 已配置

检查通过后启动 `dlio_localization.launch.py`

#### `docker/entrypoint.orin.sh`

用于当前 Orin WildOS 镜像

主要职责:

- 加载 ROS 和工作空间环境
- 把环境变量转换为 ROS launch 参数
- 固定使用 robot profile、platform localization、无仿真时间和无 RViz 模式
- 固定禁止 Orin 启动第二套 D-LIO 和相机静态 TF
- 默认选择实机 `wildos_nav_conf.yaml`，可通过 `WILDOS_VISUAL_CONFIG` 覆盖
- 调用 `verify_runtime.py` 检查模型和 GPU
- 启动 `start_wildos_elevation.sh`

传入 `--check-config` 时只打印转换后的 launch 参数，不启动 WildOS

#### `docker/entrypoint.camera.orin.sh`

用于 Orin `cameras` 服务

主要职责:

- 强制要求 front、left、right 三个相机序列号
- 分别启动三个 `realsense2_camera` 节点
- 只启用彩色流，关闭深度、红外和相机 IMU
- 根据可选外参发布 `base_link -> camera_link`
- 容器退出时统一停止所有相机和静态 TF 进程

#### `docker/entrypoint.sh`

旧的通用 WildOS 入口，保留用于兼容其他镜像或手工运行

当前 `Dockerfile.orin` 使用 `entrypoint.orin.sh`，不直接使用此文件

### 3.4 健康检查

#### `docker/healthcheck.lidar.mid360.sh`

检查以下内容:

- `livox_ros_driver2_node` 进程存在
- 原始点云类型是 `sensor_msgs/msg/PointCloud2`
- IMU 类型是 `sensor_msgs/msg/Imu`
- 点云字段包含 D-LIO 去畸变所需的 `timestamp`
- IMU topic 能收到数据

雷达未通电、主机网卡地址不正确或 JSON 中 IP 配置错误时该服务会显示 `unhealthy`

#### `docker/healthcheck.localization.sh`

检查以下内容:

- `dlio_odom_node` 进程存在
- `dlio_tf_adapter` 进程存在
- `dlio_output_guard` 进程存在
- `/odom` 能收到一条消息
- `/cloud_registered` 能收到一条消息

没有真实雷达数据时该容器会显示 `unhealthy`，这是预期行为

#### `docker/healthcheck.camera.orin.sh`

加载 ROS 环境并调用独立的 Python 健康检查器

#### `docker/healthcheck.camera.orin.py`

创建临时 `rclpy` 节点，同时订阅 front、left、right 的彩色 `CameraInfo`

该检查器不使用 ROS 2 CLI daemon，避免健康检查超时或并发执行导致 daemon 状态损坏

任意相机未启动、序列号错误或 USB 不稳定都会使容器显示 `unhealthy`

#### `docker/healthcheck.sh`

检查 WildOS ROS launch 进程是否存在

当前 `Dockerfile.orin` 使用此健康检查

### 3.5 构建和运行辅助文件

#### `docker/build_x86_localization.sh`

构建 x86 雷达定位工作空间:

- 使用 CMake 编译并安装 Livox SDK2
- 将 Livox ROS Driver 2 切换到 ROS 2 package 描述
- 使用 colcon 编译 Livox 驱动、D-LIO 和定位适配包
- 构建结束后清理中间目录

#### `docker/config/MID360_config.example.json`

单台 MID360 的驱动配置模板

- `host_net_info` 填写 x86 雷达网卡的静态 IP 和主机端口
- `lidar_configs.ip` 填写 MID360 的 IP
- `extrinsic_parameter` 仅控制驱动发布的设备外参，需要与现场 TF 和 D-LIO 配置保持一致

#### `docker/configure_apt_mirrors.sh`

三个镜像共用的 APT 换源脚本

主要职责:

- 把 amd64 的 Ubuntu Archive 和 Security 源切换到清华 Ubuntu 镜像
- 把 arm64 的 Ubuntu Ports 源切换到清华 Ubuntu Ports 镜像
- 把已有 ROS 2 官方源切换到清华 ROS 2 镜像
- 同时处理传统 `.list` 和 deb822 `.sources` 格式
- 关闭构建过程不需要的 `deb-src` 源码索引
- 清理基础镜像继承的旧软件索引
- 缺少 CA 证书时允许首次使用 HTTP，安装证书后切换为 HTTPS

`auto` 参数根据 CA 证书是否存在选择协议，`http` 和 `https` 参数用于强制指定协议

#### `docker/build_workspace.sh`

把 Orin ROS 工作空间分成依赖阶段和应用阶段构建

- `dependencies`: 构建消息、Elevation Mapping、graaf 和几何依赖
- `application`: 构建 WildOS 应用包
- `all`: 构建全部包

脚本固定 graaf commit，并在构建后清理日志和上游 Git 数据

#### `docker/install_ros_humble.sh`

为 Orin NVIDIA 基础镜像安装 ROS 2 Humble、编译工具、图像依赖、TF、DDS 和 RViz

该脚本调用 `configure_apt_mirrors.sh`，不再独立维护 Ubuntu 换源逻辑

#### `docker/requirements-runtime.txt`

定义 WildOS 推理、视觉、几何、模型加载和 ROS Python 构建所需的 Python 依赖版本

#### `docker/ros_environment.sh`

为 Orin 容器和调试 shell 加载:

- ROS 2 Humble
- `/opt/wildos_ws` 工作空间
- 项目 Python 路径
- 模型目录
- 项目虚拟环境

#### `docker/verify_runtime.py`

WildOS 启动前检查:

- 实际 CPU 架构与镜像目标一致
- 必需模型文件存在
- PyTorch 能访问 CUDA
- CuPy 能发现 CUDA 设备
- `rclpy` 可以导入

任意条件不满足时阻止 WildOS 启动

#### `docker/benchmark_visual_model.orin.py`

使用 front、left、right 三路真实压缩图像构造一个三图 batch，加载与 Orin 部署一致的 FP16 RADIO、SigLIP2、traversability head 和 frontier head

输出模型加载时间、目标文本编码时间、三图 batch 推理平均值与 P95、输出张量尺寸和 CUDA 显存统计

该脚本不依赖雷达、里程计、导航图或相机外参，适合在完整导航链尚未接入时单独验收相机到视觉模型的链路

## 4. 部署前准备

### 4.1 仓库位置

三个 Compose 的构建上下文都是仓库父目录，建议保持以下结构:

```text
wildos_ws/src/
  elevation_mapping_cupy/
  graaf_vendor/
  nebula2-wildos/
```

所有命令均在 `nebula2-wildos` 根目录执行

### 4.2 主机要求

x86 主机:

- Docker Engine 和 Docker Compose v2
- 连接 Livox MID360 的有线网卡已设置静态 IPv4 地址
- 主机网卡 IP、MID360 IP 与驱动 JSON 位于同一子网
- 防火墙允许 JSON 中的 Livox UDP 端口
- 已准备现场 DLIO 参数文件

AGX Orin:

- Docker Engine 和 Docker Compose v2
- NVIDIA Container Runtime
- JetPack 对应的 NVIDIA PyTorch 基础镜像
- 三台 RealSense 可通过 `/dev/bus/usb` 访问
- WildOS 模型目录完整

### 4.3 DDS 和时间同步

两台主机必须保持以下配置一致:

```dotenv
ROS_DOMAIN_ID=2
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

部署前确认:

- x86 和 Orin 系统时间已同步
- 防火墙允许 ROS 2 DDS 通信
- 两台主机位于可互通网络
- 不存在第二个 `/odom` 或相同 TF child frame publisher

## 5. x86 雷达和 DLIO 部署

### 5.1 准备环境文件

```bash
cp .env.x86_64.lidar-dlio.example .env.x86_64.lidar-dlio
cp docker/config/MID360_config.example.json /absolute/path/MID360_config.json
```

至少修改:

```dotenv
MID360_CONFIG_FILE=/absolute/path/MID360_config.json
LIVOX_PUBLISH_FREQ=10.0
POINTCLOUD_INPUT_TOPIC=/livox/lidar
IMU_INPUT_TOPIC=/livox/imu
LIDAR_FRAME=lidar_link
IMU_FRAME=imu_link
BASE_FRAME=base_link
GLOBAL_FRAME=odom
POINTCLOUD_OUTPUT_TOPIC=/cloud_registered
ODOM_OUTPUT_TOPIC=/odom
DLIO_CONFIG_FILE=/absolute/path/to/robot_dlio.yaml
```

`MID360_CONFIG_FILE` 和 `DLIO_CONFIG_FILE` 必须是 x86 主机上的绝对路径

编辑 `MID360_config.json`:

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",
      "push_msg_ip": "192.168.1.5",
      "point_data_ip": "192.168.1.5",
      "imu_data_ip": "192.168.1.5",
      "log_data_ip": ""
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.12"
    }
  ]
}
```

将 `cmd_data_ip`、`push_msg_ip`、`point_data_ip` 和 `imu_data_ip` 替换为 x86 雷达网卡地址，将 `lidar_configs.ip` 替换为 MID360 地址

未采集雷达内部日志时保留 `log_data_ip` 为空，不要随意修改模板端口

检查网卡和雷达连通性:

```bash
ip -4 address
ping -c 3 192.168.1.12
```

### 5.2 验证 Compose

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  config --quiet
```

### 5.3 构建镜像

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  build lidar localization
```

### 5.4 启动和查看日志

首次部署建议先单独启动并验收雷达:

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  up -d lidar

docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  logs -f lidar
```

确认原始数据正常后启动 D-LIO:

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  up -d localization

docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  logs -f localization
```

日常启动可以直接执行 `up -d`，Compose 会等待 `lidar` 健康后启动 `localization`

### 5.5 验收

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic info /livox/lidar --verbose
ros2 topic echo /livox/lidar --field fields --once
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic echo /cloud_registered --field header --once
ros2 run tf2_ros tf2_echo odom base_link
```

确认:

- `/livox/lidar` 类型为 `sensor_msgs/msg/PointCloud2` 且字段中包含 `timestamp`
- `/livox/imu` 类型为 `sensor_msgs/msg/Imu`
- 原始点云和 IMU 时间戳同步，时间基准与 x86 系统时间一致
- 点云和里程计时间戳持续递增
- `odom -> base_link` 只有一个发布者
- 运动时轨迹方向和尺度正确
- 点云可以在 `odom` frame 下稳定显示

## 6. Orin 单 WildOS 部署

相机容器尚未准备好时使用此方式

### 6.1 准备环境文件

```bash
cp .env.orin.wildos.example .env.orin.wildos
```

至少修改:

```dotenv
ROS_DOMAIN_ID=2
POINTCLOUD_INPUT_TOPIC=/cloud_registered
ODOM_INPUT_TOPIC=/odom
WILDOS_CKPT_DIR=/absolute/path/to/nebula2-wildos/ckpts
WILDOS_ORIN_IMAGE_TAG=orin-jp62
WILDOS_ORIN_BASE_IMAGE=nvcr.io/nvidia/pytorch:24.10-py3-igpu
```

### 6.2 验证运行参数

```bash
docker compose \
  --env-file .env.orin.wildos \
  -f compose.orin.wildos.yaml \
  config --quiet

docker compose \
  --env-file .env.orin.wildos \
  -f compose.orin.wildos.yaml \
  run --rm wildos --check-config
```

### 6.3 构建和启动

```bash
docker compose \
  --env-file .env.orin.wildos \
  -f compose.orin.wildos.yaml \
  build wildos

docker compose \
  --env-file .env.orin.wildos \
  -f compose.orin.wildos.yaml \
  up -d wildos

docker compose \
  --env-file .env.orin.wildos \
  -f compose.orin.wildos.yaml \
  logs -f wildos
```

### 6.4 验收

复用已经完成的实机验收项:

- 能接收 x86 的 `/cloud_registered` 和 `/odom`
- 轨迹方向和尺度正确
- 高程图随机器人运动正确更新
- NavigationGraph 和 Path 正常生成
- 运动执行链能够消费正确路径

## 7. Orin WildOS 和三相机部署

### 7.1 确认相机序列号

连接三台相机后，在 Orin 上执行:

```bash
rs-enumerate-devices
```

记录三台设备序列号，并固定 front、left、right 角色，不使用 `/dev/videoN` 作为身份

### 7.2 准备环境文件

```bash
cp .env.orin.wildos-cameras.example .env.orin.wildos-cameras
```

填写相机序列号:

```dotenv
FRONT_CAMERA_SERIAL=...
LEFT_CAMERA_SERIAL=...
RIGHT_CAMERA_SERIAL=...
```

填写标定外参:

```dotenv
FRONT_CAMERA_TRANSFORM="x y z qx qy qz qw"
LEFT_CAMERA_TRANSFORM="x y z qx qy qz qw"
RIGHT_CAMERA_TRANSFORM="x y z qx qy qz qw"
```

外参表示 `base_link -> <name>_link`

外参暂时留空时可以验证图像，但不能完成视觉评分和三维目标定位验收

同时确认:

```dotenv
WILDOS_CKPT_DIR=/absolute/path/to/nebula2-wildos/ckpts
WILDOS_ORIN_BASE_IMAGE=nvcr.io/nvidia/pytorch:24.10-py3-igpu
WILDOS_CAMERA_BASE_IMAGE=ros:humble-ros-base-jammy
```

### 7.3 验证 Compose

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  config --quiet
```

### 7.4 先构建和启动相机

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  build cameras

docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  up -d cameras

docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  logs -f cameras
```

检查三路相机:

```bash
ros2 topic hz /spot1/realsense/front/color/image_raw
ros2 topic hz /spot1/realsense/left/color/image_raw
ros2 topic hz /spot1/realsense/right/color/image_raw
ros2 topic echo /spot1/realsense/front/color/camera_info --once
```

检查相机 TF:

```bash
ros2 run tf2_ros tf2_echo base_link front_color_optical_frame
ros2 run tf2_ros tf2_echo base_link left_color_optical_frame
ros2 run tf2_ros tf2_echo base_link right_color_optical_frame
```

### 7.5 启动 WildOS

相机和 x86 定位输出均正常后启动:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  build wildos

docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  up -d wildos

docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  logs -f wildos
```

也可以在两个镜像均已构建后一次启动:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  up -d
```

### 7.6 单独测试三相机视觉模型

完整 WildOS 容器已经运行时先停止该服务，避免同时加载两份模型:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  stop wildos
```

保持相机服务运行，执行三相机真实图像基准:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  run --rm --no-deps wildos \
  python3 /opt/wildos_ws/src/nebula2-wildos/docker/benchmark_visual_model.orin.py \
  --runs 100 \
  --object-query "blue bucket"
```

测试完成后恢复 WildOS:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  start wildos
```

该测试只验证三路图像解码、视觉模型前向和目标文本编码，完整视觉评分仍要求 `/odom`、`/cloud_registered`、导航图和相机外参可用

## 8. 日常操作

### 8.1 查看服务状态

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  ps
```

### 8.2 单独重启相机

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  restart cameras
```

### 8.3 单独重启 WildOS

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  restart wildos
```

### 8.4 停止服务

x86:

```bash
docker compose \
  --env-file .env.x86_64.lidar-dlio \
  -f compose.x86_64.lidar-dlio.yaml \
  down
```

Orin:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  down
```

命名 volume 中的 ROS 日志默认保留，执行 `down` 不会删除日志 volume

## 9. 修改后的重建范围

| 修改内容 | 需要重建 |
|---|---|
| `.env.*` 环境配置 | 通常只需重新创建对应服务 |
| DLIO YAML | 重新创建 `localization` |
| `Dockerfile.x86_64` 或定位代码 | 重建 `localization` |
| 相机入口脚本或相机 Dockerfile | 重建 `cameras` |
| WildOS Python 或 ROS 代码 | 重建 `wildos` |
| 模型文件 | 模型通过挂载提供，重启 `wildos` |

重新创建单个服务:

```bash
docker compose \
  --env-file .env.orin.wildos-cameras \
  -f compose.orin.wildos-cameras.yaml \
  up -d --force-recreate cameras
```

## 10. 常见故障

### 10.1 `localization` 显示 unhealthy

依次检查:

- LiDAR 和 IMU 驱动是否持续发布
- topic 名称是否与环境文件一致
- DLIO YAML 是否挂载成功
- 点云逐点时间字段是否满足 DLIO 配置
- `/odom` 和 `/cloud_registered` 是否有输出

### 10.2 `cameras` 显示 unhealthy

依次检查:

- `/dev/bus/usb` 是否存在并已挂载
- 三个序列号是否正确
- USB 带宽和供电是否稳定
- D435if 是否能被当前 `realsense2_camera` 识别
- 三路 `CameraInfo` 是否都有输出

### 10.3 `wildos` 无法启动

依次检查:

- NVIDIA runtime 是否可用
- `WILDOS_CKPT_DIR` 是否为绝对路径
- 模型文件是否完整
- PyTorch 和 CuPy 是否能访问 CUDA
- `/cloud_registered`、`/odom` 和 TF 是否来自 x86

### 10.4 跨主机看不到 topic

依次检查:

- 两台主机 `ROS_DOMAIN_ID` 是否一致
- `ROS_LOCALHOST_ONLY` 是否为 `0`
- `RMW_IMPLEMENTATION` 是否一致
- 防火墙和交换机是否允许 DDS 流量
- 两台主机系统时间是否同步

## 11. 推荐启动顺序

```text
1. 启动 x86 LiDAR 和 IMU 驱动
2. 启动 x86 localization
3. 验证 /cloud_registered、/odom 和 TF
4. 启动 Orin cameras
5. 验证三路图像、CameraInfo 和相机 TF
6. 启动 Orin wildos
7. 验证高程图、NavigationGraph、Path 和运动
8. 启用视觉评分和目标搜索
```
