# 实机 Docker 部署说明

本文是 `sim2real` 分支在 Unitree GO2 上的部署入口

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

相机 AGX 的 `eno1=192.168.123.99/24` 专供 GO2 底层主机直连, 不参与 Docker 或 DDS 配置, 不得断开. x86 的 USB 网卡 `enx00e03b8511b9=192.168.1.50/24` 专供地址为 `192.168.1.136` 的 MID360. 跨机 DDS 使用 x86 `eno1=192.168.50.1/24` 和相机 AGX `192.168.50.2/24`, 具体恢复命令见 [环境配置](environment.md)

## 2. 配置和部署参数

仓库配置定义当前实机的固定运行契约:

- Domain 2、FastDDS、非 localhost 模式
- MID360 输入 `/livox/lidar`、`/livox/imu`
- D-LIO 输出 `/cloud_registered`、`/odom`，frame 为 `odom`、`base_link`、`lidar_link`、`imu_link`
- 三相机 namespace、640 × 480、15 Hz、WildOS 目标搜索和 `robot` profile
- x86 与 AGX 镜像名、基础镜像和容器资源限制

这些值在 Compose、launch 和仓库 YAML 中维护，不应写入 `.env` 覆盖

`.env` 只保存主机或设备相关参数:

| 平台 | 参数 |
|---|---|
| x86 | MID360、D-LIO 挂载路径和注册点云输出频率 |
| 相机 AGX | 三台相机序列号、USB 路径、`base_link` 外参、`WILDOS_CKPT_DIR` |

## 3. Compose 文件

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

## 4. Docker 文件

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

## 5. Docker 控制脚本

所有 Compose 日常操作使用 `scripts/wildos_docker.sh`:

```bash
scripts/wildos_docker.sh <x86|orin> <action> [service...]
```

| 操作 | 行为 | 示例 |
|---|---|---|
| `config` | 验证 Compose 插值与结构 | `scripts/wildos_docker.sh orin config` |
| `build` | 构建镜像 | `scripts/wildos_docker.sh orin build cameras` |
| `update` | 拉取基础镜像、重建并重新创建容器 | `scripts/wildos_docker.sh x86 update localization` |
| `start` | 后台启动服务 | `scripts/wildos_docker.sh orin start wildos` |
| `stop` | 停止服务但保留容器 | `scripts/wildos_docker.sh x86 stop localization` |
| `down` | 停止并移除该平台全部容器, 保留 named volumes | `scripts/wildos_docker.sh orin down` |
| `restart` | 重启运行中的服务 | `scripts/wildos_docker.sh orin restart cameras` |
| `logs` | 查看最近 200 行日志, `-f` 持续跟随 | `scripts/wildos_docker.sh x86 logs -f localization` |
| `status` | 查看容器状态和健康检查 | `scripts/wildos_docker.sh orin status` |

自定义环境文件使用 `--env-file`，必须放在 action 后、service 前:

```bash
scripts/wildos_docker.sh x86 update --env-file /path/to/x86.env localization
```

`update` 只更新 Docker 镜像和容器，不执行 Git 同步

## 6. x86 配置

仓库已保存当前实机配置:

```dotenv
MID360_CONFIG_FILE=./docker/config/MID360_config.json
DLIO_CONFIG_FILE=./graph_construction/configs/dlio/mid360.yaml
OUTPUT_POINTCLOUD_RATE_HZ=0.0
```

MID360 配置中主机网卡地址和雷达 IP 必须与现场一致

验证、构建和启动:

```bash
scripts/wildos_docker.sh x86 config
scripts/wildos_docker.sh x86 update
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

## 7. 相机 AGX 配置

仓库已保存当前三台相机的序列号、USB 路径、近似外参和权重目录. 相机实测标定完成后直接更新 `.env.orin.wildos-cameras` 中的三条外参

关键参数:

```dotenv
FRONT_CAMERA_SERIAL=...
LEFT_CAMERA_SERIAL=...
RIGHT_CAMERA_SERIAL=...
FRONT_CAMERA_TRANSFORM="x y z qx qy qz qw"
LEFT_CAMERA_TRANSFORM="x y z qx qy qz qw"
RIGHT_CAMERA_TRANSFORM="x y z qx qy qz qw"
WILDOS_CKPT_DIR=./ckpts
```

模型文件必须与仓库中的 `ckpts/manifest.json` 完全匹配。容器启动和模型反序列化前都会校验文件大小与 SHA256, 校验失败时直接停止, 不会自动下载替代文件

更新模型时必须同步评审并更新清单中的版本、来源、大小和 SHA256。当前两个 head checkpoint 使用 `weights_only=True` 和固定类型 allowlist 加载；RADIO checkpoint 仍含模型构造元数据, 只允许在哈希校验通过后加载

相机服务只会打开环境文件中前、左、右三条序列号. GO2 狗头保留的额外 RealSense 即使被系统枚举也不会自动分配或启动. 主机没有 `rs-enumerate-devices` 时，先构建相机镜像，再逐台登记:

```bash
scripts/wildos_docker.sh orin build cameras
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
scripts/wildos_docker.sh orin config
scripts/wildos_docker.sh orin update
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

## 8. 推荐启动顺序

1. 运行 `scripts/wildos_docker.sh x86 start lidar`
2. 验证原始点云和 IMU
3. 运行 `scripts/wildos_docker.sh x86 start localization`
4. 验证 `/cloud_registered`、`/odom` 和 TF
5. 运行 `scripts/wildos_docker.sh orin start cameras`
6. 验证三路图像、CameraInfo 和相机 TF
7. 运行 `scripts/wildos_docker.sh orin start wildos`
8. 验证高程图、NavigationGraph、视觉输出和 Path

## 9. 重建范围

| 修改 | 操作 |
|---|---|
| `.env.*` | `restart` 对应服务 |
| MID360 JSON | `restart lidar` |
| D-LIO YAML | `restart localization` |
| x86 定位代码或 Dockerfile | `update` 或 `update localization` |
| 相机脚本或 Dockerfile | `update cameras` |
| WildOS 代码或配置 | `update wildos` |
| 模型 | `restart wildos` |

## 10. 常见故障

### 没有雷达 topic

- 检查 MID360 和主机网卡 IP
- 检查 JSON 挂载路径
- 检查雷达供电和网线
- 运行 `scripts/wildos_docker.sh x86 logs lidar`

### D-LIO 无输出或发散

- 检查点云逐点时间和 IMU 时间
- 检查 LiDAR/IMU frame 和外参
- 检查 10 度安装倾角和倾斜轴向
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
use_initializer_at_start=false
robot_blind_zone_radius=0.4
robot_blind_zone_elevation_search_radius=1.5
robot_ground_height_offset=0.65
```

### 容器 healthy 但没有数据

当前健康检查主要检查进程和 DDS 发送线程, 必须继续执行 topic 频率、类型和单帧内容检查

## 11. 日常状态

```bash
scripts/wildos_docker.sh x86 status
scripts/wildos_docker.sh orin status
```

测试完成后保持服务运行, 除非明确要求停止
