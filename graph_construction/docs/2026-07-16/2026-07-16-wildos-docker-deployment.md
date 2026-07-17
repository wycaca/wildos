# WildOS 双架构 Docker 部署

日期: 2026-07-16

## 1. 部署范围

容器包含当前完整 WildOS elevation 主链路:

```text
pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> graph_construction
  -> WildOS and ExploRFM
  -> object_target_fusion
  -> object_search_goal_mux
  -> graphnav_planner
```

`path_follower_node` 会被编译并保留，但仍不在默认集成启动链中运行

传感器驱动、Unity、Isaac Sim、DLIO 和机器人底盘控制不放入本容器。它们通过 ROS2 topic 与容器内 WildOS 通信

## 2. 镜像

| 平台 | Dockerfile | 基础镜像 | GPU 软件栈 |
|---|---|---|---|
| x86_64 | `docker/Dockerfile.x86_64` | `nvidia/cuda:12.6.3-cudnn-runtime-ubuntu22.04` | PyTorch 2.7.0, CUDA 12.6, CuPy 13.6 |
| AGX Orin | `docker/Dockerfile.orin` | `nvcr.io/nvidia/pytorch:25.04-py3-igpu` | JetPack 6.2, NVIDIA PyTorch 2.7, CuPy 13.6 |

两个镜像都安装 ROS2 Humble，并编译当前主链路 ROS package。`graaf` 固定到已验证提交 `2a4715ff46a25048e06078ea20afc325d59f987a`，避免镜像构建随上游 `main` 漂移

Orin 镜像必须运行在 JetPack 6.2 和 L4T 36.4 系列上。不同 JetPack 版本必须同步更换 `WILDOS_ORIN_BASE_IMAGE`，不能只更换容器内 CUDA package

## 3. 构建上下文

Compose 使用 ROS workspace 的 `src` 目录作为构建上下文，目录必须保持为:

```text
wildos_ws/src/
  nebula2-wildos/
  elevation_mapping_cupy/
  graaf_vendor/
```

Dockerfile 专用 ignore 文件会排除 Git、虚拟环境、测试产物、旧 build/install/log 和未使用的 ONNX 文件

镜像内重新生成的 `build` 目录必须保留，因为 ROS2 `--symlink-install` 的 C++ executable 和 component library 会引用该目录。只删除构建日志，不删除本次镜像构建产物

## 4. 模型文件

完整镜像直接包含运行所需权重，因此构建前必须确认以下文件存在:

```text
ckpts/c-radio_v3-b_half.pth.tar
ckpts/frontier_ckpt_new.ckpt
ckpts/traversability_ckpt.ckpt
ckpts/siglip2/
```

模型文件没有提交到 Git。构建机器需要提前下载或从已验证机器复制模型目录。Dockerfile 会建立 `frontier_head.ckpt` 和 `trav_head.ckpt` 兼容链接，使独立组件入口也能使用当前权重

## 5. Compose 配置

复制环境模板:

```bash
cp .env.docker.example .env
```

真实机器人保持:

```dotenv
WILDOS_TOPIC_PROFILE=robot
USE_SIM_TIME=false
```

Unity 改为:

```dotenv
WILDOS_TOPIC_PROFILE=unity
USE_SIM_TIME=true
```

Isaac Sim 改为:

```dotenv
WILDOS_TOPIC_PROFILE=isaac
USE_SIM_TIME=true
```

Compose 使用 `network_mode: host` 和 `ipc: host`。host network 保证 ROS2 multicast、Zenoh 和传感器 topic 可以直接通信，host IPC 避免 Fast DDS shared memory 位于不同 IPC namespace

## 6. x86_64 部署

主机要求:

- Ubuntu 22.04
- NVIDIA 驱动和 NVIDIA Container Toolkit
- Docker Engine 与 Docker Compose plugin

构建并启动:

```bash
docker compose -f compose.x86_64.yaml build
docker compose -f compose.x86_64.yaml up -d
docker compose -f compose.x86_64.yaml logs -f wildos
```

停止:

```bash
docker compose -f compose.x86_64.yaml down
```

## 7. AGX Orin 部署

Orin 主机要求:

- JetPack 6.2
- L4T 36.4
- Ubuntu 22.04
- NVIDIA Container Runtime
- Docker Engine 与 Docker Compose plugin

建议直接在 Orin 上构建，避免在 x86 上使用 QEMU 模拟执行 CUDA ARM 层:

```bash
docker compose -f compose.orin.yaml build
docker compose -f compose.orin.yaml up -d
docker compose -f compose.orin.yaml logs -f wildos
```

## 8. 验证

入口脚本每次启动都会检查:

- 容器 CPU 架构与镜像类型一致
- PyTorch 能访问 CUDA
- CuPy 能枚举 CUDA device
- RADIO、Frontier、Traversability 和 SigLIP2 模型存在
- ROS2 Python 环境可以导入

手动检查:

```bash
docker compose -f compose.x86_64.yaml run --rm wildos \
  python3 /opt/wildos_ws/src/nebula2-wildos/docker/verify_runtime.py
```

Orin 使用相同命令，只需替换 Compose 文件名

启动后至少检查:

```bash
docker compose -f compose.x86_64.yaml exec wildos ros2 node list
docker compose -f compose.x86_64.yaml exec wildos ros2 topic list
```

2026-07-16 当前验证结果:

- 两份 Compose 配置均通过 `docker compose config`
- x86_64 镜像完成纯净构建，镜像 ID 为 `sha256:01bbdf09ea488028b294c88870bc09e8844b30c188bed60f614d1d079e322a63`
- x86_64 镜像大小约 16.9GB，包含约 4.9GB 模型、完整 CUDA/PyTorch 依赖、ROS2 Humble 和 RViz
- 镜像内 10 个 ROS package 全部编译通过，`planner_node` 和 `path_follower_node` 均存在
- 镜像内 NumPy 1.26.4、SciPy 1.15.3、PyTorch 2.7.0+cu126、CuPy 13.6.0、OpenCV、ExploRFM、RADIO、`rclpy` 和 `ros2_numpy` 导入通过
- 当前 x86 构建机的 NVIDIA 驱动不可通信，因此 GPU 启动被 `verify_runtime.py` 正确阻止，仍需在驱动正常的 NVIDIA 主机完成 CUDA 和传感器闭环运行
- Orin 基础镜像 manifest 已确认只提供 `linux/arm64`，但本次没有在真实 Orin 上执行镜像构建和 GPU 闭环验证

## 9. RViz

默认 `LAUNCH_PAPER_RVIZ=false`，部署容器按无界面模式运行

需要使用主机 X11 显示 RViz 时:

```bash
xhost +local:root
LAUNCH_PAPER_RVIZ=true docker compose -f compose.x86_64.yaml up
```

Compose 已挂载 `/tmp/.X11-unix` 并传递 `DISPLAY`

## 10. 架构边界

不能把 x86 镜像直接复制到 Orin，也不能把普通 PyPI x86 CUDA wheel 安装到 Orin。两种镜像共享 WildOS 源码和 ROS 接口，但 PyTorch/CUDA 基础层必须分别维护

Orin 镜像使用 NVIDIA 官方 iGPU tag，因为 Jetson GPU 驱动和 CUDA 用户态库与 JetPack 绑定。升级 JetPack 时先查 NVIDIA PyTorch for Jetson compatibility matrix，再更新基础镜像和运行验证结果

官方兼容性依据:

- [NVIDIA PyTorch for Jetson compatibility matrix](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)
- [NVIDIA NGC PyTorch tags](https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/pytorch/-/tags?_lr=1)
- [PyTorch previous versions](https://pytorch.org/get-started/previous-versions/)
- [JetPack 6.2.1 installation guide](https://docs.nvidia.com/jetson/jetpack/6.2.1/install-setup/index.html)

## 11. 文件清单

- `docker/Dockerfile.x86_64`: x86_64 CUDA 12.6 镜像
- `docker/Dockerfile.orin`: JetPack 6.2 AGX Orin 镜像
- `docker/install_ros_humble.sh`: 两个平台共用的 ROS2 Humble 和系统依赖安装
- `docker/requirements-runtime.txt`: WildOS 共用 Python 运行依赖
- `docker/build_workspace.sh`: 固定 Graaf 并构建 10 个 ROS package
- `docker/entrypoint.sh`: 环境初始化、运行前检查和默认集成启动
- `docker/verify_runtime.py`: 架构、模型、PyTorch CUDA、CuPy CUDA 和 ROS Python 检查
- `docker/healthcheck.sh`: 集成 launch 进程健康检查
- `docker/Dockerfile.x86_64.dockerignore`: x86 构建上下文过滤
- `docker/Dockerfile.orin.dockerignore`: Orin 构建上下文过滤
- `compose.x86_64.yaml`: x86_64 构建和部署
- `compose.orin.yaml`: AGX Orin 构建和部署
- `.env.docker.example`: Compose 参数模板
