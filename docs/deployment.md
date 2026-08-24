# GO2 实机部署

## 部署结构

| 主机 | 服务 | 职责 |
| --- | --- | --- |
| x86 | `lidar` | MID360 点云和 IMU |
| x86 | `localization` | D-LIO、注册点云、odom 和 TF |
| 相机 AGX | `cameras` | 三路 RealSense 和静态 TF |
| 相机 AGX | `wildos` | 高程图、导航图、视觉、目标融合和 Planner |

两台主机使用 ROS Domain 2、FastDDS 和专用 `192.168.50.0/24` 有线链路. 现场网卡和地址保存在部署配置中, 不要改动 GO2 底层直连或 MID360 独立网段

## 配置入口

- x86: `.env.x86_64.lidar-dlio`, `compose.x86_64.lidar-dlio.yaml`
- AGX: `.env.orin.wildos-cameras`, `compose.orin.wildos-cameras.yaml`
- 网络与 frame: `graph_construction/configs/topic_profiles.yaml`
- D-LIO: `graph_construction/configs/dlio/mid360.yaml`
- 高程图和图构建: `graph_construction/configs/*.yaml`
- 视觉、Goal Mux 和 Planner: 对应 package 的 `configs/` 或 `config/`

`.env` 只保存主机路径、设备身份、标定和模型目录. 固定运行参数保存在 Compose、launch 和 YAML

## 构建和启动

日常操作统一使用:

```bash
scripts/wildos_docker.sh <x86|orin> <config|build|update|start|stop|restart|logs|status> [service...]
```

首次或代码更新后:

```bash
scripts/wildos_docker.sh x86 config
scripts/wildos_docker.sh x86 update
scripts/wildos_docker.sh orin config
scripts/wildos_docker.sh orin update
```

推荐启动顺序:

```bash
scripts/wildos_docker.sh x86 start lidar
scripts/wildos_docker.sh x86 start localization
scripts/wildos_docker.sh orin start cameras
scripts/wildos_docker.sh orin start wildos
```

`update` 只重建镜像和容器, 不执行 Git 同步. 修改 `.env` 后使用 `start` 让 Compose 重建受影响服务, 只替换只读挂载的模型文件时使用 `restart`, 修改代码、Dockerfile 或依赖时使用 `update`

## 网络

当前专用链路:

- x86 `eno1`: `192.168.50.1/24`
- 相机 AGX USB 网卡: `192.168.50.2/24`
- x86 MID360 网卡: `192.168.1.50/24`, 雷达为 `192.168.1.136`
- 相机 AGX `eno1`: GO2 底层直连, 不参与 DDS

x86 专用链路需要恢复时执行:

```bash
bash scripts/configure_x86_agx_link.sh eno1
```

两端必须使用同一 Domain、RMW 和 FastDDS profile. 跨机只允许一个 relay 订阅 `/cloud_registered`, 避免高密度点云重复传输

## D-LIO 和 TF

```text
/livox/lidar + /livox/imu
  -> D-LIO
  -> dlio_tf_adapter -> /odom + /tf
  -> dlio_output_guard -> /cloud_registered
```

`base_link` 当前是 D-LIO 雷达参考原点. GO2 机身外参应由单独的 `go2_base_link -> base_link` 表达, 不要重复写入 D-LIO 内部 LiDAR/IMU 外参

定位异常时 guard 停止转发不可信点云. 容器 `healthy` 只证明进程存在且发送线程未阻塞, 不证明 topic、时间或 TF 正确

## 相机和模型

相机服务只打开 `.env.orin.wildos-cameras` 中配置的前、左、右序列号. 外参为空或仍为近似值时只能验收图像, 不能验收目标定位

模型目录只读挂载. `ckpts/manifest.json` 是来源、版本、大小和 SHA256 的唯一清单, 容器启动和反序列化前都会校验

登记相机:

```bash
scripts/wildos_docker.sh orin build cameras
bash scripts/deploy_orin_cameras.sh --assign front
bash scripts/deploy_orin_cameras.sh --assign left
bash scripts/deploy_orin_cameras.sh --assign right
```

## 上线验证

```bash
scripts/wildos_docker.sh x86 status
scripts/wildos_docker.sh orin status
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic hz /spot1/realsense/front/color/image_raw/compressed
ros2 topic hz /spot1/realsense/left/color/image_raw/compressed
ros2 topic hz /spot1/realsense/right/color/image_raw/compressed
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /spot1/nav_graph
ros2 topic hz /spot1/scored_nav_graph
ros2 run tf2_ros tf2_echo odom base_link
```

按传感器、时间、TF、高程图、导航图、视觉、Path 的顺序排查. 定位错误已经污染地图时必须重启 `wildos`

常用日志:

```bash
scripts/wildos_docker.sh x86 logs -f localization
scripts/wildos_docker.sh orin logs -f cameras
scripts/wildos_docker.sh orin logs -f wildos
```

尚未完成的实机验收只记录在 [TODO](todo.md)
