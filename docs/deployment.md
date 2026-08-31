# GO2 实机部署

## 部署结构

| 主机 | 服务 | 职责 |
| --- | --- | --- |
| x86 | `lidar` | MID360 点云和 IMU |
| x86 | `localization` | D-LIO、注册点云、odom 和 TF |
| x86 | `navigation` | 局部代价地图、A*、路径跟踪和速度生成 |
| 相机 AGX | `cameras` | 三路 RealSense 和静态 TF |
| 相机 AGX | `wildos` | 高程图、导航图、视觉、目标融合和 Planner |
| 相机 AGX 宿主机 | `wildos-go2-motion-gateway` | 最终限速、断流停车和 Unitree Sport API |

两台主机使用 ROS Domain 2、FastDDS 和专用 `192.168.50.0/24` 有线链路. 现场网卡和地址保存在部署配置中, 不要改动 GO2 底层直连或 MID360 独立网段

## 配置入口

- x86: `.env.x86_64.lidar-dlio`, `compose.x86_64.lidar-dlio.yaml`
- AGX: `.env.orin.wildos-cameras`, `compose.orin.wildos-cameras.yaml`
- 网络与 frame: `graph_construction/configs/topic_profiles.yaml`
- D-LIO: `graph_construction/configs/dlio/mid360.yaml`
- 高程图和图构建: `graph_construction/configs/*.yaml`
- 视觉、Goal Mux 和 Planner: 对应 package 的 `configs/` 或 `config/`
- RViz 图可视化: `wildos_visualization/configs/visualization.yaml`
- 局部导航和速度网关: `wildos_navigation/config/navigation.yaml`

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
scripts/go2_motion_gateway.sh start
scripts/wildos_docker.sh x86 start navigation
```

`navigation` 必须最后启动. 停机时先停止 `navigation`, 等待网关断流归零后再停止网关

生产入口默认使用 `launch_paper_rviz:=false`, 不启动 RViz 或可视化订阅. 现场调试时显式设为 `true`, 会启动独立 `wildos_visualization/graph_visualizer` 和 RViz, 并从公开 graph、scored graph、TargetEstimate 和 odom 生成拓扑、评分环和目标协方差; 两者停止不会停止图构建、视觉评分或目标融合

`update` 只重建镜像和容器, 不执行 Git 同步. 修改 `.env` 后使用 `start` 让 Compose 重建受影响服务, 只替换只读挂载的模型文件时使用 `restart`, 修改代码、Dockerfile 或依赖时使用 `update`

## 网络

当前专用链路:

- x86 `eno1`: `192.168.50.1/24`
- 相机 AGX USB 网卡: `192.168.50.2/24`
- x86 MID360 网卡: `192.168.1.50/24`, 雷达为 `192.168.1.136`
- 相机 AGX `eno1`: `192.168.123.99/24`, GO2 底层为 `192.168.123.161`

x86 专用链路需要恢复时执行:

```bash
bash scripts/configure_x86_agx_link.sh eno1
```

感知和规划链路使用 FastDDS Domain 2. GO2 底层只在 `eno1` 的 CycloneDDS Domain 0 可见, 不允许把底层 DDS 扩展到跨机网卡

导航容器通过 `192.168.50.1 -> 192.168.50.2:9999/UDP` 发送固定格式速度报文. AGX 网关只接受 `192.168.50.1`、校验 CRC、重新限幅并在 0.25 秒断流后归零

跨机只允许一个 relay 订阅 `/cloud_registered`, 避免高密度点云重复传输

## GO2 motion gateway

AGX 已安装的 `/home/agx/agent_ws` 提供 `motion motion_node`. 网关以 systemd 服务运行, 使用以下脚本管理:

```bash
scripts/go2_motion_gateway.sh install
scripts/go2_motion_gateway.sh start
scripts/go2_motion_gateway.sh status
scripts/go2_motion_gateway.sh logs
scripts/go2_motion_gateway.sh restart
scripts/go2_motion_gateway.sh stop
```

`install` 构建宿主机 `wildos_navigation` package 并安装服务, 保持服务停止且禁止开机自启
安装时仅授权 `agx` 用户免密启停和重启该网关服务, 其他 `sudo` 操作仍需密码

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
ros2 topic info /goal_pose -v
ros2 topic hz /combined_grid
ros2 topic info /path -v
ros2 topic info /spot1/graphnav_planner/path -v
ros2 topic hz /cmd_vel
ros2 topic hz /spot1/realsense/front/color/image_raw/compressed
ros2 topic hz /spot1/realsense/left/color/image_raw/compressed
ros2 topic hz /spot1/realsense/right/color/image_raw/compressed
ros2 topic hz /elevation_mapping_node/elevation_map_raw
ros2 topic hz /spot1/nav_graph
ros2 topic hz /spot1/scored_nav_graph
ros2 topic echo /diagnostics --once
ros2 run tf2_ros tf2_echo odom base_link
```

AGX 底层域验证:

```bash
source /opt/ros/humble/setup.bash
source /home/agx/agent_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eno1" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
ros2 topic info /cmd_vel -v
ros2 topic info /api/sport/request -v
```

按传感器、时间、TF、高程图、导航图、视觉、Path 的顺序排查. 定位错误已经污染地图时必须重启 `wildos`

常用日志:

```bash
scripts/wildos_docker.sh x86 logs -f localization
scripts/wildos_docker.sh x86 logs -f navigation
scripts/wildos_docker.sh orin logs -f cameras
scripts/wildos_docker.sh orin logs -f wildos
scripts/go2_motion_gateway.sh logs
```

导航详细接口、失效行为和测试步骤见[局部导航](navigation.md)

尚未完成的实机验收只记录在 [TODO](todo.md)
