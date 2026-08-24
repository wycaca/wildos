# 实机环境配置

## 1. 配置文件职责

| 文件 | 内容 |
|---|---|
| `graph_construction/configs/topic_profiles.yaml` | `robot` profile 的 topic、frame 和 DDS |
| `graph_construction/configs/dlio/mid360.yaml` | x86 D-LIO 参数和传感器外参 |
| `graph_construction/configs/elevation_mapping.yaml` | GO2 高程图和启动先验 |
| `graph_construction/configs/graph_construction_elevation.yaml` | 实机地图分类、盲区和图参数 |
| `visual_navigation/configs/wildos_nav_conf.yaml` | 三相机视觉配置 |
| `.env.x86_64.lidar-dlio` | x86 MID360 和 D-LIO 宿主机文件路径 |
| `.env.orin.wildos-cameras` | 相机 AGX 序列号、USB 路径、外参和模型路径 |

YAML 和 Compose 保存全系统固定配置，`.env` 只保存主机路径、设备身份和标定数据

## 2. DDS 和网络

两台部署容器固定使用:

```dotenv
ROS_DOMAIN_ID=2
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

x86 和相机 AGX 通过 `192.168.50.0/24` 有线网络通信

当前 GO2 网络分工:

| 主机 | 网卡 | 地址 | 用途 |
|---|---|---|---|
| 相机 AGX | `wlP1p1s0` | `10.72.15.133/24` | 无线管理 |
| 相机 AGX | `eno1` | `192.168.123.99/24` | GO2 底层主机直连, 不得修改或断开 |
| 相机 AGX | `enx00e03a151de5` | `192.168.50.2/24` | x86 DDS 专用链路, 无默认网关 |
| 点云 x86 | `wlp1s0` | `10.72.249.164/23` | 无线管理 |
| 点云 x86 | `eno1` | `192.168.50.1/24` | 相机 AGX DDS 专用链路, 无默认网关 |
| 点云 x86 | `enx00e03b8511b9` | `192.168.1.50/24` | MID360 专用链路, 雷达地址为 `192.168.1.136` |

在 x86 上配置或恢复 AGX 专用链路:

```bash
cd /home/ks-x86/wildos_ws/src/nebula2-wildos
bash scripts/configure_x86_agx_link.sh eno1
```

脚本只创建或更新 `wildos-agx-link`, 不会触碰雷达网卡、无线管理网卡或默认路由. 两端均应显示物理载波后再验证:

```bash
ping -c 3 192.168.50.1  # 在相机 AGX 执行
ping -c 3 192.168.50.2  # 在点云 x86 执行
```

Domain 和 RMW 是实机 topic 契约的一部分，变更时必须同时修改两台主机和 `robot` profile

## 3. x86 传感器和定位

x86 环境文件配置挂载路径和注册点云输出频率:

```dotenv
MID360_CONFIG_FILE=./docker/config/MID360_config.json
DLIO_CONFIG_FILE=./graph_construction/configs/dlio/mid360.yaml
OUTPUT_POINTCLOUD_RATE_HZ=0.0
```

MID360 JSON 中主机网卡地址和雷达地址必须与现场网络一致

## 4. 相机 AGX `robot` profile

```text
/cloud_registered
  -> pointcloud_relay
  -> /spot1/cloud_registered_local, 约 10 Hz, frame=dlio_odom
  -> elevation mapping

/odom
  -> odom_frame_adapter
  -> /spot1/odom_for_scoring
```

不得只修改点云 header 来代替坐标变换

`pointcloud_relay` 不修改点坐标、字段和 header, 只限制跨机大点云的本机扇出和输出频率
输入 frame 不是 `dlio_odom` 时会丢弃消息, 应先修复 D-LIO 输出或增加独立 TF 转换节点

## 5. TF

必须存在连续且无冲突的链路:

```text
odom -> dlio_odom -> base_link -> lidar_link
                             -> imu_link
                             -> front_link -> front_color_optical_frame
                             -> left_link  -> left_color_optical_frame
                             -> right_link -> right_color_optical_frame
```

当前 `base_link` 是雷达参考原点。三相机位于雷达下方同一平面, 朝向分别为前、左、右；`base_link -> <name>_link` 当前使用近似值, 结构固定后必须重新标定

MID360 当前安装倾角为 10 度。接入 GO2 机身 TF 后使用一条 `go2_base_link -> base_link` 表达安装外参, 不得再次修改 D-LIO 的雷达内部旋转

## 6. 高程图和图构建

`use_initializer_at_start` 当前关闭, 以避免人工地面先验越过真实墙体或障碍。站立状态注册点云的地面峰值约为 -0.44 m, 同时雷达原点约为 0.22 m, 因此站立高度回退值使用 0.65 m:

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `resolution` | 0.2 m | 高程图分辨率 |
| `map_length` | 30 m | rolling map 边长 |
| `max_height_range` | 1.0 m | 排除天花板 |
| `initialize_tf_offset` | -0.65 m | 当前禁用, 仅保留为站立状态无地面样本时的名义值 |
| `initialize_tf_grid_size` | 1.0 m | 初始锚点方形边长 |
| `dilation_size_initialize` | 2 cell | 初始化膨胀 |
| `robot_blind_zone_radius` | 0.4 m | 只修补雷达正下方三相机结构盲区 |
| `robot_blind_zone_elevation_search_radius` | 1.0 m | 从邻近可见地面估计盲区高程 |
| `robot_ground_height_offset` | 0.65 m | 附近无可见地面时的站立状态回退值 |
| `robot_ground_elevation_tolerance` | 0.2 m | 防止机身或低障碍被误选为地面 |

调整启动先验时必须同时检查人工区域是否连接真实地面、是否越过墙体以及是否覆盖近场障碍

## 7. 三相机

- 前相机 D435if
- 左右相机 D435i
- 狗头保留的 D435i 不在三台已配置序列号中, 容器不会打开它. 现场条件允许时应断开以释放 USB 带宽和供电余量
- 驱动彩色流 640 × 480, 15 Hz
- WildOS 处理 10 Hz
- 图像 topic 使用压缩传输
- 相机时间戳保持驱动原值

外参为空时只能验证图像, 不能验收视觉评分、LiDAR 投影和三维目标定位

## 8. 上线检查

```bash
ros2 topic list -t
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic hz /spot1/realsense/front/color/image_raw/compressed
ros2 topic hz /spot1/realsense/left/color/image_raw/compressed
ros2 topic hz /spot1/realsense/right/color/image_raw/compressed
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link front_color_optical_frame
```

按源数据、时间、TF、高程图、导航图、视觉、Path 的顺序排查
