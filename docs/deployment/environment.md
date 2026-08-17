# 实机环境配置

## 1. 配置文件职责

| 文件 | 内容 |
|---|---|
| `graph_construction/configs/topic_profiles.yaml` | `robot` profile 的 topic、frame 和 DDS |
| `graph_construction/configs/dlio/mid360.yaml` | x86 D-LIO 参数和传感器外参 |
| `graph_construction/configs/elevation_mapping.yaml` | 小推车高程图和启动先验 |
| `graph_construction/configs/graph_construction_elevation.yaml` | 实机地图分类、盲区和图参数 |
| `visual_navigation/configs/wildos_nav_conf.yaml` | 三相机视觉配置 |
| `.env.x86_64.lidar-dlio` | x86 传感器路径和容器变量 |
| `.env.orin.wildos-cameras` | 相机 AGX 序列号、外参和模型路径 |

带 `sim` 的文件名是历史兼容命名, 当前实机 Docker 入口仍加载这些文件

## 2. DDS 和网络

两台主机统一设置:

```dotenv
ROS_DOMAIN_ID=2
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

x86 和相机 AGX 通过 `192.168.50.0/24` 有线网络通信

修改 RMW、Domain 或 FastDDS profile 后必须重启相关 ROS 进程

## 3. x86 传感器和定位

x86 环境文件至少配置:

```dotenv
POINTCLOUD_INPUT_TOPIC=/livox/lidar
IMU_INPUT_TOPIC=/livox/imu
POINTCLOUD_OUTPUT_TOPIC=/cloud_registered
ODOM_OUTPUT_TOPIC=/odom
GLOBAL_FRAME=odom
BASE_FRAME=base_link
LIDAR_FRAME=lidar_link
IMU_FRAME=imu_link
MID360_CONFIG_FILE=/absolute/path/to/MID360_config.json
DLIO_CONFIG_FILE=/absolute/path/to/graph_construction/configs/dlio/mid360.yaml
```

MID360 JSON 中主机网卡地址和雷达地址必须与现场网络一致

## 4. 相机 AGX `robot` profile

```text
/cloud_registered
  -> pointcloud_relay
  -> /spot1/cloud_registered_local, 2 Hz, frame=dlio_odom
  -> elevation mapping

/odom
  -> odom_frame_adapter
  -> /spot1/odom_for_scoring
```

不得只修改点云 header 来代替坐标变换

`pointcloud_relay` 不修改点坐标和字段, 只限制跨机大点云的本机扇出和输出频率

## 5. TF

必须存在连续且无冲突的链路:

```text
odom -> dlio_odom -> base_link -> lidar_link
                             -> imu_link
                             -> front_link -> front_color_optical_frame
                             -> left_link  -> left_color_optical_frame
                             -> right_link -> right_color_optical_frame
```

三相机 `base_link -> <name>_link` 当前使用近似值, 结构固定后必须重新标定

## 6. 高程图和图构建

当前小推车参数:

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `resolution` | 0.2 m | 高程图分辨率 |
| `map_length` | 30 m | rolling map 边长 |
| `max_height_range` | 1.0 m | 排除天花板 |
| `initialize_tf_offset` | -0.90 m | 雷达到地面高度 |
| `initialize_tf_grid_size` | 1.0 m | 初始锚点方形边长 |
| `dilation_size_initialize` | 2 cell | 初始化膨胀 |
| `robot_blind_zone_radius` | 0.8 m | 脚下种子半径 |
| `robot_blind_zone_elevation_search_radius` | 2.0 m | 最大连通搜索半径 |
| `robot_ground_height_offset` | 0.90 m | odom 原点到地面 |

调整启动先验时必须同时检查人工区域是否连接真实地面、是否越过墙体以及是否覆盖近场障碍

## 7. 三相机

- 前相机 D435if
- 左右相机 D435i
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
