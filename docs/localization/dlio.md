# x86 D-LIO 坐标与 TF

## 1. 职责

x86 主机上的 `localization` 容器使用 MID360 点云和内置 IMU 估计 GO2 连续位姿

```text
/livox/lidar + /livox/imu
  -> dlio_odom_node
  -> /wildos/dlio/odom_raw
  -> dlio_tf_adapter
  -> /odom + /tf

/wildos/dlio/pointcloud/deskewed_raw
  -> dlio_output_guard
  -> /cloud_registered
```

## 2. Frame

当前 frame 契约:

```text
odom
  -> dlio_odom
  -> base_link
  -> lidar_link
  -> imu_link
```

| 数据 | Frame |
|---|---|
| D-LIO 局部 odom | `dlio_odom -> base_link` |
| canonical odom | `odom -> base_link` |
| 注册点云 | `dlio_odom` |
| 原始点云 | `lidar_link` |
| IMU | `imu_link` |

`dlio_tf_adapter` 在没有外部参考 odom 时使用启动位姿建立固定 `odom -> dlio_odom`

## 3. 当前参数

参数文件为 `graph_construction/configs/dlio/mid360.yaml`

关键设置:

- `pointcloud/deskew: true`
- `map/waitUntilMove: false`, 静止时仍发布注册点云
- `cropBoxFilter/size: 0.4`, 排除雷达下方三相机一体结构
- `voxelFilter/res: 0.25`
- `maxCorrespondenceDistance: 0.5`
- `maxIterations: 32`

GO2 新安装后的 MID360 相对水平姿态和精确 LiDAR 外参仍需测量

## 4. Topic 所有权

| Topic | 发布者 |
|---|---|
| `/livox/lidar` | `lidar` 容器 |
| `/livox/imu` | `lidar` 容器 |
| `/odom` | `dlio_tf_adapter` |
| `/cloud_registered` | `dlio_output_guard`, 健康时默认限频 2 Hz |
| `/tf` | `dlio_tf_adapter` |

不得同时启动第二个 `/odom` 或相同 TF child frame 发布者

## 5. 验收

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /cloud_registered
ros2 topic hz /odom
ros2 topic echo /cloud_registered --field header --once
ros2 run tf2_ros tf2_echo odom base_link
```

`OUTPUT_POINTCLOUD_RATE_HZ` 控制 x86 跨机输出频率, 默认 2.0 Hz。D-LIO 内部 deskewed 点云保持原始频率, 设为 0 可关闭跨机限频

需要确认:

- 原始点云约 10 Hz
- 点云包含有效逐点 `timestamp`
- IMU 和点云使用同一时钟
- 静止时 `/cloud_registered` 仍以配置频率持续发布
- 前进、后退和转弯方向正确
- 长时间运动不出现跳变或发散

容器 `healthy` 当前只证明相关进程存在且 DDS 发送线程未阻塞, topic 内容仍需使用上述命令验收
