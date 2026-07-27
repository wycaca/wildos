# DLIO 坐标与 TF

> 更换 LiDAR、IMU、外参或 topic 时参见 [环境配置](environment.md)

## 1. 这条链路解决什么问题

DLIO 使用 LiDAR 和 IMU 估计机器人的连续位姿

Unity 真值里程计只做两件事:

- 启动时确定 DLIO 地图在 `odom_3D` 中的初始位置和方向
- 运行时作为误差诊断参考

Unity 真值不能持续替换 DLIO 位姿, 否则建图点云、odom 和 TF 会来自不同位置源

## 2. 当前坐标链

```text
odom_3D
  -> dlio_odom       启动时锁定的固定对齐
  -> base_link       DLIO 持续发布的机器人位姿
  -> livox_frame     LiDAR 和 IMU 外参
  -> *_camera        三路相机外参
```

对应消息:

| 数据 | frame 表达 |
|---|---|
| DLIO 原始 odom | `dlio_odom -> base_link` |
| 对齐后的 odom | `odom_3D -> base_link` |
| 主链动态 TF | `odom_3D -> dlio_odom -> base_link` |
| DLIO 输出点云 | `dlio_odom` |
| 三路相机 | `base_link -> front/left/right_camera` |

对齐后的 odom 与两段 TF 链表示同一个位姿

## 3. 启动对齐

系统在 DLIO IMU 标定稳定后, 查找与 DLIO 时间最接近的 `/unity/odom`

```text
固定对齐 = Unity 参考位姿 × DLIO 初始位姿的逆
```

该对齐只计算一次

后续机器人运动完全来自 DLIO, 不会随 Unity 参考里程计反复校正

## 4. LiDAR 和 IMU 外参

Unity 当前把 LiDAR 和 IMU 视为同一个 `livox_frame`

```text
base_link -> livox_frame
translation = [0.093, 0.0, 0.334] m
rotation = identity
```

DLIO 源码中 `baselink2imu` 和 `baselink2lidar` 均按 `base_link -> sensor` 使用, 平移单位是米

由于 LiDAR 和 IMU 的 child frame 相同, DLIO 原始 TF 中可能出现两条相同的 `base_link -> livox_frame`

适配器只转发一条, 避免重复发布同一外参

真机上如果 LiDAR 和 IMU 不共用同一物理坐标系, 必须改为各自真实的 frame 和标定外参

DLIO 环境参数位于 `graph_construction/configs/dlio/<profile>.yaml`

其中 frame、外参、重力和 `pointcloud/deskew` 属于硬件接入参数, GICP、voxel、keyframe 和 `odom/geo/*` 属于数据质量调优参数

## 5. TF 发布权

DLIO 模式的发布权如下:

| TF | 唯一 owner |
|---|---|
| `odom_3D -> dlio_odom` | `dlio_tf_adapter` |
| `dlio_odom -> base_link` | `dlio_tf_adapter` |
| `base_link -> livox_frame` | `dlio_tf_adapter` 转发 DLIO 外参 |
| `base_link -> *_camera` | 三个静态相机 TF 节点 |

官方 DLIO 的 `/tf` 被改到 `/spot1/dlio/odom_node/tf_raw`

Unity 原始 `/tf` 与 WildOS 使用的 `/spot1/tf` 隔离

因此主链中不会同时出现 Unity 和 DLIO 发布同一个动态 child frame

## 6. 方向差诊断

每 30 秒的低频日志区分四个值:

| 日志字段 | 含义 |
|---|---|
| 启动原始方向差 | DLIO 原始 yaw 减 Unity 参考 yaw |
| 当前对齐后方向差 | 启动对齐后, 当前 DLIO yaw 减参考 yaw |
| 累计变化 | 对齐后方向差相对启动时继续变化了多少 |
| 最大对齐后方向差 | 本次运行中出现过的最大绝对方向差 |

判断方法:

- 启动原始方向差较大, 但当前对齐后和累计变化接近 0, 属于固定初始坐标差
- 当前对齐后方向差随时间持续增长, 属于累计漂移
- 方向差增长同时伴随地图重影和相机投影偏移, 优先检查 IMU 方向、外参和时间同步

该诊断不修改位姿, 也不参与健康门控

## 7. 自动化验证

当前测试覆盖:

- 静止输入不会产生累计 yaw 变化
- DLIO 和参考同时旋转 90 度时, yaw 方向和角度单位正确
- 固定初始偏置与后续累计漂移可以分开
- 对齐 odom 与 TF 父子 frame 和组合位姿一致
- LiDAR 和 IMU 共用 frame 时不会重复转发外参

2026-07-23 验证结果: DLIO 相关 37 项自动化测试通过, `graph_construction` 构建通过

Unity 完整运行后仍需根据新日志确认真实漂移是否下降
