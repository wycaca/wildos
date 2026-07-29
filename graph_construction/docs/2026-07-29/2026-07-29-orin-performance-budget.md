# AGX Orin 实机性能预算

日期: 2026-07-29

## 1. 结论

既定部署方案可以继续使用:

- x86 主机接入 LiDAR 和 IMU, 运行 D-LIOM
- AGX Orin 接收 `/odom` 和 `/cloud_registered`
- AGX Orin 运行三路相机、视觉模型、高程图、导航图和轻量导航
- 轻量导航按照最多占用 2 个 CPU 核心估算
- WildOS 视觉处理限制为 10 Hz, 相机继续发布 15 Hz

该方案的 CPU 和内存余量充足, 主要风险是视觉模型与高程图竞争 GPU, 以及跨主机 PointCloud2 的 DDS 带宽放大

## 2. 实机平台

实测设备为 NVIDIA Jetson AGX Orin Developer Kit:

| 项目 | 实测值 |
|---|---:|
| CPU | 12 核 Cortex-A78AE |
| CPU 最高频率 | 约 2.2 GHz |
| 内存 | 64 GB |
| 功耗模式 | MAXN |
| 当前可用内存 | 约 51 GB |
| 三相机输入 | 640 x 480, 15 Hz |

视觉模型已经加载但尚未接入实机点云和里程计时:

| 容器 | CPU | 内存 |
|---|---:|---:|
| WildOS | 约 16%, 即 0.16 个核心 | 约 1.62 GB |
| 三相机 | 约 26%, 即 0.26 个核心 | 约 170 MB |

此时 WildOS 在等待上游输入, GPU 为 0%, 不能用该状态代表完整运行负载

## 3. 三相机视觉模型性能

使用三路真实压缩图像进行 500 次持续推理:

| 指标 | 实测值 |
|---|---:|
| 图像批次 | 3 x 640 x 480 |
| 平均耗时 | 44.4 ms |
| P50 | 44.5 ms |
| P95 | 45.3 ms |
| 最大耗时 | 46.0 ms |
| 最大吞吐 | 22.5 个三相机批次每秒 |
| GPU 使用率 | 88% 至 99% |
| 推理进程 CPU | 约 0.5 至 1.1 个核心 |
| CUDA 峰值分配 | 约 1.67 GB |
| CUDA 峰值保留 | 约 3.94 GB |

相机 15 Hz 对应每 66.7 ms 到达一组三相机图像, 模型 P95 为 45.3 ms, 只留下约 21.4 ms 处理图像解码、投影、评分和发布

因此视觉处理改为显式限制 10 Hz:

```yaml
processing_rate_hz: 10.0
```

处理定时器由该参数计算, 单帧输入缓冲设置为保留最新同步帧, 不追赶已经过期的图像

10 Hz 下模型理论 GPU 时间占比约为:

```text
44.4 ms x 10 Hz = 444 ms/s, 约 44%
```

这可以为 CuPy 高程图、目标投影和瞬时 CUDA 峰值留下更稳定的余量

## 4. 完整系统资源预算

以下估算假设 D-LIOM 在 x86 主机运行, AGX Orin 只承担 canonical 点云和里程计的接收与下游处理:

| 模块 | AGX CPU 估算 | GPU 估算 | 内存估算 |
|---|---:|---:|---:|
| 三路 RealSense | 0.2 至 0.5 核 | 0% | 0.2 至 0.5 GB |
| 10 Hz 视觉模型和解码 | 0.5 至 1.2 核 | 约 40% 至 55% | 2 至 5 GB |
| 点云接收、TF 和 DDS | 0.3 至 0.8 核 | 0% | 0.5 至 1 GB |
| CuPy 高程图 | 0.5 至 1.2 核 | 约 5% 至 20% | 1 至 3 GB |
| 导航图构建 | 0.6 至 1.5 核 | 0% | 0.5 至 2 GB |
| 轻量导航 | 约 2 核 | 0% | 小于 1 GB |
| ROS2、监控和其他节点 | 0.5 至 1 核 | 0% | 0.5 至 1 GB |
| 合计 | 约 4.6 至 8 核 | 约 45% 至 75% | 约 6 至 13 GB |

预计剩余资源:

- CPU 平均剩余约 4 至 7 核
- 瞬时高负载下保守剩余约 2 至 4 核
- 系统可用内存预计仍超过 40 GB
- GPU 通常剩余约 25% 至 55%

如果 D-LIOM 改为在 AGX Orin 运行, 预计还会增加 2 至 4 个 CPU 核心和 2 至 6 GB 内存, 总 CPU 可能达到 7 至 11 核, 不建议作为首选方案

## 5. `/cloud_registered` 四订阅者风险

2026-07-29 在 AGX Orin 实机检查:

```text
Topic: /cloud_registered
Type: sensor_msgs/msg/PointCloud2
Publisher count: 0
Subscription count: 4
```

检查时 x86 D-LIOM 尚未启动, 因此 publisher 为 0, 但 AGX WildOS 侧已经存在 4 个订阅者

实测 endpoint:

| 订阅节点 | 订阅数量 | 用途 |
|---|---:|---|
| `object_target_fusion` | 1 | 点云与视觉 Mask 融合 |
| `elevation_mapping_node` | 1 | 高程图更新 |
| `pipeline_performance_monitor` | 2 | 原始点云和对齐点云性能监测 |

当前 platform profile 中 `raw_lidar_topic` 和 `aligned_pointcloud_topic` 都解析为 `/cloud_registered`, 导致 `pipeline_performance_monitor` 对同一 topic 建立两个订阅

其中一个监控订阅属于可消除的冗余, 在修复前会重复执行统计回调, 并且可能使跨机 DDS publisher 建立额外 reader 数据路径

这是明确的性能风险点

跨主机 DDS 可能为多个 reader 分别发送序列化后的 PointCloud2 数据, 实际带宽取决于 DDS 实现、QoS、传输模式和订阅者发现结果

以 100,000 点、每点 32 字节、10 Hz 为例:

```text
单份点云流量 = 100000 x 32 x 10 = 32 MB/s
四份独立传输 = 32 x 4 = 128 MB/s
```

四份独立传输可能接近或超过千兆以太网的实际有效带宽, 同时增加 x86 序列化和 AGX 反序列化开销

接入 x86 D-LIOM 后必须记录:

- x86 网卡发送吞吐
- AGX 网卡接收吞吐
- `/cloud_registered` 实际频率和消息大小
- DDS publisher 到 reader 的连接数量
- 四个 subscription endpoint 是否仍然存在
- `pipeline_performance_monitor` 的重复订阅是否已经消除
- 点云消息源时间到 AGX 回调时间的 P50 和 P95
- 是否出现丢帧、突发积压或延迟持续增长

建议:

- 原始 LiDAR 点云只在 x86 本地供 D-LIOM 使用
- AGX 只接收 deskew 后的必要点云
- 高频点云使用 Best Effort 和 Keep Last 1
- 可视化使用独立降采样点云
- 如确认跨机产生多份传输, 在 AGX 增加单一跨机接收入口并在本机转发
- 不在缺少实测数据时直接统一切换 DDS 实现

## 6. 完整链路验收指标

接入 x86 D-LIOM 和轻量导航后至少持续运行 15 分钟, 验收:

- 三路相机输入保持 15 Hz
- WildOS 视觉处理保持 10 Hz 左右
- 视觉输入只处理最新帧, source age 不持续增长
- 导航图输出不低于 1.8 Hz
- 导航图消息年龄 P95 不高于 800 ms
- CPU 平均总占用不高于 9 个核心
- 导航控制核心不被其他节点长期抢占
- GPU 不长期保持 95% 以上
- `/cloud_registered` 网络流量不接近链路上限
- 系统可用内存保持 10 GB 以上
- 容器不存在持续增长的消息队列或内存占用

最终性能结论需要以点云、D-LIOM、视觉、高程图和轻量导航同时运行时的实机数据为准
