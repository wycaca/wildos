# DLIO 接入方案

日期: 2026-07-17

## 1. 结论

- 仿真和真机都可以接入 DLIO
- 真机默认必须使用 DLIO, 或使用经过同等验证的 6DoF LiDAR-inertial odometry
- Unity 和 Isaac Sim 保留平台真值 odom 作为评测基准, DLIO 模式下不得把真值 pose 或 TF 接入 WildOS 运行链路
- DLIO 作为 WildOS 外部组件运行, 默认不加入 WildOS GPU 容器
- WildOS 继续只依赖标准 ROS2 odom, TF 和 PointCloud2 接口, 不在 graph, visual scoring 或 planner 内部耦合 DLIO

## 2. 当前状态和缺口

| 项目 | 当前状态 | 接入 DLIO 前必须完成 |
|---|---|---|
| WildOS odom 输入 | `/spot1/odom_for_scoring` | 上游切换为 DLIO odom 和 DLIO TF |
| elevation 点云 | 原始点云经过 `pointcloud_axis_adapter` | DLIO 模式改为直接消费 odom frame 的 deskewed 点云 |
| Unity 和 Isaac Sim | 使用平台 odom 或 TF | 增加 IMU, 点时间字段和 DLIO 轨迹评测 |
| robot profile | 仍是占位配置 | 核对真实 Ouster raw points, IMU, frame 和 TF topic |
| DLIO 参数 | 仓库内没有平台配置 | 增加仿真模板和真机标定配置入口 |
| 启动 | 不启动也不检查 DLIO | 增加显式 localization backend 和启动前检查 |
| TF ownership | 当前由平台链路提供 | DLIO 模式下只允许一个 `odom -> base_link` 发布者 |

当前 `pointcloud_axis_adapter` 只保留 XYZ 字段, 会丢弃 intensity, ring 和逐点时间等字段, 因此 DLIO 必须直接订阅传感器原始 PointCloud2, 不能订阅 adapter 输出

## 3. 上游和社区参考

### 3.1 官方 DLIO ROS2 分支

[DLIO ROS2 branch](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/tree/feature/ros2) 已验证 Ubuntu 22.04 和 ROS2 Humble, 输入为 `sensor_msgs/msg/PointCloud2` 和 6 轴 `sensor_msgs/msg/Imu`

本方案检查时的 ROS2 分支 head 为:

```text
c8acc37100e349d70a9d8432d656cbce7e5072cd
```

官方实现要求重点包括:

- LiDAR 和 IMU 时间同步
- `base_link` 到 LiDAR 和 IMU 的外参
- IMU bias 和 scale-misalignment 等内参
- 原始点云保留 DLIO 所需的逐点时间信息
- `use_sim_time` 与运行环境一致

[官方 ROS2 launch](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/feature/ros2/launch/dlio.launch.py) 的主要输出为:

```text
dlio/odom_node/odom
dlio/odom_node/pose
dlio/odom_node/path
dlio/odom_node/keyframes
dlio/odom_node/pointcloud/keyframe
dlio/odom_node/pointcloud/deskewed
```

[官方 odom 实现](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/blob/feature/ros2/src/dlio/odom.cc) 还会发布 `odom -> base_link`, `base_link -> imu` 和 `base_link -> lidar` TF, deskewed cloud 的 frame 为 odom

官方 `feature/ros2` 是开发分支, 首轮构建使用上述 commit, 通过验证后再作为项目固定版本, 不直接跟随浮动分支

### 3.2 社区 WildOS 实现

本地参考仓库 `external_references/nebula2-wildos-main_ws` 对应 [社区 WildOS 仓库](https://github.com/TIKTOKDAD/nebula2-wildos-main_ws), 当前记录 commit:

```text
156270a38635db38b4291cc43b15590cee37514c
```

其中 `dlio_odom_twist_adapter` 解决社区 A300 仿真中 DLIO twist 实际沿世界轴表达, 但消息声明为 `base_link` 表达的问题, 核心变换为:

```text
v_body = R(world_from_body)^T * v_world
```

该适配器只可作为条件兼容层, 不是 DLIO 本体, 也不应默认复制到当前 Go2 和 ROS2 Humble 链路

只有实测确认 DLIO twist 坐标语义错误, 且 Nav2 或控制器确实消费该 twist 时才启用适配器, 当前 graph 和 visual scoring 主要消费 pose, 不依赖该适配器

## 4. 目标架构

```text
raw PointCloud2 ----------------------+
                                      |
raw Imu ------------------------------+-> external DLIO
                                             |
                                             +-> DLIO odom
                                             |     |
                                             |     +-> odom_frame_adapter
                                             |             |
                                             |             +-> /spot1/odom_for_scoring
                                             |
                                             +-> odom -> base_link TF
                                             |
                                             +-> deskewed PointCloud2
                                                       |
                                                       +-> elevation_mapping_cupy
                                                               |
                                                               +-> current WildOS mainline

simulator ground truth odom or TF -> evaluation only
```

接口职责:

- DLIO 消费原始 LiDAR 和 IMU, 输出连续 odom 和 deskewed cloud
- `odom_frame_adapter` 只统一 topic, frame 和 pose 来源, 不发布动态 TF
- 官方 deskewed cloud 已在 odom frame, DLIO 模式默认绕过当前 `pointcloud_axis_adapter`
- 仿真原始点云轴向需要修正时, 优先配置 DLIO LiDAR 外参, 必须新增 adapter 时也要保留原始 fields 和 timestamp
- WildOS 主链继续消费 `/spot1/odom_for_scoring` 和 elevation GridMap
- 仿真真值只用于 DLIO 误差统计, 不参与定位 fallback

## 5. Topic 和 frame 契约

### 5.1 建议内部接口

| 语义 | 建议 topic | 消息类型 |
|---|---|---|
| DLIO 原始点云输入 | profile 指定, 不统一改名 | `sensor_msgs/msg/PointCloud2` |
| DLIO IMU 输入 | profile 指定, 不统一改名 | `sensor_msgs/msg/Imu` |
| DLIO odom | `/spot1/dlio/odom_node/odom` | `nav_msgs/msg/Odometry` |
| DLIO deskewed cloud | `/spot1/dlio/odom_node/pointcloud/deskewed` | `sensor_msgs/msg/PointCloud2` |
| WildOS 标准 odom | `/spot1/odom_for_scoring` | `nav_msgs/msg/Odometry` |
| elevation 对齐点云 | profile 的 `aligned_lidar_topic` | `sensor_msgs/msg/PointCloud2` |
| 仿真真值 odom | 平台原 topic | `nav_msgs/msg/Odometry` |

`topic_profiles.yaml` 建议新增平台接口字段:

```yaml
dlio_pointcloud_input_topic: <raw sensor pointcloud>
dlio_imu_input_topic: <raw sensor imu>
dlio_odom_topic: /spot1/dlio/odom_node/odom
dlio_deskewed_topic: /spot1/dlio/odom_node/pointcloud/deskewed
imu_frame: <platform imu frame>
ground_truth_odom_topic: <simulation only>
```

这些字段只保存平台 topic 和 frame 差异, DLIO 算法参数和标定值不放进 topic profile

### 5.2 TF ownership

DLIO 模式必须满足:

- 动态 `odom -> base_link` 只有一个 owner
- 平台真值 TF, 旧 odom-to-TF bridge 和 DLIO 不得同时发布同一 transform
- 官方 DLIO scan-rate TF 隔离到 raw topic, adapter 只转发 `base_link -> imu/lidar` 外参
- canonical `odom -> base_link` 从 DLIO odom 重建, 避免原始 TF 落后于 IMU-rate odom
- DLIO 输出 frame 必须与 profile 的 `global_frame`, `odom_parent_frame`, `odom_child_frame` 一致
- 当前 graph launch 对 `/tf` 存在 namespace remap, 实施前必须用 live ROS graph 确认 DLIO 和所有消费者位于同一个 TF channel
- 外部 DLIO 实现也必须保证 canonical TF 与 odom pose 同源且时间戳一致

真机不允许在 DLIO TF 丢失时静默回退到消息 pose 或其他定位源, 应停止主链并输出明确错误

## 6. 配置设计

建议新增独立目录:

```text
graph_construction/configs/dlio/
  unity.yaml
  isaac.yaml
  robot.example.yaml
```

配置边界:

- `unity.yaml` 和 `isaac.yaml` 保存可复现的仿真参数
- `robot.example.yaml` 只说明字段, 不提交伪造外参或默认真机标定
- 真实机器人配置由部署目录或挂载文件注入
- 配置必须覆盖 `use_sim_time`, frame, deskew, preprocessing, IMU calibration, LiDAR/IMU extrinsics 和 GICP 参数
- 同一次实验同时记录 DLIO commit, 参数文件 hash, simulator scene 和 sensor rate

官方默认外参和 IMU 内参只能用于启动验证, 不能作为真机验收配置

## 7. 启动设计

保留唯一主入口 `scripts/start_wildos_elevation.sh`, 不新增第二套 WildOS 启动脚本

在现有集成 launch 增加:

```text
localization_backend:=platform|dlio
launch_dlio:=false|true
dlio_params_file:=<path>
```

语义:

- `platform`, 保持当前 Unity 和 Isaac Sim 真值定位链路
- `dlio`, WildOS 只消费 DLIO odom, TF 和 deskewed cloud
- `launch_dlio:=false`, DLIO 由宿主机或独立 sidecar 启动, WildOS 只检查接口
- `launch_dlio:=true`, 当前 ROS workspace 已安装固定版本 DLIO 时由 launch include 启动
- DLIO backend 自动设置 `publish_lidar_static_tf:=false` 和 `launch_pointcloud_axis_adapter:=false`

推荐默认值:

| profile | 默认 backend | `use_sim_time` |
|---|---|---|
| `unity` | `platform` | `true` |
| `isaac` | `platform` | `true` |
| `robot` | `dlio` | `false` |

仿真 DLIO 实验显式传入:

```bash
WILDOS_TOPIC_PROFILE=isaac \
  ./scripts/start_wildos_elevation.sh \
  localization_backend:=dlio \
  launch_dlio:=true
```

外部 DLIO 与 WildOS 容器必须使用相同 ROS domain, 兼容 RMW 和可达的 TF topic, 容器继续使用 host network

## 8. 仿真接入

仿真接入的目的有两个:

- 在有真值轨迹时调通 DLIO 参数和 frame 契约
- 提前暴露逐点时间, IMU 同步, TF ownership 和重定位失败问题

Unity 和 Isaac Sim 分别执行以下前置检查:

1. 确认 PointCloud2 fields, 尤其是 DLIO 所需的逐点时间字段
2. 确认 IMU 同时提供 angular velocity 和 linear acceleration
3. 确认点云与 IMU stamp 使用同一仿真时钟
4. 确认 `/clock` 连续且 DLIO `use_sim_time=true`
5. 确认 simulator 不再向运行链路发布冲突的 `odom -> base_link`
6. 单独运行 DLIO, 再接入 elevation mapping 和 WildOS

如果仿真点云缺少可靠逐点时间, 不能仅靠 topic 名称宣称完成 DLIO 接入, 应先修改传感器 publisher 或明确记录该仿真只能做接口测试

评测时同时记录:

- DLIO 与 ground truth 的 ATE 和 RPE
- 静止漂移, 回环后漂移和快速转向表现
- deskewed cloud 相对原始 cloud 的墙面和地面稳定性
- DLIO 输出频率, 延迟, timestamp 单调性和丢帧
- DLIO-only 模式下 elevation map, graph 和目标搜索是否连续

真值指标阈值应在首轮基线数据后确定, 不在没有传感器频率和场景长度的情况下预设虚假精度门槛

## 9. 真机接入

真机首选 Ouster 点云和同一设备的 IMU, 以降低时间同步和外参误差

上线前必须完成:

1. 获取未经过 XYZ-only 处理的 Ouster raw PointCloud2
2. 核对点云逐点时间字段和 Ouster profile
3. 完成硬件时间同步或验证统一时钟源
4. 标定 `base_link -> lidar` 和 `base_link -> imu`
5. 标定 IMU bias 和 scale-misalignment
6. 用 rosbag 单独验证 DLIO 初始化, 运动和重启
7. 确认唯一动态 TF owner
8. 再接入 elevation mapping, graph 和完整目标搜索

当前 robot profile 的 `/spot1/ouster/front/points_filtered` 不能直接认定为 DLIO 原始输入, 必须通过真机 `ros2 topic info -v` 和 PointCloud2 fields 核对后再确定 raw topic

## 10. 社区 twist adapter 的采用条件

只有同时满足以下条件才引入社区式 `dlio_odom_twist_adapter`:

- 实测 DLIO twist 随机器人航向变化后仍沿 odom 世界轴表达
- 下游 Nav2 或控制器需要标准 body-frame twist
- 旋转前后线速度和角速度向量模长保持一致
- 适配后的直行 `linear.x` 和横向 `linear.y` 符合实车运动

不满足上述条件时不启用, 防止对已经正确的 body-frame twist 做二次旋转

## 11. 故障处理原则

- 缺少 IMU, 时间戳回退, TF 冲突或 odom 出现 NaN 时立即判定 DLIO 不健康
- deskewed cloud 超时后暂停 elevation 输入, 不改用原始 cloud 静默继续
- 仿真 DLIO 失败时不自动切回 ground truth, 需要显式重启并切换 backend
- 真机 DLIO 失败时停止下发新路径, 保留诊断数据, 不使用未验证定位继续执行
- QoS 不匹配通过 profile 或 launch 显式修正, 不在算法节点内硬编码平台例外

## 12. 实施步骤和 TODO

### P0, 接口探测

- [x] 构建并验证候选 DLIO commit `c8acc37100e349d70a9d8432d656cbce7e5072cd`
- [x] 记录 Unity 点云 fields, IMU topic, rate, QoS 和 stamp
- [ ] 记录 Isaac Sim 点云 fields, IMU topic, rate, QoS 和 stamp
- [ ] 记录真机 Ouster raw points, IMU, frame, rate, QoS 和时间同步方式
- [x] 核对 Unity `/tf`, DLIO `/spot1/tf` 和 `/spot1/tf_static` 的真实发布与订阅关系

### P1, DLIO standalone

- [ ] 新增 DLIO 参数模板和真机外部标定文件入口
- [ ] 在 rosbag, Unity 和 Isaac Sim 中分别启动 DLIO
- [ ] 验证 odom, TF, deskewed cloud, stamp 和 frame 一致性
- [ ] 确认 DLIO 初始化和重启行为

### P2, WildOS 接线

- [x] 扩展 `topic_profiles.yaml` 和 `topic_profiles.py`
- [x] 增加 `localization_backend` 和 `launch_dlio` launch 参数
- [x] 让 DLIO odom 经 adapter 输出 `/spot1/odom_for_scoring`
- [x] 让 odom frame 的 deskewed cloud 直接进入 elevation mapping
- [x] DLIO 模式关闭 pointcloud axis adapter 和重复 LiDAR static TF
- [x] 使用隔离 TF channel 禁止平台真值参与 DLIO 运行链路
- [x] 更新 `docs/details/topics.md` 和接入文档

### P3, 验证和回归

- [x] 增加 profile key 和 launch wiring 测试
- [ ] 增加运行期重复 TF owner 检查
- [ ] 对比 raw cloud 与 deskewed cloud 的 elevation map 稳定性
- [ ] 完成 Unity 和 Isaac Sim 的 DLIO-only 全链路实验
- [x] 完成 Unity DLIO-only 到 navigation graph 的全链路冒烟验证
- [ ] 完成真机静止, 直行, 转向, 上下坡和长时间运行测试
- [ ] 完成 graph, planner 和 object search 全链路回归
- [ ] 根据实测决定是否引入 twist adapter

## 13. 完成标准

只有满足以下条件才可把 DLIO 接入标记为完成:

- WildOS 运行链路只有 DLIO 一个定位源
- 动态 `odom -> base_link` 只有一个发布者
- raw PointCloud2 和 IMU 时间同步已验证
- 真机外参和 IMU 内参来自实际标定
- `/spot1/odom_for_scoring` 连续, timestamp 单调, 无 NaN 和明显跳变
- elevation mapping 直接消费 odom frame 的 DLIO deskewed cloud
- 仿真真值只出现在评测工具中
- Unity, Isaac Sim 和真机至少分别完成一次记录完整的验证
- DLIO 进程退出或数据超时时, WildOS 能显式停止而不是静默 fallback

## 14. Unity 接入实现

2026-07-17 已完成 Unity 静态接线:

- Unity DLIO 原始点云为 `/livox/lidar`
- Unity DLIO IMU 为 `/livox/imu`
- DLIO odom 输出为 `/spot1/dlio/odom_node/odom`
- DLIO deskewed cloud 输出为 `/spot1/dlio/odom_node/pointcloud/deskewed`
- DLIO odom 经 `odom_frame_adapter` 以 message pose 输出 `/spot1/odom_for_scoring`
- elevation mapping 直接消费 odom frame 的 deskewed cloud
- DLIO 模式绕过 `pointcloud_axis_adapter`, 原始点云直接进入 DLIO
- 官方 DLIO TF 隔离到 `/spot1/dlio/odom_node/tf_raw`, `dlio_tf_adapter` 重建 `/spot1/tf`
- DLIO 与 WildOS 使用 `/spot1/tf` 和 `/spot1/tf_static`, Unity真值继续保留在 `/tf` 用于评测
- DLIO 配置位于 `graph_construction/configs/dlio/unity.yaml`
- 固定依赖位于 `dependencies/dlio.repos`

导入并构建 DLIO:

```bash
cd /mnt/hhd/han/wildos_ws
vcs import src < src/nebula2-wildos/dependencies/dlio.repos
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select direct_lidar_inertial_odometry graph_construction \
  --symlink-install
```

启动 Unity DLIO 全链路:

```bash
cd /mnt/hhd/han/wildos_ws/src/nebula2-wildos
WILDOS_TOPIC_PROFILE=unity \
  ./scripts/start_wildos_elevation.sh \
  localization_backend:=dlio \
  launch_dlio:=true \
  do_object_search:=true
```

运行前仍需核对:

- 当前 `/livox/lidar` 包含 FLOAT32 `x`, `y`, `z`, `intensity` 和 UINT8 `line`, 但没有逐点时间字段
- Unity DLIO 配置因此显式关闭 deskew, 发布端补充有效 Livox `timestamp` 后再启用
- `/livox/imu` 的 angular velocity, linear acceleration, frame 和 stamp 有效
- 点云与 IMU stamp 使用同一 `/clock`
- `/livox/imu.header.frame_id` 已确认是 `livox_frame`
- Unity 实测 `base_link -> livox_frame` 外参为 `[0.093, 0.0, 0.334]m`，DLIO LiDAR 和 IMU 外参必须保持一致

### 14.1 Unity 实测结果

2026-07-17 在 `ROS_DOMAIN_ID=89`, `rmw_zenoh_cpp` 下完成实测:

- `/livox/lidar` 和 `/livox/imu` 均为 `RELIABLE`, `KEEP_LAST(10)`, `VOLATILE`, 实测发布频率分别约 `10 Hz` 和 `100 Hz`
- 点云 fields 为 FLOAT32 `y`, `z`, `x`, `intensity` 和 UINT8 `line`, 没有 `timestamp`, `time` 或 `t`
- IMU 包含 angular velocity 和 linear acceleration, `frame_id=livox_frame`
- 固定 commit DLIO 完成 IMU 初始化并发布 `/spot1/dlio/odom_node/odom`, `/spot1/dlio/odom_node/pointcloud/deskewed` 和 `/spot1/tf`
- DLIO odom 由 adapter 转为 `/spot1/odom_for_scoring`, timestamp 保持原值
- elevation mapping 成功消费 DLIO cloud 并发布 GridMap
- graph construction 成功收到 DLIO odom 和 GridMap, 首帧导航图包含 19 个节点和 35 条边
- WildOS 模型加载完成并收到首帧同步输入, planner 成功订阅 DLIO odom

2026-07-20 坐标与高程复核:

- DLIO 使用独立 `dlio_odom`，避免与 Unity `odom_3D` 同名但不同原点
- IMU 标定稳定后用 `/unity/odom` 做一次启动锚定，运动增量仍由 DLIO 提供
- 修正 LiDAR/IMU 安装偏移和重复重力姿态后，DLIO 点云转换到 Unity 世界坐标的逐点误差中位数约 4.1 mm，最大约 6.1 mm
- 机器人脚下地图地面由错误的约 -0.51 m 恢复到约 -0.058 m，首帧导航图为 26 个节点和 40 条边

后续长时间验证修正:

- `/livox/lidar` 实测约 `10 Hz`, `/livox/imu` 实测约 `100 Hz`
- Unity 每帧约 40000 点中约一半为无效零点, 进入 DLIO crop 后保留约 19800 个有效点
- 官方 `adaptive: true` 在当前 Unity 稀疏扫描上约 40 秒后发散, position 达到数万米且 bias 饱和
- 固定 `adaptive: false` 后独立 DLIO 静止运行约 98 秒, distance to origin 保持约 3 mm
- WildOS 动态 TF QoS 改为 `RELIABLE`, 与 Unity 和 DLIO TF publisher 一致
- 清理旧 launch 后 `/spot1/tf`, DLIO odom 和 scored graph 均只有一个 publisher
- 修复后完整链路持续运行超过 2 分钟, DLIO position 保持毫米级, `/spot1/scored_nav_graph` 稳定约 `2 Hz`
- 启动脚本增加重复 launch 检查, 旧 elevation launch 未退出时拒绝启动第二套实例

2026-07-20 输入链路修正:

- Unity 发布端已修复重复 timestamp, 删除 WildOS 内部 LiDAR 和 IMU 去重逻辑
- 故障日志显示输入转发节点每 10 秒只收到约 97 帧 IMU, 与 LiDAR 帧数近似相同
- DLIO 改为直接订阅 `/livox/lidar` 和 `/livox/imu`, Python 节点只在输出侧门控约 10 Hz 的 deskewed cloud
- 健康失败不再停止原始传感器输入或要求重启, 后续健康 odom 可自动恢复 canonical odom、TF 和点云输出
- 删除输入节点后实测原始 `/livox/imu` 仍约 10.02 Hz, header stamp 间隔约 0.1 s, 说明 Unity 的 200 Hz 配置尚未反映到 ROS publisher

该验证证明 Unity 接线和 DLIO-only 主链可运行, 但不等于完成算法精度验收。由于当前点云没有逐点时间字段, DLIO 将传感器识别为 unknown 并关闭 deskew, 因此还需补充有效 Livox `timestamp` 后评测 ATE, RPE 和运动去畸变效果
