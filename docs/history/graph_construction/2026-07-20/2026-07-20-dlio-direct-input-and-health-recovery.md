# DLIO 直接输入与健康恢复修复

## 故障证据

故障日志每 10 秒的累计增量约为:

```text
pointcloud +97
imu +97
```

用户侧预期 `/livox/imu` 已提升到 200 Hz, 但故障链路实际只收到约 10 Hz。随后健康检查记录位置误差仅 0.043 m、速度 0.179 m/s, 但姿态误差达到 10.76 deg, 触发永久冻结。冻结后 canonical TF、odom、GridMap 更新和评分路径依次停止。

删除输入中转并让 DLIO 直接订阅后再次实测:

- `/livox/imu` 到达频率约 10.02 Hz
- 连续 header stamp 间隔约 0.1 s
- `/livox/lidar` 到达频率约 10.01 Hz
- DLIO odom 内部输出约 100 Hz

因此 10 Hz 不是去重节点造成, Unity 的 200 Hz 配置尚未实际作用到 ROS publisher。直接输入仍用于彻底删除去重逻辑并避免不必要的 Python 传感器中转, 但不能补偿缺失的 IMU 测量。

## 修复

```text
/livox/lidar ------------------------------> DLIO
/livox/imu --------------------------------> DLIO
                                              |
                                              +-> odom -> dlio_tf_adapter
                                              |            |
                                              |            +-> aligned odom
                                              |            +-> /spot1/tf
                                              |            +-> healthy
                                              |
                                              +-> deskewed_raw
                                                     |
                                              dlio_output_guard
                                                     |
                                              deskewed -> elevation
```

- 删除 LiDAR 和 IMU timestamp 去重代码及输入转发节点
- DLIO 直接订阅原始 LiDAR 和 IMU, 避免不必要的 Python 传感器中转
- 新增 `dlio_output_guard`, 只门控低频 DLIO 输出点云
- 健康失败时不再停止原始传感器输入, 只暂停异常 odom、TF 和下游点云
- adapter 持续检查后续 odom, 恢复健康后自动恢复发布, 不再要求重启链路

## 验证

- graph construction 全量 64 项测试通过
- `graph_construction` 使用 `--symlink-install` 编译通过
- install 目录中的旧 `dlio_input_filter` 可执行文件已移除
- 最小 DLIO launch 已确认官方节点直接订阅 `/livox/lidar` 和 `/livox/imu`, 输出点云经过独立健康门控
- 静止运行中 aligned odom 与 Unity 位置误差约 4.7 mm, `healthy=true`
- `graphnav_planner` 14 项核心 GTest 通过
- 静止完整链路持续超过 5 分钟保持 `healthy=true`, 未再次触发 10 deg 姿态冻结

## 相机同步与路径恢复

DLIO 修复后继续发现 Unity 相机 stamp 约 294 s, 同时 odom 和 nav graph 已约 1490 s。WildOS 的 8 路 ApproximateTimeSynchronizer 无法配对, 所以没有 scored graph 和 Path。

Unity profile 新增 `camera_stamp_mode=now`, `camera_stamp_adapter` 将三路 compressed image 和 camera info 重打到当前 `/clock`。最终完整启动记录:

- 第一帧相机 stamp 从 424.700 s 重打到 2144.156 s
- WildOS 收到第一帧同步输入并立即找到相机 TF
- 高程初始化平均地面高度约 -0.018 m
- 首帧盲区状态为 `known_ground`, 脚下修补为 0
- planner 选择 `reason=initial_forward_route`
- planner 实际发布 `poses=3, frame=odom_3D` 的 Path

机器人在验收期间保持静止, 约 20 秒后出现 `no_path_progress` 属于未执行路径后的预期恢复逻辑, 不代表 Path 未发布。

Unity ROS publisher 的原始 IMU 仍需真正提升到 200 Hz, 再完成运动、转向和长期高程一致性验收。

当前固定 DLIO 源码已将硬编码 100 Hz odom timer 参数化为 `odom/publishRate`, Unity 设置为 20 Hz。该设置只减少重复 odom 和 pose 消息, 不改变 10 Hz IMU 输入、LiDAR scan matching 或内部状态传播, 因此不能替代 Unity IMU 频率修复。

完整验收命令:

```bash
./scripts/start_wildos_elevation.sh do_object_search:=true
```

重点检查:

```bash
ros2 topic hz /livox/imu
ros2 topic hz /spot1/dlio/odom_node/odom
ros2 topic hz /spot1/dlio/odom_node/pointcloud/deskewed
ros2 topic echo --once /spot1/dlio/odom_node/healthy
ros2 topic echo --once /spot1/scored_nav_graph
ros2 topic echo --once /spot1/graphnav_planner/path
```
