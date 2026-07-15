# 多视角目标融合

## 一句话说明

WildOS 负责看见目标, Target Fusion 负责记住目标位置, Goal Mux 负责选择目标, Planner 负责规划路线

## 四个模块

```text
三路相机
  -> WildOS
  -> Target Fusion
  -> Goal Mux
  -> Graph Planner
```

### WildOS

输出当前帧目标 Mask、目标名称和每路相机置信度

WildOS 不保存目标世界坐标

### Target Fusion

输入目标 Mask 和相机位姿, 可选读取 LiDAR 点云

它按照论文方法使用固定数量粒子融合不同位置、不同相机的观测

两个有效视角形成粗目标后即可引导远距离导航, LiDAR 不是定位前置条件

LiDAR 点落入目标 Mask 时可以更快收紧或锁定位置, 没有匹配点云时继续纯视觉定位

### Goal Mux

按以下顺序选择唯一高层目标:

```text
任务完成
  > 稳定融合目标
  > 两视角视觉粗目标
  > 初始探索方向
```

粒子位置只有明显变化或置信度明显提高时才更新高层目标

旧视觉目标仍发布红色 marker 用于对照, Mux 不订阅其 Pose, 不允许它控制导航

### Graph Planner

接收高层目标, 选择目标附近可达 graph 节点并发布 graph path

执行路径不会直接追加物体中心坐标

## 融合状态

- `EMPTY`: 还没有目标
- `PENDING`: 只有一个有效视角
- `TRACKING`: 已有至少两个不同视角, 输出论文式远距离粗目标
- `STABLE_VISION`: 纯视觉目标已经稳定, 可以控制导航
- `LIDAR_LOCKED`: 连续两帧 LiDAR 测量一致
- `REACHED`: Mux 确认最终完成后停止更新

## 关键 Topic

- `/spot1/object_mask`: WildOS 输出的目标 Mask
- `/spot1/object_target_estimate`: Target Fusion 输出的粗目标或稳定目标估计
- `/spot1/graphnav_goal_pose`: Goal Mux 输出的唯一高层目标
- `/spot1/object_search_completed`: Goal Mux 输出的最终任务完成状态

调试可视化:

- `/spot1/object_target_estimate_viz`: 黄色表示视觉粗目标, 绿色表示稳定目标
- `/spot1/object_target_particles`: 当前目标粒子点云

## 稳定性规则

- 相同位置和方向的重复观测不增加视角数
- 沿目标射线直行不产生横向视差, 不能增加多视角深度支持
- 与稳定目标方向明显不一致的 Mask 直接丢弃
- 单帧 LiDAR 命中不能锁定目标
- LiDAR 缓存覆盖视觉推理延迟, 优先匹配 Mask 原始时间附近的点云
- 相机 TF、目标 Mask 和 LiDAR 匹配统一使用三路图像的中位时间
- odom 和图像允许宽松同步, 但 odom 时间不能代替目标观测时间
- Mask 内 LiDAR 点先剔除局部地面, 再选择最密集高点簇作为目标可见表面
- `PENDING` 不显示位置, `TRACKING` 显示带较大误差的黄色粗目标
- 稳定目标短暂不可见时继续保存
- 小幅粒子抖动不改变导航目标
- WildOS reached 只表示当前视觉近距离候选证据, 不允许终止融合
- Mux 同时确认稳定目标、`2.0m` 距离和视觉证据后永久完成任务
- Fusion 和 WildOS 只接受 Mux 的 completed 通知并停止后续目标更新

## 当前状态

代码和启动链路已经接通, 下一步需要在 Unity 中验证目标误差、稳定时间和路径更新次数

修改消息或融合节点后需要重新构建 `object_search_msgs`、`triangulation3d`、`visual_navigation` 和 `graph_construction`
