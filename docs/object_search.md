# 目标搜索

目标搜索从文本查询开始, 在没有目标坐标时探索, 形成可靠目标位置后接近并完成近距离确认

## 模块分工

| 模块 | 职责 |
| --- | --- |
| WildOS | Frontier 评分、目标 Mask 和近距离视觉证据 |
| target fusion | 多视角粒子滤波和 LiDAR 距离精修 |
| Goal Mux | 高层目标、状态切换和最终完成 |
| Planner | 图上路线、探索记忆和 Path 验证 |
| `wildos_navigation` | 目标点到局部路径、避障和停车 |

Goal Mux 是高层目标和最终完成状态的唯一 owner

## 证据流程

```text
文本目标
  -> SEARCH: 按视觉评分探索
  -> PENDING: 单视角只有方向, 原地观察或横向换位
  -> COARSE: 多视角形成粗位置, 到安全距离观察
  -> STABLE/LIDAR_LOCKED: 稳定位置或连续 LiDAR 支持
  -> REACHED: 新鲜近距离证据通过完成门控
```

单视角粒子均值不能作为导航坐标. PENDING 只使用 Mask 质心射线方向获取第二视角

视觉稳定目标由独立视角、粒子分布和置信度共同决定. LiDAR 只提供近距离精修, 单帧 LiDAR 不能覆盖已有视觉轨迹

完成必须同时满足稳定目标、距离门槛和新鲜视觉或 LiDAR 证据. 仅到达估计坐标附近不能宣布完成

## 探索和恢复

Planner 在 scored graph 上用 Dijkstra 计算可达路线, 状态机只决定选择哪个方向以及何时允许回头

```text
FOLLOW_BRANCH
  -> CHECK_DEAD_END
  -> CHOOSE_BRANCH
  -> BACKTRACK
  -> FOLLOW_BRANCH 或 EXPLORATION_EXHAUSTED
```

- 活动路线有进展时保持稳定, 不因 Frontier 分数小变化频繁换路
- 路径持续失效或无进展后才确认死路
- 岔路按位置和方向记忆, 回退优先最近未探索分支
- 普通延伸限制回头, 只有显式 `BACKTRACK` 可以明显反向
- graph、odom 或目标证据异常时冻结失败计时
- 目标证据过期后恢复抢占前的探索方向和分支记忆

## 状态接口

Goal Mux 通过 `/spot1/object_search_status` 发布强类型 `ObjectSearchStatus`. `pending_protection` 明确告诉 Planner 是否冻结探索失败计时

运行时切换目标:

```bash
ros2 topic pub --once /spot1/object_search_target \
  std_msgs/msg/String "{data: 'red fire extinguisher'}"
```

切换目标会清理旧文本特征、Mask 确认窗口、粒子、目标估计、完成锁存和搜索方向. 更早时间戳的数据不能污染新任务

## 目标融合边界

- Mask 必须经过空间过滤和连续帧确认
- 使用观测时刻相机 TF
- 重复视角不增加独立支持数
- 沿目标方向直行不产生横向视差
- Mask 与点云只在时间接近时融合
- LiDAR 点先去地面并聚类, 连续一致后才能锁定
- 调试 Marker 和粒子点云没有订阅者时不构造

具体门槛以以下配置为准:

- `visual_navigation/configs/wildos_nav_conf.yaml`
- `visual_navigation/configs/object_search_goal_mux.yaml`
- `graphnav_planner/config/planner.yaml`

## Planner 与局部导航

Planner 发布 Path 变化事件, 不是心跳. graph 过期且 odom 新鲜时最多发布一次 hold Path, odom 过期或时间戳异常时停止发布

Goal Mux 发布 `/goal_pose`. 导航算法使用 odom 和注册点云构建局部代价地图, 通过 A* 生成控制路径并发布 `/cmd_vel`. GraphNav Planner Path 不直接驱动已迁移的路径跟随器. 具体契约见 [Planner](../graphnav_planner/README.md)和[导航](navigation.md)

## 代码入口

- WildOS: `visual_navigation/visual_navigation/wildos/nav.py`
- Goal Mux: `visual_navigation/visual_navigation/object_search_goal_mux.py`
- ROS target fusion: `visual_navigation/visual_navigation/object_target_fusion.py`
- 粒子滤波: `triangulation3d/triangulation3d/target_particle_filter.py`
- Planner: `graphnav_planner/src/`

行为修改运行对应 package 测试, 全仓验证使用 `./scripts/test_repo.sh`
