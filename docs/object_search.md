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
scripts/go2_search_test.sh target "red fire extinguisher"
```

切换目标会清理旧文本特征、Mask 确认窗口、粒子、目标估计、完成锁存和搜索方向. 更早时间戳的数据不能污染新任务

## GO2 实机测试

在 x86 主机运行统一脚本. 脚本复用现有 Docker 和运动网关控制脚本, 默认通过 `agx@192.168.50.2` 管理相机 AGX. 首次连接会要求确认 SSH 主机指纹和输入 AGX 密码

需要免除每次 SSH 密码输入时执行一次 `ssh-copy-id agx@192.168.50.2`. 运动网关仍按宿主机 sudo 策略要求确认权限

```bash
# 启动传感器、定位、视觉和运动链路, 最后发布搜索目标
scripts/go2_search_test.sh start "chair"

# 测试过程中切换目标
scripts/go2_search_test.sh target "red fire extinguisher"

# 查看整体状态或单个模块的持续日志
scripts/go2_search_test.sh status
scripts/go2_search_test.sh logs wildos
scripts/go2_search_test.sh logs navigation
scripts/go2_search_test.sh logs gateway

# 结束运动测试, 保留相机、雷达、定位和 WildOS 服务
scripts/go2_search_test.sh stop
```

`stop` 固定先停止 x86 导航速度源, 等待网关超时发布零速度后再停止 AGX 网关. 网络变化时通过 `AGX_HOST` 和 `AGX_REPO_ROOT` 覆盖默认连接参数

## 目标融合边界

- Mask 必须经过空间过滤和连续帧确认
- 使用观测时刻相机 TF
- 重复视角不增加独立支持数
- 沿目标方向直行不产生横向视差
- 横向观察只选择当前导航图可达安全区域内未访问的位置
- 附近没有新安全观察点时保持当前位置, 等待图或目标证据更新
- Mask 与点云只在时间接近时融合
- LiDAR 点先去地面并聚类, 连续一致后才能锁定
- 目标融合只发布 `TargetEstimate`, 内部粒子不发布为调试点云

目标可视化由 `wildos_visualization` 订阅公开 `TargetEstimate` 和 odom 重建. 目标球三轴尺寸来自位置协方差, 稳定目标为青色, 达到粗目标接管门槛时为黄色; 绿色线连接当前 odom 与目标. 切换搜索目标时外部进程清除旧 Marker

在 1500 粒子基线上, 旧核心 Marker 与粒子点云构造中位耗时合计 0.246 ms, 每次额外复制 18 KB 粒子数据. 拆分后核心可视化耗时和粒子调试带宽均为 0, 外部目标 Marker 构造中位耗时为 0.136 ms

## 评分可视化隔离

WildOS 核心只把归一化 `frontier_scores` 写入公开 `/spot1/scored_nav_graph`, 不再同步生成或发布 score ring. `wildos_visualization/graph_visualizer` 在 RViz 模式下订阅 scored graph, 用单个 `LINE_LIST` 重建相同方向弧段和颜色

在 100 个 Frontier、每个 16 个方向的合成输入上, 旧核心生成 1601 个 Marker 的中位耗时为 382.254 ms. 拆分后核心耗时为 0 ms; 外部进程生成 2 个 Marker 的中位耗时为 85.108 ms, 比旧构造快 4.49 倍. 生产模式关闭可视化时不订阅 scored graph

旧 `model_visualization` 和 `within_range_geofrontiers` 没有运行时消费者, 且模型拼图依赖未公开的逐像素内部张量. 当前直接删除这两条调试流及绘图工具, 不新增内部热图 Topic. 三路 256×256 合成输入下, 核心每帧减少 6.298 ms 拼图开销和约 1.20 MiB 图像构造. 将来只有在现场诊断明确需要时, 才增加低频、best effort、depth 1 的专用调试输出

WildOS 使用固定容量窗口记录解码、投影、推理、检测、评分、图复制、发布、输入频率和链路年龄, 每 30 秒向 `/diagnostics` 发布标准标量. 不再构造两段周期性能日志. 合成满指标消息的构造加序列化中位耗时为 0.359 ms, 大小 3738 bytes, 折算每秒核心开销约 0.012 ms

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
- RViz score ring: `wildos_visualization/wildos_visualization/graph_visualizer.py`

行为修改运行对应 package 测试, 全仓验证使用 `./scripts/test_repo.sh`
