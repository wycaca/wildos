# TIKTOKDAD nebula2-wildos-main_ws 提交参考分析

日期: 2026-07-02

目标: 阅读 `TIKTOKDAD/nebula2-wildos-main_ws` 的 `main` 分支提交记录, 判断其改动方向是否和当前 WildOS 复现目标一致, 并提取本项目后续可以学习参考的部分

参考仓库:

```text
https://github.com/TIKTOKDAD/nebula2-wildos-main_ws/commits/main/
```

## 提交概览

截至 2026-07-02, 该仓库 `main` 分支只有 3 个提交:

| 提交 | 日期 | message | 判断 |
| --- | --- | --- | --- |
| `a2c6d9a` | 2026-06-28 | `Initial import of WildOS workspace` | 初始导入, 主要作为基线, 不体现具体工程判断 |
| `e587c19` | 2026-07-01 | `Initial import for GitHub upload` | 包含较多新增工程内容, 其中 `graphnav_builder`, `graphnav_nav2_bridge`, sync tuner, goal mux 对本项目有参考价值 |
| `225be74` | 2026-07-01 | `Update visual navigation changes` | 主要调整三相机仿真 topic, camera mapping, sync queue 和初始粗目标参数, 和本项目三相机适配方向相关, 但不可直接照搬 |

## 和当前项目目标是否一致

结论: 部分一致, 但运行目标和适配对象不同

一致的部分:

- 都围绕 WildOS 缺失的几何图构建, 视觉评分, graph planner 和执行闭环补齐工程链路
- 都把 `NavigationGraph` 作为几何图到视觉评分和 planner 的接口
- 都关注三相机输入, GridMap / traversability map, TF, QoS, ApproximateTimeSynchronizer 和 frontier 稳定性
- 都把 graph builder / graph construction 视为几何记忆层, 不直接做语义目标决策

不同的部分:

- 该仓库面向 Clearpath A300 / Jazzy / `odom` / `/a300_0000/...` 仿真链路
- 当前项目面向 Unitree Go2 / Isaac Sim / Humble / ROS_DOMAIN_ID=3 / `/unitree_go2/...` 和 `/spot1/...` 适配链路
- 该仓库的 `graphnav_builder` 直接消费 `grid_map_msgs/GridMap`, 当前项目已有 `graph_construction` 同时支持 LiDAR OccupancyGrid baseline 和 elevation GridMap 实验后端
- 该仓库的 `graphnav_nav2_bridge` 已经尝试接 Nav2 执行层, 当前项目文档明确 Nav2 和 cmd_vel 执行暂缓
- 最新提交将 camera 名称改为 `"0", "1", "2"`, 当前项目文档目标仍倾向 `front`, `left`, `right` 语义, 只在仿真 topic 必须时做映射

因此, 该仓库不是当前项目的直接上游, 也不能整体合并, 但其中有几类工程方法值得吸收

## 可直接学习参考的部分

### 1. graph builder 的分层边界

`graphnav_builder` 明确把 ROS 通信和论文算法拆开:

```text
Odometry + local GridMap
    -> ApproximateTimeSynchronizer
    -> bounded MessageBuffer + historical TF
    -> TraversabilityGrid
    -> SparseGraphBuilder
    -> NavigationGraph
```

可参考点:

- ROS node 只负责 topic, QoS, TF, 参数, 诊断和发布
- `SparseGraphBuilder` 作为纯算法层, 不依赖 `rclpy`
- GridMap 解码, 坐标变换, 线段碰撞, graph state 和 ROS message 转换拆成独立工具
- 这样更容易单元测试, 也避免 TF / QoS 问题污染图构建算法

当前项目已有类似分层:

```text
node.py
graph_builder.py
grid_adapter.py
frontier_detector.py
edge_builder.py
graph_memory.py
msg_utils.py
viz.py
```

后续可学习它的边界清晰度, 但不需要重命名或整体替换当前模块

### 2. GridMap 解码和 rolling map 坐标处理

`graphnav_builder` 的代码分析文档重点提到:

- `GridMap` 可能有 `data_offset`
- 图层存储可能涉及行列布局和循环缓冲区
- 地图可能带 yaw
- `world_to_map_axes`, `map_axes_to_world`, `xy_to_cell`, `cell_to_xy` 必须互为逆变换

这和当前项目 2026-07-01 的问题高度相关:

```text
如果 GridMap row / column 到 world 坐标映射不一致, graph marker 会相对 GridMap surface 出现整体转置, 镜像或平移
```

可参考做法:

- 增加 GridMap 解码专项测试
- 用非零 offset, rolling buffer, yaw, known point 做 round-trip 验证
- 不只用 RViz 视觉判断, 要用可复现的小型 GridMap 单元测试确认坐标约定

### 3. frontier owner 验证的性能开关

该仓库新增 `validate_frontier_paths`

语义:

- `true`: 新 frontier 和历史 frontier 都验证 owner 直线路径安全
- `false`: 保留 safe component 和基础过滤, 但跳过直线路径证明, 使用近似性能模式

这个开关对当前项目有参考价值, 因为 Isaac Sim 中 LiDAR / elevation map 更新频率和 frontier 数量可能让路径验证成为瓶颈

建议学习方式:

- 不直接关闭安全验证
- 先在当前 `frontier_detector.py` 和 `edge_builder.py` 中增加阶段耗时统计
- 如果确认 frontier path check 是主要耗时, 再考虑引入类似诊断参数
- 默认仍应保持安全验证开启

### 4. robot anchor 和 current node 安全策略

`graphnav_builder` 有 `ensure_robot_anchor` 思路:

- 当没有安全可达图节点时, 如果机器人当前格安全, 添加确定性 anchor node
- `current_node_idx` 优先选择机器人可安全到达的最近节点
- 没有安全候选时才回退几何最近点, 并保留诊断状态

当前项目已经有 current node 和 graph memory, 但在 elevation backend 质量波动时可能出现:

```text
nodes=0
edges=0
current node 离 robot marker 不稳定
```

可参考做法:

- 给 current node 匹配增加更明确的安全失败诊断
- 在可通行 GridMap 较碎时, 评估是否需要临时 robot anchor
- anchor 只应作为恢复连接性的保守机制, 不能掩盖地图分类错误

简短理解:

- 这类问题通常出现在 elevation map 很碎, robot 附近没有采到安全节点, 或 robot 到最近节点的直线路径被 unknown / obstacle 切断时
- 修复目标不是强行让 graph 有节点, 而是先找 robot 可安全到达的最近节点
- 如果找不到安全可达节点, 但 robot 当前 cell 是 free, 才在 robot 位置生成一个确定性 anchor node 作为可信起点
- 如果 robot 当前 cell 也不是 free, 不应添加 anchor, 只能回退几何最近点并输出诊断, 因为这通常说明地图分类或坐标映射仍有问题
- 可以把 anchor 理解成“机器人脚下临时放一个起步石墩”: 它帮助 graph 从机器人真实位置接上, 但不能把水坑或悬崖伪装成安全地面

### 5. 诊断日志和 stage timing

该仓库在 graph builder 中记录:

- graph size
- connected components
- current component size
- reachable / unreachable frontier
- degree statistics
- frontier path checks
- edge candidates / validations
- update latency
- TF drops
- processing-buffer drops
- `distance_fields`, `update_nodes`, `sample_nodes`, `update_frontiers`, `build_edges`, `current_node` 等阶段耗时

当前项目后续应优先学习这个方向

建议:

- 在 `graph_construction` 首帧统计之外, 增加周期性 stage timing
- 对 elevation backend 的问题, 同时输出分类统计, graph 统计, TF 等待和耗时统计
- 慢帧 warning 应打印具体慢在哪个阶段, 不只打印总耗时

### 6. 同步调参工具

`wildos_sync_tuner.py` 和 `objmask_sync_tuner.py` 是很有价值的运行时诊断工具

它们的核心思路:

- 只观察 topic, 不改变运行系统
- 读取每个 topic 的频率, header stamp, 接收时间, QoS endpoint
- 估算 `syncsub_slop`, `syncsub_queue_size`, `qos_history_depth`, TF cache depth
- 对 BEST_EFFORT publisher 和 RELIABLE subscriber 不匹配给出 warning
- 对 `ObjectMaskWithTf + PointCloud2` 也提供单独的同步参数建议

这和当前项目多次遇到的问题直接相关:

```text
相机, odom, nav_graph stamp 差过大
ApproximateTimeSynchronizer 不触发
TF buffer stale message
camera_info 低频导致同步堵塞
```

建议优先移植为当前项目的诊断工具, 但需要改为本项目 topic 和 camera naming:

```text
/camera/front/color/image/compressed
/camera/left/color/image/compressed
/camera/right/color/image/compressed
/spot1/nav_graph
/spot1/odom_for_scoring
/elevation_mapping_node/elevation_map_raw
```

### 7. initial goal mux

`initial_goal_mux.py` 的逻辑适合 object search 初期目标缺失场景:

- 在三角定位还没输出真实目标前, 发布一次配置的粗目标
- 等待 output topic 出现订阅者后再发布, 避免一次性消息丢失
- 第一条有效三角定位目标到来后永久切换到真实目标
- 拒绝 NaN 或 infinite 目标
- 检查输入输出 topic 不能相同, 避免消息回环

这个对当前项目有参考价值, 但前提是我们恢复 object search 或需要自动给 `graphnav_planner` 一个粗目标

当前阶段如果只是手动发布 `/spot1/imgnav_waypoint`, 可以先不实现

## 只能作为后续参考的部分

### graphnav_nav2_bridge

该仓库的 `graphnav_nav2_bridge` 提供了完整思路:

```text
/scored_nav_graph + /imgnav_waypoint
    -> graphnav_planner/path
    -> /goal_pose
    -> /graphnav_navigate_to_pose
    -> Navfn + MPPI
    -> velocity_smoother
    -> collision_monitor
    -> /a300_0000/cmd_vel
```

其中 `goal_pose_to_nav2.py` 值得参考:

- 对 look-ahead goal 做 rate limit
- 目标变化超过距离或 yaw 阈值才更新 Nav2 action
- 输入 goal 超时后 cancel active Nav2 goal
- 把 3D graph goal flatten 成 2D Nav2 goal
- 拒绝空 frame, NaN, infinite 和 zero quaternion
- 使用独立 action name `/graphnav_navigate_to_pose`, 避免和普通 `/navigate_to_pose` 客户端互相抢占

但当前项目文档已经把 Nav2 / cmd_vel 执行暂缓, 所以它只能作为下一阶段参考

不建议现在照搬的原因:

- 它绑定 A300 topic, footprint, LaserScan, Jazzy Nav2 参数
- 当前仿真是 Unitree Go2 / Isaac Sim, command topic, base frame, LiDAR scan 输入和安全层不同
- 当前优先级仍是 3 相机视觉, LiDAR baseline, elevation backend 对齐和 graph/path 质量

### integrated_traversability edge cost

该仓库支持边代价模式:

```text
euclidean
integrated_traversability
```

这和论文系统更接近, 但当前项目应先保证:

- GridMap elevation/traversability frame 对齐
- graph nodes 不采到墙面或高处板面
- edge 不穿过 obstacle 或 unknown
- planner path 可稳定生成

只有 elevation backend 的 traversability layer 质量稳定后, 才适合把 edge cost 从欧氏距离扩展到风险积分

## 不建议照搬的部分

### 数字 camera mapping

最新提交把多处 camera mapping 从:

```text
0 -> front
1 -> left
2 -> right
```

改成:

```text
0 -> 0
1 -> 1
2 -> 2
```

并配合 topic:

```text
/a300_0000/sensors/camera_{}/color/compressed
```

这和当前项目目标不完全一致

当前项目的三相机语义应继续保留:

```text
front
left
right
```

如果 Isaac Sim runtime 的 topic 只能按数字命名, 也应通过配置或映射层适配, 不应在核心导航代码里把语义名称改丢

### A300 Nav2 配置

`nav2_a300_odom.yaml` 中的 footprint, costmap size, MPPI velocity limit, LaserScan topic, collision monitor topic 都是 A300 专用

当前项目不能直接使用:

```text
/a300_0000/sensors/lidar3d_0/scan
/a300_0000/cmd_vel
base_link
odom
```

只能学习结构:

- rolling odom costmap
- 独立 graphnav action name
- velocity smoother 和 collision monitor 放在 Nav2 输出后
- look-ahead goal stale 后 cancel

### 大量中文代码注释

该仓库把一些 Python docstring 和注释改成中文

当前项目用户要求是代码需要简洁注释, 英文标点, 无句号, 复杂逻辑函数加注释说明

后续本项目代码注释应保持简洁, 不应照搬长段中文注释

## 对当前项目的建议优先级

### 优先级 A, 近期可做

- 增加 WildOS 输入同步诊断工具, 参考 `wildos_sync_tuner.py`
- 增加 ObjectMask / LiDAR 同步诊断工具, 参考 `objmask_sync_tuner.py`
- 为 `graph_construction` 增加 stage timing 和 frontier path check 计数
- 为 GridMap 直连路径补充 rolling buffer, data offset, yaw, row/column 映射单元测试
- 梳理 current node 安全匹配失败时的诊断输出

### 优先级 B, 地图稳定后再做

- 评估 `validate_frontier_paths` 类性能开关
- 评估 robot anchor 机制
- 评估 integrated traversability edge cost
- 评估 initial goal mux, 用于 object search 初始粗目标

### 优先级 C, 下一阶段执行闭环

- 参考 `goal_pose_to_nav2.py` 的 look-ahead goal rate limit 和 stale cancel
- 参考 mapless rolling Nav2 costmap 结构
- 重新为 Unitree Go2 / Isaac Sim 设计 footprint, cmd_vel topic, collision monitor 输入和 action namespace

## 总结

该仓库的提交方向和 WildOS 工程复现目标部分一致, 但运行平台, ROS 版本, topic/frame 命名和阶段优先级与当前项目不同

最值得学习的不是具体 topic 参数, 而是以下工程方法:

- ROS 适配和纯 graph 算法层分离
- GridMap rolling buffer 和坐标约定的严格测试
- frontier reachability, historical edge validation 和 current node safety 的诊断化
- 运行时同步/QoS/TF 调参工具
- 执行闭环中 look-ahead goal 的限频, 过滤和超时取消策略

当前项目不应整体迁移该仓库代码, 应按上述优先级提取小块设计, 先服务于现有 `graph_construction`, 三相机 visual scoring 和 elevation backend 对齐问题
