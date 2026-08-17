# 提交分支和路径发布去抖

## 问题

Unity 自研导航直接消费 `/spot1/graphnav_planner/path`, `path_follower_node` 不参与当前执行链路

旧实现每次收到 scored graph 都重新执行 Frontier 排序并发布 Path, 同时只要候选路径和旧路径共享任意节点就可能继承活动分支

路口候选通常共享机器人附近的公共路径前缀, 因此侧向或后向候选可能被误认为当前走廊延伸, 下游会立即把这种高层变化转换为新的 `/corrected_path`

## 状态模型

`ActiveBranch` 现在保存:

- 当前尾部 Frontier UUID 和位置
- 从分支提交点到当前尾部的完整有序节点 UUID
- 与 UUID 对齐的完整高层路径点
- 路径尾部切向
- 机器人在提交路径上的最大弧长进度和最后进展时间
- 是否正在恢复 deferred branch

完整有序路径同时承担拓扑锁定、进度测量和缓存执行路线三项职责, 不再维护无序路径节点集合和任意 odom 位移进度

## 分支继承

正常 Frontier 延伸必须满足:

1. 候选 graph path 有序经过当前活动尾节点
2. 候选在尾节点之后仍有新节点
3. 新延伸首段不能沿旧尾部切向反向

仅共享机器人附近公共前缀不满足继承条件, 因此 Y 路口和环路中的其他候选不能在活动分支有效时抢占路线

旧 Frontier UUID 从 graph 中消失时允许空间迁移, 但候选必须位于旧尾部 `frontier_continuity_radius` 内且不能落在旧尾部切向后方

Unity 将该既有参数从 `10m` 收紧为 `5m`, 不新增参数

## 分支释放

活动分支只在以下事件释放:

- 已提交路径中两个仍存在的相邻节点失去 graph edge
- 机器人在 `frontier_progress_timeout` 内没有增加路径弧长进度
- Object Search 状态或高层 goal 发生真实变化并重置探索状态

Frontier 短暂消失、视觉分数变化、横向移动和向后滑动都不会释放活动分支

路径弧长每增加 `0.25m` 更新一次进展时间, `12s` 表示没有沿当前路径前进, 不是连续 `12s` 没有检测到目标

## 发布去抖

`planner_node` 仍在每次 Graph 更新时调用 planner, 以便及时验证 edge、更新进度和发现当前尾部的有序延伸

planner 返回 `PlanningResult.path_changed`, 仅以下情况为 true:

- 首次提交探索分支
- 当前分支沿有序尾部产生新延伸
- 当前路径失效或无进展后恢复其他分支
- 真实目标路径的剩余拓扑发生变化
- 已发布路线失效且没有替代路线, 需要发布空 Path 停止旧路线

同一 Frontier 重复出现、Frontier 短暂消失、机器人沿已发布路径前进导致起始前缀缩短时, `path_changed` 为 false, 不重复发布

提交路径需要延伸或恢复时, 新 Path 从当前 odom 开始并接到最近路径段的前向端, 不重新包含机器人身后的最近离散节点

该去抖位于 `graphnav_planner`, Unity 自研导航继续直接消费高层 Path, 不需要为本次修改增加底层过滤逻辑

## 修改文件

- `graphnav_planner/include/graphnav_planner/planner.hpp`
- `graphnav_planner/src/planner.cpp`
- `graphnav_planner/src/planner_node.cpp`
- `graphnav_planner/launch/graphnav_planner.launch.yml`
- `graphnav_planner/test/test_deferred_branch.cpp`
- `graph_construction/launch/elevation_visual_navigation_sim.launch.py`
- `graph_construction/docs/AGENT_README.md`
- `graph_construction/docs/2026-06-22/2026-06-22-principles.md`
- `graph_construction/docs/2026-07-14/2026-07-14-frontier-lifecycle-and-deferred-branch.md`

## 验证场景

- 同一 Frontier 重复 Graph 更新不产生新 Path 版本
- Frontier 短暂消失时继续缓存路线
- 有序经过旧尾部的新 Frontier 才发布延伸路径
- 共享公共前缀的 Y 路口侧向候选不能抢占活动分支
- 横移和后退不能重置无进展计时
- 已提交 edge 消失后立即释放分支并恢复 deferred branch
- 旧 Frontier UUID 消失后只允许前向近邻迁移
- 旧 Frontier UUID 消失后拒绝连续半径内的后向近邻迁移
- 真实目标路径只缩短已走前缀时不重复发布

## 验证结果

- `graphnav_planner` 隔离构建通过
- committed branch 和 deferred branch 九项 GTest 通过
- `cppcheck` 和 `lint_cmake` 通过
- 九项 GTest 在 AddressSanitizer 下通过
- 包级 `uncrustify` 仍报告现有 C++ 整体风格与 ROS 默认规则不一致, 本次不重排无关文件
- `xmllint` 在受限网络中无法下载 ROS package schema, 与本次代码逻辑无关
