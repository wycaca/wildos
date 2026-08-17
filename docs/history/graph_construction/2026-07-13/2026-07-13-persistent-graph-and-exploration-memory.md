# Persistent Graph and Exploration Memory

## 参考基线

- 社区仓库 `TIKTOKDAD/nebula2-wildos-main_ws`
- 核对提交 `156270a38635db38b4291cc43b15590cee37514c`
- 参考其 rolling GridMap 外节点和边保留、`explored_radius` 单调增长和 planner frontier 连续性
- 未接入该提交中的 Nav2 和 DLIO 适配

## Persistent Graph

- unknown 和滚动窗口外区域不否定历史节点
- 历史节点低于新边 clearance 阈值时继续保留, 只有自由圆完全消失或明确 obstacle 才删除
- disconnected component 不再提供删除开关, 所有历史 component 持续发布
- historical edge 固定执行可见障碍验证, unknown 和窗口外区域不能删除边
- robot anchor 固定启用, 连边半径复用 `edge_radius`, 连边数量复用 `current_node_max_edge_neighbors`

## Active Frontier

- Frontier 是当前 rolling GridMap 派生状态, 不是持久图拓扑
- 历史点只有仍位于当前地图且满足 free/unknown 边界时才继续活动
- 移出窗口后清除 `frontier_points` 和 `is_frontier`, owner 节点与 edge 不删除
- 当前可见边界使用世界坐标键保持 owner 稳定, 避免相邻帧 UUID 抖动
- 新边界落入任一持久节点的 `explored_radius` 时不再创建

## Deferred Branch

- planner 将未选择 Frontier 转成独立 `DeferredBranch`
- 只保存稳定 owner UUID、位置、发现方向和发现顺序, 不保存历史视觉分数
- 当前 `ActiveBranch` 正常前进时 deferred branch 不参与候选排序
- 当前分支持续无 odom 进展后, 优先恢复最早发现且当前图仍可达的 deferred branch
- 返回分支入口附近后由当前地图重新生成的 Frontier 接管, 不恢复旧 `frontier_points`

## Exploration Direction

- 删除 `StableFrontierSelector`, goal mux 不再订阅或选择 scored graph frontier
- goal mux 根据首帧 odom 固定探索原点和 heading, 粗目标姿态与配置后的 heading 保持一致
- planner 订阅 Object Search 状态, 只在 `SEARCHING_WITH_INITIAL_GOAL` 使用方向探索模式
- 方向探索把有限粗目标转换成沿初始 heading 持续前移的虚拟目标, 越过初始 `30m` 位置后不会反向吸引路径
- 方向探索不启用 goal radius 到达停止, `30m` 只作为前向 lookahead 而不是搜索终点
- planner 是唯一 frontier 决策层
- planner 使用显式 `ActiveBranch` 保存 Frontier、路径 UUID、局部方向、进度和最后有效高层路径
- planner 使用 `DeferredBranch` 保存未选择的稳定拓扑入口
- 同一走廊仍有可达候选时硬保持, 不按瞬时视觉分数或总代价切换
- 走廊拓扑暂时失配时复用最后有效路径后缀, 不立即重新选择其他方向
- 缓存终点已经位于活动分支方向后方时发布当前位置 hold path, 禁止沿旧路径回头
- 只有分支持续无进展后才释放 `ActiveBranch`, 临时屏蔽失败邻域并恢复保存分支

## Traversal Memory

- planner 根据连续 `current_node UUID` 二值记录实际经过的 graph edge
- Dijkstra edge weight 对已走边增加一次固定重复访问代价
- 经过边不会删除或禁用, 真实死路只有原路可退时仍能生成路径
- virtual goal 加入前先计算持久图 shortest paths, 避免候选通过 virtual goal 互相短接
- `frontier_points` 只表示当前地图探索方向, 可执行 path 停在安全 graph node

## 参数清理

- 删除 Graph Construction 的 `validate_historical_edges`, `ensure_robot_anchor_node`, `robot_anchor_edge_radius`, `robot_anchor_max_edges`, `prune_disconnected_nodes`
- 删除 Graph Construction 的 deadend observation、removed frontier suppression 和 trajectory spacing 参数
- 删除 goal mux 的 graph topic、selected frontier topic、frontier selector 和初始 goal 重算参数
- planner 删除旧的 local frontier timer 和 planner node path 缓存参数
- visual navigation 删除 removed Frontier 坐标记忆和 `min_frontier_separation`
- planner 保留 `frontier_continuity_radius`, `frontier_progress_timeout`, `revisit_cost_factor`

## 无目标探索发布

- goal mux 收到首帧 odom 后持续发布固定粗目标, 未检测到目标不会停止探索
- 删除 goal publisher 的 subscriber 数量门控, 避免 Zenoh discovery 尚未稳定时抑制粗目标发布
- planner 输出 `/spot1/graphnav_planner/path`, Unity 自研导航消费后发布 `/corrected_path`
- 集成 launch 停止启动 `path_follower_node`, 避免重复消费 `/corrected_path`
- 运行前必须重新构建 `visual_navigation`, `graph_construction` 和 `graphnav_planner`, 避免源码与 install 二进制不一致

## 探索路径稳定性

- goal mux 周期重发的同一粗目标只更新 planner 内缓存时间戳, 不再以 5 Hz 重复触发相同规划
- planner 仍在每帧 graph 更新时重规划, 保留环境变化和机器人前进后的路径更新
- 同分支通过候选路径与上一轮路径节点集合的拓扑重叠判断, 不再只依赖单个 frontier UUID
- 拓扑重叠路径的首段仍需保持非反向, 空间回退同时要求方向夹角不超过 60 度
- progress timeout 使用变换到 graph frame 的真实 odom 位移, 不再依赖可能抖动或停留的 current graph node
- Frontier 短暂失配期间继续用真实 odom 刷新执行进度, 机器人仍在前进时不会误判死路
- 后方 Frontier 只保存为 deferred branch, 不携带历史视觉分数参与当前方向选择
- 缓存路径从离机器人最近的 waypoint 开始发布, 已执行的路径前缀不会再次下发
- Object Search 状态变化时先清除旧 goal 和探索状态, 等新状态 goal 到达后再规划, 避免目标出现瞬间发布旧探索路径

## 验证

- 图算法测试覆盖多次滚动窗口后起点节点和边仍存在
- 图算法测试覆盖窗口外 Frontier 取消活动、owner 拓扑保留和当前可见 owner 稳定继承
- 图算法测试覆盖 explored radius 阻止已探索区域重新生成 Frontier
- planner 测试覆盖正常前进时后方分支不激活、确认无进展后恢复最早分支
- 视觉测试覆盖历史评分下一帧删除和多相机当前证据融合
- goal mux 测试覆盖机器人移动转向后固定粗目标不变
- goal mux 测试覆盖配置 heading 同时作用于粗目标位置和姿态
- `graphnav_planner` 完成编译和定向 GTest
