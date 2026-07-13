# Persistent Graph and Exploration Memory

## 参考基线

- 社区仓库 `TIKTOKDAD/nebula2-wildos-main_ws`
- 核对提交 `156270a38635db38b4291cc43b15590cee37514c`
- 参考其 rolling GridMap 外节点和边保留、`explored_radius` 单调增长、历史 frontier 可见性验证和 planner frontier 连续性
- 未接入该提交中的 Nav2 和 DLIO 适配

## Persistent Graph

- unknown 和滚动窗口外区域不否定历史节点
- 历史节点低于新边 clearance 阈值时继续保留, 只有自由圆完全消失或明确 obstacle 才删除
- disconnected component 不再提供删除开关, 所有历史 component 持续发布
- historical edge 固定执行可见障碍验证, unknown 和窗口外区域不能删除边
- robot anchor 固定启用, 连边半径复用 `edge_radius`, 连边数量复用 `current_node_max_edge_neighbors`

## Persistent Frontier

- `frontier_detector` 不再每帧清空全部 `frontier_points`
- 第一阶段保留窗口外历史点, 并重新验证窗口内历史点是否仍为 free/unknown 边界
- 第二阶段只补充新 frontier, 使用世界坐标量化键去重
- 已确认的历史 owner 优先继承, 避免最近邻细微变化导致 frontier UUID 和方向抖动
- 当前可见点成为普通 known free、unknown、obstacle 或 owner 路径不可达时才删除

## Exploration Direction

- 删除 `StableFrontierSelector`, goal mux 不再订阅或选择 scored graph frontier
- goal mux 根据首帧 odom 计算一次固定粗目标, 不使用超时或到达半径重新生成
- planner 是唯一 frontier 决策层
- planner 使用 UUID 和空间邻域继承当前探索分支
- 当前分支在 `frontier_switch_margin` 范围内继续保持, 明显更优的新分支才允许切换
- 分支持续无进展后临时屏蔽其邻域, 没有其他候选时回退到完整候选集

## Traversal Memory

- planner 根据连续 `current_node UUID` 记录实际经过的 graph edge
- Dijkstra edge weight 按经过次数乘以重复访问代价
- 经过边不会删除或禁用, 真实死路只有原路可退时仍能生成路径
- virtual goal 加入前先计算持久图 shortest paths, 避免候选通过 virtual goal 互相短接
- `frontier_points` 继续只表示探索方向, 可执行 path 停在安全 graph node

## 参数清理

- 删除 Graph Construction 的 `validate_historical_edges`, `ensure_robot_anchor_node`, `robot_anchor_edge_radius`, `robot_anchor_max_edges`, `prune_disconnected_nodes`
- 删除 Graph Construction 的 deadend observation、removed frontier suppression 和 trajectory spacing 参数
- 删除 goal mux 的 graph topic、selected frontier topic、frontier selector 和初始 goal 重算参数
- planner 删除旧的 local frontier timer 和 planner node path 缓存参数
- planner 保留 `frontier_continuity_radius`, `frontier_progress_timeout`, `frontier_switch_margin`, `revisit_cost_factor`

## 验证

- 图算法测试覆盖多次滚动窗口后起点节点和边仍存在
- 图算法测试覆盖窗口外 frontier 保留、重新观察后删除和 owner 稳定继承
- goal mux 测试覆盖机器人移动转向后固定粗目标不变
- `graphnav_planner` 使用独立临时 build 和 install 目录完成编译
