# Graph Construction 原理和当前代码对应

日期: 2026-06-22

修订: 2026-07-13

目标: 用当前代码逻辑解释 WildOS Graph Construction 的算法原则, 图论对象, 关键字段和模块边界

## 核心原则

Graph Construction 的任务是把机器人局部看到的几何地图转换成能长期记忆, 能扩展, 能规划的稀疏导航图

当前默认输入是 elevation/2.5D GridMap:

```text
elevation GridMap
  -> ClassifiedGrid
  -> free nodes
  -> frontier nodes
  -> safe edges
  -> NavigationGraph
```

2D OccupancyGrid 是 fallback:

```text
PointCloud2 or aligned PointCloud2
  -> livox_grid_builder
  -> OccupancyGrid
  -> ClassifiedGrid
  -> NavigationGraph
```

这张 graph 后续会被 WildOS 视觉模块加上 frontier scores, 再交给 `graphnav_planner` 做高层路径搜索

## 图论对象

论文中的导航图:

```text
G_nav_t = (V_t, E_t)
```

当前代码中的对应关系:

- `G_nav_t`, `GraphState`
- `V_t`, `InternalNode`
- `E_t`, `InternalEdge`
- `F_geo_t`, `is_frontier=true` 的 node 集合
- `current_node_idx`, 机器人当前所在 graph node 或 robot anchor node

可以把它理解为:

```text
node = 机器人可以站的位置
edge = 两个 node 之间的安全连接
edge cost = graph planner 使用的代价
frontier node = 附近存在 free / unknown 边界的 node
frontier point = frontier node 关联的未知边界采样点
```

## 代码入口

当前 ROS 入口在:

```text
graph_construction/graph_construction/node.py
```

`node.py` 只负责 ROS 适配:

- 声明和读取参数
- 订阅 GridMap 或 OccupancyGrid
- 查询 odom / TF
- 调用 `SparseGraphBuilder`
- 发布 `NavigationGraph`
- 发布可视化 `MarkerArray`
- 输出诊断日志

纯算法入口在:

```text
graph_construction/graph_construction/graph_builder.py
```

`SparseGraphBuilder` 接收:

- `ClassifiedGrid`
- robot position
- stamp seconds

它不直接依赖 ROS message

## Grid 分类原则

所有地图输入先转成 `ClassifiedGrid`

对应文件:

```text
grid_adapter.py
grid_types.py
```

当前支持:

- `OccupancyGrid`, 2D fallback
- `grid_map_msgs/GridMap`, elevation 主线

分类结果至少需要表达:

- free, 可通行区域
- obstacle, 明确障碍
- unknown, 未观测区域
- elevation, elevation path 中 node 和 path 的 z 来源
- distance fields, edge clearance 和 free radius 的几何依据

elevation path 的当前处理:

- 直接消费 `/elevation_mapping_node/elevation_map_raw`
- 用 traversability layer 和 elevation layer 分类
- 对被 free 包围的小洞做 free 后处理
- 只为已判定 free 的小型 NaN elevation cell 补 elevation
- 不修改原始 GridMap topic

## Node 更新原则

node 是稀疏 graph 的长期记忆单位

对应文件:

```text
graph_memory.py
graph_builder.py
```

每个 node 需要维护:

- stable UUID
- pose
- free radius
- explored radius
- frontier points
- frontier 标记
- robot anchor 标记

当前原则:

- 新 node 从当前 free cells 中采样
- 采样点之间保持 `min_node_separation`
- 已有 node 跨帧保留, 避免 UUID 抖动
- 明确落入障碍或不可用区域的 node 会被移除或失效
- 当前局部地图外的历史 node 可作为 graph memory 保留
- `prune_disconnected_nodes` 默认关闭, 避免 current node 短时误判时清空大部分 graph

## Robot Anchor 原则

脚下点云缺失时, robot pose 对应 cell 可能是 unknown

如果强行要求机器人当前位置到 graph node 的整条线都 known free, planner 可能找不到可用起点

当前做法:

- `ensure_robot_anchor_node=true`
- current node 找不到 collision-free graph node 时创建 robot anchor
- robot anchor 不参与 frontier assignment
- robot anchor 只连接近邻 graph node
- unknown 不会直接否决 anchor 边
- 当前可见 obstacle 会否决 anchor 边

这不是把脚下 unknown 改成 free, 而是给 planner 一个临时 graph 起点

## Frontier 检测原则

frontier 的基本定义:

```text
frontier cell = free cell next to unknown cell
```

对应文件:

```text
frontier_detector.py
```

当前逻辑:

- 在 `ClassifiedGrid` 中检测 free / unknown 边界
- 忽略局部地图边缘噪声
- 以 `frontier_candidate_spacing` 降采样候选点
- 将 frontier points 分配给可安全到达的 owner node
- 用 `frontier_min_points` 和 `frontier_min_span` 过滤孤立噪声
- 更新 node 的 `is_frontier` 和 `frontier_points`

frontier point 不直接作为 graph node, 它是 frontier node 的几何属性

这样可以把密集未知边界压缩成少量可评分, 可规划的 graph frontier

## Edge 构建原则

edge 表示两个 node 之间可以安全通行

对应文件:

```text
edge_builder.py
grid_types.py
```

当前 edge 规则:

- 只在 `edge_radius` 内搜索候选
- 每个 node 只保留最近的 `max_edge_neighbors`
- 当前 node 可以使用更大的 `current_node_max_edge_neighbors`
- edge line 不能穿过 obstacle
- edge line 不能穿过 unknown
- edge line 需要满足 `min_obstacle_clearance`
- edge cost 默认以几何距离为主

这避免局部 free space 内出现近似全连接, 也避免 RViz 中路线或 graph edge 贴墙, 切角或穿障碍

## Historical Edge 原则

局部地图是滑窗, 机器人转向或移动后, 已走过区域可能暂时变成 unknown 或离开当前地图

如果每帧只按当前局部地图重建 edge, graph topology 会剧烈抖动

当前做法:

- `validate_historical_edges=true`
- 新 edge 仍要求整条线段 known free 且 clearance 足够
- 历史 edge 只用当前可见段做证伪
- 当前不可见或 unknown 的历史段不会直接删除旧边
- 当前可见段出现 obstacle 或 clearance 不足时删除旧边

这样保持空间记忆, 同时不会忽略新出现的障碍

## 从 G_nav 到 G_score

`graph_construction` 输出的是几何 graph:

```text
G_nav_t = (V_t, E_t)
```

WildOS 视觉模块会把它变成 scored graph:

```text
G_score_t = (V_t, E_t, S_t, D_t)
```

对应字段:

```text
key = frontier_scores
value = [score_bin_0, score_bin_1, ..., score_bin_15]

key = is_default_scored
value = [0.0] 或 [1.0]
```

`frontier_scores` 属于 `visual_navigation/wildos/nav.py`, 不属于 `graph_construction`

因此 graph construction 的关键是提供稳定 frontier geometry, 不生成语义分数

## Object Search 原则

当前目标搜索不再只是“初始方向 goal”

对应文件:

```text
visual_navigation/visual_navigation/object_search_goal_mux.py
visual_navigation/visual_navigation/stable_frontier_selector.py
visual_navigation/visual_navigation/wildos/nav.py
```

当前策略:

- 无目标时从 scored graph 选择稳定 frontier
- 默认优先 odom 前方 frontier
- 前方没有候选时回退到任意 frontier
- graph frontier UUID 抖动时按位置继承近邻 frontier
- 目标出现时优先发布目标方向上的安全 graph frontier
- 目标短时遮挡时可用 target latch 继续保持目标方向
- 目标到达或视觉近距离确认后进入 reached latch
- reached latch 持续发布当前位置 hold goal

这让目标搜索在“无目标探索, 目标记忆, 目标到达停止”之间稳定切换

## Planner 原则

`graphnav_planner` 在 scored graph 上求路径

对应文件:

```text
graphnav_planner/src/planner.cpp
graphnav_planner/src/planner_node.cpp
graphnav_planner/src/path_follower_node.cpp
```

当前原则:

- 输入 `graphnav_msgs/NavigationGraph`
- 输入 profile 配置的 goal pose topic
- 使用 `traversability_cost` 作为 graph edge 权重
- virtual goal 只参与搜索
- 默认不把 virtual goal 加入可执行 path
- 默认不把 unknown frontier point 加入可执行 path
- 到达 goal 半径内时发布当前位置单点 path
- `path_follower_node` 输出 `/spot1/tracking_goal_pose`

这保证 planner 选择目标时能利用 frontier score, 但最终 path 仍优先由安全 graph node 组成

## 当前完整算法流程

当前实现可以概括为:

```text
function GraphConstructionStep(input_map, robot_pose, previous_graph):
    classified_grid = DecodeAndClassify(input_map)

    graph = previous_graph
    graph = RemoveInvalidVisibleNodes(graph, classified_grid)

    new_nodes = SampleFreeNodes(classified_grid, graph)
    graph = MergeStableNodes(graph, new_nodes)

    frontier_cells = DetectFreeUnknownBoundary(classified_grid)
    graph = AssignFrontierPoints(graph, frontier_cells, classified_grid)

    current_node = FindCollisionFreeCurrentNode(graph, robot_pose, classified_grid)
    if current_node is missing and ensure_robot_anchor_node:
        current_node = CreateOrUpdateRobotAnchor(graph, robot_pose)

    new_edges = BuildVisibleSafeEdges(graph, classified_grid)
    graph.edges = MergeHistoricalEdges(graph.edges, new_edges, classified_grid)

    graph.current_node_idx = current_node.index
    return graph
```

这个流程的核心不是单次 frontier detection, 而是跨帧稳定 graph memory

## 当前难点

工程难点集中在:

- UUID 稳定性, 同一个物理区域不应每帧生成新 node
- frontier 生命周期, 已探索或消失的 frontier 要及时移除
- historical edge validation, 不能把当前看不到误判成不能走
- current node 匹配, 脚下 unknown 时要通过 robot anchor 接回 graph
- GridMap rolling buffer, row / column 和 frame 映射必须正确
- edge clearance, 避免路线贴墙, 切角或穿障碍
- frontier 密度, 节点太密会拖慢更新和规划, 节点太稀会断图
- object search 稳定性, 无目标时要避免 goal 每帧跳变或回头

## 当前配置落点

```text
graph_construction/configs/graph_construction_elevation.yaml
  elevation 主线 graph 参数

graph_construction/configs/graph_construction.yaml
  2D fallback graph 参数

graph_construction/configs/topic_profiles.yaml
  profile 级 topic, frame, path, goal 和 object search 参数

visual_navigation/configs/object_search_goal_mux.yaml
  object search goal mux 默认参数

graphnav_planner/launch/graphnav_planner.launch.yml
  planner 和 path follower 参数
```

## 当前启动落点

推荐入口:

```bash
./scripts/start_wildos_elevation.sh
```

Unity 目标搜索:

```bash
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh do_object_search:=true
```

2D fallback:

```bash
ros2 launch graph_construction wildos_2d_sim.launch.py topic_profile:=unity
```
