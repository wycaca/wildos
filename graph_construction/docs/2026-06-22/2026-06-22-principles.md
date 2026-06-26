# Graph Construction 原理详解

日期: 2026-06-22

目标: 用更具体, 更通俗的方式理解 WildOS 论文中未开源的 Graph Construction, 包括它为什么需要图论, 论文里有哪些相关公式, 以及后续复现时可以按什么算法拆解

## 先说结论

Graph Construction 的核心任务是把机器人局部看到的几何地图, 转换成一个能长期记忆, 能扩展, 能规划的稀疏导航图

在 WildOS 中, 机器人不能只依赖当前相机画面或当前局部地图, 因为野外环境很大, 局部深度感知范围有限, 走进死路后还需要记住之前哪里没探索完

所以 Graph Construction 做的是这件事:

```text
局部 traversability map
    -> 可走区域
    -> 未知区域边界
    -> 稀疏 free nodes
    -> frontier nodes
    -> collision-free edges
    -> NavigationGraph
```

这张图后面会被 ExploRFM 加上视觉评分, 再交给 graphnav_planner 用 Dijkstra 做高层路径规划

## 图论视角

论文中把导航图写成:

```text
G_nav_t = (V_t, E_t)
```

解释:

- `G_nav_t` 是时间 `t` 的导航图
- `V_t` 是节点集合, 每个节点代表一个机器人可以到达的位置
- `E_t` 是边集合, 每条边代表两个节点之间有一条可走连接
- 边不是普通连线, 它带有 `traversability cost`, cost 越大表示越难走或越不安全

可以把它理解成一张野外地图上的路线网:

```text
节点 = 可以站的位置
边 = 两个位置之间能不能走
边权重 = 这段路有多难走
frontier node = 站在这里可以继续进入未知区域
```

## Graph Construction 在论文中的定义

论文方法章节写到, 局部几何可通行地图 `T_geo_t` 会被增量集成到导航图中:

```text
T_geo_t -> G_nav_t = (V_t, E_t)
```

其中:

```text
v_i in V_t
```

表示一个可达节点

```text
(v_i, v_j) in E_t
```

表示节点 `v_i` 和 `v_j` 之间有一条边

frontier 节点集合定义为:

```text
F_geo_t = {v_i in V_t | v_i.isFrontier = true}
```

解释:

- `F_geo_t` 是所有几何 frontier nodes 的集合
- 如果某个节点 `v_i` 的 `isFrontier` 为 true, 它就是 frontier node
- frontier node 的含义是, 它附近存在 free space 和 unknown space 的边界
- 这些节点是后续探索候选目标

## 为什么需要 frontier node

传统 frontier exploration 的思想很简单:

```text
已知 free 区域和未知区域之间的边界 = frontier
机器人往 frontier 走 = 探索新区域
```

但 WildOS 不是直接在 dense grid 上规划, 而是把 frontier 绑定到稀疏 graph node 上

这样做有几个好处:

- 图更小, 比 dense grid 更适合长距离规划
- 图能保留历史, 局部地图滑窗移动后不会丢掉所有记忆
- frontier node 可以被视觉模块评分, 形成 `scored_nav_graph`
- planner 可以用标准图搜索算法, 比如 Dijkstra 或 A*

## 关键字段如何对应代码

开源代码中的 `graphnav_msgs/msg/NavigationGraph.msg` 对应论文里的 `G_nav_t`

```text
std_msgs/Header header
string[] trav_classes
Node[] nodes
Edge[] edges
uint32 current_node_idx
```

解释:

- `nodes` 对应 `V_t`
- `edges` 对应 `E_t`
- `current_node_idx` 是机器人当前所在或最近的节点
- `trav_classes` 表示可通行性类别, WildOS 默认使用 `default`

`NodeTraversabilityProperties.msg` 是理解 Graph Construction 的重点:

```text
bool is_frontier
geometry_msgs/Point[] frontier_points
float32 explored_radius
float32 free_radius
KeyValue[] properties
```

解释:

- `is_frontier`, 当前节点是不是 frontier node
- `frontier_points`, 这个节点关联的未知边界点
- `explored_radius`, 该节点周围多大范围已经被探索过
- `free_radius`, 该节点周围多大范围是自由可通行区域

`EdgeTraversability.msg`:

```text
float32 traversability_cost
KeyValue[] properties
```

解释:

- `traversability_cost` 是边权重
- graph planner 会把它作为 Dijkstra 的图搜索 cost

## 论文 Algorithm 1 的整体流程

论文给出的主流程可以理解为:

```text
function UpdateNavigationGraph(previous_graph, local_traversability_map):
    unknown_sdf, obstacle_sdf = ComputeSDF(local_traversability_map)

    graph = UpdateExistingNodes(previous_graph, unknown_sdf, obstacle_sdf)

    new_nodes = SampleNewNodes(local_traversability_map, graph)
    graph.nodes = graph.nodes union new_nodes

    graph.nodes = UpdateFrontierNodes(local_traversability_map, graph.nodes)

    new_edges = BuildEdges(graph.nodes, local_traversability_map)
    graph.edges = graph.edges union new_edges

    return graph
```

解释:

1. 先从局部地图里算出 unknown 和 obstacle 的距离场
2. 用距离场更新已有节点, 判断节点附近是否还安全, 是否已经探索
3. 在 free cells 里采样新节点, 补充图覆盖
4. 找 free 和 unknown 的边界, 更新 frontier nodes
5. 在节点之间建立 collision-free edges
6. 返回新的 `G_nav_t`

这里的关键词是 "incrementally integrated", 也就是增量更新, 不是每一帧完全重建

## SDF 是什么

论文伪代码中使用了:

```text
SDF_unk_t
SDF_obs_t
```

通俗理解:

- `SDF_unk_t(p)` 表示点 `p` 到最近 unknown 区域的距离
- `SDF_obs_t(p)` 表示点 `p` 到最近 obstacle 的距离

它们可以用来回答两个问题:

```text
离障碍物还有多远
离未知区域还有多远
```

这正好对应 `free_radius` 和 `explored_radius`

## 更新已有节点

论文 Algorithm 2 可以简化为:

```text
function UpdateNodes(graph):
    for each node v_i in graph within current map bounds:
        free_radius = min(SDF_obs(v_i), SDF_unk(v_i), max_free_radius)
        explored_radius = max(previous_explored_radius, SDF_unk(v_i))

        if free_radius == 0:
            remove v_i and its edges

    return graph
```

解释:

- 如果节点离障碍物太近, `free_radius` 可能变成 0, 说明节点不再可用
- `explored_radius` 只增不减, 表示这个节点曾经帮我们确认过多大范围的空间
- 删除失效节点时也要删除相关边, 否则图会出现不可走的连接

这个步骤是空间记忆的基础

## 采样新节点

论文 Algorithm 3 的思想是, 在当前局部 free cells 中采样新的 graph nodes

简化伪代码:

```text
function SampleNewNodes(local_map, graph, num_samples):
    new_nodes = empty set

    for k in 1 to num_samples:
        p = random sample from free cells

        if p is too close to existing node:
            continue

        if p is inside obstacle or unknown:
            continue

        create node at p
        compute free_radius for node
        add node to new_nodes

    return new_nodes
```

解释:

- 采样点必须在 free 区域
- 不能和已有节点太近, 否则图会过密
- 新节点要带 pose, uuid, free_radius, explored_radius
- 后续会用这些节点建边

通俗讲, 这是在可走区域里撒一些稀疏路标点

## 检测 frontier cells

frontier cell 的定义可以通俗写成:

```text
frontier cell = free cell next to unknown cell
```

对应伪代码:

```text
function DetectFrontierCells(local_map):
    frontier_cells = empty set

    for each cell c in local_map:
        if c is free and any neighbor of c is unknown:
            add c to frontier_cells

    return frontier_cells
```

解释:

- free cell 表示机器人已经知道这里能走
- unknown neighbor 表示旁边还有没看过的区域
- 这个边界就是探索最有价值的位置

## 把 frontier cells 分配给节点

论文 Algorithm 4 的重点是, frontier cell 不直接作为 graph node, 而是分配给附近已有节点

简化伪代码:

```text
function UpdateFrontierNodes(local_map, nodes):
    for each node v_i:
        clear temporary frontier_points

    frontier_cells = DetectFrontierCells(local_map)

    for each frontier cell c_f:
        p_cf = center position of c_f

        if p_cf is inside explored_radius of any node:
            continue

        v_star = nearest node that has collision-free connection to p_cf

        if v_star exists:
            add p_cf to v_star.frontier_points

    for each node v_i:
        if count(v_i.frontier_points) >= N_min
           and span(v_i.frontier_points) >= S_min:
            v_i.isFrontier = true
        else:
            v_i.isFrontier = false

    return nodes
```

解释:

- `frontier_points` 是 unknown 边界点
- 一个 frontier node 可以对应多个 frontier points
- 如果某个 frontier cell 已经落在别的节点 explored_radius 内, 就说明它不再是有效探索边界
- 找最近 collision-free node 是为了保证这个 frontier 可以从图上安全到达
- `N_min` 和 `S_min` 用于过滤孤立或过短的噪声 frontier
- 局部滑窗边界附近的 frontier cell 应被过滤, 避免把局部地图外边缘当作探索边界

这一步的效果是, 把密集边界点压缩成少量 frontier nodes

## 建立 edges

论文 Algorithm 5 很直接:

```text
function BuildEdges(nodes, local_map):
    new_edges = empty set

    for each node v_i in nodes:
        neighbors = GetSpatialNeighbors(v_i, nodes, edge_radius)

        for each node v_j in neighbors:
            if CollisionFree(v_i, v_j):
                add edge (v_i, v_j) to new_edges

    return new_edges
```

解释:

- 先找空间距离足够近的节点
- 再检查两点之间是否 collision-free
- 从 collision-free 候选中保留最近的少量近邻, 避免局部 free space 内近似全连接
- 如果中间没有障碍或不可通行区域, 且节点仍在近邻上限内, 就加边
- 边的 cost 可以来自距离, traversability, slope, roughness 等

当前第一版实现采用:

```text
E_i = nearest K collision-free neighbors within edge_radius
```

解释:

- `edge_radius` 控制候选搜索范围
- `K` 控制普通节点的最大近邻边数量
- 当前机器人所在节点使用更大的 `K_current`, 让机器人附近保留更多路径选择
- 这样更符合 sparse navigation graph 的目标, 也能避免 RViz 中边线过密遮挡地图

最小复现时可以先用:

```text
edge_cost = EuclideanDistance(v_i, v_j)
```

后续再升级为:

```text
edge_cost = distance_cost + traversability_penalty + slope_penalty
```

## 从 G_nav 到 G_score

Graph Construction 输出的是几何图:

```text
G_nav_t = (V_t, E_t)
```

WildOS 视觉模块会把它变成 scored graph:

```text
G_score_t = (V_t, E_t, S_t, D_t)
```

解释:

- `S_t` 是 frontier node 的视觉语义评分
- `D_t` 是这个评分上次更新时机器人到该 frontier 的距离
- 开源代码中, `S_t` 被写入 node.properties 的 `frontier_scores`

对应代码字段:

```text
key = frontier_scores
value = [score_bin_0, score_bin_1, ..., score_bin_15]
```

这表示同一个 frontier node 在不同目标方向上有不同分数

## 论文中的规划代价公式

论文把到目标的代价拆成两部分:

```text
Cost(x_T, p_goal) = argmin over v_i in F_geo_t [
    d(x_T, v_i) + z(s_i) * d(v_i, p_goal)
]
```

解释:

- `x_T` 是机器人当前位姿
- `p_goal` 是估计出来的目标位置
- `v_i` 是候选 frontier node
- `d(x_T, v_i)` 是从机器人走到 frontier 的已知局部代价
- `d(v_i, p_goal)` 是从 frontier 到目标的大致距离
- `s_i` 是视觉语义分数, 分数越高表示越值得去
- `z(s_i)` 是把分数转成代价缩放因子的函数

论文中给出:

```text
z(s_i) = 1 - alpha * log(s_i + epsilon)
```

解释:

- `alpha` 是调节视觉分数影响力的参数
- `epsilon` 防止 `log(0)` 数值错误
- `s_i` 越大, `log(s_i + epsilon)` 越大, `z(s_i)` 越小
- `z(s_i)` 越小, 说明这条 frontier 到 goal 的启发式代价越低

也就是说, 视觉认为更有希望的 frontier 会被 planner 偏好

## auxiliary goal node 是什么

论文和代码都用了一个技巧, 把目标点变成图上的虚拟节点:

```text
v_goal_hat = auxiliary goal node
```

然后把每个 frontier node 连到这个虚拟目标节点:

```text
edge = (v_i, v_goal_hat)
edge_cost = z(score_i) * safe_distance(v_i, p_goal)
```

这样一来, 原本的目标选择问题就变成了标准最短路问题:

```text
path = Dijkstra(G_score_t, v_start, v_goal_hat)
```

解释:

- `v_start` 是离机器人最近的当前节点
- `v_goal_hat` 是临时加进图里的目标节点
- Dijkstra 会自动在所有 frontier 中选择总代价最低的一条路径
- 搜索结束后删掉虚拟目标节点即可

这就是 Graph Construction 和图论的直接关系

## 和开源 planner 的对应关系

`graphnav_planner/src/planner.cpp` 中的逻辑和论文公式基本对应:

```text
graph_.add_vertex(node, i)
graph_.add_edge(edge.from_idx, edge.to_idx, weight)
current_node_idx_ = graph->current_node_idx
```

解释:

- ROS 消息里的 nodes 被转成图顶点
- ROS 消息里的 edges 被转成图边
- `traversability_cost` 作为边权重

planner 会读取 node.properties 中的:

```text
frontier_scores
```

然后根据目标方向选择对应 bin 的分数:

```text
best_bin = BinIndex(goal_heading)
frontier_score = frontier_scores[best_bin]
```

再计算 frontier 到虚拟目标节点的边代价:

```text
frontier_cost = frontier_path_distance * (1 - frontier_score_factor * log(frontier_score))
```

最后执行:

```text
dijkstra_shortest_path(graph, current_node_idx, virtual_goal)
```

这说明复现 Graph Construction 时, 最重要的是把 `NavigationGraph` 消息喂对

## 一个完整但简化的 Graph Construction 伪代码

下面是后续复现时可以使用的版本:

```text
function GraphConstructionStep(local_grid, robot_pose, previous_graph):
    graph = previous_graph

    unknown_sdf = DistanceToUnknown(local_grid)
    obstacle_sdf = DistanceToObstacle(local_grid)

    for each node in graph.nodes:
        if node is outside valid local area:
            keep node as memory node
            continue

        node.free_radius = min(obstacle_sdf(node.pose), unknown_sdf(node.pose), max_free_radius)
        node.explored_radius = max(node.explored_radius, unknown_sdf(node.pose))

        if node.free_radius <= min_free_radius:
            remove node and connected edges

    new_nodes = SampleFreeNodes(local_grid, graph.nodes)
    graph.nodes = MergeNodes(graph.nodes, new_nodes)

    frontier_cells = DetectFreeUnknownBoundary(local_grid)

    for each node in graph.nodes:
        node.frontier_points = empty list
        node.is_frontier = false

    for each frontier_cell in frontier_cells:
        p = CellCenter(frontier_cell)

        if IsInsideAnyExploredRadius(p, graph.nodes):
            continue

        owner = FindNearestCollisionFreeNode(p, graph.nodes, local_grid)

        if owner exists:
            owner.frontier_points.append(p)
            owner.is_frontier = true

    graph.edges = RebuildOrUpdateEdges(graph.nodes, local_grid)
    graph.current_node_idx = FindNearestNode(robot_pose, graph.nodes)

    return graph
```

## 最小复现版本可以怎么降级

为了先跑通 WildOS 开源部分, 可以先做一个 2D grid 版本:

```text
输入:
    occupancy grid
    robot pose

输出:
    NavigationGraph

节点:
    在 free cells 中按固定间距采样

frontier:
    free cell 旁边有 unknown cell

边:
    距离小于 edge_radius 且 Bresenham 直线无障碍

cost:
    欧氏距离

current_node_idx:
    离 robot_pose 最近的节点
```

这个版本虽然不够野外机器人真实使用, 但可以验证:

- `visual_navigation/wildos/nav.py` 能不能投影 frontier
- `frontier_scores` 能不能写回 graph
- `graphnav_planner` 能不能生成 path
- RViz 中 free nodes, frontier nodes, frontier_points, edges 是否合理

## 真正难点

Graph Construction 的难点不在单独检测 frontier, 而在下面这些工程细节:

- UUID 稳定性, 同一个物理 frontier 不应该每帧变成新节点
- frontier 生命周期, 已探索或消失的 frontier 要及时移除
- 在局部滑窗和稀疏点云输入下, removed frontier suppression 不能过强, 否则短暂消失的合法 frontier 会被长期抑制
- 图连通性, 节点太稀会断图, 节点太密会拖慢规划
- collision check, 边必须确实可走
- 局部地图滑窗, 当前地图范围外的历史节点要保留还是冻结
- current node 匹配, 机器人当前位置要可靠映射到 graph node
- cost 设计, cost 太简单会忽略地形风险, cost 太复杂会难以调试

## 本阶段建议

后续复现可以按这个顺序推进:

```text
1. fake graph publisher
2. 2D grid graph construction
3. frontier detection and node assignment
4. stable UUID and incremental update
5. edge collision check and cost
6. elevation_mapping_cupy integration
7. WildOS full pipeline integration
```

最重要的是先让 `NavigationGraph` 消息契约稳定, 因为 WildOS 开源部分已经围绕这个消息设计好了

## 资料来源

- WildOS paper, https://arxiv.org/abs/2602.19308
- WildOS repository, https://github.com/nasa-jpl/nebula2-wildos
- 本地代码, `external/nebula2-wildos/graphnav_msgs`
- 本地代码, `external/nebula2-wildos/graphnav_planner`
- 本地代码, `external/nebula2-wildos/visual_navigation`
