# Graph Construction 实现方案

日期: 2026-06-23

目标: 在 `external/wildos` 中新增 `graph_construction` 模块, 先实现一个可运行, 可调试, 可接入 WildOS 现有评分和路径规划链路的基础版本

核心链路:

```text
traversability grid + odom
    -> graph_construction
    -> /spot1/nav_graph
    -> WildOS visual scoring
    -> /spot1/scored_nav_graph
    -> graphnav_planner
    -> path
```

## 设计边界

本阶段只复现 Graph Construction 的基础几何图能力, 不重写 WildOS 已开源的视觉评分和图规划器

需要完成:

- 从局部可通行栅格中生成稀疏导航图
- 生成 free node, frontier node, edge, current node
- 输出 `graphnav_msgs/NavigationGraph`
- 接入 WildOS scoring 所需要的 frontier 几何字段
- 给 `graphnav_planner` 提供可连通, 可计算代价的图
- 提供 RViz Marker 可视化, 方便检查节点, 边, frontier, 半径
- 加入基础死路回退逻辑, 避免刚消失的 frontier 反复被选中

暂不完成:

- 不实现论文中未开源的雷达与视觉融合可通行区域生成
- 不直接生成语义 `frontier_scores`
- 不修改 `graphnav_msgs`
- 不重写 `graphnav_planner`
- 不实现 Web UI 或项目主页视频中的完整交互界面

## ROS2 版本

实现目标版本为 ROS2 Humble

模块形式为 Python ROS2 package:

```text
external/wildos/graph_construction
```

启动方式:

```text
ros2 launch graph_construction graph_construction.launch.py ns:=spot1
```

## 模块目录

```text
graph_construction/
  package.xml
  setup.py
  resource/graph_construction
  configs/graph_construction.yaml
  launch/graph_construction.launch.py
  docs/
  graph_construction/
    __init__.py
    node.py
    graph_builder.py
    grid_adapter.py
    frontier_detector.py
    edge_builder.py
    graph_memory.py
    deadend_recovery.py
    msg_utils.py
    viz.py
```

各文件职责:

- `node.py`, ROS2 节点入口, 负责订阅 grid 和 odom, 周期发布 graph 和 marker
- `graph_builder.py`, 图构建主流程, 串联节点更新, 采样, frontier, 建边, current node
- `grid_adapter.py`, 把 `OccupancyGrid` 转成 free, obstacle, unknown 三类栅格, 并提供 collision check
- `frontier_detector.py`, 检测 free 和 unknown 的边界, 并分配给附近 graph node
- `edge_builder.py`, 根据距离和碰撞检测生成图边
- `graph_memory.py`, 保存内部节点, 边, UUID, frontier 生命周期状态
- `deadend_recovery.py`, 记录已经消失的 frontier, 降低死路附近反复探索
- `msg_utils.py`, 把内部图转换为 `NavigationGraph`
- `viz.py`, 输出 RViz MarkerArray 可视化

## 输入

第一版输入保持最小:

```text
/spot1/odom, nav_msgs/Odometry
/spot1/traversability_grid, nav_msgs/OccupancyGrid
```

`OccupancyGrid` 的含义:

- 小于 `free_threshold` 的 cell 视为 free
- 大于等于 `obstacle_threshold` 的 cell 视为 obstacle
- 小于 0 或处于中间值的 cell 视为 unknown

后续如果接入更接近论文的实现, 输入可替换为:

```text
grid_map_msgs/GridMap
elevation_mapping_cupy output
terrain traversability map
```

## 输出

主要输出:

```text
/spot1/nav_graph, graphnav_msgs/NavigationGraph
```

调试输出:

```text
/spot1/graph_construction_viz, visualization_msgs/MarkerArray
```

输出图需要满足 WildOS 当前代码的接口期望:

```text
NavigationGraph
  header
  nodes
  edges
  current_node_idx
  trav_classes = ["default"]
```

每个 node 至少需要:

```text
uuid
position
free_radius
explored_radius
trav_properties[0].is_frontier
trav_properties[0].frontier_points
trav_properties[0].traversability_score
```

每条 edge 至少需要:

```text
start_node_idx
end_node_idx
weight
trav_properties[0].traversability_score
```

## 与 WildOS 的关系

Graph Construction 输出的是几何图, 不直接决定目标在哪里

WildOS 视觉模块会读取 `/spot1/nav_graph`, 找出 `is_frontier = true` 的节点, 把它们投影到图像上, 根据视觉可通行性, 视觉 frontier, 目标相似度生成 `frontier_scores`

之后视觉模块发布:

```text
/spot1/scored_nav_graph
```

`graphnav_planner` 实际使用 scored graph, 它会在图上运行 Dijkstra, 并把 frontier score 转成到虚拟目标节点的代价

所以 Graph Construction 最重要的契约是:

- frontier node 要稳定
- frontier points 要指向 unknown 边界
- graph edge 要表示可通行连通关系
- current node 要能匹配机器人当前位置
- UUID 不要频繁抖动, 否则视觉评分缓存会失效

## 核心图论模型

Graph Construction 维护一个无向加权图:

```text
G_t = (V_t, E_t)
```

解释:

- `G_t` 是时刻 `t` 的导航图
- `V_t` 是节点集合, 每个节点表示一个可站立或可经过的位置
- `E_t` 是边集合, 每条边表示两个节点之间可以直线通行

节点:

```text
v_i = (p_i, r_i, e_i, f_i)
```

解释:

- `p_i` 是节点世界坐标
- `r_i` 是 free radius, 表示节点附近安全可通行半径
- `e_i` 是 explored radius, 表示节点附近已经被观察过的范围
- `f_i` 表示是否为 frontier node

边:

```text
e_ij = (v_i, v_j, w_ij)
```

第一版边权:

```text
w_ij = ||p_i - p_j||_2
```

解释:

- 两个节点之间直线无碰撞时才建立边
- 权重先用欧氏距离, 后续可以叠加坡度, 粗糙度, traversability cost

## 节点采样公式

从 free cell 中采样节点, 但需要保证节点之间不要太密:

```text
min_j ||p_new - p_j||_2 >= d_min
```

解释:

- `p_new` 是新候选节点
- `p_j` 是已有节点
- `d_min` 是最小节点间距, 对应配置 `min_node_separation`
- 这样可以把 dense grid 压缩成 sparse graph

节点安全半径:

```text
r_i = min(d_obstacle(p_i), d_unknown(p_i), r_max)
```

解释:

- `d_obstacle(p_i)` 是到最近 obstacle 的距离
- `d_unknown(p_i)` 是到最近 unknown 的距离
- `r_max` 是最大半径上限
- 这个半径既用于可视化, 也用于判断 explored area

## Frontier 定义

frontier cell 是 free 和 unknown 的边界:

```text
c is frontier <=> c is free and exists n in N(c), n is unknown
```

解释:

- `c` 是一个 free cell
- `N(c)` 是它的邻域
- 如果它旁边有 unknown cell, 说明从这里附近继续走可能进入未探索区域

frontier node 不是每个 frontier cell 都单独生成一个节点

第一版做法是:

```text
frontier cell -> nearest collision-free graph node -> node.frontier_points
```

这样可以保持图稀疏, 也符合 WildOS 对 frontier node 的接口期望

## Frontier 分配公式

给一个 frontier point `p_f`, 找最近的可达 graph node:

```text
v* = argmin_v ||p_v - p_f||_2
```

约束:

```text
||p_v - p_f||_2 <= R_frontier
line(p_v, p_f) is collision-free
p_f not in removed_frontiers
```

解释:

- `R_frontier` 对应 `frontier_assign_radius`
- collision-free 保证该 frontier 可以从图节点附近安全接近
- removed frontier suppression 用来减少死路附近反复尝试

如果一个节点挂载的 frontier points 足够多:

```text
|frontier_points(v_i)| >= N_min
```

则:

```text
is_frontier(v_i) = true
```

否则:

```text
is_frontier(v_i) = false
```

## 建边公式

两个节点满足距离约束:

```text
||p_i - p_j||_2 <= R_edge
```

并且两点连线穿过的 grid cell 都不是 obstacle:

```text
collision_free(p_i, p_j) = true
```

才建立边:

```text
(v_i, v_j) in E_t
```

第一版边权:

```text
w_ij = ||p_i - p_j||_2
```

后续可扩展为:

```text
w_ij = distance_ij * (1 + alpha * terrain_cost_ij)
```

解释:

- `distance_ij` 是几何距离
- `terrain_cost_ij` 可以来自坡度, 粗糙度, 雷达可通行性
- `alpha` 控制地形代价对路径的影响

## 死路回退

死路的典型现象:

```text
frontier node 被选中
机器人走过去
局部地图更新后发现该 frontier 不再连接新的 unknown 区域
planner 又因为几何距离近反复选择同一片区域
```

第一版处理方式:

```text
removed_frontiers = recently disappeared frontier points
```

如果新的 frontier point 离 removed frontier 太近:

```text
||p_f - p_removed||_2 <= R_suppress
```

则跳过它

解释:

- 这不是完整的全局行为树回退
- 但可以让图构建层不要把刚失败的边界马上重新发布出去
- planner 在没有这个 frontier 后, 会倾向选择其它 frontier

## 视觉边界节点探索

论文中的视觉探索不是 Graph Construction 单独完成的, 而是几何 frontier 和视觉 scoring 共同完成

Graph Construction 负责:

```text
is_frontier
frontier_points
node position
node uuid
graph edges
```

WildOS 视觉模块负责:

```text
project frontier into camera images
estimate visual traversability
estimate visual frontier direction
estimate object similarity
write frontier_scores
```

planner 负责:

```text
choose best frontier by graph cost and score
run Dijkstra
publish path
```

因此第一版 Graph Construction 只要把几何 frontier 做稳定, 就可以接入后续视觉评分链路

## 主流程伪代码

```text
function UpdateGraph(occupancy_grid, odom):
    grid = ClassifyGrid(occupancy_grid)

    obstacle_sdf = DistanceToObstacle(grid)
    unknown_sdf = DistanceToUnknown(grid)

    for each existing node v:
        if v is outside current grid:
            keep v as memory node
            continue

        if v is no longer free:
            remove v
            continue

        v.free_radius = min(obstacle_sdf[v], unknown_sdf[v], max_free_radius)
        v.explored_radius = max(v.explored_radius, unknown_sdf[v])

        if v.free_radius < min_obstacle_clearance:
            remove v

    for each sampled free cell c:
        p = CellCenter(c)

        if DistanceToNearestNode(p) < min_node_separation:
            continue

        if obstacle_sdf[c] < min_obstacle_clearance:
            continue

        create graph node at p

    frontier_cells = DetectFrontierCells(grid)

    for each node v:
        clear v.frontier_points
        v.is_frontier = false

    for each frontier cell c_f:
        p_f = CellCenter(c_f)

        if NearRemovedFrontier(p_f):
            continue

        owner = NearestNodeWithinRadius(p_f, frontier_assign_radius)

        if owner exists and CollisionFree(owner.position, p_f):
            owner.frontier_points.append(p_f)

    for each node v:
        if count(v.frontier_points) >= frontier_min_points:
            v.is_frontier = true

    UpdateDeadendMemory()

    edges = BuildCollisionFreeEdges(nodes, grid, edge_radius)
    current_node = NearestCollisionFreeNode(odom.position)

    return NavigationGraph(nodes, edges, current_node)
```

## 当前参数

```text
free_threshold: 20
obstacle_threshold: 65
sample_stride: 8
min_node_separation: 1.0
max_free_radius: 4.0
min_obstacle_clearance: 0.5
edge_radius: 8.0
frontier_assign_radius: 5.0
frontier_min_points: 2
deadend_observation_count: 3
removed_frontier_suppression_radius: 1.0
publish_rate_hz: 2.0
```

这些参数是第一版调试值, 重点是输出稳定, 可观察, 能接入 planner

后续实测时需要根据地图分辨率和机器人尺寸调整:

- `sample_stride`, 控制节点密度
- `min_node_separation`, 控制图稀疏程度
- `edge_radius`, 控制图连通性
- `frontier_assign_radius`, 控制 frontier 能挂到多远的节点
- `min_obstacle_clearance`, 控制安全距离

## 可视化

第一版包含 RViz Marker 可视化, 不包含项目主页视频中的完整 Web UI

可视化内容:

- free nodes
- frontier nodes
- graph edges
- frontier points
- free radius 或 explored radius
- current node

用途:

- 检查节点是否落在 free space
- 检查 edge 是否穿过 obstacle
- 检查 frontier points 是否贴近 unknown 边界
- 检查 current node 是否跟随机器人位置
- 检查死路 frontier 是否会消失

## 与论文公式的对应关系

论文中的 Graph Construction 主要提供:

```text
T_geo = (G_t, F_geo_t)
```

解释:

- `G_t` 是稀疏导航图
- `F_geo_t` 是几何 frontier node 集合
- 本模块输出的 `NavigationGraph` 对应这个几何图

frontier node 集合:

```text
F_geo_t = {v_i in V_t | is_frontier(v_i) = true}
```

视觉评分后, planner 使用 scored graph 计算路径

简化代价可以理解为:

```text
cost(path) = sum edge_cost + frontier_virtual_cost
```

其中 frontier virtual cost 会被视觉分数影响:

```text
frontier_virtual_cost = d_frontier_goal * (1 - beta * log(score))
```

解释:

- `score` 越高, 该 frontier 越可能指向目标
- `frontier_virtual_cost` 越小, Dijkstra 越倾向选择它
- Graph Construction 不计算 `score`, 但必须保证 frontier node 和 frontier_points 正确

## 第一版验收标准

构建:

```text
colcon build --packages-select graph_construction graphnav_msgs graphnav_planner visual_navigation
```

启动:

```text
ros2 launch graph_construction graph_construction.launch.py ns:=spot1
```

Topic 检查:

```text
ros2 topic echo --once /spot1/nav_graph
ros2 topic echo --once /spot1/graph_construction_viz
```

应看到:

- `nodes` 非空
- `edges` 非空
- `trav_classes` 包含 `default`
- `current_node_idx` 在 nodes 范围内
- frontier nodes 有 `frontier_points`
- RViz 中节点和边不明显穿过障碍

## 风险和后续工作

当前第一版可以帮助接通链路, 但还不是论文级完整实现

主要风险:

- `OccupancyGrid` 表达不了完整 2.5D 地形, 坡度和粗糙度暂时缺失
- 节点采样是规则 stride, 不是论文中更灵活的局部随机采样
- frontier 生命周期只做基础 suppression, 还没有完整任务层回退策略
- 没有接入雷达生成的 traversability cost
- 没有实现主页视频里的 UI

后续优先级:

1. 用真实 WildOS bag 或仿真 grid 验证 graph topic
2. 调整节点密度和 edge radius, 保证 planner 可以连通
3. 检查视觉模块是否能正确投影 frontier
4. 接入 elevation 或 radar traversability, 替换简单 OccupancyGrid 阈值
5. 增强死路回退, 增加 frontier failed count 和低价值 frontier 记忆
6. 如果需要主页视频效果, 再单独实现 graph debug UI 或接入现有 RViz/Web 可视化

## 本阶段结论

本方案先把 Graph Construction 定义为 WildOS 中的几何图生成层

它不直接做目标识别, 也不直接做语义评分, 而是负责把可通行区域组织成稳定的稀疏图, 并把未知边界表达成 frontier nodes

只要 `/spot1/nav_graph` 满足 WildOS 的消息契约, 后续视觉 scoring 和 `graphnav_planner` 就可以在这个图上继续工作

