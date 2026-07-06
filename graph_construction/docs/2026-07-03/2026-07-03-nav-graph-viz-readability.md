# nav_graph_viz 可读性调整

日期: 2026-07-03

## 背景

RViz2 中 `/spot1/nav_graph_viz` 会显示 WildOS visual navigation 看到的导航图

用户观察到:

- 障碍物后方仍有 graph node 显示
- 这些 node 和当前机器人所在区域没有连通
- 红色 graph edge 过淡, 在地图上不够显眼

## 节点显示语义

`/spot1/nav_graph_viz` 显示的是当前导航图中的节点, 不是只显示当前可达节点

因此障碍物后面的节点可能有两种情况:

- 它们属于历史 graph memory 或其它连通分量
- 它们在局部地图中是 free node, 但当前和机器人所在连通分量不连通

如果没有边连接到当前分量, 它们对当前 planner 来说不可达

这类节点仍有诊断意义:

- 检查 graph memory 是否保留了历史区域
- 检查 obstacle / unknown 是否把图切成多个连通分量
- 检查 frontier score 是否错误地落到不可达分量

但在主导航视图中, 不能把“显示出来”理解成“当前可达”

## 可达性判断

当前可达性应优先看:

```text
graph_construction 慢帧日志中的 连通分量, 当前分量大小, 当前节点状态
```

以及 planner 是否能从当前 node 找到 path

如果需要 RViz 更明确地区分可达和不可达, 后续可以在 `nav_graph_viz` 中按 current connected component 给节点和边分层着色

## 本次修改

只调整 `/spot1/nav_graph_viz` 的红色 edge 可读性:

```text
edge line width: 0.006 -> 0.018
edge alpha: 0.18 -> 0.42
```

不改变 graph 数据, 不改变 planner 行为

## 验证

重启 WildOS 后在 RViz2 中打开:

```text
/spot1/nav_graph_viz
/spot1/score_rings
/spot1/graph_construction_viz
```

预期:

- 红色 graph edge 比之前更容易看见
- score ring 和目标检测 marker 不应被红线完全压住
- 障碍物后方不连通节点仍可能显示, 但不代表当前可达

