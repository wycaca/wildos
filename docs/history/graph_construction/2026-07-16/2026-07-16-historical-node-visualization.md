# 历史导航图节点可视化修复

日期: 2026-07-16

## 1. 问题

`/spot1/graph_construction_viz` 中历史红色图边正常显示，但边的端点几乎不可见，容易误判为导航图只保存了边，没有保存历史节点

图数据没有丢失。历史普通节点已经发布在 `memory_free_nodes` namespace 中，但原来的绿色亮度、透明度和尺寸都明显低于当前节点，在深色 RViz 背景和大范围视角下无法辨认

## 2. 修改

历史普通节点的 Marker 参数调整为:

```text
color: (0.0, 0.8, 0.0)
alpha: 0.9
scale: 0.25m
```

历史节点仍使用独立的 `memory_free_nodes` namespace，便于在 RViz 中单独开关，但其可见性与当前绿色节点保持一致

本次不延长紫色 `frontier_points` 的生命周期。Frontier 只表达当前仍有效的 free/unknown 边界，失效后继续清除，避免旧边界误导视觉评分和路线规划

## 3. 验证

新增 `test_graph_viz_keeps_historical_nodes_visible`，检查历史节点仍被写入 Marker，并验证尺寸、颜色和透明度不会退回不可见状态
