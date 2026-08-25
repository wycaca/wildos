# 持久导航图

`graph_construction` 把 elevation `GridMap` 和 canonical odom 转换为持久 `NavigationGraph`

## 数据语义

- `free`: 当前确认可通行
- `obstacle`: 当前确认不可通行, 优先于人工先验和历史记忆
- `unknown`: 当前不可见, 不生成新节点或新边
- rolling map 滑出后的 unknown 不直接删除已确认的历史安全路线

## 更新流程

```text
GridMap + odom
  -> 分类和变化检测
  -> 受限脚下盲区修补
  -> 更新历史节点并采样新节点
  -> 更新 Frontier 和 current node
  -> 只重查变化区域影响的边
  -> 发布完整 NavigationGraph 和修补后高程图
```

## 关键边界

- 只在机器人可达的 free 分量采样节点
- 节点使用稳定 UUID
- 新边必须有完整已知安全走廊
- 新障碍删除冲突节点和边
- free 变 unknown 时保留已确认历史边
- 稳定帧不得重复扫描全部历史节点和局部 pair
- 对外发布完整图, 增量索引只属于内部实现

## GO2 脚下盲区

MID360 下方存在三相机结构遮挡. 当前关闭 elevation mapping 的启动平面, graph construction 修补与机器人直接连通的近场 unknown, 并将相同结果发布到 `/spot1/elevation_map_repaired` 供 RViz 显示

修补必须满足:

- 不覆盖 obstacle
- 到候选栅格的射线不穿过墙体或受保护高程突变
- 优先使用附近真实地面高程
- 初始化成功后固定在世界坐标, 不跟随机器人移动
- 后续真实观测可以覆盖人工先验

当前范围和高度参数以 `graph_construction/configs/graph_construction_elevation.yaml` 为准. 该行为仍需按 [TODO](todo.md) 完成 GO2 实机验收

## Frontier 和规划起点

Frontier 是 free 与 unknown 的边界, 挂在附近安全节点上. rolling map 外边缘不是有效 Frontier

`current_node` 优先选择机器人附近可安全直达的普通节点. 图发布前必须确认它属于当前可达分量

## 入口和验证

- 实现: `graph_construction/graph_construction/graph_builder.py`
- ROS adapter: `graph_construction/graph_construction/node.py`
- 配置: `graph_construction/configs/graph_construction_elevation.yaml`
- 测试: `graph_construction/test/`

实机检查:

- `ros2 topic hz /spot1/elevation_map_repaired`
- 脚下修补不穿墙、不覆盖机身和近场障碍
- 运动后历史节点和边保持稳定
- 新障碍删除危险拓扑
- graph 更新耗时不随历史图无界增长
