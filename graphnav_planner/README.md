# GraphNav Planner

`graphnav_planner` 消费视觉评分后的 `NavigationGraph`, 在持久安全图上运行 Dijkstra 并发布 `nav_msgs/msg/Path`

## 职责边界

- Goal Mux 决定高层搜索状态和目标
- Planner 选择图上路线并维护探索分支记忆
- 仓库外控制器执行 Path、局部避障和停车
- Planner 不把虚拟目标加入图, 不规划穿过 unknown 的路线
- Planner 不发布 Marker 或调试 GridMap, `frontier_scores` 保留在公开 scored graph 中供规划和外部可视化消费

探索路线保持、死路确认和岔路恢复见[目标搜索](../docs/object_search.md)

## 输入失效

- Path 只在路线、停止状态或观察姿态变化时发布, 不是心跳
- graph 过期且 odom 新鲜时最多发布一次当前位置 hold Path
- odom 过期或输入时间戳超出容差时停止发布
- 输入恢复后重新验证当前路线

外部控制器必须独立监测 odom, 拒绝过期或未来 Path, 故障时停车并丢弃缓存路线. 当前阈值以 `config/planner.yaml` 为准

## 性能边界

Planner 核心只维护规划所需的内部 unexplored distance map. 每次成功规划不再检查两个 debug publisher, 也不在回调中序列化 MarkerArray 或双层 GridMap. 可视化订阅者不能增加 Planner 回调工作量

## 代码和验证

- ROS adapter: `src/planner_node.cpp`
- 图与路径工具: `src/planner.cpp`
- 探索状态: `src/planner_exploration.cpp`
- 候选和路线: `src/planner_planning.cpp`
- 岔路记忆: `src/exploration_memory.cpp`
- 配置: `config/planner.yaml`
- 测试: `test/`

在仓库根目录执行:

```bash
./scripts/test_repo.sh
```
