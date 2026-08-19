# 图导航 Planner

`graphnav_planner` 消费已评分的导航图并发布可执行 `Path`, 不控制机器人底盘

## 职责

- 在当前持久图上运行 Dijkstra
- 让正在执行的探索分支在持续有进展时保持稳定
- 确认死路后才回退到未探索分支
- 为粗目标和稳定目标生成安全观察位姿
- 只在路线, 停止状态或观察位姿变化时发布 Path

`ObjectSearchGoalMux` 是高层搜索状态和任务完成的唯一 owner. Planner 只为当前状态选择并维护路径

## 代码结构

| 文件 | 职责 |
| --- | --- |
| `src/planner.cpp` | 图更新, 路径工具和已走边记忆 |
| `src/planner_exploration.cpp` | 活动分支生命周期, 进展检查, 冷却和恢复状态 |
| `src/planner_planning.cpp` | 候选选择, Dijkstra, 方向约束和路径输出 |
| `src/exploration_memory.cpp` | 岔路栈和未探索方向记忆 |
| `src/planner_node.cpp` | ROS, TF, Goal Mux 集成和 Path 发布 |
| `src/planner_visualization.cpp` | Planner Marker |

## 关键边界

- Planner 不会把虚拟探索目标加入图, 也不会穿过 `unknown`
- 普通延伸不能无限制回退, 只有显式 `BACKTRACK` 可以明显回头
- 图或 odom 缺失时暂停规划, 但保留恢复状态
- 目标抢占会保留探索记忆, 过期目标证据失效后可以恢复原分支

## 外部导航安全契约

- `Path` 按路线变化发布, 不是心跳, 不应只用消息间隔判断 Planner 是否健康
- graph 过期且 odom 新鲜时, Planner 最多发布一次当前位置单点 hold path
- odom 过期时 Planner 停止发布, 外部导航必须独立监测 odom, 超过 1.0 秒立即停车并丢弃当前路线
- 外部导航接收新 `Path` 时应校验时间戳, 超过 2.0 秒或来自未来超过 0.1 秒时拒绝执行
- 恢复执行前必须同时取得新鲜 odom 和新鲜 Path, 不得继续执行故障前缓存路线

状态转换, 参数, 恢复行为和测试映射见 [目标搜索](../object_search/target_search.md)
