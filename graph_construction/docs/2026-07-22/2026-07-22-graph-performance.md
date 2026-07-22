# 导航图性能优化

## 修改内容

- 向量化 GridMap 连通域、高程尖峰、脚下盲区、reachable component 和 Frontier cell 检测
- 比较相邻 GridMap, 记录变化 cell 和新 free cell
- 新边只由新 free cell、新节点和移动 anchor 触发
- 历史边只在走廊穿过变化 cell 时复查
- 复用稳定节点和边的 ROS 消息对象
- 日志增加变化 cell、重建节点和受影响边数量

## 测试

- `graph_construction` 全量测试通过, 93 passed
- `colcon build --packages-select graph_construction --symlink-install` 通过
- 完整 WildOS、DLIO、高程图、导航图和 Planner 链路完成三轮 Unity 运行
- 运行期间节点和边持续增长, Planner 持续发布非空路线

## 性能结果

相近图规模对比:

| 版本 | 图规模 | 总耗时 | 图更新 | graph 消息年龄 P95 |
|---|---:|---:|---:|---:|
| 优化前 | 153 nodes / 356 edges | 458 ms | 318 ms | 约 800 ms |
| 优化后 | 145 nodes / 359 edges | 166 ms | 143 ms | 785 ms |

低变化帧只重建 7/73 个节点和 35/194 条边

最终总耗时 P95 为 252 ms, 比 250 ms 目标高约 2 ms

## 未完成项

最终实现尚未完成 243 nodes 以上长距离复测

Unity 当前运动速度达到约 5.2 至 5.9 m/s, DLIO 在运行中频繁暂停和恢复, 前一轮还出现约 33 m 位姿跳变, 跳变后的数据不用于性能结论

下一步先限制 Unity 运动速度并稳定 DLIO, 再进行长距离复测

如果稳定输入下 P95 仍超过目标, 优先增加 Frontier 碰撞结果缓存和历史 Frontier 局部失效, 暂不直接改写 C++
