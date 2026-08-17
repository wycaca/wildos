# 探索走廊循环和回头路径修复

日期: 2026-07-16

## 1. 现场问题

Unity 长时间目标搜索仍处于 `SEARCHING_WITH_INITIAL_GOAL`，没有稳定目标估计时，机器人在走廊角落反复移动

实时日志显示 Planner 在多个局部 Frontier 之间循环:

```text
恢复历史候选分支, 中继点=(-3.50,-9.50)
延续当前走廊, 中继点=(-3.50,-8.30)
路径持续无进展
再次恢复前一个候选
```

实时 `/spot1/graphnav_planner/path` 同时出现:

```text
当前位置=(-3.74,-8.35)
  -> (-3.50,-9.50)
  -> (-3.50,-8.30)
```

截图中的黄色点属于 `/spot1/graph_construction_viz` 的历史 `trajectory`，不是 Planner 的未来路线

## 2. 根因

旧实现存在四个相互叠加的问题:

1. deferred branch 恢复排序使用保存于发现时的方向，可能把已经位于机器人后方的候选继续判断为方向一致
2. Planner 只保存一个 stalled frontier，新失败会覆盖旧记录，不同 UUID 也无法继承同一物理走廊的失败状态
3. 分支延伸把新尾点追加到历史缓存路线，deferred handoff 可形成先到旧尾点再返回附近新尾点的 U 形路线
4. 当前路线中的 edge 单帧消失就立即释放分支，rolling map 的短暂波动会触发频繁切换

## 3. 代码修改

### 3.1 多失败走廊记忆

`Planner` 使用 `failed_branches_` 保存多条失败走廊，每条记录包含:

- Frontier UUID
- 失败位置
- 走廊轴线方向
- 失败次数
- 允许重试时间

同一 UUID，或 `frontier_failure_merge_radius` 范围内且轴线一致的候选共享失败记录。轴线按无向方向判断，因此同一走廊中相反行进方向不会绕过冷却

首次冷却为 `frontier_failure_cooldown`，重复失败按次数增加，目标搜索状态切换时统一清空

### 3.2 历史候选使用实时指标

deferred branch 恢复不再用旧发现方向覆盖实时方向一致度

排序优先级为:

1. 当前仍满足前进量和回退量约束的候选
2. 当前路线代价与实际回退量更小的候选
3. 普通目标接近模式继续按发现顺序恢复

前向候选最小进展从 `0.0m` 调整为 `0.5m`，避免机器人附近几乎没有进展的小 Frontier 被反复当作前向延伸

### 3.3 当前路线替换历史追加

活动分支仍保留分支身份、失败计时和方向语义，但执行路径改为当前 graph 节点到新 Frontier 的最新 Dijkstra 简单路径

分支延伸和 deferred handoff 不再向旧路径尾部追加点，并拒绝包含重复 UUID 或元数据长度不一致的候选路线

初始方向探索的进度改为相对初始 odom 方向的单调投影，因此路线重建不会重置无进展判断

### 3.4 路径失效去抖

`committed_path_invalid` 仍检查当前剩余路径的 edge，但只有持续超过 `path_invalid_confirm_duration` 才释放活动分支

单帧 edge 恢复时立即清除失效计时

## 4. 参数

```yaml
directional_min_forward_progress: 0.5
frontier_failure_cooldown: 60.0
frontier_failure_merge_radius: 2.5
path_invalid_confirm_duration: 1.5
```

## 5. 回归测试

`graphnav_planner/test/test_deferred_branch.cpp` 新增或调整:

- `ReleasesBranchWhenCommittedEdgeDisappears`，单次 edge 消失保持原分支，持续失效才恢复
- `FailedCorridorsSuppressNearbyUuidAliases`，A/B 分支失败后附近新 UUID 不得重新进入循环
- `DeferredHandoffRebuildsPathWithoutReturningToOldTail`，handoff 后路线直接到新 Frontier，不得经过旧尾点再返回

功能测试结果:

```text
14 tests passed
```

主工作区编译和定向测试结果:

```text
colcon build --packages-select graphnav_planner: passed
test_deferred_branch: 14/14 passed
```

完整包级 lint 仍会报告仓库原有的 `uncrustify` 基线差异，`xmllint` 还会因当前环境无法访问 ROS schema 而失败。这两项不影响本次功能测试结论

## 6. 部署注意

运行中的进程来自 `/mnt/hhd/han/wildos_ws/install`，不能使用仓库目录内另一套 `build/install` 作为最终部署结果

本次已在 `/mnt/hhd/han/wildos_ws` 主工作区重新编译 `graphnav_planner`。`planner_node` 是未变化的启动壳，实际修改已写入重新生成的 `libgraphnav_planner_component.so`

当前未停止正在运行的旧进程。需要先退出旧进程，再重新执行完整启动命令，运行时才会加载新组件库

本次代码修复不能代替 Unity 长时间闭环验证，重启后仍需确认日志不再在两个相邻 Frontier UUID 之间周期循环，并检查每次新 Path 不含返回当前位置附近的尾段
