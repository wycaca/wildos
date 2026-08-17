# Frontier Revisit Memory Plan

> 状态: 已于 2026-07-14 被 Frontier 生命周期分层和 Deferred Branch 方案替代, 本文仅保留历史设计背景

当前实现不再保存 removed Frontier 坐标或长期视觉评分, 重复探索由持久节点 `explored_radius` 抑制, 未选择分支由 planner 保存稳定拓扑入口

## 目标

在目标尚未找到的探索阶段保存已经消失的 Frontier 位置, 新 Frontier 若重新出现在历史位置附近, 将其全部方向分数置零, 避免机器人反复选择已经探索过的区域

本阶段只处理 Frontier 重访记忆, 不实现分支队列, 不改变持久稀疏图, 不接入 Nav2, DLIO 或多视角粒子目标融合

## 重构前代码审计

当前代码已经包含社区实现中的基础逻辑, 不是从零开始接入:

- `WildOS_Nav` 使用 `frontier_uuid_to_scores` 保存当前 Frontier 的历史评分
- `remove_old_frontiers` 对比历史 UUID 和当前 `NavigationGraph` Frontier UUID
- 历史 Frontier 不再出现在 graph 中时, 其节点位置写入 `removed_frontier_positions`
- 当前相机可评分 Frontier 和未进入相机视野的默认评分 Frontier 都会与历史位置比较
- 三维距离小于 `min_frontier_separation` 时, 对应 `frontier_scores` 全部置零
- planner 已将零分限制为 `1e-3` 后再计算对数代价, 不会产生 `log(0)` 或非有限代价

现有实现位置:

```text
visual_navigation/visual_navigation/wildos/nav.py
  WildOS_Nav.__init__
    frontier_uuid_to_scores
    frontier_uuid_to_scoring_distance
    removed_frontier_positions

  remove_old_frontiers
    检测已从 NavigationGraph 消失的 Frontier UUID
    保存历史位置

  update_navgraph_with_scores
    对视觉评分和默认评分执行重复区域抑制

visual_navigation/configs/wildos_nav_sim_conf.yaml
  min_frontier_separation: 0.75

graphnav_planner/src/planner.cpp
  将 Frontier 分数限制到 [0, 1]
  使用 max(score, 1e-3) 计算对数代价
```

## 重构前实现问题

现有代码能够工作, 但不适合作为长期路线记忆直接保留:

1. Frontier 生命周期判断、历史位置存储和距离查询全部内嵌在 `nav.py` 的评分大函数中, 职责不清晰
2. 视觉评分 Frontier 和默认评分 Frontier 各自重复一遍距离计算和置零逻辑
3. Frontier 消失时只删除 `frontier_uuid_to_scores`, 没有同步删除 `frontier_uuid_to_scoring_distance`, 会持续积累失效 UUID
4. `removed_frontier_positions` 每次通过 `np.vstack` 扩展, 长距离任务中会持续复制数组
5. 每个新 Frontier 都线性扫描全部历史位置, 总开销随探索距离增长
6. 当前使用 XYZ 三维距离, 同一平面位置可能因 elevation 高度抖动而绕过抑制
7. 没有独立单元测试证明 Frontier 消失、重新生成和距离边界行为

## 实现方案

### 1. 提取纯逻辑 Frontier 记忆

新增 `RemovedFrontierMemory`, 放在视觉导航包内, 不依赖 ROS message 和 `rclpy`

职责只包括:

- 保存已确认消失的 Frontier 世界坐标 XY
- 使用抑制半径对历史位置做空间分桶
- 判断候选位置是否落入任何历史抑制区域
- 对相邻历史位置去重, 避免同一区域反复增加记录
- 暴露历史区域数量供测试和低频诊断使用

建议接口:

```python
class RemovedFrontierMemory:
    def __init__(self, suppression_radius: float): ...
    def remember(self, position_xy: tuple[float, float]) -> bool: ...
    def contains(self, position_xy: tuple[float, float]) -> bool: ...
    def __len__(self) -> int: ...
```

空间分桶尺寸直接复用 `suppression_radius`, 查询当前桶及周围八个桶, 不新增容量、过期时间或距离模式参数

### 2. 明确 Frontier 生命周期

只有满足以下条件才记为已消失 Frontier:

- 该 UUID 之前存在于 `frontier_uuid_to_scores`
- 最新 `NavigationGraph` 中该 UUID 已经不存在或不再标记为 Frontier

以下情况不能记为已消失:

- Frontier 仍在 graph 中, 但当前不在任何相机视野内
- Frontier 仍在 rolling GridMap 外被持久图保存
- 视觉同步暂时没有产生该 Frontier 的图像评分

该规则保证 Frontier 记忆建立在 Graph Construction 已确认的 Frontier 生命周期之上, 不用相机可见性替代几何探索状态

Frontier 消失后同步清理:

```text
frontier_uuid_to_scores
frontier_uuid_to_scoring_distance
```

### 3. 统一评分抑制入口

在 `nav.py` 增加一个小型辅助函数, 所有 Frontier 分数在写入 graph property 前统一调用:

```python
def suppress_revisited_frontier_scores(self, position, scores): ...
```

该函数负责:

- 查询 `RemovedFrontierMemory`
- 命中历史区域时返回同形状的零分数组
- 未命中时保持原评分
- 最后继续使用现有 `normalize_frontier_scores`

视觉评分 Frontier 和默认评分 Frontier 共用同一个入口, 不再复制距离判断

### 4. 使用二维世界距离

Frontier 重访判断使用 graph frame 下的 XY 距离, 不使用 Z

原因:

- 当前机器人主要沿地表运动
- elevation map 的局部高度可能随点云更新产生小幅变化
- Frontier 重访语义对应平面探索区域, 不是三维空间中的独立目标

### 5. 保持任务级记忆

Frontier 记忆生命周期与 `wildos` 节点进程一致:

- 单次目标搜索任务中持续保留完整历史
- 不增加自动过期时间
- 节点重启后清空
- 当前查询文本由启动配置固定, 暂时不增加动态任务重置接口

后续若支持运行时切换目标查询, 必须在任务切换时显式清空 Frontier 记忆

### 6. 与目标检测隔离

Frontier 重访记忆只抑制探索评分, 不能阻止已确认目标进入目标导航链路

当前代码先从目标增强后的原始 `nav_data` 选择 `object_target_candidate`, 再把 Frontier 分数写入 `scored_nav_graph`, 因此历史 Frontier 零分不会覆盖已经确认的直接目标 pose

## 实际修改文件

本次代码阶段只修改以下文件:

### 已新增

`visual_navigation/visual_navigation/wildos/frontier_memory.py`

- 实现纯 Python `RemovedFrontierMemory`
- 使用 XY 空间分桶和邻域查询
- 不导入 ROS package

`visual_navigation/test/test_frontier_memory.py`

- 覆盖历史位置保存和邻近位置去重
- 覆盖半径内抑制和半径外保留
- 覆盖相同 XY 不同 Z 的调用方行为
- 覆盖长序列查询不会退化为全量数组复制

### 已修改

`visual_navigation/visual_navigation/wildos/nav.py`

- 用 `RemovedFrontierMemory` 替换 `removed_frontier_positions`
- Frontier 消失时同步清理评分距离缓存
- 抽取统一分数抑制函数
- 视觉评分和默认评分共用抑制逻辑
- 保留现有 `frontier_uuid_to_scores` 和按接近距离更新评分的语义

`visual_navigation/configs/wildos_nav_sim_conf.yaml`

- 保留现有 `min_frontier_separation: 0.75`
- 只补充其作为历史 Frontier 抑制半径的说明
- 不新增参数

### 未修改

`graph_construction/graph_construction/frontier_detector.py`

- 继续负责几何 Frontier 生命周期和持久 owner
- 不保存语义上的已探索区域评分

`graphnav_planner/src/planner.cpp`

- 已安全处理零分 Frontier
- 继续消费 `frontier_scores`, 不感知记忆实现细节

`graph_construction/configs/topic_profiles.yaml`

- Frontier 记忆不是平台 topic 或 frame 差异
- 不把抑制参数复制到 profile

launch 文件

- 不增加节点和 topic
- 不改变当前 Unity 自研导航链路

## 实施结果

- `RemovedFrontierMemory` 已从 `nav.py` 提取为独立纯逻辑模块
- 历史位置按 XY 空间分桶, 查询只访问当前桶和周围八个桶
- 相邻 removed Frontier 会去重, 同一区域不会重复增加记忆
- Frontier 消失时同步清理评分缓存和最近评分距离缓存
- 视觉评分与默认评分共用统一抑制函数
- 历史区域抑制优先于按接近距离更新评分, 已缓存高分不能绕过零分覆盖
- 保留现有 `min_frontier_separation`, 没有新增运行参数
- planner、Graph Construction、launch 和 topic profile 均未修改
- 新增 6 项单元测试, 覆盖半径边界、去重、不同 elevation、负坐标和长路线记忆

## 验证标准

1. Frontier UUID 从 graph 消失后, 世界 XY 被保存一次
2. 新 UUID Frontier 出现在抑制半径内时, 所有方向分数为零
3. 新 Frontier 位于抑制半径外时, 视觉评分或默认评分保持不变
4. 相同 XY 但 elevation Z 不同的 Frontier 仍被判定为同一历史区域
5. Frontier 仍在 graph 但当前相机不可见时, 不加入 removed memory
6. Frontier 消失后评分缓存和距离缓存都不保留旧 UUID
7. planner 消费零分后生成有限代价, 不崩溃且优先选择其他有效 Frontier
8. 目标确认后的直接目标 pose 不受 Frontier 探索抑制影响

## 后续关系

Frontier 重访记忆解决的是不重复选择已探索区域, 不能替代分支记忆

下一阶段分支记忆需要独立保存:

- 当前活动分支
- 尚未探索的候选分支
- 分支进度和失败原因
- 确认死路的条件
- 死路后恢复哪个历史分支

只有分支记忆完成后, 才能严格实现当前方向成为死路后再切换到保存分支
