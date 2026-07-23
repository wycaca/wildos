# 边更新性能优化 TODO

日期: 2026-07-23

状态: 边处理性能尚未完成, 当前方案改为论文式 free radius 稀疏节点和局部半径图

- 低变化区间总耗时能够回落到平均 275 ms, 说明变化触发后的边更新放大是主要增量瓶颈
- 性能汇总中的阶段耗时是时间窗口平均值, `最近工作量` 是最后一帧统计, 两者当前没有逐帧对齐, 不能用单条汇总精确解释某一帧耗时

## 性能问题复盘: 为什么最近优化仍未解决

### 日志结论

最近完整运行日志与修改前文档中的约 500 nodes 阶段相比, 没有证明边更新性能得到改善:

| 指标 | 修改前记录 | 2026-07-23 最近运行 |
|---|---:|---:|
| 图规模 | 约 500 nodes | 478 nodes、1754 edges |
| 总耗时平均值 | 约 376 至 425 ms | 高峰 620 ms |
| 总耗时 P95 | 未完整记录 | 高峰 951 ms |
| 边更新平均值 | 约 245 至 264 ms | 高峰 455 ms |
| graph 消息年龄 P95 | 未完整记录 | 高峰约 1.31 s |

两次运行的地图变化强度并不完全一致, 因此不能只根据高峰值判断单个提交发生了性能回退, 但最近结果已经足以否定“性能问题已解决”

日志中最关键的反例是:

- 仅 9 个新增 free cell, 仍然选中了 154 个新 free 附近旧节点
- 最终重建 194 个节点的边, 占当前 250 个局部节点的约 78%
- 生成并检查 2905 条候选边, 复查 461 条历史边
- 边更新平均耗时达到 455 ms
- `unknown` 阻挡候选长期增长到约 6700 条
- 大幅地图变化时, 270 个新增 free cell 会触发 193 个附近旧节点和 1149 条 unknown 候选重试

这说明当前耗时仍然主要由“少量栅格变化被扩大成大范围节点重建”决定, 尚未达到修改文档中“成本主要取决于真实局部变化”的设计目标

### 最近提交解决了什么, 又留下了什么

| 提交 | 已解决的问题 | 仍然存在的问题 |
|---|---|---|
| `3c73df3` | 引入持久图、局部更新和性能统计 | 更新入口仍以节点重建为中心 |
| `5eedd82` | 优化长期增量更新和消息复用 | 没有建立边候选的完整生命周期 |
| `99a8075` | 新障碍改为通过空间索引定位历史边, 稳定帧减少空转 | 新 free 仍需要扩大到附近节点 |
| `ceb5006` | 增加新 free 局部重连和初始化盲区隔离 | 新 free 影响半径约 4.78 m, 离散变化容易覆盖大部分局部节点 |
| `2a7a711` | 记录 unknown 阻挡候选, 增加低连接节点重试 | 精确候选命中后又转成两个端点的整邻域重建 |

最近修改持续给同一套节点重建流程增加事件、索引和重试状态, 修复了若干正确性问题, 但没有改变边更新的所有权和失效粒度

### 当前边更新逻辑的结构性缺陷

1. 栅格变化和实际执行粒度不一致

   `_node_ids_near_newly_free()` 使用约 `0.5 × edge_radius + clearance + cell_diagonal`, 当前约为 4.78 m 的影响半径查找旧节点

   少量分散的新增 free cell 就可能覆盖 30 m 局部窗口中的大部分节点, 每个命中节点随后又查询最多 24 个近邻候选

   实际传播链为:

   `changed cell -> 半径内节点 -> 每个节点的近邻候选 -> 节点全部关联历史边`

   这不是边级增量更新, 只是通过空间索引缩小过的节点级批量重建

2. unknown 重试丢失了精确索引带来的收益

   `_blocked_unknown_retry_nodes()` 已经能够找到真正经过新增 free 附近的候选 pair, 但随后把 pair 转成两个端点 ID

   `build_edges()` 不会只重试命中的 pair, 而是重新生成两个端点各自的近邻候选

   精确传播链被放大为:

   `changed cell -> exact blocked pair -> 两个端点 -> 两端点全部近邻候选`

   1149 条命中候选还会共享大量端点, 当前统计只记录去重后的节点集合, 没有记录每种触发原因各自造成的候选检查量

3. 候选选择和历史边保留存在语义冲突

   `build_edges()` 对普通节点只选择最近 4 条有效边, 但 `merge_historical_edges()` 会把未被当前障碍证伪的受影响历史边重新加入

   因此 `max_edge_neighbors=4` 只限制本次新选择, 不是持久图的不变量

   当近邻排名变化时, 旧的安全边不会因为已经不在前 4 名而退出, 新边又可以继续加入

   这有三个后果:

   - 节点实际度数和边数量可能随历史选择累积
   - 重建节点时需要复查越来越多的关联历史边
   - 普通近邻边、已走通历史边和关键桥接边没有明确的角色和不同生命周期

   当前 478 nodes、1754 edges 的平均度数约为 7.34, 仅凭这个数值还不能证明边已经无界增长, 但现有代码确实没有维护明确的持久度数或边角色不变量

4. unknown 阻挡候选缺少淘汰策略

   `_sync_unknown_blocked_candidates()` 会保存本次检查中被 unknown 阻挡的 pair

   候选只有再次被检查或端点删除时才会移除, 没有以下生命周期约束:

   - 端点已经离开当前 rolling window
   - pair 已经不在任一端点的候选预算内
   - 已有更短或更稳定的替代边
   - 长期没有对应区域的观测变化
   - 候选只属于已断开的旧近邻排名

   因此候选数量可以随探索历史增长, 最近已经达到约 6700 条, 每次地图开放都可能唤醒大量过期或低价值候选

5. 局部替换仍产生大量重复工作

   当前流程先收集重建节点的全部关联边, 再执行:

   - 对所有重建节点重新查询和排序近邻
   - 对所有候选重新做走廊和净空检查
   - 对所有受影响历史边再次做局部冲突检查
   - 从邻接索引和边空间索引删除旧边
   - 创建新 `InternalEdge` 并重新插入索引

   即使只有一条候选走廊状态变化, 同一节点其他无关边也会经历检查和索引更新

6. 现有基准覆盖的是有利路径, 没有复现长期运行负载

   当前固定图基准验证了稳定地图、少量新障碍和 free 变 unknown

   其中稳定地图和 free 变 unknown 本来就可以直接跳过大部分旧边工作, 没有覆盖最近日志中的主要负载:

   - 分散的 unknown 变 free
   - rolling map 持续移动
   - 新节点和新 free 同时出现
   - unknown 阻挡候选长期累积后集中开放
   - 低连接重试与普通重建叠加
   - 近邻排名变化后的旧边淘汰

   因此 1.1 至 7.3 ms 的微基准结果不能代表 10 分钟运行后的边更新性能

7. 当前性能日志不能建立单帧因果关系

   阶段耗时使用一段时间内的平均值, `最近工作量` 只保存最后一帧

   例如最后一条日志显示最后一帧没有重建节点和候选, 但边阶段窗口平均仍为 148 ms

   这不代表空操作本身耗时 148 ms, 而是统计窗口中仍包含之前的重负载帧

   在改成逐帧耗时与逐帧工作量配对前, 只能判断总体相关性, 不能可靠定位单个子步骤占比

## 设计结论: 回到论文的单一拓扑规则

现有方案不再继续增加多种边状态和重试队列

新的实现以 WildOS 论文 Algorithm 3 和 Algorithm 5 为准:

- 使用 free radius 控制节点密度
- 连接半径内全部安全节点
- 每帧重新计算当前局部地图中的全部 pair
- 历史图只保留当前地图无法判断的部分
- 不再区分 local、historical 和 bridge 等边角色

参考来源:

- [WildOS 论文](https://arxiv.org/html/2602.19308)
- [WildOS 官方 graphnav_mapper](https://github.com/nasa-jpl/nebula2-wildos/tree/main/graphnav_mapper)
- [社区 Python graphnav_builder](https://github.com/TIKTOKDAD/nebula2-wildos-main_ws/tree/main/graphnav_builder)

论文中的 sparse graph 主要通过稀疏节点实现, 不是在密集节点上只保留最近 4 条边

### 只保留一个安全扩展

论文使用 `E_t = E_{t-1} ∪ E_new`, 没有详细说明新障碍出现后如何删除旧边

当前项目继续保留一条部署安全规则:

- 当前地图明确看到 obstacle 或净空不足时, 删除冲突的历史边
- 当前地图变成 unknown 时, 保留以前确认过的边
- 新边必须完全位于当前已知 free 区域

除此之外不再增加边类型、桥接规则和低连接重试

## 新的图更新流程

```text
分类当前 GridMap
  -> 计算 obstacle 和 unknown 距离场
  -> 更新当前窗口内历史节点的 free radius 和 explored radius
  -> 在 reachable free 中随机采样稀疏新节点
  -> 更新 Frontier
  -> 用 cKDTree 生成当前窗口内全部半径 pair
  -> 对全部局部 pair 执行统一安全检查
  -> 只提交实际增加和删除的 edge delta
  -> 更新 current node 并发布完整图
```

### 1. 使用 free radius 保持节点稀疏

每个节点保存:

```text
free_radius = min(
  distance_to_obstacle,
  distance_to_unknown,
  max_free_radius
)
```

节点采样按论文执行:

1. 从机器人 reachable free 区域随机选择 cell
2. cell 到 obstacle 和 unknown 的距离必须大于机器人安全半径
3. 查询候选附近已有节点
4. 如果候选落在任一已有节点的 free radius 内, 拒绝候选
5. 接受候选后记录它自己的 free radius
6. 重复固定次数, 论文参数为每帧 1000 次

实现时还要把本帧刚接受的节点加入覆盖检查, 防止同一帧产生重叠节点

固定 `random_seed`, 保证同一组输入可以复现, 进程内随机状态持续推进

保留和新增的主要参数:

| 参数 | 建议值 | 作用 |
|---|---:|---|
| `node_sample_count` | 1000 | 每帧随机采样次数, 对齐论文 |
| `max_free_radius` | 4.0 m | 限制开阔区域的最大节点间距 |
| `min_obstacle_clearance` | 0.5 m | 节点和边的基本安全距离 |
| `edge_radius` | 8.0 m | 局部节点的最大连边距离 |
| `random_seed` | 固定值 | 保证测试和离线回放可复现 |

删除现有固定网格规则:

- 删除 `sample_stride`
- 删除 `min_node_separation`
- 删除 `_adaptive_lattice_multiple()`
- 删除 `_world_lattice_bounds()`
- 不再要求节点落在固定世界网格交点

预期效果:

- 开阔区域 free radius 大, 节点自然稀疏
- 障碍和 unknown 附近 free radius 小, 节点自然变密
- 采样点不再受固定网格交点限制, 可以填补视觉上为空地的覆盖洞
- 节点减少后, 半径内全部连边仍能保持可接受的 pair 数量

### 2. 半径内安全节点全部连边

对当前 GridMap 内的节点构建一次 `cKDTree`

使用 `query_pairs(edge_radius)` 一次返回全部无向 pair

每个 pair 只检查一次, 不排序, 不截断:

- 删除最近 24 个候选限制
- 删除普通节点最近 4 条边限制
- 删除 current node 最近 12 条边限制
- 不再单独生成 bridge edge

统一安全规则:

| pair 当前状态 | 当前地图结果 | 处理 |
|---|---|---|
| 不存在 | 整条走廊为已知 free 且净空足够 | 添加边 |
| 不存在 | 经过 unknown 或 obstacle | 不添加 |
| 已存在 | 当前可见部分出现 obstacle 或净空不足 | 删除边 |
| 已存在 | 当前可见部分变成 unknown | 保留历史边 |
| 已存在 | 两端都在当前地图内, 但距离超过 `edge_radius` | 删除边 |
| 已存在 | 两端不都在当前地图内 | 保持不变 |

新边可以增加少量 creation hysteresis, 例如在 0.5 m 安全距离上额外要求 0.15 m

旧边仍按正常安全距离保留, 避免安全边在阈值附近反复增删

### 3. 全部局部 pair 每帧重算

当前局部节点经过 free radius 稀疏化后, 每帧直接重算全部局部 pair

这样 unknown 变 free 后, 下一帧会自然重新检查相关 pair 并补边

删除以下状态和补偿逻辑:

- `_node_ids_near_newly_free()`
- `_unknown_blocked_pairs`
- `_unknown_blocked_index`
- `_blocked_unknown_retry_nodes()`
- `_sync_unknown_blocked_candidates()`
- low-degree retry queue
- `merge_historical_edges()`
- 按节点收集 `edge_rebuild_node_ids`
- local、historical、bridge、blocked 和 retired 多状态设计

边只有存在和不存在两种拓扑状态

是否已经走过可以继续作为边的普通属性保存, 不能改变基本连边规则

### 4. 使用 edge delta 更新持久图

每帧得到局部 pair 检查结果后, 计算:

```text
edges_to_add
edges_to_remove
edges_to_keep
```

只对 add 和 remove 更新邻接表、消息缓存和边空间索引

不能清空再重建整个边集合

当前地图外的节点和边保持不变, 继续提供长距离返回路线

## Python 性能优化

### 已经使用编译实现的部分

以下部分已经由 SciPy 或 NumPy 的编译代码执行, 暂时不更换:

| 功能 | 当前实现 | 结论 |
|---|---|---|
| obstacle 和 unknown 距离场 | `scipy.ndimage.distance_transform_edt` | 保留 |
| reachable free 连通分量 | `scipy.ndimage.label` | 保留 |
| 栅格数组筛选 | NumPy 向量运算 | 保留 |

距离场在最近日志中平均约 4 至 5 ms, 不是当前主要瓶颈

### 立即替换邻居查询

当前 `NodeSpatialIndex.query_radius()` 使用 Python 循环

新的局部 pair 枚举改为:

```python
positions = np.asarray(local_node_xy, dtype=np.float64)
pairs = cKDTree(positions).query_pairs(
    edge_radius,
    output_type="ndarray",
)
```

本机简单基准使用 250 个随机节点、5509 个半径 pair:

| 实现 | 单次耗时 |
|---|---:|
| 当前 Python 空间桶逐节点查询 | 约 32.4 ms |
| `cKDTree` 建树并执行 `query_pairs` | 约 0.32 ms |

这个基准只测 pair 生成, 不包含走廊安全检查, 但已经说明该替换值得直接采用

节点采样的覆盖查询可以继续使用持久空间桶作为第一版

如果采样阶段变成热点, 再对批量候选使用 `cKDTree.query_ball_point()`

### 走廊安全检查的处理

当前 Bresenham 是 Python generator, 每条边还会创建 list 和 NumPy 数组

这是完成 `cKDTree` 替换后最可能的热点

第一版先保持现有安全语义, 同时做以下简单优化:

- pair 的世界坐标和 grid 坐标批量转换
- 先检查端点和长度, 再生成线段 cell
- 一个 pair 只生成一次线段 cell
- obstacle、unknown 和两个 SDF 使用同一组索引
- 历史边检查复用同一个结果, 不再第二次栅格化

暂不使用 Shapely 或逐边 OpenCV 画线替换 Bresenham

它们需要为每条边创建几何对象或临时图像, 还容易改变栅格净空语义

如果实测走廊检查仍超过边阶段预算, 再实现一个小型 pybind11 扩展:

- 输入局部 pair 数组、节点 grid 坐标、obstacle、unknown 和两个 SDF
- 在 C++ 中批量执行 Bresenham 或论文官方实现中的 SDF 递归线段检查
- 返回每个 pair 的 `free`、`unknown` 或 `obstacle` 结果
- Python 只负责图状态更新

是否增加 C++ 扩展必须由 profiler 决定, 不在第一阶段提前增加构建复杂度

### 不建议的替换

- 不用 NetworkX 构建或更新热路径图
- 不用 Shapely 判断每条栅格边的碰撞
- 不为 4 至 5 ms 的距离场切换 OpenCV 实现
- 不引入尚未安装的 Numba 作为第一选择
- 不在性能数据不足时把整个 graph construction 重写成 C++

## 实施步骤

### 阶段 0: 建立论文基线

- [ ] 为当前实现记录同一地图下的局部节点数、局部 pair 数、有效边数和各阶段耗时
- [ ] 增加逐帧耗时与逐帧工作量配对日志
- [ ] 保存 2026-07-23 问题场景的 classified grid, 用于新旧实现离线对比
- [ ] 增加论文 Algorithm 3 和 Algorithm 5 的最小单元测试
- [ ] 修正 `details/graph_update.md` 中已经过时的边更新说明

### 阶段 1: 替换节点采样

- [ ] 使用 reachable free 随机采样替换固定世界网格
- [ ] 增加 `node_sample_count` 和 `random_seed`, 复用安全距离与最大 free radius 参数
- [ ] 按已有节点 free radius 拒绝重复候选
- [ ] 本帧新节点之间也执行覆盖检查
- [ ] 保持节点 UUID、free radius、explored radius 和窗口外持久记忆
- [ ] 删除固定网格函数及 `sample_stride`、`min_node_separation` 等无用参数
- [ ] Graph Construction 重启后从空图测试, 不混用旧固定网格生成的节点
- [ ] 对比开阔地、窄路、障碍边缘和截图场景的节点数量与覆盖洞

### 阶段 2: 替换边构建

- [ ] 使用 `cKDTree.query_pairs()` 生成全部局部半径 pair
- [ ] 对每个 pair 应用统一安全规则
- [ ] 删除两端都在当前地图内但已经超过 `edge_radius` 的旧边
- [ ] 删除最近邻截断、邻居数量上限和 current node 特殊边数
- [ ] 删除 unknown blocked candidate 和 low-degree retry
- [ ] 删除历史边回灌, 改为单次 pair 检查直接生成 edge delta
- [ ] 保留新障碍删除旧边、unknown 不误删历史边的安全语义
- [ ] 只更新实际变化的边对象和索引

### 阶段 3: 优化走廊检查

- [ ] 单独记录 pair 生成、grid 坐标转换、Bresenham、SDF 查询和 edge delta 耗时
- [ ] 合并同一 pair 的新边和历史边重复检查
- [ ] 对比 Python Bresenham 和批量实现
- [ ] 只有走廊检查仍超过预算时才增加 pybind11 C++ 扩展
- [ ] C++ 扩展必须与 Python 参考实现逐 pair 输出一致

### 阶段 4: 回归和长任务测试

- [ ] 开阔区域验证 free radius 大时节点明显变少
- [ ] 障碍和 unknown 附近验证节点会自然变密
- [ ] 验证所有半径内 collision-free pair 都存在边
- [ ] 验证 unknown 变 free 后无需 retry cache 就能补边
- [ ] 验证新 obstacle 会删除冲突历史边
- [ ] 验证 rolling map 移动后窗口外节点和边不会消失
- [ ] 验证同一 reachable free 区域不再因最近 4 邻居限制而断开
- [ ] 连续运行至少 1000 帧, 检查节点、边、内存和耗时趋势
- [ ] 使用 Unity 连续运行至少 10 分钟, 检查路径、回头和消息年龄

## 验收条件

### 节点

- 新节点只生成在 reachable free 且净空足够的位置
- 候选落入已有节点 free radius 时不会重复生成节点
- 开阔地节点密度明显低于当前固定网格实现
- 障碍附近仍有足够节点表达狭窄通路
- 截图中的可达空地不再因固定网格交点失效而形成覆盖洞

### 边

- 当前窗口内所有距离不超过 8.0 m 且安全的 pair 都有边
- 不再存在最近 4 条边、最近 24 个候选和 bridge 特殊规则
- unknown 变 free 后下一次局部 pair 重算可以直接补边
- 新 obstacle 可以删除当前可见的冲突边
- unknown 和窗口外区域不会误删历史安全路线

### 性能

- 局部 pair 生成平均不高于 5 ms
- 500 total nodes 场景平均边更新不高于 150 ms
- 总耗时 P95 不高于 500 ms
- graph 消息年龄 P95 不高于 800 ms
- 导航图输出不低于 1.8 Hz
- 长任务中性能主要由当前局部节点和 pair 数决定, 不再受历史 blocked candidate 数量影响

实现提交: 待提交
