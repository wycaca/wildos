# 启动空洞、图孤岛与边性能 TODO

日期: 2026-07-27

状态: 第一阶段代码和自动化测试已完成, 等待 Unity 长任务验收

关联文档:

- [2026-07-23-edge-update-performance-todo.md](../2026-07-23/2026-07-23-edge-update-performance-todo.md)
- [2026-07-20-elevation-startup-blind-zone-fix.md](../2026-07-20/2026-07-20-elevation-startup-blind-zone-fix.md)
- [graph_update.md](../../../graph_construction/graph_update.md)

## 1. 结论

这次实测暴露了三个相互关联但层级不同的问题:

1. 高程图源头仍然发布机器人周围的 unknown 空洞
2. 栅格 free 连通不等于 NavigationGraph 已经连通
3. unknown 边界产生大量密集节点, 全半径 pair 每帧重算后形成高负载

上一版 `ad7dd54` 的启动盲区修补没有真正解决现场问题, 主要原因不是搜索半径还不够大, 而是修补放错了层级:

- 修补只修改 `graph_builder` 当前收到的一份 `ClassifiedGrid`
- 它没有修改高程图源头 `/elevation_mapping_node/elevation_map_raw`
- `robot_blind_zone_initial_only=true` 时, 一旦某帧被判定为完成, 后续帧不再修补
- 下一帧重新从原始 GridMap 分类时, 源头的 unknown 空洞会再次出现
- 当前自动化测试还明确验证了“初始化完成后第二帧保持 unknown”, 因此测试通过不代表人工安全区域可以跨帧存在

所以应先把人工启动安全区域移到高程图层, 再让图构建只消费最终地图

## 1.1 2026-07-27 已完成修改

启动区域:

- 高程图源头 `initialize_tf_grid_size` 从 1 m 扩大到 12 m
- 图层人工 free 改为固定世界坐标先验, 后续帧恢复同一区域
- 机器人移动后不会在新位置继续填充 unknown
- 真实 free 和 obstacle 优先于人工先验
- 增加 `initial_prior_restored` 和 `initial_prior_disconnected` 状态

图连通:

- `_reachable_free_mask()` 只返回机器人所在 free 分量
- 隔着 unknown 的外围 free 不再提前生成节点
- 边更新后从 current node 执行 BFS
- 增加图分量数、current 分量节点数和局部连通状态

节点和边:

- unknown 变 free 后只压缩变化区域附近的冗余普通节点
- current node 和 Frontier owner 不参与压缩
- 稳定帧不再生成或检查 pair
- 新节点只检查 incident pair
- 新 obstacle 只复查边空间索引命中的历史边
- unknown 变 free 只生成变化区域附近端点的半径 pair
- 斜边使用双向栅格线 supercover, 修复障碍角点漏检
- 节点只更新 Z 高程时不再重建关联边的 XY 空间索引
- RViz 默认只显示生成森林, `viz_show_full_edges=true` 才显示全部实际边

自动化结果:

- 图构建和持久图核心测试 63 项通过
- ROS 镜像中与本次修改相关的 138 项测试通过
- 另外 8 项 launch 测试因镜像安装空间 topic profile 旧于当前工作区失败, 与本次修改无关
- ROS `graph_construction` 包在临时工作区构建通过
- flake8 忽略项目已有的 E501 和 W503 后通过

250 个手工密集节点、13497 条安全边的合成压力测试:

- 首帧完整构图约 642 ms
- 相同地图稳定帧不检查 pair, 总更新约 17 ms
- 单个新 obstacle 检查 1527 个受影响 pair, 总更新约 70 ms

## 2. 问题一: 启动空洞仍然存在

### 2.1 图片现象

第一张图中, 机器人位于中央矩形蓝色区域, 其四周仍有一圈大面积黑色 unknown, 外围才是 LiDAR 已观测地面

这说明当前人工区域只形成了一个较大的脚下岛, 没有持续填满该岛到外围真实地面之间的启动盲区

### 2.2 当前代码为什么失效

当前有两套彼此独立的启动处理:

1. `elevation_mapping_cupy` 在首帧点云前调用 `initialize_map()`
2. `graph_builder` 收到 GridMap 后调用 `_repair_robot_blind_zone()`

高程图初始化当前使用:

- `initialize_frame_id: ["base_link"]`
- `initialize_tf_grid_size: 12.0`
- `dilation_size_initialize: 5`
- `resolution: 0.2`

连续两次膨胀后, 它会生成约 16 m 宽的有限人工地面, 但仍不能根据首帧点云识别并填满到外围真实地面之间的完整盲区

图构建层的修补还有以下限制:

- 原最大搜索半径固定为 6 m
- 只改本帧分类结果, 不写回高程图
- 完成条件只检查栅格 free 是否接触外围 free
- 完成后通过 `_blind_zone_initialization_complete` 永久跳过后续修补
- RViz 继续显示高程图源头, 因此黑色空洞不会消失

只增大 `robot_blind_zone_elevation_search_radius` 不能单独解决跨帧丢失问题, 仍需保存固定世界坐标启动先验并让真实 obstacle 优先覆盖

### 2.3 2026-07-27 现场问题复查

最新现场图中的断连不是连边碰撞检查造成的, 而是启动盲区修补提前结束

参数尺度如下:

- 高程图源头初始化方形边长为 12 m
- `dilation_size_initialize=5`, 分辨率为 0.2 m
- 初始化函数连续膨胀两次, 人工 free 方形最终约为 16 m 宽
- 图层原搜索半径只有 6 m, 整个搜索圆仍位于人工方形内部

因此脚下人工 free 分量会直接到达搜索边界, 代码误报 `known_ground` 和 `connected=true`, 实际没有看到外围真实点云, 也不会填充中间 unknown 接缝

已修正:

- 将默认和部署配置的搜索半径改为 12 m, 确保越过约 8 m 的人工区域半宽
- 保持 4 m 只作为盲区种子半径, 不扩大跟随机器人移动的人工区域
- 增加 `robot_blind_zone_center_reaches_search_limit` 和 `robot_blind_zone_boundary_free_components` 诊断字段
- 增加 16 m 人工方形, 1 m unknown 接缝和外围真实 free 的同形回归测试
- 测试同时覆盖首帧连接外围节点和第二帧恢复固定世界坐标通道

修正后图构建和持久图核心测试共 63 项通过

### 2.4 解决方案

把启动安全先验移到 `elevation_mapping_cupy`, 在地图源头保存, 并允许真实点云覆盖

建议流程:

```text
创建脚下基础安全区域
  -> 融合首帧有效点云
  -> 找到与机器人脚下区域相连的 unknown 分量
  -> 在受限启动范围内向外填充
  -> 直到接触高度相容的已观测可通行地面
  -> 将填充 cell 标记为低置信度人工先验
  -> 后续真实点云覆盖人工高程、方差和 traversability
```

具体规则:

- 只处理与机器人脚下人工区域四连通的 unknown
- 外围目标必须是已观测 free, 且高程与机器人脚下地面相容
- 已观测 obstacle、明显台阶、坑和帘状噪声不能被覆盖
- 人工 cell 使用较大方差或单独的 prior mask, 真实观测优先级更高
- 初始化完成条件必须在连续多帧中保持, 不能只看一次临时分类
- 启动先验随 rolling map 一起移动时, 只保存已经建立的世界坐标区域, 不能跟随机器人不断填充新 unknown

需要注意:

unknown 本身不能证明物理安全, 因此“填满所有盲区”只能作为明确的启动安全假设使用, 不能扩展为运行过程中的通用 unknown 填充

### 2.5 修改任务

- [ ] 在 `elevation_mapping_cupy` 增加可持久保存的启动 prior mask
- [ ] 调整初始化顺序, 使首帧点云先提供真实地面边界, 再补齐脚下连通盲区
- [ ] 增加不清空已有点云结果的 unknown 填充接口, 不再通过重复 `initialize_map()` 修补
- [ ] 人工 cell 使用高方差, 真实点云能够覆盖其高程和分类
- [ ] 将 `graph_builder` 的盲区填充降为诊断或删除, 避免地图层和图层各维护一套地面真值
- [ ] 发布人工 cell 数量、人工区域到真实 free 的最短距离和初始化状态

### 2.6 验收条件

- 首帧稳定后, 原始 `elevation_map_raw` 中机器人脚下分量已经接触真实观测 free
- 连续输入相同地图时, 人工安全区域不会在第二帧消失
- 后续点云在人工区域发现障碍时, 人工 free 被真实 obstacle 覆盖
- 机器人移动后, 系统不会在新位置继续无条件填充 unknown
- 超出允许范围仍找不到真实地面时, 明确报告 `startup_prior_not_connected`, 不得误报初始化完成

## 3. 问题二: 脚下节点不能连接正常节点

### 3.1 图片现象

第二张图中蓝色栅格在视觉上已经比第一张图完整, 但脚下节点所属分量仍没有稳定接入左右两侧正常图

这说明“地图上存在 free cell”没有转化为“从 current node 可以沿安全 edge 到达外部节点”

### 3.2 当前逻辑缺陷

#### 完成条件检查错了层级

`_repair_robot_blind_zone()` 的 `robot_blind_zone_connected` 只检查栅格连通

它没有在节点采样和连边完成后执行以下检查:

```text
current node
  -> NavigationGraph 邻接表
  -> 外围真实观测节点
```

因此一条过窄的 free 通道也可能被判定为完成, 但该通道可能因为 0.5 m 净空要求无法放置节点或连接边

#### `_reachable_free_mask()` 混入了实际不可达分量

当前函数除了机器人所在 free 分量, 还会加入 8 m 内“没有明确 obstacle 视线阻挡”的其他 free 分量

unknown 不会阻止这些外围分量被采样, 但新边又要求整条走廊均为已知 free

结果是:

- 外围正常区域会生成节点
- 脚下岛也会生成节点
- 两组节点之间经过 unknown 时不能加边
- 一个更新周期内主动制造出多个图分量

#### 图更新缺少连通性后置条件

当前更新顺序在构建 edge delta 后直接发布完整图, 没有验证:

- current node 是否有邻边
- current node 分量是否包含真实观测节点
- current node 分量是否包含可用 Frontier
- 是否存在栅格已连通但节点采样遗漏的窄通道

### 3.3 解决方案

#### 只在真实可达 free 中采样

`_reachable_free_mask()` 默认只返回机器人所在的 free 连通分量

不要把隔着 unknown 的外围 free 分量称为 reachable, 也不要提前在这些分量中生成规划节点

如果需要保留外围地图节点用于历史图显示, 应单独标记为 `observed_unreachable`, 不能进入当前可规划候选

#### 在图构建结束后执行 BFS 连通验收

每次完成节点和边更新后:

1. 从 `current_node_id` 对邻接表执行 BFS
2. 统计 current 分量中的节点数和 Frontier 数
3. 检查是否到达至少一个真实观测 free 上的普通节点
4. 只向 Planner 暴露 current 分量内的 Frontier 和目标候选

如果栅格已连通但图未连通:

- 沿栅格 free 通道提取一条安全中心线
- 在中心线上按 free radius 补充必要节点
- 只为新节点计算半径内安全边
- 再次执行 BFS

如果不存在满足净空要求的 free 通道:

- 保持 `startup_graph_not_connected`
- 不发布不可执行路径
- 等待更多真实观测或地图源头完成安全先验
- 禁止用跨 unknown 的虚假直线边强行连接

#### 区分人工区域和真实区域

节点需要知道脚下 cell 是真实观测还是人工 prior

图初始化的完成条件应同时满足:

- current node 位于安全 free
- current 分量至少包含一个真实观测节点
- current 分量存在离开脚下人工区域的安全边

### 3.4 修改任务

- [x] 删除 `_reachable_free_mask()` 对相邻 unknown 隔离分量的合并
- [ ] 为人工 prior cell 增加来源标记并传入图构建
- [x] 增加 current node 分量 BFS 和图级初始化状态
- [ ] FrontierDetector 和 Planner 只使用 current node 可达分量
- [ ] 增加栅格已连通但图断开的安全桥接节点补采样
- [ ] 记录图分量数、current 分量节点数、可达真实节点数和可达 Frontier 数

### 3.5 验收条件

- 初始化完成时, current node 通过真实 edge 到达至少一个外围真实观测节点
- `robot_blind_zone_connected=true` 但图不连通时, 系统不能报告可导航
- 所有发布路径中的节点都属于 current node 分量
- 路径的每条边均经过已知或显式人工 prior free, 且满足净空要求
- 没有任何边直接跨越未授权 unknown

## 4. 问题三: 连边过密, 图更新仍然很慢

### 4.1 图片现象

两张图中脚下区域有大量交叉红线, 局部形态接近全连接图

这不仅影响 RViz 可读性, 还代表当前实现确实需要保存和检查大量 pair

### 4.2 根因

当前边更新规则是:

```text
当前 GridMap 内全部节点
  -> cKDTree.query_pairs(8 m)
  -> 每个 pair 执行一次栅格走廊检查
  -> 每帧重复
```

主要放大因素:

1. unknown 边界使 `free_radius` 变小, 节点沿空洞边缘变密
2. 8 m 连边半径大于脚下岛内大多数节点间距, 大量节点互相成为候选
3. `query_pairs()` 虽然由 SciPy C++ 实现, 但返回的 pair 数量本身没有减少
4. 每个 pair 仍在 Python 循环中调用一次线段栅格检查
5. 地图稳定时也会重新生成并复查全部局部 pair
6. unknown 变为 free 后, 旧节点的 free radius 会扩大, 但重叠旧节点不会被合并或删除
7. 探索过程中积累的密集节点会永久增加后续 pair 数

对于局部节点数 `N`, 当大部分节点都位于 8 m 内时, pair 数接近:

```text
N × (N - 1) / 2
```

因此只把邻域查询换成 `cKDTree` 不能解决走廊检查的二次增长

### 4.3 优化顺序

#### 第一阶段: 先减少错误产生的密集节点

- 修复高程图源头空洞, 避免人工岛边缘长期产生很小的 free radius
- 真实观测扩展后重新计算旧节点 free radius
- 对当前局部已知 free 区域执行节点压缩
- current node、路径关键节点和 Frontier owner 先保留
- 普通节点落入已保留节点的 free radius 时, 删除被覆盖节点
- 删除节点后重新连接受影响局部 pair, 并执行 BFS 验证

节点压缩必须是可重复的不变量, 不能只限制新节点, 却永久保留历史密集节点

#### 第二阶段: 保留论文拓扑, 改为真正的增量检查

仍保留“半径内安全节点全部连边”的单一拓扑规则, 但不再每帧重复检查没有变化的 pair

最小增量规则:

- 新节点只生成与自身相关的半径 pair
- 删除节点只删除其关联边
- obstacle 或净空变化只复查走廊经过 dirty cell 的现有边
- unknown 变为 free 时, 只重试走廊经过对应 dirty cell 的阻挡 pair
- 节点集合和栅格均未变化时, 直接复用上一帧边结果
- 缓存只保存 pair、走廊包围盒和最后状态, 不再引入多种拓扑边类型

这里需要恢复“变化区域到 pair”的精确索引, 但边的拓扑规则仍然只有存在和不存在两种, 不恢复过去的 bridge、low-degree 和近邻预算等混合规则

#### 第三阶段: 将批量走廊检查移到编译实现

对同一批必须复查的 pair:

- 使用连续 NumPy 数组传入端点和分类栅格
- 用 C++、Cython 或 Numba 批量执行栅格线段和净空检查
- 一次返回 free、unknown、obstacle 三态数组
- Python 只负责生成 edge delta

SciPy `cKDTree` 已经解决 pair 搜索, 需要替换的是逐 pair 的 Python 调度和线段结果分配

#### 第四阶段: 性能仍不达标时再修改拓扑

如果完成节点压缩和增量检查后仍超出实时预算, 再评估稀疏规划图:

- Delaunay 或半径图的几何骨架
- 保留 current node 和 Frontier 的必要冗余边
- 图分量之间只增加最短安全补边
- 每次裁剪后执行 BFS 和最短路一致性检查

该方案会偏离论文“半径内安全节点全部连边”的直接实现, 不应作为第一步

### 4.4 可视化单独处理

计算图和 RViz 显示需要分开:

- Planner 继续使用完整有效图
- RViz 默认只显示生成树、当前路径或每个节点最近的少量边
- 增加调试开关显示完整边集合

只减少 RViz 红线不能改善图更新耗时, 因此这项只能作为显示优化

### 4.5 修改任务

- [ ] 增加节点来源、受保护状态和局部压缩流程
- [x] unknown 变 free 后删除已被扩大 free radius 覆盖的冗余普通节点
- [ ] 增加 pair 状态缓存和 dirty cell 到 pair 的精确索引
- [x] 稳定帧跳过 pair 生成和走廊检查
- [x] 新节点只检查自身 incident pair
- [x] dirty cell 使用边索引或变化区域近端点缩小 pair
- [ ] 实现批量编译走廊检查基准, 与当前 `skimage.draw.line` 逐 pair 方式比较
- [x] RViz 默认切换为稀疏边显示, 完整边作为调试选项
- [ ] 若前述优化仍不达标, 再单独评审稀疏规划拓扑

### 4.6 性能统计和验收

每帧同时记录耗时和工作量, 不能再把窗口平均耗时与最后一帧工作量混用

需要记录:

- local nodes
- total edges
- average degree
- candidate pairs
- revalidated pairs
- skipped cached pairs
- compressed nodes
- graph components
- edge pair、edge validation、edge delta 和总更新时间

目标:

- 稳定帧 `revalidated_pairs=0`
- 单个局部变化的检查量由受影响 pair 决定, 不再接近全部局部 pair
- 开阔稳定区域的节点数和边数不随运行时间持续增长
- 图更新 P95 小于 200 ms, 满足 5 Hz 输入频率
- 边阶段 P95 小于 100 ms
- RViz 完整边关闭时, 不影响 Planner 使用的真实图

## 5. 自动化测试补充

### 5.1 高程图源头

- [ ] 首帧点云外存在不规则环形盲区, 人工 prior 能填到真实 free
- [x] 第二帧相同输入时人工区域仍存在
- [x] 后续真实 obstacle 覆盖人工 free
- [x] 高程不相容的外围地面不能作为连接目标
- [x] 启动范围外的普通 unknown 不会被填充

### 5.2 图连通

- [x] 使用与第一张图相似的“中央岛、unknown 环、外围 free”栅格
- [ ] 使用与第二张图相似的狭窄连接和不规则障碍栅格
- [ ] 栅格连通但净空不足时, 图初始化保持未完成
- [ ] 补采样后 current node 可以 BFS 到外围真实节点
- [x] 隔着 unknown 的外围节点不能进入 current 可达分量
- [ ] Planner 不会选择不可达分量的 Frontier

### 5.3 节点和边性能

- [x] unknown 边界消失后, 重叠历史节点会被压缩
- [ ] 节点压缩后 current 分量和最短路径仍存在
- [x] 完全稳定的第二帧不重新检查全部 pair
- [x] 单个 obstacle cell 只复查相交边
- [x] 单个 unknown 变 free 只重试变化区域附近 pair
- [ ] 运行 10 分钟等价回放后, 节点、边和缓存数量保持稳定
- [ ] 增加 50、100、200 和 500 个局部节点的性能基准

## 6. 实施顺序

### P0: 增加现场可验证统计

- [x] 同帧输出地图初始化、图分量、节点、pair 和耗时
- [ ] 保存问题发生时的一帧 GridMap 和 NavigationGraph 作为回归数据

### P1: 修复高程图源头

- [x] 使用 `elevation_mapping_cupy` 已有高方差初始化能力并扩大启动先验
- [ ] 验证真实点云覆盖和跨帧保持

### P2: 修复图连通判定

- [x] 只采样真实 reachable free
- [ ] 增加 current 分量 BFS 后置条件和安全补采样

### P3: 清理密集节点

- [x] 在 free radius 扩大后压缩当前局部冗余节点
- [ ] 保护 current、Frontier 和必要历史路径节点

### P4: 优化边更新

- [x] 稳定 pair 缓存
- [x] dirty cell 精确失效
- [ ] 批量编译走廊检查

### P5: Unity 长任务验收

- [ ] 机器人能够从初始化区域走入真实观测区域
- [ ] current node 始终存在到外部图的安全路径
- [ ] 长时间探索后图更新耗时不持续增长
- [ ] 完整边调试显示关闭后 RViz 清晰, Planner 结果不变

## 7. 预计涉及文件

高程图源头仓库:

- `elevation_mapping_cupy/scripts/elevation_mapping_node.py`
- `elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py`
- `elevation_mapping_cupy/elevation_mapping_cupy/initializer_utils.py`
- 对应配置和测试

当前仓库:

- `graph_construction/graph_construction/graph_builder.py`
- `graph_construction/graph_construction/edge_builder.py`
- `graph_construction/graph_construction/graph_memory.py`
- `graph_construction/graph_construction/frontier_detector.py`
- `graph_construction/graph_construction/node.py`
- `graph_construction/test/test_graph_builder.py`
- `graph_construction/test/test_persistent_graph.py`
- `graph_construction/configs/elevation_mapping_sim.yaml`
- `graph_construction/configs/graph_construction_elevation.yaml`
