# WildOS 当前实现原理

> 本文随当前实现长期维护，核心原理变化时必须同步更新

## 1. 几何地图原则

当前唯一几何输入是 elevation mapping 发布的 `GridMap`

`grid_adapter.py` 负责:

1. 根据 GridMap layout 和 circular buffer 起点恢复二维 layer
2. 读取 traversability、elevation、variance 和辅助 layer
3. 将 layer 统一分类为 free、obstacle 和 unknown
4. 对固定的小孔洞做后处理
5. 生成带 origin、resolution、frame 和高度查询能力的 `ClassifiedGrid`

不再支持:

- `OccupancyGrid` 输入
- free 和 obstacle 的 2D 数值阈值
- transpose 或 flip 兼容开关
- 关闭当前必要后处理的兼容分支

## 2. 稀疏图构建

`SparseGraphBuilder.update` 的核心过程:

```text
ClassifiedGrid
  -> distance fields
  -> current node sampling
  -> historical node validation
  -> edge building and validation
  -> frontier detection and assignment
  -> robot anchor update
  -> GraphUpdateResult
```

`GraphUpdateResult` 只返回当前 graph 和 classified grid，不携带 stage timing 或统计诊断对象

### 2.1 节点

节点使用世界坐标对齐的自适应 lattice 采样:

- unknown 附近保持更密集
- 已充分探索区域减少重复节点
- `min_node_separation` 防止滚动窗口产生近距离副本
- 历史节点通过稳定 ID 保留

机器人当前位置使用独立 anchor node 表达，不与持久 graph 节点身份混合

### 2.2 边

边必须满足:

- 两端节点距离在连接半径内
- 线段在当前可见地图中 collision free
- corridor clearance 满足 obstacle 和 unknown 约束
- 历史边在局部窗口变 unknown 时不会被误删
- 当前可见障碍明确阻断时会失效

### 2.3 Frontier

frontier 是 free cell 与 unknown 邻域的边界，不是每个 cell 都创建 graph node

处理过程:

1. 扫描有效 free cell
2. 过滤 rolling grid 外边缘
3. 按米制 spacing 选代表 cell
4. 将候选分配给附近 collision-free graph owner
5. 按点数、跨度和历史 owner 规则过滤噪声

历史窗口外分支由 planner 的 deferred branch 逻辑处理，不继续伪装成活动 frontier

## 3. 视觉评分

WildOS 对三路相机做 ExploRFM 推理，获得:

- frontier confidence
- traversability confidence
- spatial feature
- query similarity mask

几何 frontier 投影到各相机后，视觉评分写入 `NavigationGraph` 的 frontier score 字段

当前 `frontier_scores` 的维护位置是:

```text
visual_navigation/visual_navigation/wildos/nav.py
```

它不属于 `graph_construction`

## 4. 目标检测和到达证据

目标检测必须先通过:

- peak score 阈值
- connected component 像素数和面积占比
- 连续帧确认

确认后的 mask 才会发布为 `ObjectMaskWithTf`

近距离完成证据使用独立的 mask fraction、pixel count 和连续帧规则，并发布到 `object_reached_topic`

视觉评分、目标位置融合和最终完成是三个不同职责，不能合并成一个布尔 latch

## 5. 目标位置融合

当前融合入口:

```text
visual_navigation/visual_navigation/object_target_fusion.py
```

纯算法:

```text
triangulation3d/triangulation3d/target_particle_filter.py
```

融合过程:

1. 第一张确认 mask 沿相机射线初始化固定数量粒子
2. 后续不同视角根据 mask 投影一致性递归更新权重
3. 重复视角和与稳定目标冲突的观测被拒绝
4. 两个有效视角后可以形成视觉跟踪目标
5. 视角数、有效粒子和方差满足条件后形成稳定视觉目标
6. LiDAR mask 内点先移除地面，再优先选择离相机最近的有效前景簇
7. 已有视觉轨迹时，单帧 LiDAR 只记录候选，不改变位置、置信度或 source，连续两帧空间一致后才更新和锁定

LiDAR 不是多视角视觉融合成立的前提

Unity 场景的粒子射线最大深度为 `30m`，其他 profile 当前保持 `100m`

## 6. Goal Mux 状态所有权

`ObjectSearchGoalMux` 是 goal 和完成状态的唯一 owner

优先级:

```text
completion latch
  > stable target estimate
  > physically valid coarse target estimate
  > initial heading exploration goal
```

关键约束:

- 初始探索目标只基于首帧 odom 和配置 heading 计算一次
- 单视角 pending 估计不能替换初始目标
- 粗目标必须通过 frame、距离、高度、水平标准差和置信度门控
- 稳定目标也必须通过 frame 和高度门控
- 稳定融合目标使用独立的小门槛持续纠偏
- 普通目标小于 `target_update_min_distance` 的抖动不会移动目标
- `object_reached` 只有在稳定目标距离满足约束时才能触发完成
- 完成后持续发布当前位置停止目标和 completed 状态

## 7. Graph Planner

`graphnav_planner` 消费:

- scored navigation graph
- odom
- high-level goal pose
- object search status

planner 内部固定使用当前 traversability class，virtual goal 只参与搜索，不追加到可执行 path

算法参数统一位于:

```text
graphnav_planner/config/planner.yaml
```

`path_follower_node` 保留为可选的 path-to-goal 适配器，通过独立 launch 启动，不属于默认集成链路

## 8. 平台适配

平台差异通过 topic profile 注入，不复制主 launch

profile 负责:

- domain 和 RMW
- point cloud axis mode 和 frame
- odom 输入、输出和 pose source
- 相机 topic 和静态 TF 约定
- graph、目标搜索和 planner topic

核心算法不得读取固定机器绝对路径

模型和仓库资源由 `repository_root()` 解析，可通过 `WILDOS_REPO_ROOT` 显式覆盖

## 9. 研究基线原则

研究仓库保留 LRN、ImgFrontier 和 GeoFrontier 等 baseline

基线和当前主线共享消息与工具时，公共契约修改必须同步所有调用方

基线可以有独立 launch 和算法配置，但不能复制或接管默认 elevation 集成入口

## 10. 验证原则

每次结构清理至少验证:

- Python 和 launch 静态编译
- YAML 可解析
- graph、frontier、目标融合和 Goal Mux 单元测试
- `object_search_msgs` 到 `visual_navigation` 的消息构建
- `graphnav_planner` C++ 干净构建
- 旧 entrypoint、旧参数和旧 topic 的静态残留扫描
