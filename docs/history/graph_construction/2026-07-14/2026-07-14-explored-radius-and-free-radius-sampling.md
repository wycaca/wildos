# Explored Radius 和 Free Radius 采样修复

## 问题

运行图曾出现 `121` 个节点、`436` 条 edge、`0` 个活动 Frontier, 其中 `64` 个节点的 `explored_radius=inf`

全 known 的局部 GridMap 不代表整个世界已探索, 但旧距离场在局部图内找不到 unknown cell 时会返回无限距离

持久 explored area 索引把该值解释为全局覆盖, 因而当前可见 Frontier 会被全部过滤

普通节点原先按固定 stride 生成规则网格, 开阔区域节点明显过密

社区实现按已有节点 free radius 覆盖去重, 密度较低但节点排列不规则, 不适合当前自研导航直接观察和调试高层路径

## 修复方案

### 有限 explored radius

- 计算 unknown 距离场时将 rolling GridMap 外部视为 unknown
- 历史节点进入更新流程时清除负数和非有限 explored radius
- 当前可见节点从本帧局部距离场重建有限 explored radius
- explored area 索引只接收有限正半径, 旧状态不能清除全部 Frontier

### 世界坐标自适应网格

- 所有候选固定在世界坐标网格上, rolling GridMap 移动时网格相位不变化
- `sample_stride * resolution` 定义最细网格间距
- free radius 决定二次幂网格层级, 层级之间保持嵌套和整齐排列
- 障碍物或 unknown 附近 free radius 小, 使用细网格补充节点
- 开阔区域 free radius 大, 只保留粗网格节点
- `min_node_separation` 防止历史节点附近重复补点, 同时用于 robot anchor breadcrumb

## 参数变化

- 不新增运行参数
- `sample_stride` 同时定义最细网格密度
- `min_node_separation` 继续用于普通节点去重和 robot anchor
- 普通节点密度由 obstacle 和 unknown clearance 对应的 free radius 自适应控制

## 修改文件

- `graph_construction/graph_construction/grid_types.py`
- `graph_construction/graph_construction/graph_builder.py`
- `graph_construction/graph_construction/frontier_detector.py`
- `graph_construction/test/test_graph_builder_diagnostics.py`
- `graph_construction/test/test_persistent_graph.py`

## 验证

- unknown 距离场无 unknown cell 时仍返回有限距离
- 历史 `explored_radius=inf` 在下一帧恢复为有限局部覆盖
- 非有限历史半径不能隐藏当前可见 Frontier
- 普通节点保持世界坐标网格排列
- unknown 附近使用细网格, 开阔区域自动切换粗网格
- `graph_construction` 包级 Python 测试 `35` 项通过
- 同一合成地图中普通节点由旧固定网格 `138` 个降至 `57` 个, 社区纯覆盖实现为 `23` 个
- `colcon build --packages-select graph_construction --symlink-install` 通过
