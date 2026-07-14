# Frontier 生命周期和 Deferred Branch

## 问题

此前持久图同时长期保存节点、edge、窗口外 Frontier 和视觉评分

窗口外 Frontier 无法由当前 rolling GridMap 验证, 旧视觉评分也不再对应当前相机视角, 但二者仍会进入 planner 候选集, 导致机器人前进时突然选择后方历史边界

## 职责分层

### Persistent Graph

- 长期保存稳定节点、edge、explored radius、current node 和 traversal memory
- unknown 或滚动窗口移出不能删除历史路线

### Active Frontier

- 只表示当前 GridMap 中可验证的 free/unknown 边界
- 当前可见范围内继续验证边界语义、owner 路径和 explored radius
- 移出窗口后清除 `is_frontier` 和 `frontier_points`, owner 节点继续保留

### Current Visual Score

- `CurrentFrontierScores` 每个视觉处理周期重新创建
- 三相机同时看到同一 Frontier 时按方向取当前最高证据
- 当前帧不可见的 Frontier 使用几何默认分数
- 已不再是活动 Frontier 的 UUID 和旧视角分数下一帧删除

### Deferred Branch

- planner 将未选择分支保存为稳定 owner UUID、位置、发现方向和发现顺序
- 当前 `ActiveBranch` 有进展时 deferred branch 不参与实时候选排序
- 当前分支持续无 odom 进展后, 临时屏蔽失败邻域并恢复最早可达分支
- 返回保存入口后由当前地图 Frontier 接管, 不恢复历史视觉评分

## 参数变化

- 删除 `min_frontier_separation`
- 不新增 deferred branch 参数
- 继续复用 `frontier_progress_timeout` 作为当前分支无进展确认时间
- 继续复用 `frontier_continuity_radius` 屏蔽刚失败的分支邻域

## 修改文件

- `graph_construction/graph_construction/frontier_detector.py`
- `graph_construction/graph_construction/graph_memory.py`
- `graph_construction/test/test_persistent_graph.py`
- `visual_navigation/visual_navigation/wildos/current_frontier_scores.py`
- `visual_navigation/visual_navigation/wildos/nav.py`
- `visual_navigation/configs/wildos_nav_conf.yaml`
- `visual_navigation/configs/wildos_nav_sim_conf.yaml`
- `visual_navigation/test/test_current_frontier_scores.py`
- `graphnav_planner/include/graphnav_planner/planner.hpp`
- `graphnav_planner/src/planner.cpp`
- `graphnav_planner/test/test_deferred_branch.cpp`
- `graphnav_planner/CMakeLists.txt`
- `graphnav_planner/package.xml`

## 验证

- Graph Construction、Current Frontier Scores 和 Object Search 相关定向测试共 `41` 项通过
- `graph_construction`、`visual_navigation` 和 `graphnav_planner` 构建通过
- deferred branch 两项 GTest 通过
- 包级 `uncrustify` 仍报告该包原有 C++ 文件整体格式差异, 本次不重排无关代码
