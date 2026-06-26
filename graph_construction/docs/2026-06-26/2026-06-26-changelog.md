# Graph Construction 变更记录

日期: 2026-06-26

## 已完成

- 修正阶段三架构说明, 明确 `/spot1/traversability_grid` 属于 LiDAR / 几何地图链路, 不属于 3 相机视觉链路
- 新增 `scripts/start_graph_construction_livox.sh`, 作为默认 LiDAR baseline graph construction 启动入口
- 保留 `scripts/start_graph_construction.sh` 作为兼容别名, 内部转到 `start_graph_construction_livox.sh`
- 将 `graph_construction_elevation_sim.launch.py` 和 `start_graph_construction_elevation.sh` 定义为 experimental elevation backend
- 将实验 elevation adapter 输出从 `/spot1/traversability_grid` 改为 `/spot1/elevation_traversability_grid`, 避免覆盖默认 LiDAR baseline 地图
- 新增 `2026-06-26-geometry-map-vision-decoupling.md`, 记录几何地图与 3 相机视觉解耦原则
- 补充 RViz 可视化说明, 明确 GridMap raw topic 需要使用 `grid_map_rviz_plugin/GridMap`, 不能使用 `Map`
- 更新 README, 增加当前仿真默认启动说明
- 在 `grid_map_to_occupancy` 中新增连通性后处理和小孔洞修复
- 后处理包含一格裂缝 majority fill, 小型 enclosed hole fill, 小型 free island removal
- 新增参数 `enable_postprocess`, `min_free_component_cells`, `fill_hole_max_cells`, `fill_hole_min_free_neighbor_ratio`, `majority_fill_iterations`, `majority_fill_min_neighbors`
- 更新 `grid_map_to_occupancy.yaml`, 默认启用保守后处理
- 更新阶段三实现文档, 说明 GridMap 后处理算法和参数约束

## 实测结果

- 重新 build `graph_construction` 成功
- 启动 `scripts/start_graph_construction_elevation.sh` 后, `/spot1/elevation_traversability_grid` 正常发布
- 首帧 graph construction 输出 `nodes=30`, `edges=52`
- 稳定后 `/spot1/elevation_traversability_grid` 统计约为 `free=1685`, `occupied=1914`, `unknown=18901`
- free 连通域数量为 6, 最大 free 连通域约占全部 free cell 的 75.1%
- `/spot1/nav_graph` 统计为 `nodes=44`, `edges=85`, `frontier=32`

## 调参结论

- 当前参数能生成非空且连通性较好的 graph, 暂不继续激进填充
- 图中长条黑色区域保留为 obstacle, 避免把真实障碍误填为 free
- 后续如果 RViz 仍有大量一格裂缝, 优先将 `majority_fill_iterations` 从 1 调到 2
- 后续如果存在小型黑色孔洞, 优先增大 `fill_hole_max_cells`
- 后续如果 free 噪声孤岛较多, 优先增大 `min_free_component_cells`
