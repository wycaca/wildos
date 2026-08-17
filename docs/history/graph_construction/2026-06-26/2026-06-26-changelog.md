# Graph Construction 变更记录

日期: 2026-06-26

## 已完成

- 修正阶段三架构说明, 明确 `/spot1/traversability_grid` 属于 LiDAR / 几何地图链路, 不属于 3 相机视觉链路
- 新增 `scripts/start_graph_construction_livox.sh`, 作为默认 LiDAR baseline graph construction 启动入口
- 保留 `scripts/start_graph_construction.sh` 作为兼容别名, 内部转到 `start_graph_construction_livox.sh`
- 将 `graph_construction_elevation_sim.launch.py` 和 `start_graph_construction_elevation.sh` 定义为 experimental elevation backend
- 将实验 elevation adapter 输出从 `/spot1/traversability_grid` 改为 `/spot1/elevation_traversability_grid`, 避免覆盖默认 LiDAR baseline 地图
- 将 elevation 版 graph construction 改为直接消费 `/elevation_mapping_node/elevation_map_raw`, 不再默认依赖 2D OccupancyGrid 投影
- graph node, edge, frontier point 的 z 坐标现在可来自 GridMap `elevation` layer, 使 RViz 可视化更接近论文高程图底图
- `graph_construction_elevation_sim.launch.py` 不再默认启动 `grid_map_to_occupancy`, 2D projection adapter 保留为 debug/兼容工具
- 新增 `elevation_visual_navigation_sim.launch.py` 和 `scripts/start_elevation_visual_navigation.sh`, 用于高程地图, graph 点边, visual scoring 一起联调
- `odom_frame_adapter` 首帧日志增加输入输出 frame 和 position, 便于排查 RViz odom 偏移
- 修正 GridMap 直连时的 cell 到 world 坐标转换, 使用 grid_map 中心和 index 负方向约定, 避免 graph 点边相对高程图镜像或偏移
- 修正 GridMap footprint marker 的绘制方向, 与 GridMap 主显示保持一致
- 新增 `2026-06-26-geometry-map-vision-decoupling.md`, 记录几何地图与 3 相机视觉解耦原则
- 补充 RViz 可视化说明, 明确 GridMap raw topic 需要使用 `grid_map_rviz_plugin/GridMap`, 不能使用 `Map`
- 更新 README, 增加当前仿真默认启动说明
- 在 `grid_map_to_occupancy` 中新增连通性后处理和小孔洞修复
- 后处理包含一格裂缝 majority fill, 小型 enclosed hole fill, 小型 free island removal
- 新增参数 `enable_postprocess`, `min_free_component_cells`, `fill_hole_max_cells`, `fill_hole_min_free_neighbor_ratio`, `majority_fill_iterations`, `majority_fill_min_neighbors`
- 更新 `grid_map_to_occupancy.yaml`, 默认启用保守后处理
- 更新阶段三实现文档, 说明 GridMap 直连 graph construction 和 debug projection adapter 的边界
- GridMap 直连 graph construction 增加分类后处理, 复用一格裂缝填充, 小孔洞填充, 小型 free island 删除思路
- `graph_construction_elevation.yaml` 增加 GridMap 直连后处理参数, 便于和 debug projection adapter 对齐调参
- graph construction 首帧日志增加 frontier 数量和 GridMap raw / postprocess 分类统计, 便于区分坐标, 分类和连通性问题
- 修正 GridMap 直连坐标映射, 按 `grid_map_core` 约定使用 row index 对应 world x, column index 对应 world y
- 修正 GridMap footprint marker 的 x / y 尺寸, 避免调试框宽高和 RViz GridMap 显示不一致
- GridMap 直连 graph node 增加 odom 高度门限, 避免墙面或高处表面被采样成可走节点
- graph construction 可视化增加白色 `robot_position` marker, 表示狗当前 XY 投影到 GridMap elevation 表面的地面点
- graph construction 可视化增加灰色 `robot_odom_position` marker, 表示原始 odom / base 位置
- graph construction 首帧 odom 日志增加 position, 便于和 RViz marker 对照
- robot position 投影支持在局部邻域查找最近有效 elevation, 避免机器人脚下小 NaN 空洞导致白点回落到原始 odom z
- 新增 `2026-06-26-paper-ui-target-correction.md`, 基于论文界面截图修正开发目标
- 明确论文 UI 目标是 2.5D terrain surface 加 graph / frontier / path overlay 分层显示, 不是所有 marker 共用一个 z 平面
- 更新阶段三验收标准, 白色 robot marker 表示 GridMap ground projection, 灰色 marker 表示 raw odom / base debug
- 修正 robot ground projection 失败时的可视化语义, 投影失败不再追加黄色 trajectory, raw odom marker 改为红色并在日志中标记 `robot_projection=missing`

## 实测结果

- 重新 build `graph_construction` 成功
- 旧版 2D projection adapter 曾验证 `/spot1/elevation_traversability_grid` 可正常发布
- 旧版 projection 首帧 graph construction 输出 `nodes=30`, `edges=52`
- 旧版 projection 稳定后 `/spot1/elevation_traversability_grid` 统计约为 `free=1685`, `occupied=1914`, `unknown=18901`
- 旧版 projection free 连通域数量为 6, 最大 free 连通域约占全部 free cell 的 75.1%
- 旧版 projection `/spot1/nav_graph` 统计为 `nodes=44`, `edges=85`, `frontier=32`
- GridMap 直连版本已完成静态检查和重新 build, 需要在仿真运行时复测 graph 数量和 RViz 贴合效果
- 修正 GridMap 坐标约定后, 短时联调可发布 graph, 当前首帧约为 `nodes=7`, `edges=0`, 后续优先复测高程图与 graph marker 的 XY 贴合
- 后处理函数离线最小验证通过, 一格 unknown 裂缝可被填成 free
- 使用 `/tmp` build/install/log 目录重新 build `graph_construction` 成功
- 使用 `grid_map_core::getPositionFromIndex` 验证 GridMap index 坐标约定, Python 侧 `grid_to_world` / `world_to_grid` 已按该约定修正
- 当前截图显示高程图本身仍有竖直面或高架面, graph 高处点属于采样过滤不足, 已增加 odom 高度门限后需要仿真复测
- 当前截图显示白色 robot marker 未贴合高程面, 已改为使用 GridMap elevation 投影后的 ground position 作为白点位置
- 论文 UI 分析完成, 后续优先级调整为 frame / GridMap surface 对齐, robot ground projection, graph node / frontier z 统一, elevation surface 质量过滤
- 当前截图中黄色点远离高程图, 判断为 robot XY 不在当前 GridMap footprint 或当前位置无有效 elevation, 需要先检查 odom 与 GridMap frame / center 是否一致

## 旧版 projection 调参结论

- 旧版 2D projection 参数能生成非空且连通性较好的 graph, 暂不继续激进填充
- 图中长条黑色区域保留为 obstacle, 避免把真实障碍误填为 free
- 后续如果 RViz 仍有大量一格裂缝, 优先将 `majority_fill_iterations` 从 1 调到 2
- 后续如果存在小型黑色孔洞, 优先增大 `fill_hole_max_cells`
- 后续如果 free 噪声孤岛较多, 优先增大 `min_free_component_cells`
