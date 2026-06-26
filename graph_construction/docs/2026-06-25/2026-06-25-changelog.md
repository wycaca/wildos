# Graph Construction 变更记录

日期: 2026-06-25

## 已完成

- 补充阶段二文档中的全链路 topic 说明
- 补充 RViz 可视化图例, 覆盖阶段一 graph marker, 阶段二 visual scoring marker, model visualization, score rings 和 planner debug 输出
- 明确 `/spot1/score_rings` 表示方向评分, 不是 obstacle 或安全半径
- 明确 `/spot1/nav_graph_viz` 默认隐藏并清理 `free_radius`, `explored_radius` 和 UUID 文本调试层
- 明确相机 debug 图中绿色点和绿色线表示投影到图像上的 geometric frontier/path 调试线
- 检查 `/spot1/score_rings` 无显示问题, 发现当前 `/spot1/nav_graph` 中 `frontier_nodes=0`
- 复算当前 `/spot1/traversability_grid` 后确认存在 free/unknown 边界, 不考虑 removed frontier suppression 时可生成 frontier
- 将当前仿真的 `removed_frontier_suppression_radius` 调整为 `0.0`, 避免局部滑窗下短暂消失的 frontier 被长期抑制
- 在 `frontier_detector.py` 中增加保护, suppression 半径小于等于 `0.0` 时跳过 removed frontier 检查
- 完成阶段二端到端实测, `/spot1/scored_nav_graph`, `/spot1/model_visualization`, `/spot1/score_rings`, `/spot1/nav_graph_viz` 和 `/spot1/graphnav_planner/path` 均有输出
- 实测 planner 可从 `/spot1/imgnav_waypoint` 生成非空 `/spot1/graphnav_planner/path`
- 修正 `graph_construction`, `livox_grid_builder`, `wildos`, `odom_frame_adapter` 在 Ctrl-C 停止时重复 shutdown 或 destroy 阶段被 SIGINT 打断导致的异常退出
- 将 planner 中有效路径未落到 frontier 节点时的 `NO FRONTIER IN PATH` 提示从 warn 降为 debug, 避免 waypoint 测试时误报
- 新增阶段三实现文档, 明确 Path Execution 与 `/unity/cmdvel` 闭环方案
- 根据论文实现重新调整阶段三优先级, 暂缓轻量 cmd_vel controller 和 Nav2 接入
- 新增阶段三 3 相机与 elevation/traversability map 适配实现文档
- 使用 UV 为当前 `.venv` 增加 `cupy-cuda12x`, `ros2-numpy`, `simple-parsing`
- 安装并 build `elevation_mapping_cupy` Humble 分支, ROS 可发现 `elevation_map_msgs` 和 `elevation_mapping_cupy`
- 重新验证 GPU runtime, 主机和 UV 环境中 `cupy` 可见 RTX 4090 且简单 GPU 计算通过, 受限沙箱下的 `cudaErrorNoDevice` 视为权限误判
- 新增 `grid_map_to_occupancy` adapter, 将 `elevation_mapping_cupy` 的 GridMap 转为实验 topic `/spot1/elevation_traversability_grid`
- 新增 `elevation_mapping_sim.yaml`, `grid_map_to_occupancy.yaml`, `elevation_mapping_sim.launch.py`, `graph_construction_elevation_sim.launch.py`
- 新增 `scripts/start_graph_construction_elevation.sh`, 用于启动 elevation mapping 版本 graph construction
- 将 `wildos_nav_sim_conf.yaml` 切换为三相机 topic, 当前三路 camera header 均为 `camera_frame`, 后续需要补独立三相机 TF
- 实测 `/elevation_mapping_node/elevation_map_raw` 已输出 `elevation`, `traversability`, `variance` layers, `/livox/lidar` 约 10 Hz
- 发现按 upstream 默认 `0.7/0.4` 解释 traversability 会导致 free cell 过少, graph construction 首帧 `nodes=0`
- `grid_map_to_occupancy` 增加首帧统计日志, 输出 valid/free/occupied/unknown 和 raw traversability 分布
- `grid_map_to_occupancy.yaml` 改为分位数归一化后使用 `free_threshold=0.2`, `occupied_threshold=0.05`
- 新增 `graph_construction_elevation.yaml`, elevation pipeline 默认使用更适合当前仿真的采样和 clearance 参数
- 确认 `ros2 topic echo /spot1/elevation_traversability_grid --once` 开头全 `-1` 不能代表整图全未知, 后续测试应统计整张 grid 的值分布

## 文档同步

- 已更新 `2026-06-24-stage2-visual-scoring-plan.md`
- 已更新 `2026-06-23-implementation-plan.md`
- 已更新 `2026-06-22-principles.md`
- 已更新 `2026-06-25-changelog.md`
- 已新增 `2026-06-25-stage3-motion-execution-plan.md`
- 已新增 `2026-06-25-stage3-three-camera-elevation-map-plan.md`
