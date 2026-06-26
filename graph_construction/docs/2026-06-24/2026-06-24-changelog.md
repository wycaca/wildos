# Graph Construction 变更记录

日期: 2026-06-24

## 已完成

- 将 Graph Construction 当前仿真输入调整为 `/spot1/traversability_grid` 和 `/unity/odom`
- 将 graph frame 修正为 `map`, 当前 TF 链路为 `map -> odom_fram -> livox_frame/camera_frame/imu_link`
- 新增 `livox_grid_builder`, 将 `/livox/lidar` 的 `sensor_msgs/PointCloud2` 转换为 `/spot1/traversability_grid`
- 新增 `livox_grid_builder.yaml` 和 `livox_grid_builder.launch.py`
- 新增 `setup.cfg`, 修正 `ros2 run graph_construction livox_grid_builder` 可执行入口安装位置
- 更新 `package.xml` 和 `setup.py`, 注册 LiDAR grid builder 依赖和入口
- 完成端到端测试: `/livox/lidar -> /spot1/traversability_grid -> /spot1/nav_graph`
- 实测 `graph_construction` 可发布非空 NavigationGraph, 最新结果约为 `nodes=146`, `edges=330-340`
- 建边算法从半径内近似全连接调整为 `edge_radius + k-nearest collision-free neighbors`
- 新增 `max_edge_neighbors` 和 `current_node_max_edge_neighbors` 参数
- 调低 RViz edge, free_radius, explored_radius marker 的透明度和线宽, 减少地图遮挡
- 修正 `current_node` marker 为固定 ID, 避免黄色当前点残留成路径
- 新增独立 trajectory marker, 将走过路径和 graph nodes 分开显示
- RViz 中区分当前局部 grid 内节点和历史 memory nodes
- 新增当前局部 grid footprint marker, 辅助判断哪些节点来自当前观测范围
- 收紧 frontier 生成条件, 增加 `frontier_min_span` 和 `frontier_border_margin`
- 将 `frontier_min_points` 默认值从 2 调整为 4, 降低孤立噪声 frontier
- 将端到端测试步骤补充到 `2026-06-23-implementation-plan.md`
- 新增 `2026-06-24-livox-to-occupancy-grid-plan.md`, 记录 LiDAR 转 OccupancyGrid 的实现说明
- 新增 `2026-06-24-stage2-visual-scoring-plan.md`, 记录阶段二 Visual Scoring 与 Graph Planner 闭环实现方案
- 新增 `wildos_nav_sim_conf.yaml`, 适配当前单相机仿真 topic 和 `map` frame
- 新增 `odom_frame_adapter`, 将 `/unity/odom` 转换为 `/spot1/odom_for_scoring`
- 新增 `wildos_sim_launch.py`, 默认使用 ROS domain 3 和 CycloneDDS 启动视觉评分链路
- 更新 `graphnav_planner.launch.yml`, 支持通过 `odom_topic` 参数选择 planner 和 path follower 的 odom 输入
- 新增 `graph_construction_sim.launch.py`, 合并启动 `livox_grid_builder` 和 `graph_construction`
- 新增 `scripts/start_graph_construction.sh`, 固化阶段一环境变量和合并 launch 启动命令
- 新增 `scripts/start_visual_navigation.sh`, 固化 UV venv, ROS 环境和视觉评分启动命令
- 更新阶段二测试说明, 补充 UV venv, `skimage` 和 ROS console script shebang 验收步骤
- 修正 `wildos_nav_sim_conf.yaml` 的 checkpoint 文件名, 使用当前 `ckpts` 目录已有模型
- 修正 `odom_frame_adapter`, 默认将 `/spot1/odom_for_scoring` 的 stamp 改为当前 ROS time, 避免视觉同步器收不到消息
- 调整 `wildos_nav_sim_conf.yaml` 同步参数为 `syncsub_queue_size=80`, `syncsub_slop=5.0`, 适配当前 nav graph 与相机约 `3.6s` 的 stamp 差
- 修正 TF lookup 缓冲处理, 遇到早于 TF buffer 的 stale message 时丢弃, 避免视觉评分一直卡在旧消息

## 待调整

- 保持蓝色点语义为 frontier node, 紫色小方块语义为 frontier_points
- 绿色 free nodes 出现在当前 LiDAR 扫描范围外通常是 graph memory 的正常表现, 因为局部 grid 是当前观测, nav graph 会保留历史节点
- 继续用 RViz 检查 `/spot1/traversability_grid` 和 `/spot1/graph_construction_viz` 的几何合理性

## 文档同步

- 建边核心算法已经同步到 `2026-06-23-implementation-plan.md`
- 建边核心算法已经同步到 `2026-06-22-principles.md`
- frontier 过滤算法已经同步到 `2026-06-23-implementation-plan.md`
- frontier 过滤算法已经同步到 `2026-06-22-principles.md`
