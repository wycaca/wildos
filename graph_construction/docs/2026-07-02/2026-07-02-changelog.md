# Graph Construction 变更记录

日期: 2026-07-02

## 已完成

- 阅读 `TIKTOKDAD/nebula2-wildos-main_ws` 的 `main` 分支提交记录
- 确认该仓库当前有 3 个提交, 包括 1 个初始导入, 1 个 GitHub 上传导入, 1 个 visual navigation 更新
- 分析最新提交 `225be74`, 主要涉及三相机数字 topic mapping, `initial_goal_mux` 注释和初始粗目标参数, WildOS sync queue 调整
- 分析导入提交 `e587c19` 中的高价值模块, 包括 `graphnav_builder`, `graphnav_nav2_bridge`, `wildos_sync_tuner`, `objmask_sync_tuner`, `initial_goal_mux`
- 对照当前项目目标, 判断该仓库方向与 WildOS 几何图构建, 视觉评分和执行闭环部分一致, 但运行平台和阶段优先级不同
- 新增 `2026-07-02-external-commit-reference-analysis.md`, 记录可学习部分, 不建议照搬部分和后续优先级
- 执行 graph builder 分层边界重构, 将 ROS 地图解码, 纯图算法, ROS message 转换和 RViz 可视化进一步拆开
- 新增 `grid_types.py`, 承载 `ClassifiedGrid`, 坐标转换, elevation 查询, collision line 和 distance field
- 将 `graph_builder.py` 改为纯算法层, 不再导入 ROS 消息或生成 `NavigationGraph`
- 将 `GraphBuilder` 保留为 `SparseGraphBuilder` 兼容别名, 维持现有调用入口稳定
- 将 `edge_builder.py`, `frontier_detector.py`, `viz.py`, `livox_grid_builder.py` 的纯类型或线段工具引用切到 `grid_types.py`
- 新增 `2026-07-02-graph-builder-layer-boundary-refactor.md`, 记录本次分层边界重构
- 新增 `graph_construction/docs/AGENT_README.md`, 作为后续 agent 的简短项目记忆和文档入口
- 新增 `wildos_2d_sim.launch.py`, 将 LiDAR GridMap, graph construction, WildOS visual scoring, graphnav planner 和 path follower 合并为 2D 完整启动链路
- 扩展 `elevation_visual_navigation_sim.launch.py`, 将 3D elevation 链路补齐 graphnav planner 和 path follower
- 将 `scripts/` 下启动入口收敛为 `start_wildos_2d.sh` 和 `start_wildos_3d.sh`, 删除 graph-only 和 visual-only 旧脚本
- 新增 `2026-07-02-startup-script-consolidation.md`, 记录启动脚本整理范围, 链路和验证方式
- 新增 `topic_profiles.yaml`, 将 Isaac Sim, Unity 和真实狗的 topic / frame 差异集中到 profile 配置
- 新增 `topic_profiles.py`, launch 启动时按 profile 读取环境差异, 避免把 topic 硬编码散落在 launch 默认值里
- `wildos_2d_sim.launch.py` 和 `elevation_visual_navigation_sim.launch.py` 新增 `topic_profile`, `topic_profile_file` 和单项 topic override 参数
- `start_wildos_2d.sh` 和 `start_wildos_3d.sh` 支持 `WILDOS_TOPIC_PROFILE=isaac|unity|robot`
- 新增 `2026-07-02-topic-profile-configuration.md`, 记录 topic profile 的结构, 优先级和扩展规则
- 将 `topic_profiles.py` 中本轮新增 docstring 改为中文
- 在 `AGENT_README.md` 中补充代码注释规则, 明确新增注释使用中文
- 移除 `/tmp/wildos_topic_profiles` 临时 YAML 方案, topic profile 覆盖项改为直接传给节点
- `graph_construction` 和 `livox_grid_builder` 新增 `--config-override key=value`
- `wildos` 新增 `--config-override key=value`, 通过 OmegaConf dotlist 覆盖基础配置
- `elevation_visual_navigation_sim.launch.py` 对 `elevation_mapping_node.py` 改用 launch parameters 覆盖 frame 和 LiDAR topic
- 将本次触及代码中的英文注释和 docstring 改为中文

## 实测结果

- GitHub API 可读取目标仓库提交列表
- 目标仓库 `main` 分支提交为 `225be74`, `e587c19`, `a2c6d9a`
- 本地仓库未发现 `graphnav_builder`, `graphnav_nav2_bridge`, `initial_goal_mux`, `wildos_sync_tuner`, `objmask_sync_tuner` 等同名实现
- 本次只新增文档, 未修改运行代码
- `python3 -m py_compile` 已通过 `grid_types.py`, `grid_adapter.py`, `graph_builder.py`, `node.py`, `edge_builder.py`, `frontier_detector.py`, `viz.py`, `livox_grid_builder.py`
- `graph_builder.py` 和 `grid_types.py` 已确认没有 `rclpy`, ROS 消息包或 `msg_utils` 导入
- 本次新增 agent 入口文档, 未修改运行代码
- `python3 -m py_compile` 已通过 `wildos_2d_sim.launch.py` 和 `elevation_visual_navigation_sim.launch.py`
- `bash -n` 已通过 `start_wildos_2d.sh` 和 `start_wildos_3d.sh`
- `colcon build --packages-select graph_construction --symlink-install` 已通过
- `./scripts/start_wildos_2d.sh --show-args` 和 `./scripts/start_wildos_3d.sh --show-args` 已通过
- `load_topic_profile` 已验证可读取 `isaac`, `unity`, `robot` 三套 profile
- 三套 profile 已验证可生成 2D 和 3D launch 节点 Action
- `WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_2d.sh --show-args` 已通过
- `python3 -m py_compile graph_construction/graph_construction/topic_profiles.py` 已通过
- `python3 -m py_compile` 已通过本次涉及的 launch 和节点入口文件
- 本地代码搜索已确认不再存在 `materialize_node_config` 或 `wildos_topic_profiles` 运行链路

## 诊断结论摘要

- `graphnav_builder` 的 ROS 适配和纯算法层分离, GridMap 解码, frontier reachability, historical edge validation, robot anchor 和 stage timing 对当前项目有参考价值
- `wildos_sync_tuner.py` 和 `objmask_sync_tuner.py` 对当前三相机, nav graph, odom, TF, ObjectMask/LiDAR 同步调参有较高参考价值
- `initial_goal_mux.py` 可作为 object search 初始粗目标机制参考, 但当前手动 waypoint 阶段不急需
- `graphnav_nav2_bridge` 只适合作为后续 Nav2 执行闭环参考, 不适合当前直接合入
- 最新提交中的数字 camera mapping 适配 A300 仿真, 不应直接替换当前 `front`, `left`, `right` 语义
- 当前项目已有分层雏形, 本次先完成边界收紧, 后续可继续补纯算法单元测试和同步诊断工具
- 后续 agent 应优先读取 `graph_construction/docs/AGENT_README.md`, 再读取当天 changelog 和任务相关说明文档
- 当前启动入口只保留 2D 和 3D 两类脚本, 每个脚本都应对应完整程序链路而不是单模块调试链路
- 后续新增仿真或真实平台 topic 时, 应优先新增或扩展 profile, 不要直接复制 launch 文件
- 后续代码注释默认使用中文, 保持简洁, 复杂逻辑函数补简短说明
- topic profile 不应再生成 `/tmp` 中间 YAML, 后续新增配置覆盖应走节点参数或 ROS launch parameters

## 文档同步

- 已新增 `2026-07-02-changelog.md`
- 已新增 `2026-07-02-external-commit-reference-analysis.md`
- 已新增 `2026-07-02-graph-builder-layer-boundary-refactor.md`
- 已新增 `graph_construction/docs/AGENT_README.md`
- 已新增 `2026-07-02-startup-script-consolidation.md`
- 已新增 `2026-07-02-topic-profile-configuration.md`

## 后续待办

- 后续若要增强诊断, 优先参考 `wildos_sync_tuner.py` 实现当前项目 topic 专用同步调参工具
- 后续若要排查 elevation/GridMap 偏移, 优先补 GridMap rolling buffer 和坐标映射单元测试
- 后续若要优化 graph construction 性能, 优先增加 stage timing, 再评估 `validate_frontier_paths` 类开关
- 后续进入执行闭环阶段时, 再参考 `goal_pose_to_nav2.py` 的 look-ahead goal 限频, 过滤和 stale cancel 逻辑
- 后续为 `SparseGraphBuilder` 增加不依赖 ROS 的单元测试
