# 2026-07-06 changelog

## 已完成

- 新增 `2026-07-06-object-search-exploration-design.md`, 设计目标不在视野时的 object search 探索链路
- 对照 WildOS 论文, 明确 object search 应先通过 scored frontier 安全探索, 目标进入视野后再切换目标引导
- 对照社区实现, 提取 `initial_goal_mux`, `obj_mask_triangulation`, `triangulation3d` 和 `graphnav_planner` 的可借鉴部分
- 明确阶段一优先实现 `object_search_goal_mux`, 阶段二再接入三角定位目标和 particle hypotheses
- 更新 `AGENT_README.md`, 记录 object search 下一步优先实现 goal mux
- 将社区参考仓库 clone 到 `external_references/nebula2-wildos-main_ws`, 当前 commit `225be74`
- 新增 `2026-07-06-external-reference-repo.md`, 记录参考仓库路径, 忽略规则和更新方式
- `.gitignore` 新增 `external_references/`, 避免外部参考仓库进入主项目提交
- 新增 `visual_navigation/visual_navigation/object_search_goal_mux.py`, 实现目标搜索初始 goal 和目标 frontier goal 切换
- 新增 `visual_navigation/configs/object_search_goal_mux.yaml`, 记录 mux 默认参数
- `visual_navigation/setup.py` 新增 `object_search_goal_mux` console script
- 2D / 3D launch 在 `do_object_search=true` 时启动 `object_search_goal_mux`
- `topic_profiles.yaml` 新增三套 profile 的 object search 初始目标和 timeout 参数
- 新增 `2026-07-06-object-search-goal-mux-implementation.md`, 记录实现链路和验证方式
- 新增 `external_references/COLCON_IGNORE`, 避免外部参考仓库被 colcon 扫描成重复包
- 调整 `.gitignore`, 继续忽略外部参考仓库内容, 但保留 `COLCON_IGNORE`
- 2D / 3D 启动脚本增加 `do_object_search=true` 预检, 缺少 `object_search_goal_mux` 时提前给中文构建提示
- 修复 `object_search_goal_mux` 启动参数类型, launch 侧将距离, 角度, timeout 转成 float, 将 latch 开关转成 bool
- 拆分 `/goal_pose` 和 path follower 输出, `/goal_pose` 只保留高层 planner goal, path follower 输出改为 `/spot1/tracking_goal_pose`
- `object_search_goal_mux` 新增 `/spot1/object_search_goal_viz` MarkerArray, 用于 RViz2 显示高层搜索目标
- 降低 WildOS 高频日志, `Received callback` 和 `TF found for camera frames` 不再每帧 INFO 输出, `Started Heavy` 改为 DEBUG

## 实测结果

- 已确认本地已有 `triangulation3d` 包和 `obj_mask_triangulation` 入口
- 已确认本地 `graphnav_planner` 已支持 goal pose + scored frontier 的探索式规划
- 已确认 `external_references/nebula2-wildos-main_ws` 远端为 `https://github.com/TIKTOKDAD/nebula2-wildos-main_ws.git`
- 已确认参考仓库浅克隆完成, 当前 commit `225be74`, 目录大小约 `338M`
- 已通过 Python 编译检查新增节点和 2D / 3D launch 文件
- 已通过 YAML 解析检查 `topic_profiles.yaml` 和 `object_search_goal_mux.yaml`
- 已通过 `bash -n` 检查 2D / 3D 启动脚本
- 首次构建失败原因为 `external_references/nebula2-wildos-main_ws` 中存在同名 `visual_navigation`, 已用 `COLCON_IGNORE` 排除
- 已执行 `colcon build --packages-select visual_navigation graph_construction --symlink-install`, 构建通过
- 已确认 `/mnt/hhd/han/wildos_ws/install/visual_navigation/lib/visual_navigation/object_search_goal_mux` 存在
- 已确认 `ros2 pkg executables visual_navigation` 能看到 `object_search_goal_mux`
- 已复现并定位 `initial_goal_distance` 字符串传入导致的 rclpy 参数类型错误
- 已用 `ros2 run visual_navigation object_search_goal_mux` 短时验证参数类型修复, 节点可启动并进入 `WAIT_FOR_SUBSCRIBER`
- 已用 `ros2 launch graph_construction wildos_2d_sim.launch.py topic_profile:=unity do_object_search:=true` 短时验证 profile 参数链路, mux 可启动并进入 `WAIT_FOR_ODOM`
- 已根据 `/goal_pose` 的 publisher 列表确认 topic 混用问题, 后续需验证 `/goal_pose` publisher count 应为 1
- 已用 Unity profile 的 `ROS_DOMAIN_ID=89` 和 `rmw_zenoh_cpp` 短时验证 topic 拆分, `/tmp_search_goal_pose` 只有 `object_search_goal_mux` 一个 publisher 和 `graphnav_planner` 一个 subscriber
- 已验证 path follower 输出已拆到 `/tmp_tracking_goal_pose`, publisher 为 `graphnav_path_follower`
- 已验证搜索目标可视化 `/tmp_search_goal_viz` 为 `visualization_msgs/MarkerArray`, publisher 为 `object_search_goal_mux`
- 已通过 Python 编译检查 WildOS 高频日志降噪改动
- 尚未完整运行到 `/goal_pose` 和 `/path2` 规划闭环, 需要用户再次启动确认

## 诊断结论摘要

- 当前缺口不是目标识别, 而是目标未出现时缺少 planner goal 来源
- 当前缺口也不是 graph planner 主逻辑, 本地 planner 已能把 scored frontier 接到 virtual goal
- 最小闭环已经通过 `object_search_goal_mux` 接到 `/goal_pose`, 让机器人在目标不在视野时开始探索
- 社区代码后续固定从 `external_references/nebula2-wildos-main_ws` 对比, 不再依赖 `/tmp/wildos_ref`
- 当前 `object_search_goal_mux` 只做阶段一, 目标三角定位和 particle hypotheses 仍是后续工作
- 本次启动失败不是模型加载问题, `torch KeyboardInterrupt` 是 launch 在找不到 mux 后主动中断 WildOS 的连带结果
- 外部参考仓库必须保持 `COLCON_IGNORE`, 否则会污染当前 workspace 构建
- mux 参数类型必须在 launch 侧转换, 不能依赖节点初始化后的解析逻辑
- `/goal_pose` 是 planner 高层输入, 不应再承载 path follower 的低层 waypoint
- WildOS 同步回调和 TF 成功匹配属于高频正常状态, 默认不应每帧 INFO 输出

## 后续待办

- 验证目标不在视野时 `/goal_pose`, `/path2`, `/spot1/scored_nav_graph` 和 `/spot1/score_rings`
- 第二阶段接入 `obj_mask_triangulation` 和 `/spot1/object_hypotheses`
- 评估 `path_follower_node` 的低层 waypoint topic 是否应与高层 `/goal_pose` 分离
