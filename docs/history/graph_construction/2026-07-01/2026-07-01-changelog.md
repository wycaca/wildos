# Graph Construction 变更记录

日期: 2026-07-01

## 已完成

- 新增 `2026-07-01-elevation-map-range-offset-analysis.md`, 记录 LiDAR 点云和 elevation GridMap 对比诊断
- 明确当前问题应拆分为高程图局部窗口, elevation backend 过滤, frame 统一, 点云轴转换, GridMap cell 坐标约定几个独立检查项
- 明确本次只完成源码和截图分析, 不修改运行代码
- 记录当前文档维护规则, 后续变更必须区分变更记录文档和具体说明文档
- 变更记录文档只写已完成事项, 实测结果, 文档同步和后续待办摘要
- 具体说明文档负责写背景, 目标, 代码链路, 问题分析, 可能原因, 验证步骤和修改建议
- 曾按运行时数据试验 `pointcloud_axis_adapter` 默认 `isaac_lidar_to_base` 轴转换
- 曾将 `isaac_lidar_to_base` 从旧的 `(-x, -y, -z)` 改为 `(y, -x, z)`, 后因高程图竖向伪影回退
- 保留旧转换为 `negate_xyz`, 便于必要时回退对比
- 曾将 elevation backend 默认 `map_length` 从 `30.0` 放大到 `80.0`, 后因高程图竖向伪影回退
- 曾将 elevation backend 默认 `max_ray_length` 从 `20.0` 放大到 `70.0`, 后因高程图竖向伪影回退
- 曾将 elevation backend 默认 `max_height_range` 从 `2.0` 放大到 `4.0`, 后因高程图竖向伪影回退
- 更新 elevation 相关 launch 参数说明, `pointcloud_axis_mode` 不再描述为固定 XYZ 转换

## 实测结果

- 当前 FastDDS 环境下只发现 `/spot1/odom_for_scoring`, 未发现 `/elevation_mapping_node/elevation_map_raw`, `/unitree_go2/lidar/points`, `/unitree_go2/lidar/points_aligned`
- `/spot1/odom_for_scoring` 当前 header frame 为 `odom`
- `/spot1/odom_for_scoring` 当前 position 约为 `(18.144, 41.778, 0.042)`
- 因高程图 topic 当前不在线, 本次无法直接读取截图时刻的 GridMap `header.frame_id`, `info.length_x`, `info.length_y`, `info.pose`
- 后续运行时复查时, `/elevation_mapping_node/elevation_map_raw` 已在线, header frame 为 `odom`
- `/elevation_mapping_node/elevation_map_raw` 当前 `resolution=0.2`, `length_x=30.0`, `length_y=30.0`
- `/elevation_mapping_node/elevation_map_raw` 当前 pose position 约为 `(17.987, 41.733, 0.000)`
- `/unitree_go2/lidar/points` 当前 header frame 为 `lidar`, xyz min 约为 `(-7.801, 1.565, -1.552)`, xyz max 约为 `(73.890, 154.710, 10.022)`
- `/unitree_go2/lidar/points_aligned` 当前 header frame 为 `base_link`, xyz min 约为 `(0.875, -11.211, -4.824)`, xyz max 约为 `(27.318, 1.434, 0.544)`
- `odom -> base_link` TF 可用
- `odom -> lidar` TF 不可用
- 当前运行中的 elevation node 仍是旧配置, 需要重启 launch 后才会应用本次源码和配置修改
- `python3 -m py_compile` 已通过 `pointcloud_axis_adapter.py` 和 elevation 相关 launch 文件
- 在 ROS 环境下曾验证试验模式 `isaac_lidar_to_base(1, 2, 3) = (2, -1, 3)`, 后已回退默认模式
- 在 ROS 环境下验证回退后 `isaac_lidar_to_base(1, 2, 3) = (-1, -2, -3)`, `isaac_y_forward_to_base(1, 2, 3) = (2, -1, 3)`
- 已执行 `colcon build --packages-select graph_construction --symlink-install`, build 成功
- 曾确认 install tree 中的 `elevation_mapping_sim.yaml` 使用 `map_length: 80.0`, `max_ray_length: 70.0`, `max_height_range: 4.0`, 后已重新 build 同步回退参数
- 已确认 install tree 中的 `elevation_mapping_sim.yaml` 使用 `map_length: 30.0`, `max_ray_length: 20.0`, `max_height_range: 2.0`
- 使用 `ROS_LOG_DIR=/tmp/ros_launch_show_args` 验证 `ros2 launch graph_construction elevation_visual_navigation_sim.launch.py --show-args` 加载成功
- launch 默认仍为 `/unitree_go2/lidar/points -> /unitree_go2/lidar/points_aligned`, `publish_lidar_static_tf=false`, `fastdds_profile=''`
- 已确认脚本默认 `/home/ks-server3/han/wildos_ws` 真实路径为 `/mnt/hhd/han/wildos_ws`
- 修复 `fastdds_profile=''` 时 launch 把空字符串传给 `IfCondition` 导致启动失败的问题
- `elevation_visual_navigation_sim.launch.py` 改用 `LaunchConfigurationNotEquals("fastdds_profile", "")` 判断是否注入 FastDDS profile 环境变量
- 已重新 build `graph_construction`
- 已验证 `bash scripts/start_elevation_visual_navigation.sh --show-args` 成功, 不再出现 `invalid condition expression`
- 修复 `scripts/start_elevation_visual_navigation.sh` 默认导出不存在 FastDDS XML 导致的 `XMLPARSER Error`
- `start_elevation_visual_navigation.sh` 现在只在 FastDDS profile 文件存在时设置 `FASTDDS_DEFAULT_PROFILES_FILE` 和 `FASTRTPS_DEFAULT_PROFILES_FILE`
- 如果用户 shell 中已有坏的 `FASTRTPS_DEFAULT_PROFILES_FILE`, 脚本会改用已验证存在的 FastDDS profile 或清空相关变量
- `elevation_visual_navigation_sim.launch.py` 的 `fastdds_profile` 默认值改为空, 只有显式传入非空值时才注入 FastDDS profile 环境变量
- 已重新 build `graph_construction`, 并确认 `ros2 launch ... --show-args` 中 `fastdds_profile` 默认值为空
- 使用坏的 `FASTDDS_DEFAULT_PROFILES_FILE` 和 `FASTRTPS_DEFAULT_PROFILES_FILE` 运行 `scripts/start_elevation_visual_navigation.sh --show-args`, 未再出现 `XMLPARSER Error`
- 复查 `elevation_mapping_cupy` 官方文档后, 已回退导致高程图竖向杂乱的参数放大和轴向试验
- `elevation_mapping_sim.yaml` 恢复 `map_length: 30.0`, `max_ray_length: 20.0`, `max_height_range: 2.0`
- `pointcloud_axis_adapter` 默认 `isaac_lidar_to_base` 曾恢复为 `(-x, -y, -z)`, 后经质量复查改为 `(-x, -y, z)`
- 保留 `(y, -x, z)` 为显式诊断模式 `isaac_y_forward_to_base` 和 `y_forward`
- 复查高程图质量差问题, 发现运行中 aligned 点云精确执行 `(-x, -y, -z)`
- 当前 raw 点云高处结构为正 Z, 全 XYZ 翻转后会变成大范围负 Z 深坑
- `/elevation_mapping_node/elevation_map_raw` 实测 elevation 最小值约为 `-36.5m`, 与 Z 翻转后的高处结构一致
- 将 `pointcloud_axis_adapter` 默认 `isaac_lidar_to_base` 改为 `(-x, -y, z)`, 保留旧全翻转为 `negate_xyz`
- 新增 `neg_xy_keep_z` 作为 `(-x, -y, z)` 的显式别名
- 已验证 `isaac_lidar_to_base(1, 2, 3) = (-1, -2, 3)`, `negate_xyz(1, 2, 3) = (-1, -2, -3)`
- 已重新 build `graph_construction`, 需要重启 launch 才会替换当前运行中的 adapter 进程

## 诊断结论摘要

- 高程图范围小的直接配置原因是 `elevation_mapping_sim.yaml` 使用 `map_length: 30.0`, `max_ray_length: 20.0`, 它不是显示 RTX LiDAR 全量 70m 点云的全局地图
- 高程图相比原始点云仍显得过小, 还可能来自 `max_height_range`, traversability / elevation finite 过滤, 以及高程后端只融合可解释为地表的点
- 偏移和镜像风险主要来自 frame 不统一, 当前源码配置仍以 `odom` 为主, 但目标文档要求当前仿真统一为 `map`
- `pointcloud_axis_adapter` 默认把点云 frame 改成 `base_link`, 这仍可能绕过真实 LiDAR 外参并导致偏移或镜像
- GridMap row / column 到 world 坐标的转换仍是必须复查项, 如果和 upstream `grid_map_core` 约定不一致, 会出现整体转置, 镜像或平移
- 运行时没有 `odom -> lidar` TF, 因此短期仍需要 aligned 点云以 `base_link` frame 输入 elevation mapping
- aligned 点云旧轴转换后范围明显被压缩, 且 Z 翻转会制造深坑, 是高程图质量差的主要修正点
- 官方文档中 `max_height_range` 用于过滤高于传感器的点以关闭天花板, `max_ray_length` 用于可见性清理射线长度, 不应把它们当作扩大显示范围的主手段
- 新截图的彩色竖向幕布与错误轴向加长射线的组合一致, 核心问题是点云坐标语义不可信时又允许远距离射线参与高程融合
- 当前最小修复先恢复保守参数和已知较稳定的默认轴向, 真正长期方案应补齐 `base_link -> lidar` TF 并让 elevation mapping 使用真实传感器 frame
- 当前高程图质量差的直接原因是 Z 轴翻转, 不是 map_length 不够

## 文档同步

- 已新增 `2026-07-01-changelog.md`
- 已新增 `2026-07-01-elevation-map-range-offset-analysis.md`
- 已补充官方 `elevation_mapping_cupy` 参数语义和本次回退结论

## 后续待办

- 重启 elevation launch, 确认 `/unitree_go2/lidar/points_aligned` 使用恢复后的默认轴转换
- 重启后确认 `/elevation_mapping_node/elevation_map_raw` 的 `length_x` 和 `length_y` 仍为 `30.0`
- 重启后重新截图确认竖向彩色幕布消失
- 如果高程图仍偏移, 下一步检查 GridMap row / column 到 world 坐标映射
- 如果高程图仍过小, 下一步统计 elevation 和 traversability finite cell 分布, 不优先继续拉长 ray tracing
- 重启脚本后确认不再出现 `XMLPARSER Error realpath failed`
