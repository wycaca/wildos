# 高程图启动盲区修复

## 问题

Unity 使用 `/livox/lidar_aligned` 建立高程图时，Livox 近场盲区会让机器人初始脚下缺少观测。虽然配置已经设置 `use_initializer_at_start: true`，当前 ROS 2 节点只读取 initializer 参数，没有调用 `ElevationMap.initialize_map`，因此这些参数此前没有实际生效。

另一个参数读取错误会把 `initialize_frame_id` 字符串数组按单字符串读取。当前配置写的是 `["base_link"]`，节点实际得到空字符串，后续即使补调用也无法正确查询初始化 TF。

## 修复设计

采用“源头初始化 + 图层保守兜底”的两层方案:

```text
第一帧 /livox/lidar_aligned
  -> 查询同时间戳的 odom -> base_link
  -> rolling map 移动到初始 base 位姿
  -> base_link 展开为 1.0m 方形的 4 个地面锚点
  -> 锚点高度减去 0.22m
  -> linear 插值并执行 5 cell 初始化膨胀
  -> 标记初始化完成
  -> 融合第一帧点云
```

初始化没有完成时不接收点云。该顺序是必要约束，因为 `ElevationMap.initialize_map` 内部先清空地图；如果先融合第一帧点云再初始化，首帧观测会被清除。

`SparseGraphBuilder` 现有的 `robot_blind_zone_radius` 内 unknown 修补继续保留。高程图 initializer 负责启动源头地面，图构建修补负责 TF 短暂不可用、滚动窗口更新和分类残留等异常场景，不能删除任一层。

## 文件级修改

### `elevation_mapping_cupy/elevation_mapping_cupy/initializer_utils.py`

- 新增纯函数 `build_initialization_points`
- 单初始化 TF 展开为 4 个方形锚点
- 4 个及以上足端 TF 直接作为锚点
- 明确拒绝含义不清且无法稳定插值的 2 或 3 个 TF 配置

### `elevation_mapping_cupy/scripts/elevation_mapping_node.py`

- 正确读取 `initialize_frame_id` 字符串数组
- 第一帧点云处理前执行一次 `_ensure_map_initialized`
- 初始化前更新 map center，避免世界坐标和局部高程基准不一致
- 初始化 TF 不可用时跳过点云并等待下一帧
- 读取并应用 `dilation_size_initialize`
- 加载持久地图后关闭启动初始化，避免下一帧点云清空已加载地图
- 输出初始化锚点数和平均地面高度日志
- 退出时只在 ROS context 仍有效时调用 `rclpy.shutdown`，避免正常 Ctrl-C 产生重复 shutdown 错误栈

### `elevation_mapping_cupy/test/test_initializer_utils.py`

- 覆盖单 `base_link` 的 1.0m 方形展开和 `-0.22m` 高度偏移
- 覆盖 4 个足端 TF 的逐点高度偏移
- 覆盖 2 个 TF 配置的显式拒绝

### `elevation_mapping_cupy/CMakeLists.txt`

- 将 initializer 单元测试加入 `colcon test`
- 安装测试资源时排除 `__pycache__` 和 `*.pyc`

### `graph_construction/configs/elevation_mapping_sim.yaml`

- 明确使用 `linear` 初始化
- 将 `initialize_tf_offset` 改为 `[-0.22]`
- 明确设置 `dilation_size_initialize: 5`
- 保持 `initialize_tf_grid_size: 1.0`

### 长期文档

- `graph_construction/docs/details/principles.md` 补充启动初始化顺序和双层职责
- `graph_construction/docs/AGENT_README.md` 增加不可回退的不变量、参数和测试清单

## 验证要求

1. `test_initializer_utils` 全部通过
2. `elevation_mapping_cupy` 和 `graph_construction` 从工作空间根目录编译通过
3. Unity 完整启动后出现一次 `高程图启动初始化完成` 日志
4. 日志平均高度应接近初始地面，不得接近机器人机身高度
5. 第一张有效 GridMap 的机器人脚下不再是大面积 unknown
6. 首个 graph 仍需检查脚下修补统计和首次 `initial_forward_route`，确认源头初始化没有放宽 obstacle 规则

## 本次运行验证

使用正式命令:

```bash
./scripts/start_wildos_elevation.sh do_object_search:=true
```

已确认:

- 从 `/mnt/hhd/han/wildos_ws` 编译 `elevation_mapping_cupy` 和 `graph_construction` 通过
- `graph_construction` 62 项测试通过，`elevation_mapping_cupy` 16 项单元测试通过
- 原有 ROS 集成测试因测试进程缺少 `ros2_numpy` 跳过 5 项，正式启动脚本下节点可正常加载该依赖
- initializer 只执行一次，日志为 `地面锚点=4, 平均高度=-0.018m`
- 第一帧导航图为 47 节点、79 边、9 个 frontier
- `脚下盲区修补=0, 盲区状态=known_ground, 地面来源=map`
- 机器人 odom 高度约 `0.202m`，初始化后的地面高度约 `0.062m`，地面投影正常
- `/spot1/nav_graph` 在启动后持续更新
- 最终增量编译后再次启动，initializer 结果保持为 `-0.018m`，`elevation_mapping_node` 在 Ctrl-C 后正常退出且不再出现重复 shutdown 错误

本轮没有得到首次 `initial_forward_route` 运行证据。运行期间 `/spot1/scored_nav_graph` 没有实际消息，planner 因而没有收到可评分导航图；原始导航图和高程图均持续发布，该现象不属于 initializer 执行失败，需在视觉评分链路恢复发布后继续完成路线验收。
