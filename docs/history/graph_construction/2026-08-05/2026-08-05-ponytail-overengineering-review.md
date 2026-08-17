# Graph Construction Ponytail Review

评审范围: `graph_construction/graph_construction/`, `graph_construction/launch/`, `graph_construction/configs/`, `graph_construction/test/`

评审依据: `docs/AGENT_README.md`, `docs/details/overview.md`, `docs/details/principles.md`, `docs/details/graph_update.md`, `docs/details/topics.md`, `docs/details/environment.md`, `docs/details/dlio.md`

`launch/elevation_visual_navigation_sim.launch.py:L103-205`, `launch/elevation_visual_navigation_sim.launch.py:L275-379`, `launch/elevation_visual_navigation_sim.launch.py:L905-921`, `launch/elevation_visual_navigation_sim.launch.py:L1069-1165`, `graph_construction/topic_profiles.py:L12-102`, `configs/topic_profiles.yaml:L27-41`: yagni: 同一个值同时存在于模块 YAML, topic profile, profile 对应 launch argument 和运行时 override 四层配置面, `PROFILE_KEY_DESCRIPTIONS` 也只服务于这层镜像, 与“平台接线放 profile, 算法参数放所属模块配置”的项目原则冲突. 保留 `topic_profile` 和 `topic_profile_file`, profile 只保存 topic, frame, RMW 和传感器接线, 算法值直接读取已有模块 YAML, 临时实验使用自定义 YAML, 约减少 350 行.

`graph_construction/pointcloud_axis_adapter.py:L69-79`, `launch/elevation_visual_navigation_sim.launch.py:L1045-1066`: delete: 当前三个 profile 只使用 `identity`, `isaac_lidar_to_base` 和 `x_forward_y_left`, 其余 5 个点云轴向别名及 2 套相机 TF convention 只出现在历史文档. 删除历史回退模式, 需要旧坐标环境时通过独立 profile 和实测外参重新加入, 约减少 16 行.

`graph_construction/dlio_tf_adapter.py:L383-496`, `graph_construction/dlio_tf_adapter.py:L520-545`: shrink: 手写四元数归一化, 乘法, 向量旋转, 逆变换, 旋转角和 yaw 提取, 仓库其他模块已经使用已安装的 `scipy.spatial.transform.Rotation`. 用 `Rotation.from_quat`, `apply`, `inv`, `magnitude` 和 `as_euler` 替换, 保留 `RigidTransform` 消息边界, 约减少 60 行.

`graph_construction/graph_memory.py:L362-395`, `graph_construction/graph_builder.py:L1356-1386`: yagni: `GraphState.nearest_node` 的全图扫描和 `max_distance` 分支没有第二个调用方, 唯一调用方已经传入半径索引结果, `_nearest_free_neighbor.excluded` 也从未传值. 收窄为必传候选集合并删除未使用的 `excluded` 分支, 约减少 20 行.

`graph_construction/graph_memory.py:L144-166`, `graph_construction/grid_types.py:L317-336`: shrink: 两处维护同一套 Bresenham 栅格线实现. `EdgeSpatialIndex` 复用现有 `bresenham_line`, 不再维护第二份算法, 约减少 18 行.

`graph_construction/pipeline_performance_monitor.py:L32-57`, `graph_construction/msg_utils.py:L23-39`: delete: 性能监控用 `TOPICS` 和 `TOPIC_LABELS` 两个平行结构保存同一组 key, 消息缓存的 `_node_ids` 只写不读. 把 label 放入 `TOPICS` 单一表并删除 `_node_ids`, 约减少 12 行.

`net: -476 lines possible.`

未列为多余设计: 持久节点和边空间索引, 增量 pair 更新, 启动盲区修补, DLIO 健康门控, 消息缓存, RViz 订阅门控, 硬件和时序校准参数, 这些都有当前主链调用方或 details 文档中的明确约束

## 执行结果

- 已删除逐项 topic 和 frame launch argument, 保留启动脚本使用的 ROS domain 和 RMW 运维覆盖
- 已删除历史点云轴向和相机 TF convention
- 已使用 SciPy `Rotation` 替换手写四元数运算
- 已收窄单调用方查询接口并复用 Bresenham 实现
- 已合并性能监控 topic 元数据并删除只写缓存字段
- 保留 topic profile 中随传感器和场景变化的视觉阈值, 这些属于硬件调优参数

`implemented net: -298 production lines, -277 total lines including tests and README.`
