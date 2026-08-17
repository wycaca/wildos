# 运行日志与性能检测

## 调整目标

- WildOS 在环境变量设置后立即启动并优先加载模型, 不再等待 `visual_start_delay`
- `visual_start_delay` 只延迟目标融合节点
- 正常统计默认每 30 秒输出一次, 目标融合每 60 秒输出一次
- 慢处理只在超过门槛时告警, 同类告警至少间隔 30 秒
- 保留首次就绪、状态变化、任务完成和真实异常日志
- 不逐帧输出性能数据
- DLIO 官方逐帧 dashboard 的标准输出由启动包装器丢弃, 标准错误仍保留

## 新增观测

| 环节 | 日志 | 指标 |
|---|---|---|
| 原始输入到最终路径 | `链路频率`, `链路延迟` | 雷达、IMU、点云、里程计、高程图、导航图、目标和路径的频率与延迟 |
| DLIO TF 适配 | `DLIO 位姿状态` | 输入频率、异常样本、最大误差、回调耗时 |
| DLIO 点云门控 | `DLIO 点云输出` | 转发率、拦截率、健康切换次数、回调耗时 |
| 导航图构建 | `导航图性能` | 地图分类、图更新、消息转换、可视化、总耗时 |
| WildOS | `WildOS 视觉性能` | 图像解码、投影、推理、目标检测、评分、发布、总耗时 |
| 目标融合 | `目标融合运行统计` | Mask 和点云频率、LiDAR 精修率、各阶段耗时 |
| 路径规划 | `路径规划性能` | 规划频率、avg/P95/max、路径变化和空路线次数 |

链路延迟由当前 ROS 时钟减去消息 `header.stamp` 得到, 日志中的三个数字依次是平均值、95% 上限和最大值。仿真必须启用 `/clock`, 真机必须完成系统时钟同步, 否则该值没有跨设备比较意义

## DLIO 健康门控

- 连续 3 帧异常才暂停输出
- 连续 5 帧进入恢复阈值才恢复输出
- 恢复阈值是故障阈值的 80%, 用滞回避免边界抖动
- NaN 或 Inf 立即暂停
- `dlio_tf_adapter` 负责状态变化日志, `dlio_output_guard` 不再重复打印同一事件

## 真机调参入口

- `diagnostics_log_period_sec`: 正常统计周期
- `slow_cycle_warning_ms`: 导航图慢周期门槛
- `slow_processing_warning_ms`: WildOS 慢帧门槛
- `slow_callback_warning_ms`: 目标融合慢回调门槛
- `slow_planning_warning_ms`: planner 慢规划门槛
- `unhealthy_confirm_frames`, `healthy_confirm_frames`, `health_recovery_ratio`: DLIO 健康去抖和滞回

上线前先观察 P95, 再根据真机算力调整慢处理门槛。不要为了减少告警而直接增大健康误差阈值

## 首轮热点优化

2026-07-21 根据运行日志完成以下优化:

- 距离场由 Python 8 连通队列改为 SciPy 精确欧氏距离变换, 保留地图外视为 unknown 的边界语义
- GridMap 和 odom 订阅队列深度改为 1, 每帧 GridMap 只构建一次 graph, 慢周期后直接处理最新地图
- 节点采样使用 KD-Tree 去重, 新边通过 KD-Tree 半径查询且只在当前 GridMap 窗口生成, 窗口外历史边继续保留并接受可见障碍校验
- WildOS 评分图仍逐帧发布, 调试图像和 Marker 默认每 2 秒最多发布一次, 没有订阅者时不构建可视化消息
- 默认关闭仅用于可视化路径回溯的 `compute_paths`, 不改变 frontier 最优分数计算

可视化性能入口:

- `visualization_publish_period_sec`: 调试可视化最小发布间隔, 默认 2 秒, 设置为 0 表示不降频
- `visualization_require_subscribers`: 无订阅者时是否跳过调试消息构建, 默认启用
- `compute_paths`: 是否为模型调试图回溯像素路径, 默认关闭

本地 152 x 152 栅格基准中, obstacle 和 unknown 距离场平均耗时均约 2.6 ms。包含 240 个节点和 588 条边的合成图连续更新约 241 至 250 ms。该结果用于确认优化方向, 真机验收仍以低频性能日志的 P95 为准

验证结果:

- `graph_construction/test` 和 `visual_navigation/test` 功能测试共 119 项通过
- uv `test` 依赖组已加入 `pytest>=7,<9`、`flake8` 和 `pydocstyle`, pytest 上限用于兼容 ROS2 Humble `launch_testing`
- 本次新增和修改的图构建文件通过 flake8, WildOS 修改通过未定义名称检查, 新发布限频模块通过兼容项目注释规则的 pydocstyle 检查
- `visual_navigation` 包级历史基线仍有 412 个 flake8 和 285 个 pydocstyle 问题, 不属于本次性能优化范围
- `graph_construction` 和 `visual_navigation` 使用 `colcon build --symlink-install` 构建通过
