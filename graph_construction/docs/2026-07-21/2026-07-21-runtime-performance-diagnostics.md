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
