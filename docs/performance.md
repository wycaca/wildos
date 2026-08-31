# 性能边界与 C++ 迁移决策

性能优化遵循先隔离观察面、再测量核心、最后决定语言迁移的顺序. 核心节点只记录单调时钟、计数器和固定容量窗口, 每 30 或 60 秒向 `/diagnostics` 发布标量. `pipeline_performance_monitor` 和 `wildos_visualization` 是独立进程, 停止它们不会改变核心 Topic、Path、速度或停车逻辑

性能 monitor 每 5 秒在自身进程读取 `/proc`、thermal sysfs 和 `nvidia-smi`, 汇总主机 CPU、内存、最高温度、GPU 利用率/显存/温度, 以及核心进程 CPU 和 RSS. 它不订阅图像、PointCloud2、GridMap 或 NavigationGraph. 当前主机单次资源采样中位耗时 16.543 ms、最大 20.388 ms, 折算约 3.31 ms/s, 全部发生在独立观察进程

## 可重复合成基线

[performance_baseline.yaml](../scripts/performance_baseline.yaml) 固定随机种子、输入规模、预热次数、重复次数和周期预算. [benchmark_performance.py](../scripts/benchmark_performance.py) 使用真实 PointCloud2 解码、当前 NumPy/OpenCV 数据路径和当前图构建器, 输出包含环境、配置 SHA256、P50/P95/最大值、阶段耗时、输出契约和迁移门槛的 JSON

```bash
source /opt/ros/humble/setup.bash
source /mnt/hhd/han/wildos_ws/install/setup.bash
export PYTHONPATH=graph_construction:wildos_navigation:${PYTHONPATH:-}
.venv/bin/python scripts/benchmark_performance.py --fail-on-budget
```

默认报告写入 `/tmp/wildos-performance-baseline/report.json`. `--quick` 只用于开发期冒烟测试, 不能作为迁移结论

当前 x86 主机连续运行三次生产规模配置, 取三次报告的中位结果:

| 场景 | 固定输入 | P50 | P95 | 周期预算 | P95 占预算 |
| --- | --- | ---: | ---: | ---: | ---: |
| `map_pub` | 200000 点, 101×101 栅格 | 44.786 ms | 50.262 ms | 100 ms | 50.3% |
| controller 点云 | 200000 点, 最终 35 候选 | 19.002 ms | 22.059 ms | 100 ms | 22.1% |
| controller 控制 | 清晰双点路径, 50 Hz | 0.056 ms | 0.083 ms | 20 ms | 0.4% |
| 图首次构建 | 160×160, 内部 Frontier 和墙体 | 97.820 ms | 110.268 ms | 500 ms | 22.1% |
| 图滚动增量 | 原点滚动和障碍切换 | 127.498 ms | 138.614 ms | 500 ms | 27.7% |

固定输出同时验证地图 10201 个 cell、controller 最多 35 个障碍候选、清晰路径速度 0.6 m/s, 以及图节点、边、Frontier 和 dirty cell 非空. 图采样是最重阶段, 但首次和增量图构建都低于 2 Hz 周期预算

## C++ 决策

C++ 迁移必须同时满足两个条件: Python 优化后核心 P95 超过周期预算, 且单一阶段占总 P95 至少 30%. 当前五个场景都未触发第一道门槛, 因此不迁移 `map_pub`、controller 或 Python 图核心

- PointCloud2 已直接解码为结构化 NumPy 数组, 过滤、投影、聚合和形态学处理在 NumPy/OpenCV 编译实现中运行
- controller 的二次复杂度路径固定限制为 35 个候选
- 图距离场和邻域查询复用 SciPy `ndimage` 与 `cKDTree`, 当前主要采样阶段仍有充足周期余量
- 视觉推理运行在 PyTorch/CUDA, 评分、融合和消息处理已使用张量或 NumPy 批处理
- Planner 本身已经是 C++

为语言统一重写会增加 ROS 构建、等价验证和双实现维护成本, 当前没有可证明的性能收益. 真实 GO2 rosbag 若出现连续 P95 超预算, 先用报告中的阶段 P95 确认热点占比, 再只迁移命中门槛的完整模块

## 尚缺实机基线

仓库当前没有 rosbag, 合成报告不能替代实机四模式验收. 仍需录制并版本化包含三相机、注册点云、odom、TF、GridMap 和目标搜索的固定 rosbag, 再分别运行核心、核心加埋点、核心加 monitor、核心加 visualization 四种模式, 补齐 CPU、GPU、RSS、显存、温度、跨机带宽和长时间队列增长. 在这些数据完成前, 当前结论是“不启动 C++ 迁移”, 不是永久禁止迁移
