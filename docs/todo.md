# WildOS 当前待办

本文只保留尚未完成项, 完成并通过验证的条目直接删除

## 1. 验证 GO2 脚下盲区修补

当前图构建只在 4.0 m 内修补人工地面, 并拒绝修补机器人与候选栅格之间存在墙体或受保护高程突变的区域. 墙体掩码额外扩展一个栅格, 用于封闭点云离散造成的小缺口

机器狗恢复站立后执行:

```bash
./scripts/wildos_docker.sh orin update wildos
./scripts/wildos_docker.sh orin restart wildos
./scripts/wildos_docker.sh orin logs wildos
```

验收条件:

- 脚下人工地面能连接外围真实地面
- 修补范围不超过 4.0 m
- 墙后栅格保持 unknown 或 obstacle
- 日志中 `盲区连通=True`
- 高程图、节点和边在墙边不越界

## 2. 性能优化与运行面拆分

目标是把核心逻辑、可视化和性能监控拆成互不阻塞的运行面, 再根据基线决定是否迁移 C++. 核心 Topic、frame、时间戳、QoS、安全边界和参数语义保持不变, 监控或可视化退出时不能影响建图、导航、目标搜索和停车

### 2.1 建立可重复性能基线

- 录制并版本化固定 rosbag, 覆盖静止、连续行走、密集点云、滚动地图扩展、障碍更新、Frontier 更新和目标搜索
- 分别记录核心独立运行、核心加轻量埋点、核心加性能监控、核心加可视化四种模式
- 记录各节点输入频率、消息年龄、丢弃数、处理频率、阶段耗时 P50/P95/最大值、CPU、GPU、内存、显存、温度和跨机带宽
- 使用统一基准报告格式保存四种实机模式, 禁止用临时日志作为验收依据
- 只有实机报告出现连续 P95 超预算且单一阶段占比达到 30% 时, 才重新打开 C++ 迁移

验收条件:

- 同一输入连续运行结果可复现, 性能报告能区分核心耗时和观察开销
- 每个核心回调的 P95 小于对应运行周期, 不出现连续周期超时
- 连续运行期间内存、图节点、图边和消息队列不出现与环境面积无关的无界增长

### 2.2 进程和部署隔离

- 核心、observability 和 visualization 使用独立进程, 诊断节点不加入核心 executor
- 核心进程优先使用保留 CPU, observability 和 visualization 使用剩余 CPU 和较低调度权重
- debug Topic 使用 best effort、depth 1, 禁止可靠队列阻塞核心 publisher
- 生产、诊断和可视化模式复用现有 Compose 和 launch 参数组合, 不复制整套启动文件
- 生产入口默认关闭性能监控、可视化和 trace, 诊断入口按需开启且可以单独停止
- 可视化优先运行在开发机, AGX 只在现场无法使用外部主机时承担渲染
- 更新 `compose.orin.wildos-cameras.yaml`、`compose.x86_64.lidar-dlio.yaml`、Docker entrypoint 和启动脚本的资源配置与模式检查

### 2.3 实机测试

- 固定 rosbag 覆盖真实三相机、点云、odom、TF、GridMap 和目标搜索链路
- 比较关闭埋点、开启埋点和启动 monitor 后的核心 P95, 观察开销不得超过性能预算
- 启动 visualization 后单独记录其 CPU、内存和带宽, 核心不得出现持续周期超时
- 高密度点云测试验证无 Python 对象爆发、无队列堆积和无额外跨机订阅
- 长时间运行检查 RSS、显存、图节点、图边、diagnostics 窗口和 DDS 队列是否有无界增长
- monitor、visualizer、RViz 和 trace 分别进行退出、卡顿和重启测试, 核心继续运行
