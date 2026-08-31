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

### 2.2 拆分轻量埋点和性能监控

- 核心进程只使用单调时钟、计数器和固定容量窗口记录耗时、频率、消息年龄、丢弃数和工作量
- 核心进程低频发布标准 `diagnostic_msgs/DiagnosticArray`, 不逐帧写日志、磁盘或构造完整调试快照
- Python 和 C++ 节点使用相同指标名称和单位, 性能面板不能依赖具体实现语言
- 性能监控作为独立进程汇总 diagnostics、进程资源和主机 CPU、GPU、内存、显存、温度
- 性能监控不再订阅图像、完整点云、GridMap、NavigationGraph 等高带宽消息来计算频率和年龄
- 点云带宽和消息年龄在实际生产者、relay 和消费者回调内计数, 避免增加第二个跨机点云订阅者
- `ros2_tracing` 只用于按需快照和专项诊断, 默认不持续写 trace
- 生产模式只启动核心和轻量埋点, 诊断模式显式启动性能监控, 埋点关闭后核心行为必须保持一致

- 更新 Docker 启动参数和部署文档, 不新增职责重复的 launch 或 Compose

### 2.3 把可视化迁出核心回调

- RViz 和 visualization package 默认不在生产模式启动, 可在诊断主机或开发机独立启动
- visualization 进程退出、阻塞或无订阅者时, 核心 Topic 的频率和输出内容保持不变

预计清理范围:

- 复用现有 launch 增加显式 visualization 开关, 不保留新旧可视化双路径

### 2.4 进程和部署隔离

- 核心、observability 和 visualization 使用独立进程, 诊断节点不加入核心 executor
- 核心进程优先使用保留 CPU, observability 和 visualization 使用剩余 CPU 和较低调度权重
- debug Topic 使用 best effort、depth 1, 禁止可靠队列阻塞核心 publisher
- 生产、诊断和可视化模式复用现有 Compose 和 launch 参数组合, 不复制整套启动文件
- 生产入口默认关闭性能监控、可视化和 trace, 诊断入口按需开启且可以单独停止
- 可视化优先运行在开发机, AGX 只在现场无法使用外部主机时承担渲染
- 更新 `compose.orin.wildos-cameras.yaml`、`compose.x86_64.lidar-dlio.yaml`、Docker entrypoint 和启动脚本的资源配置与模式检查

### 2.5 测试设计

固定测试输入:

- PointCloud2 包含空云、NaN、边界点、不同高度、密集重复点、窗口外点和时间戳异常
- GridMap 序列包含 free、unknown、obstacle 转换、滚动窗口、墙边缺口、脚下盲区、断流和乱序时间戳
- NavigationGraph 包含空图、单节点、多分量、Frontier 迁移、节点删除、边删除和大图
- 导航序列包含正常路径、目标切换、路径丢失、点云过期、静态障碍、动态障碍和紧急停车
- 固定 rosbag 覆盖真实三相机、点云、odom、TF、GridMap 和目标搜索链路

单元测试:

- 埋点关闭、开启和降采样时不改变核心返回值, 固定容量窗口不会无界增长
- diagnostics 的指标名称、单位、时间窗口和 reset 行为一致, 非有限时间和时间倒退不会污染统计
- 外部 graph visualizer 对节点、边、Frontier、current node 和删除事件生成正确 Marker
- 点云到栅格测试精确比较 origin、resolution、尺寸、frame、stamp 和每个 cell
- 点云过滤测试覆盖 NaN、高度、距离、边界和最大样本数
- Controller 测试精确检查停车条件和状态切换, 浮点速度使用配置定义的容差
- 图测试检查 obstacle 和 unknown 不生成新节点、新障碍删除冲突节点和边、current node 可达、Frontier 生命周期和持久图不回退

契约和集成测试:

- 生产模式不启动 observability、visualization、RViz 或 tracing, 但保留低频轻量 diagnostics
- 诊断模式只新增轻量指标订阅, 不订阅跨机完整点云和图像
- 可视化模式启动独立 visualizer, 核心 package 不再依赖 `visualization_msgs`
- 任意停止或阻塞 monitor、visualizer 和 RViz, 核心 Topic 频率、内容和停车行为不变
- Python/C++ 替换前后 Topic、类型、frame、stamp、QoS、参数名和 launch owner 一致
- 禁止两个节点同时发布 `/cmd_vel`、`/spot1/nav_graph` 或 `/combined_grid`

等价和回归测试:

- Python 和 C++ `map_pub` 对同一合成点云输出完全一致的 OccupancyGrid
- Python 和 C++ Controller 对同一消息序列产生一致的停车、转向和速度状态, 数值差异在配置容差内
- 图实现对同一 GridMap 和 odom 序列比较节点安全性、边连通性、current node、Frontier 和 Planner 可达结果
- 随机采样使用固定种子, 不要求不同随机库产生相同节点顺序, 但必须满足相同安全和连通契约
- 使用现有 Planner、Goal Mux 和 WildOS 测试验证公开 NavigationGraph 不发生兼容性回归

性能和稳定性测试:

- 基准脚本每种模式至少预热后重复运行, 报告中位数而不是单次最好结果
- 比较关闭埋点、开启埋点和启动 monitor 后的核心 P95, 观察开销不得超过性能预算
- 启动 visualization 后单独记录其 CPU、内存和带宽, 核心不得出现持续周期超时
- 高密度点云测试验证无 Python 对象爆发、无队列堆积和无额外跨机订阅
- 长时间运行检查 RSS、显存、图节点、图边、diagnostics 窗口和 DDS 队列是否有无界增长
- monitor、visualizer、RViz 和 trace 分别进行退出、卡顿和重启测试, 核心继续运行

测试清理:

- hardware launch 测试删除“生产 launch 必须包含旧性能监控节点”的断言, 改为验证生产、诊断和可视化模式边界
- C++ 节点通过等价验收后, 将仍表达公共契约的 Python 测试迁为 gtest 或 launch test, 删除只覆盖已删除私有实现的重复测试
- 每次删除实现时使用仓库搜索确认 helper、依赖、entry point、配置键和测试无调用方, 不保留 skipped 或永久兼容测试

统一验证命令需要覆盖:

- Python 最小相关 pytest
- C++ package 的 `colcon test` 和结果检查
- launch 契约测试
- 固定 rosbag 等价回放
- 四种运行模式性能基准
- `git diff --check` 和未使用依赖检查
