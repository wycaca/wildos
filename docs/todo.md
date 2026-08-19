# WildOS 代码整改 TODO

> 状态: 第一轮人工审核完成, 全部整改项已批准
>
> 本文只定义修改方案和验收条件. 只有标记为已批准的整改项才能进入代码修改

## 1. 使用方式

每个整改项使用以下状态:

| 状态 | 含义 |
| --- | --- |
| 待审核 | 方案尚未批准, 不能修改代码 |
| 已批准 | 修改范围、兼容性和验收条件已确认 |
| 实现中 | 只修改已批准范围 |
| 待验证 | 代码完成, 等待测试和实机验证 |
| 已完成 | 测试、文档和实机验收均通过 |

人工审核每个整改项时需要确认:

- 风险判断是否成立
- 修改边界是否足够小
- Topic、frame、消息和参数是否需要兼容旧版本
- 故障时应该停止、保持还是降级
- 测试是否覆盖正常路径、异常路径和恢复路径
- 是否有明确回滚方式

## 2. 建议实施顺序

| 批次 | 内容 | 开始条件 |
| --- | --- | --- |
| 1 | P0 地图安全问题 | 方案和地图语义审核通过 |
| 2 | P1 定位、TF、Planner 和目标融合问题 | 对应故障策略审核通过 |
| 3 | 其他安全与部署风险 | 兼容性和部署影响审核通过 |
| 4 | 低风险性能优化 | 已获得修改前性能基线 |
| 5 | `ObjectSearchGoalMux` 拆分 | 行为测试补齐且全部通过 |
| 6 | 可维护性、CI 和文档 | 测试入口和依赖边界审核通过 |

每个批次单独提交、测试和评审. 不把安全修复、格式化和架构重构放入同一个提交

## 3. 整改项总览

| ID | 优先级 | 模块 | 修改内容 | 状态 |
| --- | --- | --- | --- | --- |
| GC-01 | P0 | `graph_construction` | obstacle 永远不能被后处理改为 free | 待验证 |
| GC-02 | P1 | `graph_construction` | 删除按帧分位数决定安全分类 | 待验证 |
| LOC-01 | P1 | D-LIO adapter 和 guard | 为健康状态增加心跳和超时租约 | 待验证 |
| VIS-01 | P1 | WildOS TF | 实机禁止 latest TF 回退 | 待验证 |
| FUS-01 | P1 | `triangulation3d` | LiDAR 锁定前必须关联视觉轨迹 | 待验证 |
| PLN-01 | P1 | `graphnav_planner` | 过期输入禁止继续规划 | 待验证 |
| GC-03 | 其他 | 点云 relay | 禁止只修改 `frame_id` 冒充坐标变换 | 待验证 |
| SEC-01 | 其他 | 模型加载 | 限制不安全 checkpoint 反序列化 | 待验证 |
| SEC-02 | 其他 | Docker 和 DDS | 缩小容器和局域网攻击面 | 待验证 |
| PERF-01 | 性能 | 跨机点云 | 在 x86 发送前完成限频 | 待验证 |
| PERF-02 | 性能 | 导航图链路 | 避免重复复制、重建和规划相同图 | 待验证 |
| PERF-03 | 性能 | 目标融合 | 复用同一帧 LiDAR 处理结果 | 已批准 |
| PERF-04 | 性能 | WildOS 推理 | 使用 inference mode 并验证收益 | 已批准 |
| ARCH-01 | 架构 | Goal Mux | 拆分 ROS adapter 和纯状态策略 | 待验证 |
| ARCH-02 | 架构 | Goal Mux 与 Planner | 用强类型消息替代字符串协议 | 待验证 |
| MNT-01 | 可维护性 | ROS package | 补齐 package 依赖 | 待验证 |
| MNT-02 | 可维护性 | 测试和 CI | 建立统一测试入口 | 待验证 |
| MNT-03 | 可维护性 | C++ | 保持 C++ 代码格式统一 | 已批准 |
| MNT-04 | 可维护性 | Planner | 删除自带 path follower | 已批准 |
| MNT-05 | 可维护性 | 文档和模型 | 补齐运行基线、模型来源和校验信息 | 已批准 |

## 4. `graph_construction`

### GC-01 obstacle 后处理安全, P0

问题:

- majority fill 会把满足邻居数量的任何非 free 单元改成 free
- hole fill 把 unknown 和 obstacle 放在同一集合中, 最后统一改成 free
- 当前实机配置同时启用了两种处理

涉及文件:

- `graph_construction/graph_construction/grid_adapter.py`
- `graph_construction/graph_construction/node.py`
- `graph_construction/configs/graph_construction_elevation.yaml`
- `graph_construction/test/test_grid_map_adapter.py`

修改方案:

1. majority fill 只允许 unknown 变成 free
2. hole fill 的目标集合只保留 unknown
3. obstacle 在所有后处理阶段保持不可变
4. 如果上游 obstacle 存在噪声, 单独修复上游分类或传感器模型, 不能直接提升为 free
5. 保留现有 free 小洞的 elevation 填充能力, 但不能借此改变 obstacle 分类

人工审核点:

- 确认 elevation mapping 输出中 obstacle 的准确语义
- 确认是否存在必须过滤的 obstacle 噪点, 以及应该在哪一层处理
- 确认历史安全先验仍然服从 obstacle 优先原则

验收条件:

- 单点 obstacle、连续 obstacle 和封闭 obstacle 均不会变成 free
- unknown 小洞仍可按配置修复
- 现有图构建测试全部通过
- 使用代表性 rosbag 对比修改前后的 free、obstacle、unknown 数量和图连通性

### GC-02 固定 traversability 安全语义, P1

问题:

- 当前每帧使用 5% 和 95% 分位数把 traversability 拉伸到 `0..1`
- 同一个原始数值会随当前画面分布变化而改变分类
- 整体低质量地图仍可能产生大量 free 单元

涉及文件:

- `graph_construction/graph_construction/grid_adapter.py`
- `graph_construction/graph_construction/node.py`
- `graph_construction/configs/graph_construction_elevation.yaml`
- `graph_construction/test/test_grid_map_adapter.py`

修改方案:

1. 先用实机 rosbag 记录 traversability 的正常、障碍和异常范围
2. 取消实机路径中的按帧分位数归一化
3. 使用上游定义的绝对范围和固定阈值分类
4. 输入范围异常或置信度不足时分类为 unknown 并告警
5. 不新增自适应阈值, 除非固定阈值在实机数据上被证明无法工作

人工审核点:

- 确认上游层的数值方向、范围和无效值定义
- 确认 free 和 obstacle 阈值的实机标定数据
- 确认是否需要为仿真保留单独 profile

验收条件:

- 整体低分输入不会因为相对排序产生 free 区域
- 改变背景分布不会改变固定单元的分类
- 阈值边界、NaN、initializer variance 均有测试
- 实机静止和移动场景不会产生明显分类闪烁

### GC-03 点云 frame 契约, 其他风险

问题:

`pointcloud_relay` 在输入和输出 frame 不同时只修改消息头, 没有转换点坐标

修改方案:

1. 当前 identity relay 始终保留输入 `frame_id`
2. 将现有 `output_frame` 参数删除, 或改成只用于校验的 `expected_frame`
3. frame 不匹配时丢弃消息并限频告警
4. 如果未来确实需要转换, 使用独立且有测量时刻 TF 测试的转换节点

已确认决策:

- `robot` profile 要求 `/cloud_registered` 始终为 `dlio_odom`
- 当前仅保留实机 profile, 不保留 frame 重写配置

验收条件:

- identity 路径不复制高密度点云
- frame 不一致时不会发布错误坐标点云
- 点字段、时间戳和输入消息保持不变

## 5. D-LIO 定位输出

### LOC-01 健康状态心跳和租约, P1

问题:

- adapter 只在健康状态变化时发布 `Bool`
- guard 保存最后一次状态, 没有超时
- adapter 卡死在最后一次 `healthy=true` 后, guard 仍会继续转发点云

涉及文件:

- `graph_construction/graph_construction/dlio_tf_adapter.py`
- `graph_construction/graph_construction/dlio_output_guard.py`
- `graph_construction/test/test_dlio_tf_adapter.py`
- `graph_construction/test/test_dlio_output_guard.py`

修改方案:

1. adapter 以固定低频周期发布当前健康状态, 建议 2 Hz
2. guard 保存最后一次健康消息的 monotonic 时间
3. 新增明确的 `health_timeout_sec`, 建议初值 1.5 秒
4. 未收到状态、收到 false 或租约过期时停止转发
5. 恢复必须收到新的连续健康心跳, 不能依赖 transient local 中的旧 true
6. 在周期诊断中输出最后心跳年龄和超时次数

人工审核点:

- 确认心跳频率和超时是否适合当前 D-LIO 频率
- 确认 adapter 重启和 DDS 短暂抖动时的恢复要求
- 确认 false 是否需要立即覆盖恢复滤波

验收条件:

- adapter 停止后, guard 在超时内停止转发
- false 立即停止转发
- 持续健康心跳不会误超时
- adapter 恢复后按既有健康滤波规则恢复输出

## 6. WildOS TF 和视觉推理

### VIS-01 禁止实机 latest TF 回退, P1

问题:

WildOS 默认在 TF 外推失败时使用 latest TF, 与测量时刻 TF 契约冲突

涉及文件:

- `visual_navigation/visual_navigation/utils/tf_lookup_sub.py`
- `visual_navigation/configs/wildos_nav_conf.yaml`
- `visual_navigation/test/test_tf_lookup_sub.py`

修改方案:

1. 默认值改为 false
2. robot profile 显式设置 false
3. 测量时刻 TF 不可用时丢弃当前输入并计数
4. 仿真如果必须回退, 由仿真配置显式开启
5. 仿真回退需要校验 latest TF 与测量时间的最大差值

人工审核点:

- 确认实机启动阶段可接受的 TF 等待和丢帧行为
- 确认仿真是否仍需要 latest fallback

验收条件:

- robot profile 不会调用 `Time()` 查询替代测量时刻
- 过去和未来外推错误均不会产生错误评分或目标证据
- TF 恢复后处理自动恢复

### PERF-04 推理模式, 性能

修改方案:

1. 记录修改前 inference 平均、P95、最大耗时和显存
2. 使用 `torch.inference_mode()` 包裹图像模型主前向过程
3. 保留当前 FP16 配置, 不同时引入 TensorRT、ONNX 或新推理后端
4. 如果收益低于测量误差, 可以不保留该修改

当前实现说明:

- `ExploRFMInference` 在图像和文本前向入口统一启用 `torch.inference_mode()`
- 保留原有 FP16/FP32 配置, 未引入新推理后端
- WildOS 周期日志增加推理平均、P95、最大耗时和 CUDA 显存统计
- 软件验证通过: 推理模式和确定性输出测试 2 passed
- 状态保持待验证, 需要 Orin 实测确认耗时与显存没有退化

验收条件:

- 模型输出在允许误差内保持一致
- 无显存持续增长
- 修改后性能数据优于基线或至少没有退化

## 7. 目标融合和 `triangulation3d`

### FUS-01 LiDAR 与视觉轨迹关联, P1

问题:

连续 LiDAR 测量只检查彼此是否一致, 没有检查是否与已有视觉轨迹一致

涉及文件:

- `triangulation3d/triangulation3d/target_particle_filter.py`
- `triangulation3d/test/test_target_particle_filter.py`
- `visual_navigation/visual_navigation/object_target_fusion.py`

修改方案:

1. 已有视觉轨迹时, 在增加 LiDAR 连续帧计数前计算关联距离
2. 复用现有 `association_min_radius` 和 `association_sigma_factor`
3. LiDAR 超出视觉后验关联半径时拒绝测量并重置连续计数
4. 只有通过关联的连续测量才能更新粒子权重和进入 `LIDAR_LOCKED`
5. 没有视觉轨迹时保留现有 LiDAR 初始化行为, 但单独测试其锁定条件
6. 增加拒绝原因和计数, 不逐帧刷日志

人工审核点:

- 确认关联使用 XY 距离还是三维距离
- 确认视觉协方差很大时允许的最大关联半径
- 确认无视觉轨迹的 LiDAR-only 目标是否允许最终完成任务

验收条件:

- 两帧彼此一致但远离视觉目标的 LiDAR 不能锁定
- 合理距离内的连续 LiDAR 可以锁定
- 一帧异常 LiDAR 会重置计数, 后续需要重新累计
- Goal Mux 不会把错误的 `LIDAR_LOCKED` 当作完成证据

### PERF-03 复用 LiDAR 处理结果, 性能

修改方案:

1. 以点云 stamp 和 frame 为键, 只缓存最近一帧解码并转换到世界坐标的点
2. 同一帧点云被多个 mask 使用时, 复用解码和 TF 结果
3. 新点云或 frame 变化时覆盖缓存, 不引入通用缓存类
4. marker 和 particle topic 没有订阅者时不构造消息
5. 分别记录 decode、transform、project 和 publish 耗时

当前实现说明:

- 缓存键使用点云 `stamp.sec`、`stamp.nanosec` 和 `frame_id`, 只保存最近一帧
- TF 暂不可用时保留解码结果但不缓存失败, 后续 Mask 允许重试 TF
- marker 和 particle topic 无订阅者时直接返回, 不构造调试消息
- 软件验证通过: `test_object_target_fusion.py` 20 passed
- 状态保持待验证, 还需在目标平台记录缓存命中率和各阶段耗时

验收条件:

- 同一 LiDAR stamp 只执行一次点云解码和世界坐标转换
- 缓存不会跨 frame 或跨点云 stamp 复用
- debug topic 无订阅者时不创建 particle cloud
- 目标估计结果与修改前一致

## 8. `graphnav_planner`

### PLN-01 过期输入 fail closed, P1

问题:

Planner 检测到 graph 或 odom 过期后仍继续规划, freshness 当前只影响失败计时

涉及文件:

- `graphnav_planner/src/planner_node.cpp`
- `graphnav_planner/test/test_path_publication.cpp`
- `graphnav_planner/test/test_committed_branch.cpp`

修改方案:

1. 在目标转换、距离判断和 `planner_.plan_to_goal` 之前检查 graph 与 odom freshness
2. graph 过期但 odom 新鲜时, 最多发布一次当前位置 hold path
3. odom 过期时不使用旧位姿生成 hold path
4. 仓库外导航必须配置 odom 超时停车和新 Path 时间戳校验, 并在 Planner 文档中记录契约
5. 输入恢复后重新验证当前路线, 不直接沿用异常期间的失败计时
6. 对未来时间戳继续使用现有容差, 超出容差按不健康处理

已确认决策:

- odom 过期时停止发布, 由仓库外导航的 odom 超时保护停车
- Path 是变化事件而不是心跳, 接收时校验时间戳, 恢复时等待新 Path
- graph 过期时只发布一次 hold path, 不周期重发

验收条件:

- stale graph 不会触发新的 Dijkstra 或路线发布, 最多发布一次 hold path
- stale odom 不会生成基于旧位姿的 hold path
- 输入恢复后可以重新规划
- 正常目标到达和观察姿态逻辑不受影响

### PERF-02 避免重复处理完整导航图, 性能

问题:

同一基础图会经过完整消息构造、WildOS 深拷贝、Planner 图重建和 Dijkstra

修改方案:

1. 先记录节点数、边数、消息大小、深拷贝耗时、图重建耗时和规划耗时
2. WildOS 在基础图或 frontier score 实际变化时立即发布, 无变化时只发布 1 Hz freshness 心跳
3. Planner 区分 topology 变化和仅 score 变化
4. topology 未变化时只更新必要属性, 不重建图和 unexplored map
5. goal、current node 或有效 score 没变化时不重复运行 Dijkstra
6. 第一阶段不引入 delta graph 消息, 只有现有门控仍无法满足性能目标时再评审

当前实现说明:

- 不新增 revision 字段, Planner 对上一帧消息做精确 topology 和 properties 比较
- current node 变化视为规划输入变化并触发图重建和重规划
- score 有效变化阈值为 0.001, 可通过 `frontier_score_publish_epsilon` 调整
- 心跳周期为 1.0 秒, 小于 Planner 默认 2.0 秒 graph freshness 租约
- WildOS 记录完整评分图大小和深拷贝耗时, Planner 记录图更新平均和最大耗时
- 软件验证通过: `visual_navigation` 93 passed、1 skipped, `graphnav_planner` 3/3 CTest passed
- 状态保持待验证, 还需用目标平台 rosbag 对比修改前后的 CPU、网络和规划耗时

验收条件:

- 相同 topology 的连续 scored graph 不重建图
- score、goal、current node 和 topology 任一有效变化仍能触发正确规划
- 长时间运行耗时不随重复消息无意义增长

## 9. `ObjectSearchGoalMux` 拆分

### ARCH-01 ROS adapter 与状态策略分离

目标:

降低 1699 行单节点的修改风险, 同时保持 Topic、参数和行为不变

建议边界:

| 文件 | 责任 |
| --- | --- |
| `object_search_goal_mux.py` | ROS 参数、订阅、发布、消息转换、日志和 timer |
| `object_search_goal_policy.py` | 搜索状态、证据时效、状态转换、完成门控和目标选择 |

第一阶段不继续拆分 goal builder、timer manager 或通用状态机框架

当前实现说明:

- 新增不依赖 `rclpy` 的 `ObjectSearchGoalPolicy`, 统一完成锁存、完成触发、目标质量、目标过期和探索回退优先级
- ROS adapter 继续拥有消息缓存、goal 构造和唯一状态字段, policy 不保存重复状态
- 当前时间和目标年龄由 adapter 计算后显式传给 policy
- 纯 policy 测试 4 passed, 原 Goal Mux 行为测试 31 passed
- 第一阶段不搬运 goal builder 和观察动作实现, 避免仅为缩短文件制造代理层

实施步骤:

1. 先为当前状态转换补齐行为测试, 不修改实现
2. 定义最小输入、持久状态和决策结果数据结构
3. 所有时间由 ROS adapter 显式传入 policy, policy 内不调用 ROS clock
4. 将 `_select_goal`、pending/observation/final/reached 转换逐组迁移到 policy
5. ROS callbacks 只更新输入并执行 policy 返回的发布决策
6. 每迁移一组状态就运行全部 Goal Mux 测试
7. 迁移完成后再删除旧方法, 不保留两套状态逻辑

保持不变的接口:

- `/spot1/object_search_target`
- `/spot1/target_estimate`
- `/spot1/object_reached`
- `/goal_pose`
- `/spot1/object_search_completed`
- 所有现有参数名称和默认值

人工审核点:

- 确认哪一个对象拥有 reached latch、目标切换时间和观察阶段
- 确认目标切换时所有状态能够一次性清空
- 确认 policy 是否允许直接依赖 ROS message 类型
- 确认拆分提交不混入行为修改

验收条件:

- 同一输入序列产生相同状态、goal 和 completion 输出
- policy 单元测试不需要初始化 `rclpy`
- ROS adapter 测试只验证订阅、发布和消息转换
- 文件拆分后不存在重复状态字段或双重 owner

### ARCH-02 强类型状态消息

问题:

Goal Mux 使用字符串拼装状态, Planner 使用前缀和子字符串解析

修改方案:

1. 在 `object_search_msgs` 增加 `ObjectSearchStatus.msg`
2. 消息只包含 Planner 实际需要的状态 enum、`pending_protection` 和 header
3. goal 继续使用现有 `PoseStamped` topic, 不在状态消息中重复坐标
4. Goal Mux 日志保留中文状态名称
5. Planner 删除字符串 parser 和硬编码状态文本
6. 确认没有外部消费者后再停止发布旧 String topic

当前实现说明:

- 新增 `ObjectSearchStatus.msg`, 使用固定编号的 11 个状态 enum 和 `UNKNOWN=0`
- Goal Mux 在 `/spot1/object_search_status_v2` 发布强类型消息, 原 String topic 保留一个迁移周期
- Planner 只订阅 v2 topic, 直接读取 enum 和 `pending_protection`, 未知状态 fail closed
- Python 覆盖全部状态到 enum 的唯一映射, C++ 覆盖全部 enum 的规划模式分类
- 软件验证通过: `visual_navigation` 102 passed、1 skipped, `graphnav_planner` 3/3 CTest passed, launch contract 8 passed
- 状态保持待验证, 现场确认外部监控完成迁移后再删除旧 String topic

人工审核点:

- 是否需要一个版本周期同时发布新旧消息
- 外部监控是否依赖当前字符串内容
- enum 状态是否需要稳定编号

验收条件:

- 字段改名可在编译或消息生成阶段发现
- `pending_protection` 不再依赖字符串搜索
- Planner 对全部状态 enum 有测试
- Topic 契约文档同步更新

## 10. 模型、Docker 和 DDS

### SEC-01 checkpoint 安全加载, 其他风险

修改方案:

1. 列出运行时实际需要的 tensor 和非 tensor metadata
2. traversability 和 frontier head 优先使用 `weights_only=True`
3. RADIO checkpoint 如果依赖 Python 对象, 在可信环境一次性转换为 state dict 或 safetensors
4. 为全部部署模型维护文件名、版本、来源和 SHA256 清单
5. `docker/verify_runtime.py` 在启动前校验文件存在性和哈希
6. 校验失败时停止启动, 不自动下载或继续加载

当前实现说明:

- 两个 head checkpoint 使用 `weights_only=True`, 仅 allowlist 其 Lightning optimizer 和 OmegaConf 固定类型
- RADIO checkpoint 依赖 `argparse.Namespace`, 当前保留原格式, 仅允许固定 SHA256 的文件进入反序列化
- SigLIP2 使用固定 revision 的 safetensors, 同时校验配置和 tokenizer 文件
- 模型更新和回滚必须通过 `ckpts/manifest.json` 评审

验收条件:

- 未授权或哈希错误的 checkpoint 不能启动运行时
- 转换前后模型输出在允许误差内一致
- 文档能够追踪每个模型的来源和版本

### SEC-02 容器和 DDS 权限, 其他风险

修改方案:

1. 明确实机局域网是否属于可信网络
2. 使用 VLAN 和主机防火墙只允许两台部署主机通信
3. camera 容器用明确的 USB device 和必要权限替代 `privileged: true`
4. 对不需要写入的挂载保持只读
5. 评估非 root 用户运行相机和 WildOS 的可行性
6. 如果网络包含不可信设备, 启用 DDS Security 身份认证和加密

当前实现说明:

- camera 仅开放 USB major 189, 去除 `privileged` 并丢弃全部 Linux capabilities
- 所有服务启用 `no-new-privileges`, 配置和模型挂载保持只读
- host network 继续用于当前跨机 DDS 发现
- 当前假定两台主机使用可信专用链路, VLAN 和主机防火墙由现场部署执行
- DDS Security 和非 root 运行需要独立实机验证后再启用

验收条件:

- 三相机、GPU、LiDAR 和跨机 DDS 在最小权限下正常工作
- 非授权主机不能向控制相关 Topic 发布数据
- Compose 重启和健康检查行为不退化

### PERF-01 x86 发送前限频, 性能

修改方案:

1. 先确认 `/cloud_registered` 是否只有相机 AGX 消费
2. 在 x86 的 D-LIO guard 发布侧增加可配置限频
3. 保持 D-LIO 内部原始频率不变, 只限制跨机输出
4. AGX relay 保留一次 identity 转发, 不再承担主要带宽控制
5. 对比修改前后的带宽、丢包、点云年龄和 elevation 更新质量

验收条件:

- 跨机点云频率符合下游需求
- 网络带宽明显下降
- elevation mapping 和目标融合没有可见退化

## 11. 可维护性

### MNT-01 补齐 ROS package 依赖

修改方案:

1. 按源码 import 和 CMake `find_package` 对照 package manifest
2. `visual_navigation` 补齐 `geometry_msgs`、`cv_bridge`、`message_filters`、`tf2_msgs` 和实际运行依赖
3. `graphnav_planner` 补齐 `grid_map_msgs` 和 `visualization_msgs`
4. 使用 rosdep 或隔离环境验证, 不能依赖开发机上偶然存在的包

当前实现说明:

- `visual_navigation` 已声明源码直接导入的消息、launch、`cv_bridge`、`message_filters` 和 TF 依赖
- `graphnav_planner` manifest 已与 CMake 对齐, 补充 `grid_map_msgs` 和 `visualization_msgs`
- PyTorch、OmegaConf 和模型依赖继续由根 `pyproject.toml` 锁定, 不在 ROS manifest 重复维护非 ROS Python 环境
- rosdep 定向检查通过, 两个 package 单包构建通过
- 功能验证通过: `visual_navigation` 102 passed、1 skipped, `graphnav_planner` 3/3 CTest passed

验收条件:

- 全新 ROS Humble 环境可按 manifest 安装依赖
- 单包构建和测试不依赖未声明 package

### MNT-02 统一测试入口和 CI

修改方案:

1. 在根文档记录唯一的本地测试命令
2. CI 至少执行 Python 功能测试、C++ GTest 和 package manifest 检查
3. Docker 镜像构建可以继续关闭测试, 但镜像构建前必须有独立测试任务通过
4. 硬件、GPU 和 10 分钟 rosbag 测试单独运行, 不阻塞普通单元测试
5. CI 固定 ROS Humble、Python 3.10 和关键依赖版本

当前实现说明:

- 根入口为 `./scripts/test_repo.sh`, 分阶段执行 manifest、构建、Python 功能、Planner CTest 和 Docker 契约检查
- 按审核决定不执行 flake8 和 pep257, C++ 格式由代码评审保持统一
- CI 使用带 `wildos` 标签的 ROS Humble 自托管 runner 和仓库 `.venv` Python 3.10
- GPU、硬件和 10 分钟 rosbag 测试继续独立运行, 不阻塞普通功能测试
- 统一入口自举通过: graph construction 149、visual navigation 102、triangulation 12 项测试和 Planner 3/3 CTest 全部通过

验收条件:

- 一个命令可以得到完整测试汇总
- CI 失败能明确定位到 package 和测试类型
- 主运行链代码不再只有 `explorfm_trainer` 的独立 CI

### MNT-03 保持 C++ 代码格式统一

修改方案:

1. 以当前主要 C++ 文件的现有风格为准
2. 修改 C++ 时保持括号、缩进、引用和换行风格一致
3. 大范围机械格式化不与功能修改混在同一提交
4. 保留当前英文标点和无句号的代码注释要求

验收条件:

- 同一文件和同一模块不存在明显混用格式
- 格式提交不改变运行行为

### MNT-04 删除自带 path follower

修改方案:

1. 删除 `path_follower_node.cpp`、对应 launch 和 CMake target
2. 删除相关未使用依赖、安装项和文档入口
3. Planner 保持发布 Path, 后续由其他导航算法负责路径执行
4. 不在本仓库新增替代路径跟踪器

验收条件:

- 源码、Compose、launch 和构建产物不再包含 path follower
- Planner 仍只发布 Path
- 删除后 `graphnav_planner` 可以正常构建和测试

### MNT-05 运行和模型文档

需要补充:

- 完整构建、测试和测试结果查看命令
- 10 分钟运行的 CPU、GPU、显存、温度、消息年龄和回调耗时基线
- graph 节点数、边数和消息大小的增长基线
- checkpoint 来源、版本、SHA256 和兼容的代码提交
- 外部控制器的 Path 超时停车契约
- 可信网络假设、端口和主机防火墙要求
- package maintainer 和故障责任人

注释要求:

- 不为简单赋值和明显流程增加注释
- 状态转换、安全降级、时间同步和坐标变换必须说明原因
- 注释使用简洁文本和英文标点, 不使用句号

## 12. 需要补充的具体测试

### 12.1 单元测试

`graph_construction/test/test_grid_map_adapter.py`:

- `test_majority_fill_never_promotes_obstacle`
- `test_enclosed_obstacle_component_is_not_filled`
- `test_unknown_hole_can_still_be_filled`
- `test_low_absolute_traversability_does_not_become_free`
- `test_background_distribution_does_not_change_fixed_cell_class`

`graph_construction/test/test_dlio_output_guard.py`:

- `test_health_lease_expires_and_suppresses_cloud`
- `test_health_heartbeat_extends_lease`
- `test_false_health_suppresses_immediately`
- `test_old_transient_true_cannot_restore_expired_lease`

`visual_navigation/test/test_tf_lookup_sub.py`:

- `test_robot_config_rejects_latest_tf_fallback`
- `test_past_extrapolation_drops_measurement`
- `test_future_extrapolation_drops_measurement`
- `test_sim_fallback_respects_max_time_delta`

`triangulation3d/test/test_target_particle_filter.py`:

- `test_two_consistent_far_lidar_frames_cannot_lock_visual_track`
- `test_associated_lidar_frames_can_lock_visual_track`
- `test_rejected_lidar_resets_consistency_count`
- `test_lidar_only_lock_policy`

`graphnav_planner/test`:

- stale graph 不运行规划也不发布新路线
- stale odom 不使用旧位姿发布 hold
- future graph 或 odom 超出容差时 fail closed
- 输入恢复后重新规划
- topology 不变时不重建图
- score、goal 和 current node 变化时正确重规划

`visual_navigation/test/test_object_search_goal_mux.py`:

- 每个状态的进入条件和退出条件
- target 切换清空全部旧状态
- pending evidence 到期恢复探索
- coarse、metric、stable 和 LiDAR evidence 的优先级
- observation 和 reposition 的次数、超时和恢复
- reached latch 只能由合法证据触发
- stale target estimate 和 stale reached 事件被拒绝
- 同一输入序列在拆分前后产生完全一致的决策

### 12.2 跨节点集成测试

- adapter 发布 true 后停止, guard 必须在租约超时内停止点云
- 测量时刻 TF 缺失时 WildOS 不发布对应评分和目标证据
- target 快速切换时旧 mask、旧 estimate 和旧 reached 不能污染新任务
- stale graph 或 odom 时 Planner 与外部控制器能够安全停车
- debug topic 无订阅者时不构造 marker 和 particle cloud
- 新旧 `ObjectSearchStatus` 迁移期间的兼容行为

### 12.3 rosbag 和实机测试

- 空旷场景、单个细障碍、连续墙体和窄通道的分类结果
- 静止、正常移动、急转弯和短时定位中断
- 三相机时间偏差和 TF 缓存不足
- 正确 LiDAR 目标、背景簇和遮挡后的重新关联
- 连续运行至少 10 分钟, 记录资源、频率、消息年龄和图规模
- 两台主机网络限速、丢包和断连后的恢复

### 12.4 建议验证命令

```bash
source /opt/ros/humble/setup.bash
source /mnt/hhd/han/wildos_ws/install/setup.bash

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest \
  graph_construction/test visual_navigation/test triangulation3d/test \
  -p no:cacheprovider

colcon build \
  --packages-select graphnav_planner \
  --cmake-args -DBUILD_TESTING=ON

colcon test --packages-select graphnav_planner
colcon test-result --verbose
```

硬件测试需要单独记录所用提交、配置、模型哈希、rosbag、开始时间和持续时间

## 13. 开工前审核清单

只有以下项目全部确认后, 对应整改项才能从待审核改为已批准:

- [ ] 问题能够由测试或实机数据复现
- [ ] 修改文件和接口范围已确认
- [ ] 正常、异常和恢复行为已确认
- [ ] 外部控制器和跨主机兼容性已确认
- [ ] 新增测试名称和验收条件已确认
- [ ] 性能修改已保存修改前基线
- [ ] 安全修改已有失败关闭策略
- [ ] 文档和 Topic 契约更新范围已确认
- [ ] 回滚方法已确认
