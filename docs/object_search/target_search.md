# 目标搜索与探索路线

## 1. 这个模块做什么

任务开始时只有文本目标, 没有目标坐标

系统需要先探索, 看到目标后切换到目标观察和接近, 最后完成近距离确认

本模块回答:

1. 还没看到目标时往哪里走
2. 看到目标后何时停止探索
3. 粗目标、稳定目标和最终完成如何切换

目标坐标如何计算见 [目标定位与视觉雷达融合](../visual_navigation/target_localization.md)

## 2. 模块分工

| 模块 | 职责 |
|---|---|
| WildOS | 给 Frontier 打分、检测目标、生成近距离视觉证据 |
| `object_target_fusion` | 计算目标三维位置和稳定性 |
| `ObjectSearchGoalMux` | 选择当前高层目标和任务状态 |
| `graphnav_planner` | 在 scored graph 上计算路径 |
| 外部控制器 | 执行 Path 和底层避障 |

Goal Mux 是高层目标和最终完成状态的唯一 owner

Goal Mux 通过强类型 `ObjectSearchStatus` 把状态和 `pending_protection` 发送给 Planner. `/spot1/object_search_status` 字符串只在迁移期提供给外部监控

## 3. 运行时更改搜索目标

配置文件中的 `object_search_config.text_queries` 是容器启动时的默认目标

实机运行期间通过 `/spot1/object_search_target` 更改目标, 不需要修改配置或重启容器:

```bash
ros2 topic pub --once \
  /spot1/object_search_target \
  std_msgs/msg/String \
  "{data: 'red fire extinguisher'}"
```

目标必须是非空文本, 连续空白会被压缩, 重复目标不会重复计算文本特征

切换目标时系统会同步清理:

- WildOS 的旧文本特征、检测确认窗口和 Frontier 评分
- 目标融合的旧粒子、旧点云缓存和目标 Marker
- Goal Mux 的旧目标、完成锁存和探索方向

切换前产生的旧 Mask 和旧目标估计会按时间戳丢弃, 避免机器人继续追踪上一个目标

Topic 是运行时覆盖, 容器重启后仍使用配置文件中的默认目标

## 4. 状态流程

```mermaid
stateDiagram-v2
    [*] --> WAIT_FOR_ODOM
    WAIT_FOR_ODOM --> STARTUP_OBSERVATION
    STARTUP_OBSERVATION --> SEARCHING_WITH_INITIAL_GOAL
    SEARCHING_WITH_INITIAL_GOAL --> TARGET_PENDING_OBSERVATION: 单视角候选
    TARGET_PENDING_OBSERVATION --> TARGET_PENDING_REPOSITION: 仍可见但证据不足
    TARGET_PENDING_REPOSITION --> TARGET_PENDING_OBSERVATION: 到达横向观察点
    TARGET_PENDING_OBSERVATION --> SEARCHING_WITH_INITIAL_GOAL: 换位后仍无粗目标
    TARGET_PENDING_OBSERVATION --> TARGET_APPROACH_COARSE: 形成两视角粗目标
    TARGET_PENDING_REPOSITION --> TARGET_APPROACH_COARSE: 形成两视角粗目标
    SEARCHING_WITH_INITIAL_GOAL --> TARGET_APPROACH_COARSE: 粗目标通过门控
    TARGET_APPROACH_COARSE --> TARGET_OBSERVATION: 到达观察位置
    TARGET_OBSERVATION --> TARGET_APPROACH_COARSE: 更换观察点
    SEARCHING_WITH_INITIAL_GOAL --> TARGET_APPROACH_METRIC: 直接获得稳定目标
    TARGET_APPROACH_COARSE --> TARGET_APPROACH_METRIC: 目标变稳定
    TARGET_OBSERVATION --> TARGET_APPROACH_METRIC: 目标变稳定
    TARGET_APPROACH_METRIC --> TARGET_FINAL_OBSERVATION: 到达安全观察距离
    TARGET_FINAL_OBSERVATION --> TARGET_FINAL_REPOSITION: 证据仍不足
    TARGET_FINAL_REPOSITION --> TARGET_FINAL_OBSERVATION: 到达新观察点
    TARGET_FINAL_OBSERVATION --> TARGET_REACHED_VIEWPOINT: 完成门控通过
```

| 状态 | 通俗说明 |
|---|---|
| `WAIT_FOR_ODOM` | 等待机器人位置 |
| `STARTUP_OBSERVATION` | 等地图和视觉评分稳定 |
| `SEARCHING_WITH_INITIAL_GOAL` | 按初始方向探索 |
| `TARGET_PENDING_OBSERVATION` | 保持位置并面向单视角候选 |
| `TARGET_PENDING_REPOSITION` | 只按射线切向横移, 获取第二观察位置 |
| `TARGET_APPROACH_COARSE` | 接近粗目标的安全观察位置 |
| `TARGET_OBSERVATION` | 面向粗目标观察或换位 |
| `TARGET_APPROACH_METRIC` | 接近稳定目标外的安全观察点 |
| `TARGET_FINAL_OBSERVATION` | 面向稳定目标做最终确认 |
| `TARGET_FINAL_REPOSITION` | 横向更换最终观察点 |
| `TARGET_REACHED_VIEWPOINT` | 完成门控通过, 停止任务 |

## 5. 启动观察

启动后不会立即发送远距离路线, 也不会默认旋转 360 度

Goal Mux 先等待:

- 预热约 3 s
- 连续有效导航图
- 连续有效 scored graph
- 图消息没有过期
- 前方存在可规划节点和 Frontier

前方可规划时直接开始探索

只有连续多帧确认前方不足时, 才左右小角度观察并回到初始朝向

启动观察期间不累计探索失败时间

## 6. 初始方向如何工作

Goal Mux 根据第一帧 odom 和配置方向生成远距离虚拟 goal

这个 goal 只用于告诉 Planner 主要探索方向, 不是必须踩到的固定坐标

机器人前进后, 虚拟 goal 会沿同一方向继续向前移动, 避免旧 goal 落到身后导致掉头

Planner 对每个可达 Frontier 综合考虑:

- 路线是否连通
- 路线长度和可通行代价
- 是否重复经过旧边
- 沿初始方向前进多少
- 需要回退多少
- Frontier 视觉分数

virtual goal 只参与候选排序, 不会被追加成穿过 unknown 的执行路径

## 7. 探索状态机

底层仍使用 Dijkstra 计算当前位置到选定节点的最短路径

状态机只决定当前探索哪个方向, 以及什么时候允许回头

```mermaid
stateDiagram-v2
    [*] --> FOLLOW_BRANCH
    FOLLOW_BRANCH --> CHECK_DEAD_END: 路径持续失效或无进展
    CHECK_DEAD_END --> FOLLOW_BRANCH: 原方向重新出现
    CHECK_DEAD_END --> CHOOSE_BRANCH: 连续观察后确认死路
    CHOOSE_BRANCH --> BACKTRACK: 选择最近岔路的未探索方向
    BACKTRACK --> FOLLOW_BRANCH: 接上新方向的实时 Frontier
    CHOOSE_BRANCH --> EXPLORATION_EXHAUSTED: 没有未探索方向
    EXPLORATION_EXHAUSTED --> CHOOSE_BRANCH: 出现新的可达 Frontier
```

| 状态 | 行为 |
|---|---|
| `FOLLOW_BRANCH` | 沿当前方向持续前进 |
| `CHECK_DEAD_END` | 停止换路, 等待连续有效地图确认 |
| `CHOOSE_BRANCH` | 从岔路记忆中选择未探索方向 |
| `BACKTRACK` | Dijkstra 沿历史安全图返回岔路并进入新方向 |
| `EXPLORATION_EXHAUSTED` | 没有已知可达分支, 等待新地图 |

## 8. 路线和岔路记忆

### ActiveBranch

当前正在执行的走廊

只要路线仍有效并持续有进展, Planner 优先延伸它, 不因附近 Frontier 分数轻微变化立即换路

每次发布新路线后至少保持 2.5 s

以下情况不需要等待保持时间:

- 已提交路径连续确认失效
- Goal Mux 切换到粗目标或稳定目标
- 正在恢复的历史分支到达入口后需要接上实时 Frontier

Frontier 位置移动不足 0.75 m 时继续执行原路线, 等累计移动达到门槛后再更新

### JunctionRecord

保存发现侧向分支时的图节点和世界位置

岔路按发现顺序压入栈中, 死路后优先处理最近发现的岔路

### BranchRecord

保存岔路处尚未探索的大方向

方向按 8 个扇区归并, 身份由“岔路 UUID + 方向扇区”组成

同一方向内 Frontier UUID 变化只更新实时入口, 不创建新的长期分支

这部分已经拆成独立的 `ExplorationBranchMemory`:

- 同一岔路可以保存多个未探索方向
- 同方向移动的 Frontier 只更新原记录
- 嵌套岔路先处理最近进入的岔路
- 同一岔路先处理最近发现的方向

候选生成、Dijkstra 和路径输出仍留在 Planner, 因为它们需要直接读取同一份实时图

### FailedBranch

已经确认失败的走廊

相同位置和方向的失败记录会合并并进入冷却, 避免新 UUID 绕过失败记忆

## 9. 如何判断路线有进展

系统检查真实执行路线, 不只看世界坐标某一个方向:

- 是否进入下一段路径
- 已走路径弧长是否增加
- 是否更靠近下一个路点
- 实际移动是否朝向下一段路线

任意一项明显改善都会刷新进展时间

这样允许正常绕障和短暂侧移

## 10. 如何限制普通回头

首次选择路线时允许最多 2.0 m 的局部绕行, 避免狭窄区域因第一段短暂向侧后方而无路可走

路线开始执行后, 每次普通延伸使用更严格规则:

- 相对当前路线最多新增 1.25 m 回退
- Frontier 相对机器人最多落后 0.5 m
- 必须属于当前路线的有序后继, 不能只共享路口附近公共路径

这三个条件只限制普通 continuation

只有状态明确进入 `BACKTRACK` 后才允许明显回头

进入新分支后, 方向基准会切换为岔路处的局部方向, 不再持续使用最初 odom 方向限制整张图

## 11. 什么时候释放路线

### 无进展

新路线先有 20 s 启动宽限, 之后连续 12 s 没有进展才释放

### 路径失效

只检查机器人尚未走过的路线段

连接边至少连续 3 帧、持续 1.5 s 消失后才确认路径失效

### 输入异常

以下情况会冻结失败计时:

- graph 过期
- odom 过期
- 目标候选正在保护
- D-LIO 定位正在恢复

## 12. 死路恢复

路线确认失败后:

1. 记录失败走廊并开始冷却
2. 进入 `CHECK_DEAD_END`, 等待约 3 s 和连续有效地图
3. 进入 `CHOOSE_BRANCH`, 从岔路栈由近到远查找未探索方向
4. 进入 `BACKTRACK`, 使用 Dijkstra 生成安全回退路线
5. 接上新方向的实时 Frontier 后回到 `FOLLOW_BRANCH`
6. 没有未探索方向时进入 `EXPLORATION_EXHAUSTED`

普通 Frontier 不能获得恢复权限

短暂空图只暂停规划, 不会清空活动分支和岔路记忆

## 13. 目标出现后如何切换

### PENDING

第一帧有效 Mask 只有方向, 深度仍不确定

系统短暂保护这份证据, 暂停远距离探索并冻结失败计时, 但不会导航到单视角粒子位置

融合节点发布 Mask 质心射线方向, Goal Mux 丢弃不可靠的距离, 在当前位置面向目标约 1.5 s

如果目标仍在持续更新但没有形成两视角粗目标, 机器人沿射线切向横移 0.6 m, 到新位置后再观察一次

横移只执行一次, 使用 0.3 m 到达半径, 不需要也不使用单视角目标距离

目标失去新鲜证据, 或换位后仍未形成粗目标时, 恢复原探索分支

### 粗目标

粗目标至少需要 2 个独立视角和 0.45 置信度, 同时通过 frame、距离、高度和误差范围检查

机器人先到目标外约 2.75 m 的观察位置:

1. 面向目标静止约 2 s
2. 目标丢失时左右约 20 度重捕获
3. 仍不稳定时横向移动约 0.75 m
4. 到新位置后继续面向目标观察

目标抢占前, Planner 会暂存当前方向、活动分支、岔路栈和失败冷却记录

粗目标 3 s 没有更新时恢复这些探索记录, 不把目标观察误记成死路或已完成分支

### 稳定目标

`STABLE_VISION` 或 `LIDAR_LOCKED` 可以进入稳定目标接近

小于 0.3 m 的目标位置变化通常不会反复移动 goal

稳定目标不再把物体坐标直接当作落脚点

机器人先到目标外约 1.75 m 的安全观察点, 然后:

1. 面向目标静止确认约 2 s
2. 目标丢失时只左右约 15 度重捕获
3. 证据不足时横向移动约 0.9 m
4. 到新观察点后继续确认

## 14. 最终完成

任务完成必须同时满足:

- 当前有稳定融合目标
- 机器人与稳定目标距离不超过 2 m
- WildOS 连续近距离视觉证据仍新鲜, 或融合状态为新鲜 `LIDAR_LOCKED`

门控通过后:

1. Goal Mux 锁定完成状态
2. 发布当前位置停止目标
3. 发布 `/spot1/object_search_completed=true`
4. 融合状态进入 `REACHED`

只到达估计坐标附近不能直接宣布完成

## 15. 关键参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `frontier_progress_start_grace` | 20 s | 新路线起步宽限 |
| `frontier_progress_timeout` | 12 s | 无进展释放时间 |
| `directional_min_forward_progress` | 0.5 m | 前向候选最低进展 |
| `directional_max_initial_backtrack` | 2.0 m | 初始路线最大回退 |
| `directional_max_continuation_backtrack` | 1.25 m | 普通延伸允许新增的最大回退 |
| `directional_min_continuation_progress` | -0.5 m | 普通延伸终点允许落后机器人的范围 |
| `frontier_route_hold_duration` | 2.5 s | 路线更新后的最短保持时间 |
| `frontier_min_update_distance` | 0.75 m | Frontier 小变化合并距离 |
| `frontier_failure_cooldown` | 60 s | 失败走廊基础冷却 |
| `branch_recovery_cost_penalty` | 3.0 | 历史分支恢复附加代价 |
| `path_invalid_confirm_duration` | 1.5 s | 路径失效持续时间 |
| `path_invalid_confirm_frames` | 3 | 路径失效连续帧数 |
| `recovery_observe_duration` | 3.0 s | 恢复前观察时间 |
| `goal_radius` | 3.0 m | 普通非目标搜索 goal 半径 |
| `coarse_goal_radius` | 0.75 m | 粗目标观察位姿到达半径 |
| `metric_goal_radius` | 0.5 m | 稳定目标安全观察位姿到达半径 |
| `final_reposition_goal_radius` | 0.35 m | 最终观察换位到达半径 |
| `pending_reposition_goal_radius` | 0.3 m | 单视角横向换位到达半径 |
| `pending_reposition_distance` | 0.6 m | 单视角最多一次的横向移动距离 |
| `final_observation_distance` | 1.75 m | 稳定目标外的观察距离 |
| `object_reached_max_target_distance` | 2.0 m | 最终完成允许的目标距离 |

## 16. 低频诊断日志

每 30 s 输出一次探索路线统计:

```text
探索路线统计, 分支变化=..., 正常延伸=..., 小变化保持=...,
负向延伸拒绝=..., 恢复=历史.../方向.../死路...,
释放=路径失效.../无进展..., 状态切换=...,
安全节点回退=..., 无路线周期=...
```

判断方式:

- `小变化保持` 增加且 `分支变化` 下降, 说明抑制 Frontier 抖动有效
- `负向延伸拒绝` 很高且经常无路, 说明 1.25 m 可能过严
- `状态切换` 和状态原因用于确认回头前是否进入 `BACKTRACK`
- `路径失效` 或 `无进展` 持续很高, 需要继续检查地图或路线质量
- `安全节点回退` 增加说明 Frontier owner 暂不可达但仍有前向安全路径
- `无路线周期` 持续增加说明当前分量没有可执行 Frontier 或安全前向节点

## 17. Planner 代码结构

Planner 保留一份公共 `Planner` 状态, 实现按职责拆分为多个编译单元:

| 文件 | 职责 |
|---|---|
| `graphnav_planner/include/graphnav_planner/planner.hpp` | Planner 公共接口、状态和参数 |
| `graphnav_planner/src/planner.cpp` | 路径公共工具、图更新、已走边记忆和未探索距离图 |
| `graphnav_planner/src/planner_exploration.cpp` | 活动路线生命周期、进展判断、失败冷却、分支关系和抢占恢复 |
| `graphnav_planner/src/planner_planning.cpp` | Frontier 候选生成、Dijkstra、方向门控、死路恢复选择和最终路线输出 |
| `graphnav_planner/src/planner_visualization.cpp` | Frontier 分数和活动分支 Marker |
| `graphnav_planner/src/exploration_memory.cpp` | 岔路栈和未探索方向记忆 |
| `graphnav_planner/src/planner_node.cpp` | ROS topic、TF、Goal Mux 状态适配、Path 发布和性能诊断 |
| `graphnav_planner/src/planner_internal.hpp` | 仅供 Planner 编译单元共享的路径工具声明 |

`plan_to_goal` 的处理顺序:

```text
当前图、目标和机器人位置
  -> 生成移动的方向目标或保留真实目标
  -> 收集 Frontier 分数和目标半径内节点
  -> 在不含虚拟目标的持久图上运行一次 Dijkstra
  -> 为每个可达 Frontier 补充路线、前进量、回退量和恢复信息
  -> 更新活动路线进展并确认路径是否持续失效
  -> 按当前探索状态选择普通延伸、死路回退或历史分支
  -> 没有 Frontier 时选择可达的安全前向节点
  -> 更新提交路线并返回机器人当前位置后的可执行后缀
```

关键边界:

- Dijkstra 只计算当前节点到所有可达节点的最短路径
- 探索状态机只决定使用哪个候选和何时允许回头
- `ExplorationBranchMemory` 只维护岔路和方向顺序, 不读取实时图
- Goal Mux 仍是目标搜索状态 owner, Planner 只暂停或恢复自己的探索事务
- `PlannerNode` 只在路线变化、停止或观察姿态变化时发布 Path

测试按行为拆分:

| 文件 | 覆盖内容 |
|---|---|
| `test/planner_test_utils.hpp` | 共享节点、边、导航图和 Planner 构造工具 |
| `test/test_exploration_preemption.cpp` | 目标抢占和探索恢复 |
| `test/test_path_publication.cpp` | 相同路线不重复发布 |
| `test/test_directional_selection.cpp` | 初始方向、前向阻塞和安全节点兜底 |
| `test/test_committed_branch.cpp` | 路线进展、延伸、防抖、失效和失败冷却 |
| `test/test_exploration_recovery.cpp` | 死路确认、回退、耗尽和显式重置 |
| `test/test_direct_goal.cpp` | 真实目标半径内的直接路线 |
| `test/test_exploration_memory.cpp` | 多方向岔路、Frontier 合并和嵌套恢复顺序 |

其他入口:

- `visual_navigation/visual_navigation/object_search_goal_mux.py`
- `visual_navigation/configs/object_search_goal_mux.yaml`
- `graphnav_planner/config/planner.yaml`
- `visual_navigation/visual_navigation/wildos/nav.py`
