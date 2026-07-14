# Agent README

目的: 帮助开发人员和 agent 快速理解 WildOS 当前项目目标, 运行架构, 模块职责和文件夹结构

## 项目目标

本仓库当前目标是把 WildOS 的开词汇目标搜索系统接入 Isaac, Unity 和真实机器人候选环境

系统核心能力:

- 从局部几何地图生成稀疏 `NavigationGraph`
- 用 graph memory 保留历史节点、edge、探索覆盖和机器人当前位置
- 用三相机视觉模型给 graph frontier 做语义评分
- 在没有目标检测时使用首帧 odom heading 和活动分支锁定持续向前探索
- 在看到目标后选择安全的目标导航点, 并通过 graph planner 输出路径
- 在目标到达或视觉近距离确认后锁存停止状态, 避免继续规划旧路径

系统不负责:

- 训练 ExploRFM 或 RADIO backbone
- 直接发布底盘 `cmd_vel`
- 替代局部控制器或 Nav2
- 把语义目标点当成精确物体坐标

## 当前主链路

默认主线是 elevation/2.5D GridMap 后端, 这和论文及社区参考实现一致:

```text
PointCloud2
  -> pointcloud_axis_adapter
  -> elevation_mapping_cupy
  -> /elevation_mapping_node/elevation_map_raw
  -> graph_construction, GridMap path
  -> /spot1/nav_graph
  -> visual_navigation WildOS scoring
     -> /spot1/scored_nav_graph -> graphnav_planner
     -> /spot1/object_search_target_pose -> object_search_goal_mux -> graphnav_planner goal
  -> path output
```

2D OccupancyGrid 只是 fallback, 用于隔离 `livox_grid_builder`, 对照 `/combined_grid`, 或在 elevation 后端不可用时调试:

```text
PointCloud2 or aligned PointCloud2
  -> livox_grid_builder
  -> /spot1/traversability_grid
  -> graph_construction, OccupancyGrid path
  -> /spot1/nav_graph
```

视觉链路只影响 semantic scoring 和 object search goal selection, 不直接修改 elevation GridMap 或 2D fallback grid

## 当前启动入口

当前仓库里的唯一脚本入口是:

```bash
./scripts/start_wildos_elevation.sh
```

常用启动方式:

```bash
./scripts/start_wildos_elevation.sh
WILDOS_TOPIC_PROFILE=unity ./scripts/start_wildos_elevation.sh do_object_search:=true
WILDOS_TOPIC_PROFILE=robot ./scripts/start_wildos_elevation.sh do_object_search:=true
```

脚本会处理:

- source `/opt/ros/humble/setup.bash`, 可用 `ROS_SETUP` 覆盖
- source workspace `install/setup.bash`, 可用 `INSTALL_SETUP` 覆盖
- 自动激活 `.venv` 或 `wildos_venv`, 可用 `VENV_ACTIVATE` 覆盖
- 设置 `PYTHONNOUSERSITE=1`
- 将仓库根目录加入 `PYTHONPATH`
- 检查 `elevation_mapping_cupy`
- 修正部分已安装 Python entrypoint 的 shebang
- 调用 `graph_construction/launch/elevation_visual_navigation_sim.launch.py`

2D fallback 仍保留 launch 文件, 但当前没有对应启动脚本:

```bash
ros2 launch graph_construction wildos_2d_sim.launch.py topic_profile:=unity
```

不要引用旧的 `scripts/start_wildos_2d.sh` 或 `scripts/start_wildos_3d.sh`, 当前 source tree 中不存在这两个脚本

## Topic Profile

平台差异集中在:

```text
graph_construction/configs/topic_profiles.yaml
```

内置 profile:

- `isaac`, Isaac Sim 5.1 Go2, 默认 `ROS_DOMAIN_ID=3`, raw LiDAR `/unitree_go2/lidar/points`, planner path `/spot1/graphnav_planner/path`, goal `/goal_pose`
- `unity`, Unity 仿真, 默认 `ROS_DOMAIN_ID=89`, RMW `rmw_zenoh_cpp`, elevation 输入 `/livox/lidar`, 2D fallback 输入 `/mapokk`, planner path `/spot1/graphnav_planner/path`, goal `/spot1/graphnav_goal_pose`
- `robot`, 真实机器人占位 profile, 需要按现场 topic 和 TF 更新

Unity 的重要约束:

- elevation 主线使用 raw `/livox/lidar`
- 2D fallback 使用已经对齐到 `odom_3D` 的 `/mapokk`
- Unity 2D fallback 必须保持 `lidar_assume_input_in_grid_frame=true`
- 不要把 raw `/livox/lidar` 直接替换成 2D fallback 输入, 否则会重新引入地图随机器人朝向旋转的问题
- Unity 高层 goal topic 使用 `/spot1/graphnav_goal_pose`, 避免触发外部 `nav_slam/astar` 的公共 `/goal_pose`
- Unity 自研导航直接消费 `/spot1/graphnav_planner/path` 并发布 `/corrected_path`
- 集成 launch 不启动 `path_follower_node`, 避免重复消费自研导航输出

## Graph Construction 架构

`graph_construction` 的职责是把局部几何地图变成稳定稀疏 graph:

```text
node.py
  ROS 配置, topic, QoS, timer, 发布和首帧状态日志

grid_adapter.py
  OccupancyGrid / GridMap 解码为 ClassifiedGrid

grid_types.py
  ClassifiedGrid, 坐标转换, elevation 查询, collision line, distance field

graph_builder.py
  SparseGraphBuilder, 纯算法入口, 不直接依赖 ROS

graph_memory.py
  GraphState, InternalNode, InternalEdge, UUID 和历史记忆

frontier_detector.py
  free / unknown 边界检测和 frontier owner 分配

edge_builder.py
  当前边生成, historical edge validation, robot anchor 连边

msg_utils.py
  GraphState 转 graphnav_msgs/NavigationGraph

viz.py
  GraphState 和 ClassifiedGrid 转 MarkerArray

livox_grid_builder.py
  2D fallback 点云转 OccupancyGrid

pointcloud_axis_adapter.py
  elevation 主线点云轴向和 frame 适配

grid_map_to_occupancy.py
  GridMap debug projection, 不在主链路中使用
```

关键实现原则:

- 纯算法层不导入 `rclpy` 或 ROS message package
- ROS message 解码集中在 `grid_adapter.py`
- ROS message 生成集中在 `msg_utils.py`
- topic, frame, 参数和发布集中在 `node.py`
- GridMap rolling buffer, row / column 映射和 frame 约定必须用测试或运行数据确认

## Graph Construction 参数约定

Graph Construction 参数按职责分为三层:

- `topic_profiles.yaml` 只保存平台相关 topic, frame, namespace 和通信环境
- `node.py` 的 `DEFAULT_CONFIG` 只保存 ROS 适配和地图解码的稳定默认值
- `GraphBuilderConfig` 是纯算法参数的唯一默认值来源
- `graph_construction_elevation.yaml` 和 `graph_construction.yaml` 只保存后端差异及偏离算法默认值的调优项
- 配置文件缺失, 未知字段或非法阈值会在启动时直接报错, 不允许静默回退

不要把同一个默认值同时复制到 Python 和 YAML, 需要调参时只在对应后端 YAML 中覆盖

当前不开放以下历史调试开关:

- GridMap 转置和轴翻转, 当前 rolling buffer 解码已有测试覆盖
- 关闭 GridMap 后处理, 当前主线始终执行拓扑清理
- graph `trav_class`, 当前消息固定使用 `default`
- historical edge validation 和 robot anchor, 当前主线固定开启
- disconnected graph pruning, 持久路线不允许按当前连通分量删除
- node 周期耗时日志参数, 原调用长期关闭且算法层诊断仍由定向测试覆盖
- `robot_namespace` 和 `debug_grid_topic`, Graph Construction 节点从未消费这两个字段

## Graph 行为

当前 graph 默认行为:

- elevation path 直接消费 `/elevation_mapping_node/elevation_map_raw`
- 小型 elevation NaN 洞会在 graph adapter 内为 free cell 补 elevation, 不修改原始 GridMap topic
- 历史节点落入 unknown 或移出当前滚动窗口时继续保留, 只有可靠可见障碍才直接删除节点
- 历史节点重新进入 free 区域时使用自身 cell 的 elevation 更新高度, 不与机器人当前高度比较
- 低于新边 clearance 阈值的历史节点继续保留, 该阈值不再删除路线记忆
- 历史 edge 固定保留, 只用当前可见障碍证伪
- Frontier 只表示当前地图可验证的 free/unknown 边界, 移出滚动窗口后清除活动状态但保留 owner 节点
- 当前可见 Frontier owner 使用世界坐标键稳定继承, 避免同一边界在相邻帧反复换 UUID
- 新 Frontier 落入任一持久节点的 explored radius 时不再创建, 重复探索由稀疏图覆盖状态抑制
- disconnected component 始终保留, current node 短时断边不会清空其他历史路线
- 脚下点云缺失时启用 robot anchor current node, anchor 移动超过节点间距后固化旧位置为 breadcrumb
- graph 数据 z 保持贴近 elevation surface, RViz marker 额外抬高显示
- graph edge 要求 line 和 clearance 都安全, 避免路径贴墙或穿障碍

## Object Search 架构

目标搜索链路:

```text
front / left / right camera
  -> visual_navigation.wildos.nav
  -> /spot1/scored_nav_graph -> graphnav_planner
  -> /spot1/object_search_target_pose -> visual_navigation.object_search_goal_mux
  -> graphnav_planner goal topic
```

当前策略:

- 目标检测必须同时满足相似度峰值, 连通区域面积和连续帧确认, 单像素弱响应不会发布目标点
- 模型可视化第一行会同时叠加 graph 和确认后的 object mask, 不再用 graph 图覆盖 mask
- `object_search_goal_mux` 不再订阅 scored graph 或选择 frontier
- 未检测到目标时, mux 根据首帧 odom 固定探索 heading, planner 沿该 heading 持续前移虚拟目标
- 目标出现时, mux 直接切换到确认后的目标 pose, 短时丢失使用目标记忆
- 视觉 Frontier 分数每帧重建, 相机不可见后不继续发布旧视角分数
- 未选择分支由 planner 保存稳定 owner UUID、位置、方向和发现顺序, 不把历史 Frontier 当成当前候选
- frontier 选择、分支连续性、无进展屏蔽和回头代价统一由 `graphnav_planner` 负责
- Unity 已启用 target latch, 支持短时遮挡后继续朝目标方向规划
- 目标到达或近距离视觉确认后, `object_search_goal_mux` 进入 reached latch 并持续发布当前位置 hold goal
- `graphnav_planner` 到达 goal 半径内时发布当前位置单点 path, 让下游停止

## Planner 架构

`graphnav_planner` 是 C++ graph planner:

- 输入 `graphnav_msgs/NavigationGraph`
- 输入 goal pose topic, 不同 profile 可不同
- 输入 Object Search 状态, 明确区分初始方向探索和真实目标导航
- 使用 graph edge 做搜索, virtual goal 只参与搜索
- 默认不把 virtual goal 或 unknown frontier point 追加进可执行 path
- 使用显式 `ActiveBranch` 保存 Frontier、路径 UUID、局部方向、进度和最后有效高层路径
- Frontier 短暂失配时复用最后有效路径后缀, 不立即切换其他分支
- 未选择分支进入 `DeferredBranch` 记忆, 正常前进时不参与实时 Frontier 排序
- 当前分支持续无进展后释放活动分支并临时屏蔽失败邻域, 优先恢复最早保存的可达分支
- 同一走廊仍有可达候选时禁止按瞬时视觉分数或总代价切换
- 按稳定 UUID edge 二值记录是否经过, 已走边增加固定代价但不会被禁止
- 初始 `30m` 粗目标只作为方向 lookahead, 越过该位置不会停止或反向规划
- planner 通过私有 `~/path` 输出 `/spot1/graphnav_planner/path`
- 集成 launch 只启动 planner, 路径细化和执行由自研导航负责

## TODO 和 Roadmap

### P0, Frontier 生命周期分层, 已完成

- 持久图只保存节点、edge、explored radius 和访问状态
- Frontier 移出当前滚动地图后立即取消活动状态, owner 节点和历史路线继续保留
- `CurrentFrontierScores` 每帧聚合三相机当前证据, 不保留旧视角评分
- 已探索区域通过持久节点 explored radius 阻止重复 Frontier, 不再维护 removed Frontier 坐标集合
- 实施记录见 `docs/2026-07-14/2026-07-14-frontier-lifecycle-and-deferred-branch.md`

### P1, 活动分支锁定, 已完成

- `graphnav_planner` 已增加显式 `ActiveBranch`, 保存路径 UUID、终点 Frontier、局部方向、odom 进度和最后有效路径
- 当前分支暂时失配时复用未执行路径后缀, 不能立即退回全局无记忆选择
- 初始探索 heading 固定为首帧 odom 前方, 有效分支允许沿走廊自然拐弯
- 当前仅以持续无 odom 进展确认分支失败, 失败后临时屏蔽该 Frontier 邻域

### P1, Deferred Branch 记忆, 已完成

- 保存未选择分支的稳定 owner UUID、位置、发现方向和发现顺序
- 当前分支确认死路后优先恢复保存分支, 不重新执行每帧全局无记忆选择
- deferred branch 只在当前活动分支持续无 odom 进展后启用, 正常前进时不能触发后方跳转

### P1, 自研导航失败反馈

- 接入局部路径不可达、执行拒绝和控制器停止反馈
- 将底层失败反馈与持续无 odom 进展共同用于死路确认

### P2, 多视角粒子目标融合

- 将现有 `obj_mask_triangulation` 和 `triangulation3d` 接入当前 Object Search 主链路
- 融合前、左、右相机的目标 Mask、相机位姿和 LiDAR 投影
- 明确视觉多视角估计、LiDAR lock、目标记忆和目标到达之间的状态切换
- 增加误检门控, 不允许单帧错误 Mask 直接覆盖持久目标

### P2, Nav2 接入

- Unity 当前继续使用同事开发的自研导航直接消费 `/spot1/graphnav_planner/path`
- Nav2 作为 Isaac 或真实机器人可选局部规划和控制后端接入, 不替换当前 Unity 链路
- 接入前明确高层 graph path 到 Nav2 goal 或 action 的限流、抢占、失败恢复和停止协议
- 保持 `path_follower_node` 与外部导航消费者互斥, 避免双重路径执行

### P2, DLIO 接入

- Unity 仿真继续使用仿真里程计, 不强制引入 DLIO
- 真实机器人根据 LiDAR、IMU 和现有定位质量决定是否使用 DLIO
- 接入时统一 odom topic、`odom -> base_link` TF、时间同步和重定位后的 graph frame 行为
- DLIO 只提供 LiDAR-inertial odometry, 不负责局部规划或路径跟踪

### P3, 真实机器狗平台接入

- 完成 `robot` topic profile, 替换当前占位 topic 和 frame
- 标定三相机、LiDAR、IMU 和 base frame 外参
- 验证 elevation map、持久 graph、目标检测和高层路径在真实传感器噪声下的稳定性
- 对接真实机器狗导航接口、急停、速度限制、跌倒保护和任务停止反馈
- 在室内受控环境完成回放和低速闭环后, 再进入室外目标搜索测试

## 文件夹结构

```text
graph_construction/
  configs/        graph, grid, elevation, topic profile 配置
  launch/         elevation 主线和 2D fallback launch
  graph_construction/
                  geometry map 到 navigation graph 的 Python 实现
  test/           graph builder, grid adapter 和诊断测试
  docs/           dated implementation notes 和本说明

visual_navigation/
  configs/        WildOS, object search goal mux 和 baseline 配置
  launch/         WildOS 视觉节点和 baseline launch
  visual_navigation/wildos/
                  ExploRFM 推理, 当前帧 frontier scoring, object target selection
  visual_navigation/object_search_goal_mux.py
                  object search goal 状态机和 planner goal 输出
  visual_navigation/utils/
                  TF, odom adapter, goal navigator, scoring 和 buffer 工具

graphnav_planner/
  src/            planner node 和 path follower node
  include/        planner C++ 接口
  launch/         planner launch 配置

graphnav_msgs/
  msg/            NavigationGraph, Node, Edge 和 traversability 消息

object_search_msgs/
  msg/            object mask 与 TF 相关消息

explorfm/
  ExploRFM inference model

nvidia_radio/
  RADIO / NACLIP / SigLIP2 backbone 相关代码

explorfm_trainer/
  ExploRFM head 训练代码

triangulation3d/
  旧 3D object triangulation 和可视化工具

external_references/
  本地社区参考代码, 需要保留 COLCON_IGNORE
```

## 主要配置文件

```text
graph_construction/configs/topic_profiles.yaml
  profile 级 topic, frame, RMW, goal, path 和目标检测默认参数

graph_construction/configs/graph_construction_elevation.yaml
  elevation/2.5D GridMap 后端差异和调优覆盖

graph_construction/configs/graph_construction.yaml
  2D OccupancyGrid fallback 后端差异和调优覆盖

graph_construction/configs/elevation_mapping_sim.yaml
  elevation_mapping_cupy 静态仿真配置, 默认关闭 visibility cleanup 避免有效地面被射线清除

graph_construction/configs/livox_grid_builder.yaml
  2D fallback 点云转 OccupancyGrid 配置

visual_navigation/configs/wildos_nav_sim_conf.yaml
  WildOS 视觉评分仿真配置

visual_navigation/configs/object_search_goal_mux.yaml
  object search goal mux 独立默认配置

graphnav_planner/launch/graphnav_planner.launch.yml
  graph planner 和 path follower 参数
```
