# 项目 Agent 指令

本文件是项目入口, 只读取当前任务需要的路由文档

## 始终生效的规则

- 修改行为前先阅读受影响代码及其调用方
- 契约, 参数, owner 或启动行为变化时, 同步更新项目根目录 `docs/<模块>/` 中的当前说明
- `docs/history/` 只作为历史证据, 不作为当前运行依据
- 当前说明只记录现有代码逻辑, 接口和操作, 需要强调时说明设计约束或原因, 不记录按日期排列的修改过程
- 文档必须提供可直接执行的启动, 验证和排障命令
- 启动命令较复杂时, 提供或更新脚本封装启动, 停止, 重启, 日志和状态检查等常见操作
- 非平凡逻辑, 配置或 launch 变更必须新增或更新最小相关测试
- 运行最小相关测试, 语法检查和 `git diff --check`, 全量检查被历史问题阻断时明确区分新增问题
- 代码和配置注释保持简洁, 业务语义使用中文, 使用英文标点, 不以句号结尾
- 为不直观或复杂逻辑补充简短说明, 包含职责和关键边界
- 保持配置和代码清晰, 清理已确认过时或未使用的实现
- 不新增职责重复的 launch, Compose 或适配节点

## 按需阅读

| 任务 | 优先阅读 |
| --- | --- |
| 理解当前实机系统 | [README.md](README.md), [系统总览](docs/system/overview.md) |
| 修改共享安全或运行约束 | [实现原则](docs/system/principles.md) |
| 修改 `graph_construction` 或地图参数 | [导航图更新](docs/graph_construction/graph_update.md) |
| 修改 topic, frame 或 ROS profile | [Topic 契约](docs/system/topics.md), [环境配置](docs/deployment/environment.md) |
| 修改 D-LIO, 点云 relay 或 TF owner | [D-LIO 与 TF](docs/localization/dlio.md) |
| 修改 Compose 或机器人启动 | [部署说明](docs/deployment/docker.md) |
| 修改视觉评分, 目标证据或目标融合 | [视觉导航](visual_navigation/README.md), [目标定位](docs/visual_navigation/target_localization.md) |
| 修改搜索状态或高层目标选择 | [目标搜索](docs/object_search/target_search.md) |
| 修改图规划或探索恢复 | [Planner 说明](docs/graphnav_planner/README.md), [目标搜索](docs/object_search/target_search.md) |
| 修改模型或训练流程 | [文档索引](docs/README.md) 和对应包 README |

## 优先级

1. 用户当前的明确指令
2. 本文件和更近的项目指令
3. 当前文档和代码
4. 历史文档
