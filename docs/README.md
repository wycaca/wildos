# 文档索引

当前文档按代码职责归类. 先读取与任务直接相关的最小文档

## 系统与部署

| 范围 | 文档 |
| --- | --- |
| 架构 | [系统总览](system/overview.md) |
| 不可破坏的行为 | [实现原则](system/principles.md) |
| ROS topic 与 owner | [Topic 契约](system/topics.md) |
| 实机验收与运行约束 | [实机验收](system/issues.md) |
| 主机和 profile 配置 | [环境配置](deployment/environment.md) |
| Docker 部署 | [部署说明](deployment/docker.md) |
| D-LIO 与 TF | [定位说明](localization/dlio.md) |

## 运行模块

| 模块 | 文档 |
| --- | --- |
| `graph_construction` | [导航图更新](graph_construction/graph_update.md) |
| `visual_navigation` | [包 README](../visual_navigation/README.md), [目标定位](visual_navigation/target_localization.md) |
| `graphnav_planner` | [Planner 说明](graphnav_planner/README.md) |
| 跨模块目标搜索 | [目标搜索](object_search/target_search.md) |
| `triangulation3d` | [包 README](../triangulation3d/README.md) |
| `explorfm` | [包 README](../explorfm/README.md) |
| `explorfm_trainer` | [包 README](../explorfm_trainer/README.md) |
| `nvidia_radio` | [包 README](../nvidia_radio/README.md) |
