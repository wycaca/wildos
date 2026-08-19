# WildOS

WildOS 是一个用于非结构化环境开放词汇目标搜索的 ROS 2 系统. 它从 elevation `GridMap` 构建持久导航图, 用 ExploRFM 评分可达 Frontier, 再通过多视角目标融合引导安全搜索

## 当前实机链路

```text
MID360 + IMU -> D-LIO -> 注册点云, odom, TF
三路 RealSense + elevation map -> NavigationGraph -> 视觉评分
视觉证据 -> 目标融合 -> ObjectSearchGoalMux -> graphnav_planner -> Path
```

x86 主机运行 LiDAR 与 D-LIO. 相机 AGX 运行相机, 建图, 图构建, 视觉推理, 目标融合和规划. Planner 只发布 Path, 底层运动执行由仓库外控制器负责

## 快速入口

| 要了解的内容 | 文档 |
| --- | --- |
| 当前架构和约束 | [系统总览](docs/system/overview.md), [实现原则](docs/system/principles.md) |
| 实机部署和配置 | [部署说明](docs/deployment/docker.md), [环境配置](docs/deployment/environment.md) |
| 图构建 | [导航图更新](docs/graph_construction/graph_update.md) |
| 目标搜索和定位 | [目标搜索](docs/object_search/target_search.md), [目标定位](docs/visual_navigation/target_localization.md) |
| Planner 行为 | [Planner 说明](docs/graphnav_planner/README.md) |
| 全部当前文档与包文档 | [文档索引](docs/README.md) |

当前行为以 `docs/` 和代码为准. `docs/history/` 中的日期记录只保留历史上下文, 不能作为部署依据

## 测试

在 ROS Humble workspace 中使用唯一入口运行构建、Python 功能测试、Planner CTest、manifest 和 Docker 契约检查:

```bash
./scripts/test_repo.sh
```

该入口固定使用仓库 `.venv` 的 Python 3.10, 不执行 flake8 或 pep257. GPU、硬件和 10 分钟 rosbag 验证按 [TODO](docs/todo.md) 单独执行

## 主要模块

| 模块 | 职责 |
| --- | --- |
| `graph_construction` | 从 elevation `GridMap` 构建持久 `NavigationGraph` |
| `visual_navigation` | ExploRFM 评分, 视觉证据, 目标融合适配和高层目标选择 |
| `triangulation3d` | 递归多视角目标粒子滤波 |
| `graphnav_planner` | 图路径规划与探索恢复 |
| `explorfm` | 视觉推理模型 |
| `explorfm_trainer` | 视觉 Head 训练流程 |

原始的论文介绍, 安装步骤, checkpoints 与引用信息保存在 [README_OLD.md](README_OLD.md)
