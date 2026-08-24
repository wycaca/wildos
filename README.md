# WildOS

WildOS 是一个用于非结构化环境开放词汇目标搜索的 ROS 2 系统. 它从 elevation `GridMap` 构建持久导航图, 用 ExploRFM 评分可达 Frontier, 再通过多视角目标融合引导安全搜索

## 系统链路

```text
MID360 + IMU -> D-LIO -> 注册点云, odom, TF
三路 RealSense + elevation map -> NavigationGraph -> 视觉评分
视觉证据 -> 目标融合 -> ObjectSearchGoalMux -> graphnav_planner -> Path
```

x86 主机运行 LiDAR 与 D-LIO. 相机 AGX 运行相机, 建图, 图构建, 视觉推理, 目标融合和规划. Planner 只发布 Path, 底层运动执行由仓库外控制器负责

## 文档入口

| 内容 | 文档 |
| --- | --- |
| 系统架构、模块和接口边界 | [架构](docs/architecture.md) |
| 实机配置、启动、验证和排障 | [部署](docs/deployment.md) |
| 高程图到持久导航图 | [图构建](docs/graph.md) |
| 探索、目标融合和完成状态 | [目标搜索](docs/object_search.md) |
| Planner 路径行为 | [Planner](graphnav_planner/README.md) |
| 尚未完成的工作 | [TODO](docs/todo.md) |

开发 Agent 先读取 [agent.md](agent.md), 再按任务读取一份领域文档. 当前行为以代码、配置和上述文档为准

## 测试

在 ROS Humble workspace 中使用唯一入口运行构建、Python 功能测试、Planner CTest、manifest 和 Docker 契约检查:

```bash
./scripts/test_repo.sh
```

该入口固定使用仓库 `.venv` 的 Python 3.10, 不执行 flake8 或 pep257

## 主要模块

| 模块 | 职责 |
| --- | --- |
| `graph_construction` | 从 elevation `GridMap` 构建持久 `NavigationGraph` |
| `visual_navigation` | ExploRFM 评分, 视觉证据, 目标融合适配和高层目标选择 |
| `triangulation3d` | 递归多视角目标粒子滤波 |
| `graphnav_planner` | 图路径规划与探索恢复 |
| `explorfm` | 视觉推理模型 |
| `explorfm_trainer` | 视觉 Head 训练流程 |

## 研究来源

- 论文: [WildOS: Open-Vocabulary Object Search in the Wild](https://arxiv.org/abs/2602.19308)
- 项目主页: [leggedrobotics.github.io/wildos](https://leggedrobotics.github.io/wildos/)
- 数据集: [Hugging Face WildOS](https://huggingface.co/datasets/leggedrobotics/wildos)
- 模型来源、版本和 SHA256: [ckpts/manifest.json](ckpts/manifest.json)
- License: [Apache 2.0](LICENSE)

```bibtex
@misc{shah2026wildosopenvocabularyobjectsearch,
  title={WildOS: Open-Vocabulary Object Search in the Wild},
  author={Hardik Shah and Erica Tevere and Deegan Atha and Marcel Kaufmann and Shehryar Khattak and Manthan Patel and Marco Hutter and Jonas Frey and Patrick Spieler},
  year={2026},
  eprint={2602.19308},
  archivePrefix={arXiv},
  primaryClass={cs.RO}
}
```
