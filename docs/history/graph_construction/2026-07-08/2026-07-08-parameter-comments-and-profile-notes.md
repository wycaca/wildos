# 参数注释和 profile 说明收敛

## 背景

当前 2D 和 3D 完整启动入口把大量 `topic_profiles.yaml` key 暴露为 launch override, `--show-args` 和 profile 文件都比较难读

这些参数大部分不是日常入口参数, 而是平台适配, grid builder 调试, object search 稳定性和 planner 联调参数

## 目标

- 保持现有默认行为不变
- 给 profile 参数补充分组注释
- 给 `--show-args` 中的 profile override 补充具体说明
- 给主要节点配置补充分组注释, 区分日常参数和调试参数

## 当前代码链路

完整启动入口:

```text
scripts/start_wildos_2d.sh
    -> graph_construction/launch/wildos_2d_sim.launch.py
    -> graph_construction/configs/topic_profiles.yaml

scripts/start_wildos_3d.sh
    -> graph_construction/launch/elevation_visual_navigation_sim.launch.py
    -> graph_construction/configs/topic_profiles.yaml
```

profile key 由 `graph_construction.topic_profiles.load_topic_profile()` 加载, launch 中 `_profile_arg()` 会把 key 暴露成可选 override

## 问题分析

参数偏多的主要原因是 profile 同时承担了平台画像和调试参数集合

日常启动一般只需要 `topic_profile`, `do_object_search`, `log_level`, 少数 topic 覆盖和路径输出覆盖

`grid_*` 和 `object_search_frontier_*` 属于调试参数, 不应该被误认为每次启动都要手动设置

## 修改内容

- `topic_profiles.yaml` 增加顶部分层说明和各 profile 的分组注释
- `topic_profiles.py` 增加 `PROFILE_KEY_DESCRIPTIONS`, 用于集中维护 profile key 说明
- 2D 和 3D launch 的 `_profile_arg()` 改为显示具体参数说明
- `livox_grid_builder.yaml` 增加 frame/topic, 尺寸, 点云过滤, rolling grid, height_diff 和累计扫描分组注释
- `graph_construction.yaml` 和 `graph_construction_elevation.yaml` 增加 topic/frame, 诊断, 分类阈值, graph 采样, frontier 分组注释
- `object_search_goal_mux.yaml` 增加输入输出, 初始探索, 目标记忆和 stable frontier selector 分组注释

## 验证步骤

```bash
python3 -m compileall -q graph_construction/graph_construction
python3 - <<'PY'
from pathlib import Path
import yaml

for path in [
    "graph_construction/configs/topic_profiles.yaml",
    "graph_construction/configs/livox_grid_builder.yaml",
    "graph_construction/configs/graph_construction.yaml",
    "graph_construction/configs/graph_construction_elevation.yaml",
    "visual_navigation/configs/object_search_goal_mux.yaml",
]:
    with Path(path).open("r", encoding="utf-8") as stream:
        yaml.safe_load(stream)
PY

export ROS_LOG_DIR=/tmp/ros2_launch_logs
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
ros2 launch graph_construction wildos_2d_sim.launch.py --show-args
ros2 launch graph_construction elevation_visual_navigation_sim.launch.py --show-args
```
