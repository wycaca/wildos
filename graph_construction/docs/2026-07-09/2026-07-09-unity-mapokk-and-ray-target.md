# Unity Mapokk And Ray Target

## 问题

Unity 2D 中视觉模型能看到目标, 但目标搜索路线仍然明显不对

同时自研 grid builder 直接订阅 `/livox/lidar` 后, 地图随机器人朝向旋转的问题再次出现

## 判断

当前 Unity 的 raw `/livox/lidar` 不满足 `livox_grid_builder` 的标准输入契约

`livox_grid_builder` 默认假设 PointCloud2 的 XYZ 在 `header.frame_id` 对应的 LiDAR 局部坐标中, 然后通过 TF 转到 `grid_frame`

现场记录和对照结果显示, `/mapokk` 才是已经对齐到 `odom_3D` 的点云, 适合 2D local grid 直接投影

## 修改

- Unity profile 的 `lidar_topic` 恢复为 `/mapokk`
- Unity profile 的 `lidar_assume_input_in_grid_frame` 恢复为 `true`
- `AGENT_README.md` 同步 Unity 2D 当前稳定输入链路
- `2026-07-09-direct-livox-grid-input.md` 标记为已回滚方案
- WildOS 目标候选新增 detection ray fallback
- 目标检测射线附近存在 graph node 时, 优先用该节点作为 `object_search_target_pose`
- 没有合适 ray node 时, 才回退到原来的 target frontier score

## 后续

如果必须取消 `/mapokk` 外部依赖, 不能直接消费 raw `/livox/lidar`

需要把 `/mapokk` 发布端的 odom_3D 点云对齐逻辑移植到本仓库, 或新增等价的 internal aligned cloud 节点

## 验证

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=$PWD/graph_construction:$PYTHONPATH python3 - <<'PY'
from graph_construction.topic_profiles import load_topic_profile
p = load_topic_profile('unity', 'graph_construction/configs/topic_profiles.yaml')
print(p['lidar_topic'])
print(p['lidar_assume_input_in_grid_frame'])
PY
```

结果:

```text
/mapokk
true
```

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
PYTHONPATH=$PWD/graph_construction:$PWD/visual_navigation:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest visual_navigation/test/test_stable_frontier_selector.py graph_construction/test/test_graph_builder_diagnostics.py
```

结果: `7 passed`
