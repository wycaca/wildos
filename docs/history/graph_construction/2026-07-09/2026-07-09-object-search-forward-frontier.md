# 2026-07-09 object search forward frontier

## 背景

目标找不到时, 预期机器狗按当前 odom 朝向继续向前搜索

此前逻辑中, 只要 scored graph 可用, `object_search_goal_mux` 会优先进入 `GEOMETRIC_EXPLORE`, 调用 `StableFrontierSelector` 从 scored graph 选择 frontier

`initial_goal_heading_deg=0.0` 只在 scored graph 不可用时作为 fallback 生效, 因此它不能约束正常 scored graph 搜索方向

## 问题

`StableFrontierSelector` 原 utility 只包含 frontier score, 距离和切换惩罚

当后方 frontier 更近, 分数更高, 或仍处于 dwell 锁定时, 无目标搜索会发布身后的 graph frontier goal, 表现为机器狗回头

## 修改

- `object_search_goal_mux` 在 target pose 为空时, 把 odom yaw 加 `initial_goal_heading_deg` 传给 selector
- `StableFrontierSelector` 为候选 frontier 计算 `heading_alignment`, 即候选方向和 odom 搜索方向的点积
- `frontier_min_forward_dot=0.0` 默认只保留前半平面候选
- `frontier_forward_weight=0.8` 默认给越接近正前方的候选加分
- `frontier_forward_fallback_to_any=true` 默认在前方完全没有候选时回退到任意 frontier, 避免卡死

## 行为

有目标或目标记忆时, selector 不使用 odom 前向过滤

无目标且 scored graph 可用时, 优先选择 odom 前方 frontier

scored graph 不可用时, 仍使用原来的 initial heading goal

## 验证

```bash
python3 -m py_compile visual_navigation/visual_navigation/stable_frontier_selector.py visual_navigation/visual_navigation/object_search_goal_mux.py graph_construction/launch/wildos_2d_sim.launch.py graph_construction/launch/elevation_visual_navigation_sim.launch.py
source /opt/ros/humble/setup.bash && source ../../install/setup.bash && PYTHONPATH=$PWD/visual_navigation:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest visual_navigation/test/test_stable_frontier_selector.py
```
