# Object Search Target Latch And Stop

## 问题

Unity 目标搜索中, 机器人已经进入目标附近后仍会继续规划, 并且目标在前方时路线可能在前后方向之间跳变

日志表现为先进入 `TARGET_REACHED_VIEWPOINT`, 随后 WildOS 的 `目标近距离确认=False` 又让 mux 回到目标记忆或 frontier 搜索状态

## 根因

- `/spot1/object_search_reached=True` 只在 `object_reached_timeout_sec` 窗口内有效, 没有终态锁存
- `object_search_goal_mux` 在看到目标时仍优先调用 `StableFrontierSelector`, 当 graph 的目标方向分数为 0 时, 选择会退化成距离最近 frontier
- `graphnav_planner` 到达 goal 后只清掉内部 goal, 没有发布停止用 path, 下游 path follower 可能继续执行旧路径
- detection-ray 选出的 graph node 是安全导航点, 不是目标真实距离, 用它做 `object_reached` 距离门控会导致视觉已经确认近距离但 mux 不及时停止

## 修改

- `object_search_goal_mux` 当时新增有超时的 reached latch, 当前实现已移除超时参数并改为永久完成锁
- 到达确认后进入 reached latch, 持续发布当前位置 hold goal, 后续 `object_reached=False` 不会恢复搜索
- `object_search_goal_mux` 新增 `object_reached_require_target_distance`, 默认 `false`, 让视觉近距离确认可以立即触发停止 latch
- 如果现场再次出现远距离误停, 可把 `object_reached_require_target_distance=true`, 或提高 `object_search_reached_mask_fraction` 和 `object_search_reached_min_pixel_count`
- Unity profile 默认 `object_search_latch_target_after_first_detection=true`, `object_search_latch_target_timeout_sec=12.0`
- 目标有效或 latch 有效时直接发布目标 frontier goal, 不再先进入普通 graph frontier selector
- `graphnav_planner` 检测到 goal 已在 `goal_radius` 内时发布当前位置单点 path, 让外部控制节点进入停止条件

## 验证

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
PYTHONPATH=$PWD/graph_construction:$PWD/visual_navigation:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest visual_navigation/test/test_stable_frontier_selector.py graph_construction/test/test_graph_builder_diagnostics.py
```

结果: `6 passed`

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select visual_navigation graphnav_planner graph_construction --symlink-install
```

结果: `3 packages finished`
