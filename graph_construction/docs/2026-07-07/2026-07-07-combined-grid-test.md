# 2026-07-07 Combined grid test

## 背景

Unity 2D 中机器人转动时, 自研 `livox_grid_builder` 生成的 OccupancyGrid 仍会出现旋转错乱, 导致 graph 节点和 planner 路线频繁跳变

为了先隔离 graph construction 和 planner 的稳定性, 曾临时切换到同事发布的局部地图 `/combined_grid` 进行测试。当前 Unity 默认已切回本仓库自研 `livox_grid_builder`

## 目标

- Unity 2D 可通过 launch 覆盖临时消费 `/combined_grid`
- Unity 2D 对照测试时不启动本仓库 `livox_grid_builder`
- 避免同事地图和自研 grid builder 同时发布同一个 topic
- Isaac 和 robot profile 暂不改变

## 问题分析

`wildos_2d_sim.launch.py` 之前把 `traversability_grid_topic` 同时用于两处:

- `livox_grid_builder.grid_topic`, 作为本仓库局部地图输出
- `graph_construction.grid_topic`, 作为 graph 输入

如果只把 `traversability_grid_topic` 改成 `/combined_grid`, 本仓库 `livox_grid_builder` 也会向 `/combined_grid` 发布, 与同事地图冲突

因此需要增加 `launch_livox_grid_builder` 开关。当前 Unity profile 默认设为 `true`, graph 默认订阅 `/spot1/traversability_grid`, 自研 grid builder 会启动

## 修改内容

- `topic_profiles.yaml` 新增 `launch_livox_grid_builder`
- Isaac profile 设置 `launch_livox_grid_builder: "true"`
- Unity profile 当前设置 `launch_livox_grid_builder: "true"`
- robot profile 设置 `launch_livox_grid_builder: "true"`
- Unity profile 当前使用 `traversability_grid_topic: /spot1/traversability_grid`
- `wildos_2d_sim.launch.py` 新增 `launch_livox_grid_builder` launch argument
- `wildos_2d_sim.launch.py` 将该开关挂到 `livox_grid_builder` 节点 condition

## 验证步骤

临时使用 `/combined_grid` 对照测试:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true launch_livox_grid_builder:=false traversability_grid_topic:=/combined_grid
```

预期行为:

- 不出现 `livox_grid_builder` 进程启动日志
- `graph_construction` 启动日志中的 grid 应为 `/combined_grid`
- `/spot1/graph_construction_viz` 和 `/spot1/nav_graph` 应来自 `/combined_grid`
- 如果 `/combined_grid` 稳定但路线仍跳, 继续排查 graph state 和 planner 目标切换
- 如果 `/combined_grid` 稳定且路线稳定, 说明主要问题在自研 `livox_grid_builder` 的 Unity 点云坐标处理

默认自研 grid builder:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true
```
