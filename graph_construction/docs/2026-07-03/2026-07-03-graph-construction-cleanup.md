# Graph Construction 遗留代码清理

日期: 2026-07-03

## 背景

本次清理来自对 `graph_construction` 现有代码的分层检查

需要修复的问题:

- 旧 graph-only, visual-only, adapter-only launch 仍会被安装
- `grid_map_to_occupancy` debug adapter 仍复制 GridMap 归一化, 朝向和后处理逻辑
- 少量 adapter helper 缺少中文注释

用户明确要求:

- 保留 `GraphBuilder = SparseGraphBuilder` 兼容别名
- 保留 `grid_map_to_occupancy` 作为 debug adapter
- 文档和入口要明确 debug adapter 不是默认 graph 输入

## 修改内容

### 启动入口

`setup.py` 不再安装所有 `*launch.*`, 改为只安装:

- `wildos_2d_sim.launch.py`
- `elevation_visual_navigation_sim.launch.py`

删除的旧 launch:

- `graph_construction.launch.py`
- `graph_construction_sim.launch.py`
- `livox_grid_builder.launch.py`
- `elevation_mapping_sim.launch.py`
- `graph_construction_elevation_sim.launch.py`

这样 install tree 中不会继续暴露绕过 `topic_profiles.yaml` 的旧启动入口

### Debug Adapter

`grid_map_to_occupancy` 保留为 console script 和 debug 工具

它现在复用 `grid_adapter.py` 中的:

- `decode_grid_map_layer`
- `normalize_grid_map_layer`
- `orient_grid_map_array`
- `postprocess_classification`

保留在 adapter 内部的逻辑只包括:

- 首帧统计日志
- GridMap 分类 mask 到 OccupancyGrid 数值的转换
- ROS topic 订阅和发布

### 注释

补充了 `GridMapToOccupancyNode`, `_on_timer`, `_convert`, `_adapter_config`, `_load_config`, `main` 等中文说明

补充了 `pointcloud_axis_adapter` 中 `_iter_xyz`, `_xyz_fields`, `main` 的中文说明

## 当前入口约定

默认完整链路只走:

```bash
./scripts/start_wildos_2d.sh
./scripts/start_wildos_3d.sh
```

手动 debug GridMap projection 时才单独运行:

```bash
ros2 run graph_construction grid_map_to_occupancy --config grid_map_to_occupancy.yaml
```

该 debug adapter 输出 `/spot1/elevation_traversability_grid`, 但默认 3D graph construction 直接消费 `/elevation_mapping_node/elevation_map_raw`

## 验证

已执行:

```bash
python3 -m py_compile graph_construction/setup.py graph_construction/graph_construction/grid_map_to_occupancy.py graph_construction/graph_construction/pointcloud_axis_adapter.py graph_construction/launch/wildos_2d_sim.launch.py graph_construction/launch/elevation_visual_navigation_sim.launch.py
PYTHONPATH=graph_construction .venv/bin/python -m pytest -q graph_construction/test/test_graph_builder_diagnostics.py graph_construction/test/test_grid_map_adapter.py
```

结果:

- 语法检查通过
- graph builder diagnostics 和 GridMap adapter 单测共 5 个通过

## 后续

- 如果需要临时只启动某个底层节点, 优先使用 `ros2 run` 加配置文件, 不新增 launch 入口
- 如果恢复旧 launch, 必须先说明为什么 2D / 3D 完整入口无法覆盖该场景
