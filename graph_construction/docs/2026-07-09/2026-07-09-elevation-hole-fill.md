# 2026-07-09 elevation hole fill

## 背景

`elevation_mapping_cupy` 输出的 GridMap 可能在地面出现小型 NaN 洞

此前 `graph_construction.grid_adapter` 已经会把被 free 包围的小 unknown/obstacle 区域补成 free, 但没有同步补 `elevation` layer

结果是 graph node 可能采到已补 free 的 cell, 但 z 仍因 elevation 为 NaN 回落到 `z_offset`, RViz 中看起来落在高程图下方

## 修改

- `classify_grid_map` 新增 `fill_elevation_holes` 和 `fill_elevation_radius_cells`
- 只对后处理后已经判定为 free 的 NaN elevation cell 做邻近有限高程中值填补
- 大块 unknown, 边界 unknown 和未判定 free 的区域不会被补成可走
- `graph_construction_elevation.yaml` 默认开启 `grid_map_fill_elevation_holes: true`

## 边界

这不会修改 `/elevation_mapping_node/elevation_map_raw` 原始消息本身

如果 RViz 直接显示 elevation_mapping_cupy 原始 GridMap, 原始洞仍可能可见

它修的是 graph construction 内部用于节点 z, 边和路径规划的 ClassifiedGrid

## 验证

```bash
python3 -m py_compile graph_construction/graph_construction/grid_adapter.py graph_construction/graph_construction/node.py
PYTHONPATH=$PWD/graph_construction PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest graph_construction/test/test_grid_map_adapter.py graph_construction/test/test_graph_builder_diagnostics.py
git diff --check
```
