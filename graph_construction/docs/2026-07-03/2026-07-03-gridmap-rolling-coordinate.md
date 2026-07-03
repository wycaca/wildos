# GridMap 解码和 rolling map 坐标处理

日期: 2026-07-03

## 背景

`2026-07-02-external-commit-reference-analysis.md` 提醒当前项目需要重点处理 GridMap 解码和 rolling map 坐标约定

问题集中在几类:

- `GridMap` layer 可能带 `layout.data_offset`
- layer 存储可能用 row / column 或 column / row 标签
- `outer_start_index` 和 `inner_start_index` 表示 circular buffer 的起点
- GridMap pose 可能带 yaw
- `world_to_grid` 和 `grid_to_world` 必须互为逆变换

## 目标

让 graph construction 直接消费 GridMap 时, 坐标处理不再依赖无 yaw 的手动特例

同时让 `grid_map_to_occupancy` debug adapter 复用同一套 GridMap layer 解码逻辑

## 当前代码链路

GridMap 输入链路:

```text
grid_map_msgs/GridMap
    -> grid_adapter.decode_grid_map_layer
    -> grid_adapter.unwrap_grid_map_buffer
    -> grid_adapter.classify_grid_map
    -> ClassifiedGrid
    -> SparseGraphBuilder
```

坐标变换链路:

```text
ClassifiedGrid.grid_to_world
    -> GridMap center, length_x, length_y, yaw
    -> world cell center

ClassifiedGrid.world_to_grid
    -> inverse yaw
    -> GridMap local axes
    -> cell index
```

debug projection 链路:

```text
grid_map_to_occupancy
    -> grid_adapter.decode_grid_map_layer
    -> OccupancyGrid debug output
```

## 修改内容

`grid_types.py`:

- `ClassifiedGrid` 增加 `grid_map_yaw`
- GridMap convention 下的 `world_to_grid` 改为先把 world XY 旋转回 GridMap 本地轴
- GridMap convention 下的 `grid_to_world` 改为从本地轴坐标旋转回 world XY
- 使用 `grid_map_center_x`, `grid_map_center_y`, `grid_map_length_x`, `grid_map_length_y` 做正逆变换

`grid_adapter.py`:

- `classify_grid_map` 从 GridMap pose quaternion 提取 yaw
- `decode_multiarray` 支持 `layout.data_offset`
- 对 `row_index / column_index` 和 `column_index / row_index` 都保留解码
- stride 不可用或超出数据长度时回退到连续 reshape
- `unwrap_grid_map_buffer` 继续根据 `outer_start_index` 和 `inner_start_index` 还原 logical cell 顺序

`grid_map_to_occupancy.py`:

- 移除本地重复的 GridMap layer 解码实现
- 改为复用 `grid_adapter.decode_grid_map_layer`

`test_grid_map_adapter.py`:

- 新增 data_offset + rolling buffer 测试
- 新增 column / row layout 测试
- 新增无 yaw round-trip 测试
- 新增 yaw=90deg round-trip 测试

## 验证步骤

已执行:

```bash
PYTHONPATH=graph_construction .venv/bin/python -m pytest -q graph_construction/test/test_grid_map_adapter.py
```

结果:

```text
4 passed
```

已执行 AST 语法检查:

```bash
python3 - <<'PY'
import ast
from pathlib import Path
files = [
    'graph_construction/graph_construction/grid_adapter.py',
    'graph_construction/graph_construction/grid_types.py',
    'graph_construction/graph_construction/grid_map_to_occupancy.py',
    'graph_construction/test/test_grid_map_adapter.py',
]
for file_name in files:
    ast.parse(Path(file_name).read_text(encoding='utf-8'), filename=file_name)
PY
```

## 修改建议

后续如果 elevation marker 仍然相对 GridMap surface 偏移, 不要先调 flip / transpose

优先记录一帧真实 GridMap:

- `info.pose.position`
- `info.pose.orientation`
- `info.length_x`
- `info.length_y`
- `info.resolution`
- `outer_start_index`
- `inner_start_index`
- 一个已知 cell 的 layer 值和对应 world 坐标

然后把这帧数据固化成测试, 确认 `world_to_grid` 和 `grid_to_world` 是否 round-trip
