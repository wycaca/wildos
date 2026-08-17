# 2026-07-09 historical edge memory

## 背景

路线乱跳时需要确认 graph 是否只有当前点云记忆

当前实现中, `GraphState.nodes` 会跨局部地图更新保留, 但 `GraphState.set_edges()` 每帧替换全部 edge

旧逻辑下, `EdgeBuilder.build_edges()` 只按当前 `ClassifiedGrid` 生成边, 线段只要经过 unknown 就拒绝, 因此机器人转向或 rolling map 导致已走过区域暂时不可见时, 历史边会消失, planner 可用拓扑会抖动

## 社区参考

`external_references/nebula2-wildos-main_ws/graphnav_builder` 的 `validate_existing_edges` 只验证旧边在当前局部地图中可见的部分

地图外或当前 unknown 的历史段不会误删旧边, 只有可见段出现障碍或净空不足时才删除

## 修改

- `ClassifiedGrid.world_line_cells_clipped()` 支持把历史边裁剪到当前 grid 可见范围再检查
- `EdgeBuilder.merge_historical_edges()` 合并当前新边和未被证伪的历史边
- `GraphBuilderConfig.validate_historical_edges` 默认开启
- `graph_construction.yaml` 和 `graph_construction_elevation.yaml` 显式设置 `validate_historical_edges: true`

## 行为

新边仍然严格要求整条线段穿过 known free, 并满足 obstacle 与 unknown clearance

历史边只用当前可见障碍证伪, unknown 不会删除历史通路

这样避免把“当前看不到”误判成“不能走”, 同时保持看到障碍时避碰优先

## 验证

```bash
python3 -m py_compile graph_construction/graph_construction/grid_types.py graph_construction/graph_construction/edge_builder.py graph_construction/graph_construction/graph_builder.py graph_construction/graph_construction/node.py
PYTHONPATH=$PWD/graph_construction PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python3 -m pytest graph_construction/test/test_graph_builder_diagnostics.py
git diff --check
```
