# 2026-07-07 Vision threshold tuning

## 背景

切换 `/combined_grid` 后, 地图和路径稳定性明显改善。继续测试时发现两个视觉侧问题:

- 物体识别略敏感, 需要把目标 mask 阈值调高一点
- RViz 中可见的视觉 frontier 节点偏少, 需要略微增加候选数量

## 目标

- Unity 物体 mask 阈值从 `0.09` 提高到 `0.10`
- Unity visual frontier 候选范围从 `9.0m` 提高到 `11.0m`
- Unity 图像 frontier 阈值从 `0.60` 降到 `0.55`
- Isaac 和 robot profile 继续保守使用原值

## 问题分析

物体识别阈值和视觉 frontier 节点数量不是同一个参数:

- `object_search_config.mask_threshold` 控制文本相似度图转目标二值 mask, 调高会减少误检
- `frontiers_range` 控制 WildOS 从 nav graph 中取多远范围内的 frontier node, 调大可以增加可评分候选
- `frontier_threshold` 控制图像 frontier 置信度筛选, 调低可以增加参与评分的视觉 frontier 区域

因此本次没有通过降低目标 mask 阈值来增加 frontier, 而是分别调整 object search 和 visual frontier 参数

## 修改内容

- `topic_profiles.yaml` 新增 `visual_frontiers_range`
- `topic_profiles.yaml` 新增 `visual_frontier_threshold`
- Unity profile 设置 `object_search_mask_threshold: "0.10"`
- Unity profile 设置 `visual_frontiers_range: "11.0"`
- Unity profile 设置 `visual_frontier_threshold: "0.55"`
- Isaac 和 robot profile 设置 `visual_frontiers_range: "9.0"`, `visual_frontier_threshold: "0.6"`
- 2D launch 将 `visual_frontiers_range` 透传为 WildOS `frontiers_range`
- 2D launch 将 `visual_frontier_threshold` 透传为 WildOS `frontier_threshold`
- 3D launch 同步透传这两个参数

## 验证步骤

启动 Unity 2D:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity do_object_search:=true
```

预期日志:

```text
目标搜索已启用, ..., mask_threshold=0.100
```

RViz 检查:

- `/spot1/score_rings` 的可见评分圆环应略多
- `/spot1/within_range_geofrontiers` 中可见 frontier marker 应略多
- 如果误检仍多, 临时提高 `object_search_mask_threshold:=0.11`
- 如果视觉 frontier 仍少, 临时提高 `visual_frontiers_range:=12.0`
