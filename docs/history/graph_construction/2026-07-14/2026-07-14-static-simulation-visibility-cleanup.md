# 静态仿真关闭 visibility cleanup

## 问题

静态仿真中的点云能够持续观测地面, 但 `elevation_mapping_cupy` 默认开启 visibility cleanup

该机制会沿传感器到点云测量点执行射线清除, TF 或测量高度存在微小波动时, 已融合地面的 validity 可能被反复降低, 最终让 `elevation` 和 `traversability` 变成 NaN

Graph Construction 会将任意基础 layer 为 NaN 的 cell 判定为 unknown, 因此长条状缺口会阻断节点连接和路线规划

## 修改

在 `graph_construction/configs/elevation_mapping_sim.yaml` 中设置:

```yaml
enable_visibility_cleanup: false
```

该配置用于静态仿真环境, 保留已经确认的地面 cell, 后续点云仍会正常融合并更新高程和可通行性

## 边界

- 不修改点云 topic、frame 和 TF 逻辑
- 不修改 Graph Construction 的 unknown 判定和小洞后处理
- 不影响真实机器人配置
- 动态障碍移除不再依赖 visibility cleanup, 后续接入真实环境时需要使用独立配置重新启用并调参

## 生效方式

重新构建 `graph_construction` 并重启高程导航启动脚本, 确保 install tree 中的配置同步更新
