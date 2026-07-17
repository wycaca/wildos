# 删除重复的低频高程图 Topic

## 问题

默认 elevation mapping 配置同时发布:

- `/elevation_mapping_node/elevation_map_raw`, 5Hz
- `/elevation_mapping_node/elevation_map_recordable`, 2Hz

`elevation_map_recordable` 只包含 raw map 已有的 `elevation` 和 `traversability` layer。当前默认 launch、WildOS 节点、planner、RViz 和录包脚本均不消费该 topic，因此它会持续序列化和发布无人使用的大体积 GridMap

## 修改

修改 `graph_construction/configs/elevation_mapping_sim.yaml`，删除 `publishers.elevation_map_recordable`，默认配置只保留 `elevation_map_raw`

同步更新 `graph_construction/docs/details/topics.md`，将该 topic 从待清理项改为已清理项

## 行为边界

- 不修改 `/elevation_mapping_node/elevation_map_raw` 的 layer、频率或 QoS
- 不修改 elevation mapping 的保存和加载服务
- 不影响用户通过自定义 elevation 配置显式增加其他 publisher
- 如未来需要低频录包，应在专用配置或 rosbag 流程中显式实现，不恢复默认重复发布

## 验证

- YAML 解析通过
- 默认配置只包含 `elevation_map_raw` publisher
- 仓库默认运行配置中不再存在 `elevation_map_recordable`
