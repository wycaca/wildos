# WildOS 当前待办

本文只保留尚未完成项, 完成并通过验证的条目直接删除

## 1. 验证 GO2 脚下盲区修补

当前图构建只在 4.0 m 内修补人工地面, 并拒绝修补机器人与候选栅格之间存在墙体或受保护高程突变的区域. 墙体掩码额外扩展一个栅格, 用于封闭点云离散造成的小缺口

机器狗恢复站立后执行:

```bash
./scripts/wildos_docker.sh orin update wildos
./scripts/wildos_docker.sh orin restart wildos
./scripts/wildos_docker.sh orin logs wildos
```

验收条件:

- 脚下人工地面能连接外围真实地面
- 修补范围不超过 4.0 m
- 墙后栅格保持 unknown 或 obstacle
- 日志中 `盲区连通=True`
- 高程图、节点和边在墙边不越界

## 2. 完成 GO2 目标搜索与导航实机闭环

- 三路图像、CameraInfo 和测量时刻 TF 持续有效
- Frontier 评分、Mask、目标融合和 Path 连续输出
- 单视角候选不会直接成为导航坐标
- 多视角和 LiDAR 精修行为符合状态契约
- Goal Mux 目标能够驱动 GO2 完成搜索和导航
- 最终 `REACHED` 只由新鲜近距离证据触发

## 3. 记录持续运行基线

- CPU、GPU、显存和温度满足连续运行
- 点云、odom、图和 Path 消息年龄稳定
- 跨机点云带宽、丢包和 DDS 阻塞可接受
- 图节点、边和处理耗时不出现无界增长
