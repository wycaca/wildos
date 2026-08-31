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
