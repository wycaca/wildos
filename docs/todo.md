# WildOS 当前待办

本文只保留尚未完成项, 完成并通过验证的条目直接删除

## 1. 验证 GO2 脚下盲区修补

当前图构建只在 1.5 m 内修补人工地面, 并拒绝修补机器人与候选栅格之间存在墙体或受保护高程突变的区域

机器狗恢复站立后执行:

```bash
./scripts/wildos_docker.sh orin update wildos
./scripts/wildos_docker.sh orin restart wildos
./scripts/wildos_docker.sh orin logs wildos
```

验收条件:

- 脚下人工地面能连接外围真实地面
- 修补范围不超过 1.5 m
- 墙后栅格保持 unknown 或 obstacle
- 日志中 `盲区连通=True`
- 高程图、节点和边在墙边不越界

## 2. 精确标定 GO2 安装外参

当前 MID360 倾角按 10 度使用, 倾斜轴向、机身到雷达的平移和三相机外参仍需精确测量

验收条件:

- `go2_base_link -> base_link` 使用实测外参
- 三相机 TF 使用实测外参
- 站立和正常行走时地面高度稳定
- 墙体在点云和高程图中不出现系统性倾斜

排障命令见[D-LIO 与 TF](localization/dlio.md)和[环境配置](deployment/environment.md)
