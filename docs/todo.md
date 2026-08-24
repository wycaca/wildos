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

## 3. 下线旧字符串状态 Topic

Planner 已使用 `/spot1/object_search_status_v2`, 旧 `/spot1/object_search_status` 仅供迁移期外部监控使用

待办:

- 确认外部监控不再订阅旧 Topic
- 删除 Goal Mux 的旧 String publisher 和配置参数
- 删除文档中的迁移期兼容说明
- 运行目标搜索和 Planner 回归测试

## 4. 收紧实机网络边界

当前跨机 DDS 依赖可信专用有线链路, 容器已使用最小权限, 主机网络边界仍需现场配置

待办:

- 记录两台部署主机的实际 DDS 和 SSH 端口
- 配置主机防火墙, 只允许部署对端和已批准的管理主机
- 链路接入不可信设备时启用 DDS Security
- 验证三相机、点云、定位和跨机 DDS 不受防火墙影响

端口采集和验证命令见[运行基线与上线验收](deployment/validation.md)

## 5. 补齐运行基线和责任人

待办:

- 运行至少 10 分钟的资源和链路采集
- 填写 CPU、GPU、显存、温度、消息年龄和回调耗时阈值
- 记录代码提交、容器镜像 ID 和模型清单哈希
- 替换 `graph_construction/package.xml` 中的占位 maintainer 邮箱
- 明确定位、视觉、导航和部署故障责任人

执行命令和结果表见[运行基线与上线验收](deployment/validation.md)
