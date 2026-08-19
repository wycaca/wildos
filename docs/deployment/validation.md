# 运行基线与上线验收

本文定义软件测试、10 分钟实机基线和上线责任边界. 实测值必须来自同一代码提交、同一模型清单和同一运行窗口, 未采集项不能写为通过

## 1. 软件构建与测试

在仓库根目录执行:

```bash
test_commit="$(git rev-parse HEAD)"
./scripts/test_repo.sh 2>&1 | tee "/tmp/wildos-test-${test_commit}.log"
less "/tmp/wildos-test-${test_commit}.log"
```

日志末尾出现 `[test] all functional test stages passed` 才表示统一入口通过. Planner 失败时可单独查看 CTest 详情:

```bash
wildos_ws_root="$(cd ../.. && pwd)"
ctest --test-dir "${wildos_ws_root}/build/graphnav_planner" --output-on-failure -R '^test_'
```

测试记录至少保存代码提交、测试日志、ROS 版本、Python 版本和 runner 名称. GPU、硬件和 rosbag 验证不包含在统一入口中

## 2. 10 分钟实机基线

### 2.1 前置条件

采集前记录:

| 字段 | 实测值 |
| --- | --- |
| 日期和场地 | 待采集 |
| 代码提交 | 待采集 |
| `ckpts/manifest.json` SHA256 | 待采集 |
| x86 CPU、GPU、内存和系统版本 | 待采集 |
| Orin 型号、JetPack、功耗模式和时钟模式 | 待采集 |
| 容器镜像 ID | 待采集 |
| rosbag 或行走路线 | 待采集 |
| 采集人 | 待指派 |

三路相机、D-LIO、高程图、视觉评分和 Planner 全部稳定后再开始. 每项命令运行 600 秒, 并记录同一个开始时间

### 2.2 资源和日志

在 Orin 宿主机的独立终端执行:

```bash
date --iso-8601=seconds
timeout 600s tegrastats --interval 1000 | tee /tmp/wildos-tegrastats.log
```

```bash
timeout 600s docker stats wildos-orin-wildos-1 wildos-orin-cameras-1 | tee /tmp/wildos-docker-stats.log
```

在已加载 ROS workspace 的终端启动只读链路监测:

```bash
timeout 600s ros2 run graph_construction pipeline_performance_monitor | tee /tmp/wildos-pipeline.log
```

同时保存 WildOS 周期诊断. 这些日志包含图构建、视觉推理、消息时间差和 Planner 回调耗时:

```bash
timeout 600s scripts/wildos_docker.sh orin logs -f wildos | tee /tmp/wildos-runtime.log
```

`tegrastats` 记录 CPU、GPU、内存和温度, `docker stats` 记录容器 CPU 和内存. 链路监测记录消息年龄, 运行日志记录各节点平均、P95 或最大回调耗时. 容器实例名与 `docker ps --format '{{.Names}}'` 不一致时使用现场实际名称

### 2.3 图规模和消息大小

图构建周期日志中的 `局部节点/总节点` 和 `总边` 是图规模记录来源. 在另外两个终端对原始图和评分图各采集 10 分钟带宽:

```bash
timeout 600s ros2 topic bw /spot1/nav_graph | tee /tmp/wildos-nav-graph-bw.log
```

```bash
timeout 600s ros2 topic bw /spot1/scored_nav_graph | tee /tmp/wildos-scored-graph-bw.log
```

`ros2 topic bw` 输出平均消息大小和带宽. 基线需要分别记录开始、5 分钟和结束时的节点数、边数、平均消息大小, 用于判断图增长是否与行走范围一致

### 2.4 基线结果

| 指标 | 平均 | P95 或峰值 | 验收阈值 |
| --- | ---: | ---: | ---: |
| x86 CPU、内存、温度 | 待采集 | 待采集 | 待审核 |
| Orin CPU、GPU、显存、温度 | 待采集 | 待采集 | 待审核 |
| 点云、odom、图和 Path 消息年龄 | 待采集 | 待采集 | 待审核 |
| 图构建、视觉推理和规划耗时 | 待采集 | 待采集 | 待审核 |

| 时间点 | 节点数 | 边数 | `/spot1/nav_graph` 平均大小 | `/spot1/scored_nav_graph` 平均大小 |
| --- | ---: | ---: | ---: | ---: |
| 开始 | 待采集 | 待采集 | 待采集 | 待采集 |
| 5 分钟 | 待采集 | 待采集 | 待采集 | 待采集 |
| 10 分钟 | 待采集 | 待采集 | 待采集 | 待采集 |

没有修改前后的同场景数据时, 只能记录绝对值, 不能声称性能提升. 阈值由实机负责人审核后填写

## 3. 模型清单

`ckpts/manifest.json` 是模型来源、版本、大小、SHA256 和代码基线的唯一事实来源. `compatible_code_commit` 表示清单与代码共同评审的基线, 不代表尚未完成的实机性能验收

查看和校验元数据:

```bash
jq '{compatible_code_commit, compatibility_status, assets}' ckpts/manifest.json
sha256sum ckpts/manifest.json
git cat-file -e "$(jq -r .compatible_code_commit ckpts/manifest.json)^{commit}"
```

容器启动和模型反序列化前会校验每个文件的大小和 SHA256. 更新任何 checkpoint 时必须同时更新来源、版本、大小、SHA256、代码基线和兼容状态, 再完成软件测试及 Orin 实测

## 4. 外部 Path 执行契约

Planner 只发布 `nav_msgs/msg/Path`, 不控制底盘. 外部导航控制器必须满足:

- 把新 `Path` 当作路线变化事件而不是心跳
- 拒绝超过 2.0 秒或来自未来超过 0.1 秒的 `Path`
- odom 超过 1.0 秒未更新时立即停车并丢弃缓存路线
- 恢复前同时取得新鲜 odom 和新鲜 `Path`
- 自行负责局部避障、速度限制、急停和执行失败上报

仓库未包含外部控制器, 因此这些行为必须在集成测试中由控制器 owner 证明. Planner 的详细停止行为见 [Planner 说明](../graphnav_planner/README.md)

## 5. 网络和防火墙

当前部署只允许 x86 `192.168.50.1` 与 Orin `192.168.50.2` 通过专用有线链路交换 ROS 2 数据. `ROS_DOMAIN_ID=2` 和 Fast DDS 接口白名单不是身份认证

上线前必须:

- 用 VLAN 或主机防火墙把 ROS 接口入站源地址限制为唯一对端
- 记录 `ss -lunp` 的实际 DDS UDP 监听端口, 仅放行现场验证过的端口
- 将管理 SSH 和 ROS 数据置于不同接口或单独规则, 不向 ROS 网段开放无关服务
- 网络无法物理隔离时先启用 DDS Security, 否则不得上线

使用以下命令记录实际端口和对端流量:

```bash
ss -lunp
ros_interface=待填写
peer_address=待填写
sudo timeout 60s tcpdump -ni "${ros_interface}" "udp and host ${peer_address}"
```

| 协议 | 本机端口 | 对端地址 | 进程或用途 | 防火墙结果 |
| --- | --- | --- | --- | --- |
| UDP | 待采集 | 唯一部署对端 | Fast DDS discovery 或 user data | 待验证 |
| TCP | 待采集 | 管理网段 | SSH 或其他已批准管理服务 | 待验证 |

DDS 默认端口会受 domain、participant 和 Fast DDS profile 影响, 文档不写未经现场验证的固定端口范围. 防火墙采用默认拒绝, 只加入表中经过验证的协议、端口和来源. 变更前后都要验证发现、点云、图、目标和 Path, 具体权限边界见 [Docker 部署](docker.md#8-网络与容器权限边界)

## 6. 维护和故障责任

package manifest 中的 maintainer 负责代码评审入口, 不自动等同于现场值班责任人:

| package | manifest maintainer | 主要故障域 | 现场责任人 |
| --- | --- | --- | --- |
| `graph_construction` | WildOS Graph Construction Maintainers | D-LIO adapter、高程图、图构建和部署链路 | 待指派 |
| `visual_navigation`、`triangulation3d`、`object_search_msgs` | Hardik Shah | 视觉推理、目标融合和 Goal Mux | 待指派 |
| `graphnav_planner`、`graphnav_msgs` | Patrick Spieler | 图规划和 Path 发布 | 待指派 |
| 仓库外导航控制器 | 仓库未声明 | Path 执行、局部避障和停车 | 待指派 |

`graph_construction/package.xml` 当前邮箱为占位地址 `todo@example.com`, 现场责任人也尚未在仓库中声明. 两项未补齐前不得把责任边界标记为已完成

故障先按 topic owner 定位: 无源数据查传感器和部署, 时间或 TF 异常查定位链, 图异常查 graph construction, 评分或目标异常查 visual navigation 和 triangulation, Path 异常查 Goal Mux 和 Planner, 路径存在但底盘不执行查外部控制器

## 7. 注释约定

- 不注释简单赋值和明显流程
- 复杂状态转换、安全降级、时间同步和坐标变换说明原因
- 注释保持简洁, 使用英文标点, 不使用句号
