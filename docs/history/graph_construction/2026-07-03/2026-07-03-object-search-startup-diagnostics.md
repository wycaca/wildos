# 目标搜索启动诊断

日期: 2026-07-03

## 背景

使用 2D Unity 启动目标搜索:

```bash
bash scripts/start_wildos_2d.sh do_object_search:=true
```

用户反馈:

- 启动日志中没有目标搜索相关日志
- `/spot1/score_rings` 没有成功发布

## 现场结论

本次 live graph 中没有 `/wildos` 节点:

```text
ros2 node list --no-daemon
```

能看到:

```text
/spot1/graph_construction
/spot1/graphnav_planner
/spot1/graphnav_path_follower
```

但没有:

```text
/wildos
```

Topic 状态:

```text
/spot1/score_rings: Publisher count = 0, Subscription count = 1
/spot1/scored_nav_graph: Publisher count = 0, Subscription count = 1
/spot1/model_visualization: topic 不存在
```

因此当前不是 `score_rings` 内容为空, 而是 WildOS 进程没有保持运行, 所以没有发布者

## 启动时序

2D launch 中 WildOS 是延迟启动:

```text
visual_start_delay = 6.0s
planner_start_delay = 7.0s
```

用户粘贴的前半段日志只到 graph construction 首帧, 还没包含 WildOS 的完整初始化输出

`launch.log` 显示 WildOS 进程确实被启动:

```text
[python3-8]: process started
```

但 live graph 后续没有 `/wildos`, 说明该进程已经退出或被中断

## 修改内容

为便于下次定位, 已做以下增强:

1. 2D / 3D launch 中 WildOS 节点 `output` 从 `screen` 改为 `both`
2. `wildos/nav.py` 在模型初始化和目标搜索文本特征计算前后打印中文状态
3. WildOS 目标搜索启用时, RADIO 主模型优先解析为本地 `ckpts/c-radio_v3-b_half.pth.tar`
4. WildOS SigLIP2 adaptor cache 优先解析为完整缓存目录 `ckpts/siglip2`
5. SigLIP2 adaptor 会把 HuggingFace repo id 解析成本地 snapshot 路径, 并使用 `local_files_only=True`
6. `ExploRFM` 增加 RADIO backbone, traversability head 和 frontier head 的阶段日志

下次日志中应能看到:

```text
WildOS 模型初始化开始, object_search=True
WildOS 加载视觉模型, model=/.../ckpts/c-radio_v3-b_half.pth.tar, adaptor=siglip2, adaptor_path=/.../ckpts/siglip2
ExploRFM 开始加载 RADIO backbone, model=/.../ckpts/c-radio_v3-b_half.pth.tar, adaptor=siglip2, adaptor_path=/.../ckpts/siglip2
ExploRFM RADIO backbone 加载完成
ExploRFM 开始加载 traversability head, ckpt=...
ExploRFM 开始加载 frontier head, ckpt=...
WildOS 模型加载完成
WildOS 计算目标搜索文本特征, text_queries=[...]
WildOS 目标搜索文本特征计算完成
WildOS 模型初始化完成
目标搜索已启用, text_queries=[...]
```

如果这些日志没有出现, 说明 WildOS 尚未进入模型初始化或输出未被当前终端捕获

如果只出现前几行后退出, 说明断点在模型加载或 text feature 计算阶段

## 模型加载卡住点

如果日志停在:

```text
WildOS 加载视觉模型, model=c-radio_v3-b, adaptor=siglip2
```

说明已经进入 `ExploRFMInference(...)` 构造函数, 但还没有返回

本次排查到的高风险点:

- `wildos/nav.py` 原先传入 `model_version=c-radio_v3-b`, `nvidia_radio/hubconf.py` 会走 `load_state_dict_from_url`
- `wildos/nav.py` 原先传入 `adaptor_ckpt_path=ckpts`, 但完整 SigLIP2 cache 位于 `ckpts/siglip2`
- `ckpts/models--google--siglip2-so400m-patch16-naflex` 下存在 `.incomplete` 文件, 表示根 `ckpts` 下的 SigLIP2 cache 不完整

因此目标搜索启动时可能命中不完整 cache 或回退到网络等待

修复后应优先走本地文件:

```text
ckpts/c-radio_v3-b_half.pth.tar
ckpts/siglip2/models--google--siglip2-so400m-patch16-naflex
```

已验证:

```bash
.venv/bin/python3 -c "from pathlib import Path; from transformers import AutoConfig, AutoProcessor; cache=Path('ckpts/siglip2'); revision=(cache / 'models--google--siglip2-so400m-patch16-naflex' / 'refs' / 'main').read_text().strip(); path=cache / 'models--google--siglip2-so400m-patch16-naflex' / 'snapshots' / revision; AutoConfig.from_pretrained(path, local_files_only=True, trust_remote_code=True); AutoProcessor.from_pretrained(path, local_files_only=True, trust_remote_code=True); print(path)"
```

该命令可离线读取本地 snapshot, 不再访问 `huggingface.co`

## 正确验证方式

重启后等待至少 10 秒, 再检查:

```bash
ros2 node list --no-daemon | grep wildos
ros2 topic info /spot1/score_rings --no-daemon
ros2 topic info /spot1/scored_nav_graph --no-daemon
ros2 topic info /spot1/model_visualization --no-daemon
```

期望:

```text
/wildos 存在
/spot1/score_rings Publisher count >= 1
/spot1/scored_nav_graph Publisher count >= 1
/spot1/model_visualization Publisher count >= 1
```

如果 `/wildos` 存在但 publisher 仍为 0, 再查三相机 topic, odom, nav graph 和 TF 同步

如果 `/wildos` 不存在, 优先看本次新增的 WildOS 日志文件

## 注意

当前目标搜索文字不是运行时 topic 输入, 而是启动时读取:

```yaml
visual_navigation/configs/wildos_nav_sim_conf.yaml
object_search_config.text_queries
```

当前代码只支持单个 query
