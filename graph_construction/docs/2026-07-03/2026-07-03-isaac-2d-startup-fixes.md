# Isaac 2D 启动问题修复

日期: 2026-07-03

## 背景

用户在 Isaac 仿真环境运行:

```bash
bash scripts/start_wildos_2d.sh
```

日志暴露出四个问题:

- `wildos` 使用系统 Python 启动, 找不到 `.venv` 中的 `omegaconf`
- 修复 `omegaconf` 后继续暴露缺少 `.venv` 中 `explorfm`
- camera static TF 使用旧参数格式, `static_transform_publisher` 输出 deprecated warning
- 2D LiDAR grid builder 在 Isaac profile 下缺少 `lidar -> map` TF 链接
- Isaac LiDAR 单帧可能是旋转扫描中的一个扇区, 旧 grid builder 只用最新一帧重建地图, RViz 中表现为栅格随雷达慢慢转圈显示

同时要求项目日志改为中文显示

## 问题分析

`wildos` 报错:

```text
ModuleNotFoundError: No module named 'omegaconf'
```

修复后继续报:

```text
ModuleNotFoundError: No module named 'explorfm'
```

本地 `.venv` 可以导入 `omegaconf` 和 `explorfm`, 但 install tree 中的 console script 仍是:

```text
#!/usr/bin/python3
```

旧启动脚本通过 `command -v wildos` 查找 console script, 但 ROS install 中的 `visual_navigation/lib/visual_navigation/wildos` 不在 `PATH` 中, 所以 shebang 修复逻辑没有真正执行

后续再次出现 `explorfm` 报错时, 命令行已经显示使用 `.venv/bin/python3`, 说明问题不再是 Python 解释器, 而是 `explorfm` 位于仓库根目录下, 没有安装进 venv site-packages

直接在仓库根目录执行 `python -c "import explorfm"` 会成功, 是因为当前目录自动进入 `sys.path`, 但 ROS launch 启动 `wildos` 时工作目录不是仓库根目录, 因此仍会找不到 `explorfm`

`static_transform_publisher` warning 来自旧式参数:

```text
x y z qx qy qz qw parent child
```

Humble 下应使用 `--x`, `--y`, `--z`, `--qx`, `--qy`, `--qz`, `--qw`, `--frame-id`, `--child-frame-id`

Isaac 2D 当前需要给 LiDAR frame 提供 fallback static TF, 否则 `livox_grid_builder` 可能在启动早期看不到 `lidar -> map` 连通链路

2D 栅格地图是 LiDAR 观测得到的局部 OccupancyGrid, 不是仿真器的全局真值地图, 因此不会直接把整张场景栅格全部显示出来

旧实现每次只使用最新 PointCloud2, 如果 Isaac LiDAR 一条消息只包含旋转扫描中的一段, 局部地图就会看起来像一个随雷达转动的扇形, 正确做法是短时累计最近扫描帧, 用累计窗口生成当前局部栅格

## 修改内容

### `start_wildos_2d.sh`

- source ROS 后先 source install, 再重新激活项目 `.venv`
- 使用 venv Python 检查 `omegaconf` 和 `explorfm`
- shebang 修复从 `command -v` 扩展为搜索 install tree 中 `*/lib/*/<executable>`
- 修复 `wildos` 和 `odom_frame_adapter` 的 shebang
- 启动 2D launch 时显式传入 `wildos_python_executable`, 让 WildOS 使用当前 venv Python
- 启动前将仓库根目录加入 `PYTHONPATH`, 让非仓库工作目录下的 ROS 子进程也能导入 `explorfm`
- 脚本自身提示改成中文

### `wildos_2d_sim.launch.py`

- 新增 `publish_lidar_static_tf`, 默认 `true`
- 新增 `lidar_parent_frame`, `lidar_frame` profile override
- 新增 `wildos_python_executable`, 非空时用指定 Python 执行 WildOS console script
- WildOS 节点通过 `additional_env` 补充仓库根目录 `PYTHONPATH`
- 为 Isaac 2D 默认发布 `base_link -> lidar` fallback static TF
- camera static TF 改成新式参数, 消除 deprecated warning

### `livox_grid_builder.py`

- 新增短时点云累计窗口, 默认累计最近 `4.0s` 或最多 `40` 帧
- 每帧点云先转换到 `grid_frame`, 再进入累计窗口
- 每次发布局部 OccupancyGrid 时使用累计窗口内所有点云重建 grid
- 未被射线观测过的 cell 仍保持 `unknown`, 避免把未观测空间错误标成 free
- 新增中文日志显示累计扫描状态

### `livox_grid_builder.yaml`

- 新增 `scan_accumulation_time_sec: 4.0`
- 新增 `max_accumulated_scans: 40`

### 中文日志

已改为中文的项目日志:

- `livox_grid_builder`
- `graph_construction`
- `odom_frame_adapter`
- `start_wildos_2d.sh` 脚本提示

ROS launch 自带的 `[INFO] [launch]` 和第三方节点日志不由本项目控制, 保持原样

## 当前 Isaac 2D 默认行为

- 默认 profile: `isaac`
- 默认 LiDAR topic: `/unitree_go2/lidar/points`
- 默认 LiDAR frame: `lidar`
- 默认 LiDAR parent frame: `base_link`
- 默认 grid frame: `map`
- 默认发布 LiDAR fallback static TF: `publish_lidar_static_tf=true`
- 默认 WildOS Python: 由 `start_wildos_2d.sh` 传入项目 `.venv/bin/python3`
- 默认 WildOS `PYTHONPATH`: 仓库根目录优先, 用于导入本地 `explorfm`
- 默认 2D 点云累计窗口: `scan_accumulation_time_sec=4.0`, `max_accumulated_scans=40`
- 如仿真器已经发布 `base_link -> lidar`, 可用以下命令关闭 fallback:

```bash
./scripts/start_wildos_2d.sh publish_lidar_static_tf:=false
```

## 验证

已执行:

```bash
bash -n scripts/start_wildos_2d.sh
python3 -m py_compile graph_construction/launch/wildos_2d_sim.launch.py graph_construction/graph_construction/livox_grid_builder.py graph_construction/graph_construction/node.py visual_navigation/visual_navigation/utils/odom_frame_adapter.py
bash scripts/start_wildos_2d.sh --show-args
source .venv/bin/activate && python -c "import omegaconf; import explorfm"
```

结果:

- shell 语法检查通过
- Python 语法检查通过
- `--show-args` 解析通过, 未启动节点
- installed `wildos` 和 `odom_frame_adapter` shebang 已修到项目 `.venv/bin/python3`
- 2D launch 已暴露 `publish_lidar_static_tf`, `lidar_parent_frame`, `lidar_frame`
- 2D launch 已暴露 `wildos_python_executable`, 并由脚本传入 `.venv/bin/python3`
- `.venv` 已确认可导入 `omegaconf` 和 `explorfm`
- `livox_grid_builder` 已改为累计最近扫描帧, 避免只显示当前旋转扇区

## 后续

- 如果运行时仍看到 `lidar -> map` TF 断链, 先检查 Isaac 是否发布 `odom -> base_link`
- 如果出现重复 static TF, 使用 `publish_lidar_static_tf:=false` 关闭 fallback
- 如果重新 build 后 shebang 变回 `/usr/bin/python3`, 再运行 2D 启动脚本会自动修正, 且 launch prefix 会继续强制 WildOS 使用 venv Python
- 如果 WildOS 使用 venv Python 仍报 `No module named 'explorfm'`, 检查子进程 `PYTHONPATH` 是否包含仓库根目录
- 如果需要更快消除旧障碍残留, 调小 `scan_accumulation_time_sec`
- 如果 Isaac LiDAR 单圈扫描时间更长, 适当调大 `scan_accumulation_time_sec` 或 `max_accumulated_scans`

## Unity / Zenoh 运行补充

用户切到 `ROS_DOMAIN_ID=89` 和 `rmw_zenoh_cpp` 后, 实测 live graph 位于 Unity profile:

```text
ros2 launch graph_construction wildos_2d_sim.launch.py topic_profile:=unity
```

诊断结果:

- `/spot1/traversability_grid` 有消息
- `/spot1/odom_for_scoring` 无消息
- `/spot1/nav_graph` 和 `/spot1/graph_construction_viz` 无消息
- `/spot1/nav_graph_viz` 无 publisher, 因为 WildOS 进程已退出

直接原因是 Unity profile 下默认仍使用 Isaac 的 odom adapter 策略:

```text
pose_source=tf
fallback_to_message=false
```

当 Unity 没有稳定提供 `map -> base_link` TF 时, `odom_frame_adapter` 会丢弃 `/unity/odom`, 后续 graph construction 因缺少 odom 不发布 graph 和 viz

已补充 profile 策略:

- Isaac 2D: `odom_pose_source_2d=tf`, `odom_fallback_to_message_2d=false`
- Unity 2D: `odom_pose_source_2d=message`, `odom_fallback_to_message_2d=true`
- robot 2D: `odom_pose_source_2d=message`, `odom_fallback_to_message_2d=true`

重启后继续发现 `odom_frame_adapter` 立即退出, launch 参数文件中 `fallback_to_message` 被写成字符串:

```text
fallback_to_message: 'true'
```

`odom_frame_adapter` 声明该参数为 bool, rclpy 会在节点初始化阶段因为类型不匹配退出, 日志文件可能为空

已在 `wildos_2d_sim.launch.py` 中新增 bool 解析辅助函数, profile 或命令行传入的 `true/false` 会转换成真正的 Python bool 后再写入参数文件

后续 WildOS 继续退出时, traceback 指向:

```text
OmegaConf.from_dotlist(custom_args.config_override)
yaml.parser.ParserError
in "<unicode string>", line 1, column 3:
  {}_camera
```

直接原因是 launch 生成了未加引号的 dotlist 覆盖项:

```text
--config-override cam_frame={}_camera
--config-override camera_img_topic=/camera/{}/color/image/compressed
```

OmegaConf 会用 YAML 解析 dotlist 右侧值, `{}` 在 YAML 中有结构含义, `{}_camera` 不是合法 YAML 标量

已在 2D 和 3D launch 的 `_config_override_args` 中增加字符串标量转义, 传给 WildOS 和 graph/livox 的覆盖值会变成:

```text
cam_frame="{}_camera"
camera_img_topic="/camera/{}/color/image/compressed"
```

graph/livox 自己的 `yaml.safe_load` 也能正常解析这种带引号字符串

命令行仍可显式覆盖:

```bash
./scripts/start_wildos_2d.sh topic_profile:=unity odom_pose_source:=tf odom_fallback_to_message:=false
```
