# GO2 局部导航

## 职责

`wildos_navigation` 不重复构建地图或运行全局 A*. WildOS GraphNav Planner 负责生成全局 Path, 本模块只负责:

- 在 x86 上跟踪 `/spot1/graphnav_planner/path`
- 使用本机 `/cloud_registered` 做短时局部碰撞检查
- 使用 `/odom` 计算机器人相对路径位置
- 发布 `/wildos/cmd_vel`
- 在输入断流、TF 失败、路径无效或前方碰撞时立即停车

当前控制器使用 Pure Pursuit 前视跟踪和恒定速度轨迹碰撞检查. 移动物体按最新点云中的当前障碍处理, 不使用缺少可靠数据关联的单目标预测

## 数据流

```text
AGX GraphNav Planner
  -> /spot1/graphnav_planner/path
  -> x86 navigation controller
       + /odom
       + /cloud_registered
       + odom -> dlio_odom -> base_link TF
  -> /wildos/cmd_vel
  -> x86 velocity sender
  -> 192.168.50.2:9999/UDP
  -> AGX velocity receiver
  -> /cmd_vel
  -> motion_node
  -> /api/sport/request
  -> GO2
```

感知和规划使用 FastDDS Domain 2. GO2 底层使用 CycloneDDS Domain 0, 两个 DDS 域只通过经过校验和限幅的速度报文连接

## 安全边界

- Path 必须位于 `odom` 且首次接收时间有效
- 注册点云必须位于 `dlio_odom`, 按测量时间转换到 `base_link`
- odom 或点云超过配置租约立即发布零速度
- Planner publisher 消失时立即发布零速度
- 预测轨迹进入局部障碍安全半径时立即发布零速度
- 安全停车绕过加速度斜坡, 正常起步和转向保留加速度限制
- AGX 网关只接受专用链路 x86 地址并校验固定长度、协议版本和 CRC
- AGX 网关独立限幅, 远端速度断流后主动发布零速度
- `motion_node` 保留自身 0.5 秒超时停车和遥控器抢占

具体参数以 `wildos_navigation/config/navigation.yaml` 为准. 初次实机采用保守速度, 制动距离和转向方向通过后才能提高

## 构建和管理

x86 导航容器:

```bash
scripts/wildos_docker.sh x86 config
scripts/wildos_docker.sh x86 build navigation
scripts/wildos_docker.sh x86 start navigation
scripts/wildos_docker.sh x86 status
scripts/wildos_docker.sh x86 logs -f navigation
scripts/wildos_docker.sh x86 restart navigation
scripts/wildos_docker.sh x86 stop navigation
```

相机 AGX 宿主机网关:

```bash
scripts/go2_motion_gateway.sh install
scripts/go2_motion_gateway.sh start
scripts/go2_motion_gateway.sh status
scripts/go2_motion_gateway.sh logs
```

`install` 不启动服务且禁止开机自启. 实机运动验收完成前只能在现场手动 `start`

## 无运动验证

保持网关关闭, 只启动导航容器. 未提供 Path 时 `/wildos/cmd_vel` 必须持续为零:

```bash
ros2 topic hz /wildos/cmd_vel
ros2 topic echo /wildos/cmd_vel
ros2 node info /wildos_navigation_controller
ros2 node info /wildos_velocity_sender
```

检查输入和 TF:

```bash
ros2 topic hz /odom
ros2 topic hz /cloud_registered
ros2 topic info /spot1/graphnav_planner/path -v
ros2 run tf2_ros tf2_echo base_link dlio_odom
```

## 运动前验证

机器狗必须处于可立即遥控接管的空旷区域. 先启动 AGX 网关, 保持导航容器关闭:

```bash
scripts/go2_motion_gateway.sh start
scripts/go2_motion_gateway.sh status
scripts/go2_motion_gateway.sh logs
```

确认 `/cmd_vel` 没有非零输出, 再启动导航容器. 首次测试只验证短距离直行、停车和原地转向, 任一方向错误立即停止 `navigation`

## 测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select wildos_navigation --symlink-install
source install/setup.bash
python3 -m pytest wildos_navigation/test -p no:cacheprovider
```

测试覆盖路径跟踪、原地转向、障碍停车、自体和地面过滤、TF 点变换、加速度限制、速度协议校验、网关限幅和当前 GO2 Topic 契约
