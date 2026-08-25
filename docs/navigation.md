# GO2 导航

## 职责

`wildos_navigation` 原样迁移导航同事实机工作区中的三段算法:

- `map_pub`: 使用注册点云和 odom 构建滚动二维代价地图
- `astar`: 接收 `/goal_pose`, 在代价地图上生成局部路径
- `start_nav`: 跟踪路径, 使用注册点云判断静态和动态碰撞并发布速度

本项目只适配 Topic、frame、点云 QoS、容器和跨机速度链路, 不重新设计导航算法或调整算法常量

原工作区的 `odom_map_tf` 不在当前 launch 中启动. D-LIO 已经是 `odom -> dlio_odom -> base_link` 的唯一 TF owner, 重复启动会让两个节点同时发布 `base_link` 动态 TF

## Topic 契约

| 数据 | Topic | 类型 |
| --- | --- | --- |
| 目标点 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` |
| odom | `/odom` | `nav_msgs/msg/Odometry` |
| 注册点云 | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` |
| 局部代价地图 | `/combined_grid` | `nav_msgs/msg/OccupancyGrid` |
| 局部路径 | `/path` | `nav_msgs/msg/Path` |
| 导航速度 | `/cmd_vel` | `geometry_msgs/msg/Twist` |

`/cloud_registered` 使用传感器 QoS. 当前 D-LIO 对齐为 identity, `/odom` 和注册点云数值处于同一局部世界坐标, 导航输出 frame 统一为 `odom`

## 数据流

```text
AGX Goal Mux /goal_pose
  -> x86 map_pub + astar
       + /odom
       + /cloud_registered
  -> /combined_grid -> /path
  -> x86 start_nav -> /cmd_vel, FastDDS Domain 2
  -> x86 velocity sender -> 192.168.50.2:9999/UDP
  -> AGX velocity receiver -> /cmd_vel, CycloneDDS Domain 0
  -> motion_node -> /api/sport/request -> GO2
```

GraphNav Planner 的 `/spot1/graphnav_planner/path` 不作为这套已测试算法的控制输入. 它可以继续用于 WildOS 图规划验证, 但不能直接驱动 `start_nav`

x86 和 GO2 使用不同 DDS domain, 两端可以使用相同的 `/cmd_vel` 名称而不会形成回环. `motion_node` 只负责把 `Twist` 适配到 GO2 Sport API, 不属于导航算法

## 配置边界

[navigation.yaml](../wildos_navigation/config/navigation.yaml) 保存原工作区的代价地图参数和当前部署接口. 路径跟踪速度、前视距离、安全半径和动态障碍预测参数仍保留在迁移源码中, 避免部署侧形成第二套算法配置

## 构建和管理

```bash
scripts/wildos_docker.sh x86 config
scripts/wildos_docker.sh x86 build navigation
scripts/wildos_docker.sh x86 start navigation
scripts/wildos_docker.sh x86 status
scripts/wildos_docker.sh x86 logs -f navigation
scripts/wildos_docker.sh x86 restart navigation
scripts/wildos_docker.sh x86 stop navigation
```

AGX 运动网关:

```bash
scripts/go2_motion_gateway.sh install
scripts/go2_motion_gateway.sh start
scripts/go2_motion_gateway.sh status
scripts/go2_motion_gateway.sh logs
scripts/go2_motion_gateway.sh stop
```

`install` 不启动服务且禁止开机自启

## 无运动验证

保持 AGX 运动网关关闭, 只启动 x86 导航容器:

```bash
ros2 node list | grep -E '^/(map_pub|astar|start_nav|wildos_velocity_sender)$'
ros2 topic info /goal_pose -v
ros2 topic hz /odom
ros2 topic hz /cloud_registered
ros2 topic hz /combined_grid
ros2 topic echo /path --once
ros2 topic echo /cmd_vel
```

尚未收到目标点和路径时 `start_nav` 不发布速度. AGX 接收端在速度链路断流 0.25 秒后发布零速度

## 运动前验证

机器狗必须位于空旷区域并可立即遥控接管. 先保持导航容器关闭, 单独启动 AGX 网关并确认 `/cmd_vel` 没有非零输出

完整目标搜索运动测试统一在 x86 主机使用 `scripts/go2_search_test.sh`, 启动、停止、切换目标和日志命令见[目标搜索](object_search.md)

首次运动只验证短距离直行、停车和转向. 任一方向、速度或障碍行为不符合导航同事算法预期时立即停止导航容器, 不在现场修改算法参数

## 测试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select wildos_navigation --symlink-install
source install/setup.bash
PYTHONPATH=wildos_navigation python3 -m pytest wildos_navigation/test -p no:cacheprovider
```

测试覆盖当前 Topic、启动节点、速度链路协议和网关边界. 导航算法行为以导航同事的原工作区测试结果为准
