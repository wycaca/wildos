# WildOS 实机实现原则

> 本文记录当前代码必须保持的行为

## 1. 地图

- 唯一几何输入是 elevation `GridMap`
- `free` 表示当前确认可通行
- `obstacle` 表示当前明确不可通行
- `unknown` 表示当前看不到, 不直接否定历史安全路线
- 高程图先初始化脚下局部地面, 再融合第一帧点云
- 启动先验只能连接近场真实地面, 不能越过墙体或障碍
- obstacle 始终优先于人工安全先验
- 图更新失败时保留上一张完整有效图

## 2. 导航图

- 节点只生成在机器人可达的 free 区域
- 节点使用稳定 UUID
- 新边不能穿过 obstacle 或 unknown
- 新障碍必须删除冲突节点和边
- free 变 unknown 时保留已确认的历史安全边
- 启动人工区域固定在初始世界坐标, 不能跟随机器人移动
- 内部增量更新, 对外仍发布完整 `NavigationGraph`
- 图更新范围不能随历史图总大小线性增长

## 3. 定位和 TF

- x86 D-LIO 是 odom、动态 TF 和注册点云的统一位姿源
- D-LIO 直接订阅 MID360 点云和 IMU
- 点云逐点时间用于运动去畸变
- 同一个 TF child frame 只能有一个 owner
- 不能通过只改 `frame_id` 冒充坐标变换
- 定位异常时暂停不可信输出
- 错误位姿污染高程图后必须重启地图

## 4. 视觉和目标定位

- 前向 D435if, 左右 D435i
- 三路图像、CameraInfo、LiDAR、odom 和 TF 使用同一时间基准
- WildOS 使用测量时刻 TF, 不使用回调时最新 TF 替代
- 单视角候选不能直接接管导航
- 完全重复帧不累计证据
- LiDAR 是近距离精修证据, 单帧 LiDAR 不能覆盖已有视觉目标
- `ObjectSearchGoalMux` 是高层目标和最终完成状态的唯一 owner

## 5. ROS 和部署

- 两台主机使用 Domain 2、FastDDS 和非 localhost 模式
- x86 运行 `lidar` 和 `localization`
- 相机 AGX 运行 `cameras` 和 `wildos`
- `/cloud_registered` 只由相机 AGX 上一个点云适配器跨机订阅
- debug topic 无订阅者时不构造高成本消息
- 不增加职责重复的 launch、Compose 或适配节点

## 6. 验证

1. 检查原始点云、IMU、图像和 CameraInfo
2. 检查时间戳持续递增
3. 检查 `odom -> base_link` 和传感器 TF
4. 检查高程图、导航图和评分图
5. 检查视觉输出、目标估计和 Path
6. 性能或状态修改至少运行 10 分钟
7. 测试结束后保持部署服务运行, 除非明确要求停止

## 7. 文档

- `docs/details/` 只描述当前实机实现
- 日期目录保留历史过程
- topic、参数和模块职责变化后同步更新长期文档
