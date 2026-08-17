# 论文风格可视化和 RViz 性能优化

日期: 2026-07-16

## 1. 修改目标

本次修改解决三个显示问题:

- 高程图随高度自动变色, 难以获得稳定一致的论文截图
- 目标定位只有目标球和粒子, 缺少论文中的目标射线关系
- RViz2 直接显示高频完整 LiDAR 点云时帧率降到个位数

本次只调整可视化输出和 RViz 显示链路, 不改变 elevation mapping、目标融合和路径规划使用的数据

## 2. 论文风格 RViz 配置

新增文件:

```text
graph_construction/rviz/wildos_paper.rviz
```

主要显示规则:

- 黑色背景
- GridMap 继续使用 `elevation` 作为高度层
- GridMap 颜色使用 `FlatColor`, 固定为浅灰色
- 保留 GridMap 网格线, 便于观察真实高程表面
- 当前导航图节点为亮绿色
- 历史节点为低亮度绿色
- Frontier 节点为蓝色, Frontier points 为紫色
- 导航图边为半透明红色
- 规划路线和目标观测射线为绿色
- 稳定目标为青色球体
- 视觉粗目标继续使用黄色, 避免把未稳定目标误认为最终定位
- 目标粒子使用白色点

固定地面颜色只改变 RViz 着色, 不会把高程数据改成平面, 地面的真实 Z 高度仍由 `elevation` layer 决定

## 3. `/spot1/graph_construction_viz` 红色图元

该 MarkerArray 中原有两类红色图元:

### 3.1 红色细线

namespace:

```text
edges
```

含义是稀疏导航图节点之间的可通行边, Planner 使用相同图结构计算路线

该显示对判断图是否断裂、错误跨障碍连接和路线连通性有价值, 因此继续保留

为提高复杂地形和高程网格背景下的辨识度, 红色边线宽由 `0.02 m` 小幅增加到 `0.03 m`, 仅改变 RViz 显示效果

### 3.2 半透明红色地面圆

namespace:

```text
free_radius
```

含义是节点的局部自由半径, 只用于 Graph Construction 专项调试, Planner 不依赖这些 Marker

旧实现还会显示 `explored_radius` 青色地面圆, 两类圆都为每个当前节点建立独立 `CYLINDER` Marker, 节点较多时会增加 RViz 场景对象数量

新增配置:

```yaml
viz_show_radius_markers: false
```

默认不再发布 `free_radius` 和 `explored_radius` 圆, 需要专项调试时可临时改为 `true`

### 3.3 青蓝色局部地图边框

namespace:

```text
grid_footprint
```

该矩形只表示当前 rolling GridMap 的覆盖范围, 不参与高程建图、导航图生成、Frontier 检测、路径规划或目标定位

为减少日常画面中的非必要图元, 已删除该 Marker 的生成代码和 `GraphVisualizer` 的 grid 可视化输入, 不影响算法继续使用 `ClassifiedGrid`

## 4. 目标定位视觉效果

修改文件:

```text
visual_navigation/visual_navigation/object_target_fusion.py
```

`/spot1/object_target_estimate_viz` 继续使用现有 `visualization_msgs/Marker` 类型, 同一话题发布两个 namespace:

```text
object_target_estimate
object_target_rays
```

显示语义:

- `TRACKING`, 黄色目标球和绿色相机到目标射线
- `STABLE_VISION`, 青色目标球和绿色相机到目标射线
- `LIDAR_LOCKED`, 青色目标球和绿色相机到目标射线
- `PENDING`, 删除目标球和射线, 避免显示不可靠单视角深度

目标球尺寸继续表示有界协方差范围, 不是固定装饰球

`/spot1/object_target_particles` 保留为白色粒子点云, 用于显示多视角粒子收敛过程

## 5. RViz 点云性能优化

最终实现不新增 LiDAR 显示 topic, 不发布:

```text
/spot1/lidar_viz
```

论文风格 RViz 配置不订阅 `/livox/lidar` 或 `/livox/lidar_aligned`, 只显示 elevation mapping 输出的高程表面、导航图和目标定位结果

完整 `/livox/lidar_aligned` 继续只供 elevation mapping 和目标 LiDAR 融合使用, 可视化调整不改变算法输入

RViz2 已经通过 Ogre/OpenGL 使用 GPU 渲染, 但完整点云的 DDS 反序列化、TF 变换、场景对象更新和 CPU 到 GPU 数据上传仍在 CPU 侧产生明显成本

因此单纯更换 GPU 或打开某个 GPU 开关不能解决高频大点云卡顿, 默认不显示完整雷达点云是当前最稳定的方案

如果临时排查原始点云, 可以在 RViz 手动添加 PointCloud2, 排查完成后应立即关闭该 Display

## 6. 启动方式

默认启动保持不自动打开 GUI, 避免无显示环境或远程服务器启动失败

需要论文风格 RViz 时运行:

```bash
./scripts/start_wildos_elevation.sh \
  do_object_search:=true \
  launch_paper_rviz:=true
```

论文风格配置默认不显示雷达 PointCloud2, 不要长期重新添加 `/livox/lidar` 或 `/livox/lidar_aligned` 的完整点云 Display

## 7. 验证结果

功能测试:

```text
graph_construction + visual_navigation: 78 passed
```

新增回归覆盖:

- 默认不发布自由半径和探索半径圆
- 不再发布 `grid_footprint` 局部地图矩形框
- 导航图红色边线宽固定为 `0.03 m`
- 专项配置仍可恢复半径圆
- 稳定目标球使用青色
- 有效目标发布绿色相机射线
- `PENDING` 状态删除目标射线

编译结果:

```text
graph_construction: passed
visual_navigation: passed
```

安装和 launch 检查:

```text
wildos_paper.rviz: installed
launch_paper_rviz arguments: found
```

`/spot1/lidar_viz` 发布节点、profile 参数、launch 参数和 RViz Display 均已删除
