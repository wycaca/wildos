# 目标搜索日志语义整理

日期: 2026-07-16

## 1. 修改背景

运行时曾出现以下连续日志:

```text
目标证据待确认, evidence=1/2, window=3/3
目标视觉证据已确认, evidence=2/2, window=3/3
探索分支延伸, reason=continuation
当前帧未检测到目标, threshold=0.100, 相机最高相似度=right/0.119
```

这些日志容易产生两个误解:

- `threshold=0.100` 看似表示 `0.119` 已超过检测门槛, 实际 `0.100` 只是生成二值 Mask 的门槛, 最终峰值门槛是 `0.120`
- `探索分支延伸` 看似表示 Planner 仍在执行初始探索, 实际目标接近模式也会通过 Frontier 中继点逐步接近目标

视觉窗口通过只表示当前滑动窗口满足发布 Mask 的条件, 不等于全局融合目标首次建立或最终完成

## 2. 日志语义分层

目标搜索日志按以下链路组织:

```text
当前视觉帧
  -> 视觉确认窗口
  -> 目标融合
  -> 导航目标仲裁
  -> Planner 路线
```

每一层只描述自己拥有的状态, 不再使用容易跨层理解的笼统描述

### 2.1 当前视觉帧

文件:

```text
visual_navigation/visual_navigation/object_detection_filter.py
visual_navigation/visual_navigation/wildos/nav.py
```

过滤器新增每路相机的拒绝诊断, 区分:

- 没有超过 Mask 阈值的连通区域
- 连通区域像素不足
- 连通区域峰值低于最终峰值门槛

典型输出:

```text
当前帧未形成有效目标, 连续帧数=1, 原因=右相机/峰值不足/峰值=0.119<0.120/区域像素=3762, Mask阈值=0.100, 视觉窗口=仍满足确认门槛, 本帧不发布目标Mask, 已有融合目标不会被当前帧清除
```

该日志明确表示:

- 当前帧因为峰值不足被拒绝
- 本帧不发布新的目标 Mask
- 当前窗口可能仍然满足 `2/3` 门槛
- 融合节点已保存的历史目标不会因此被清除

正常丢帧使用 `INFO`, 首帧和配置间隔帧输出, 避免把正常视觉波动误报为异常

### 2.2 视觉确认窗口

窗口日志改为以下三种状态:

```text
视觉窗口正在累计
当前视觉窗口已通过
视觉窗口已失效
```

日志使用中文字段 `有效帧`、`窗口占用`、`候选`，不再使用缺少上下文的 `evidence`、`window`、`components`

窗口失效时明确说明历史融合目标继续由融合节点维护

### 2.3 目标融合

文件:

```text
visual_navigation/visual_navigation/object_target_fusion.py
```

融合状态在中文说明后保留协议状态码:

```text
等待多视角(PENDING)
多视角跟踪(TRACKING)
视觉稳定(STABLE_VISION)
雷达锁定(LIDAR_LOCKED)
任务完成(REACHED)
```

状态变化日志同时输出位置、置信度、有效视角和雷达支持点

### 2.4 导航目标仲裁

文件:

```text
visual_navigation/visual_navigation/object_search_goal_mux.py
```

Goal Mux 日志明确区分:

- 按初始方向探索
- 接近视觉粗目标
- 接近稳定融合目标
- 目标到达观察点

视觉粗目标或稳定融合目标接管导航时记录目标位置、融合状态、来源、置信度和有效视角

`/spot1/object_search_status` 的消息内容继续保留英文协议状态码, 仅运行日志增加中文含义, 不改变模块间接口

### 2.5 Planner 路线

文件:

```text
graphnav_planner/src/planner.cpp
graphnav_planner/src/planner_node.cpp
```

原来的统一名称 `探索分支` 拆分为:

- `初始方向探索分支`
- `目标接近中继分支`
- `直接目标路线`

目标附近已经存在可达导航图节点时, Planner 输出直接目标路线

目标附近暂时没有可达节点时, Planner 选择 Frontier 作为中继点, 日志输出目标位置、中继点位置、中继到目标距离、选择原因和总代价

分支原因同时保留内部代码, 例如:

```text
原因=延续当前走廊(continuation)
原因=恢复历史候选分支(deferred_recovery)
原因=初始前向确认阻塞后改选(confirmed_forward_blocked)
```

## 3. 行为边界

本次修改只调整日志和过滤诊断数据, 未修改:

- Mask 阈值和峰值门槛
- 连通区域过滤规则
- `2/3` 视觉确认窗口规则
- 目标融合状态机
- Goal Mux 导航目标选择规则
- Planner Frontier 选择和路线代价

因此日志变化不会改变目标检测结果或路线行为

## 4. 测试和编译

新增过滤诊断测试:

- 大区域峰值不足时返回 `peak_below_threshold`
- 高峰值区域过小时返回 `component_too_small`

验证结果:

```text
visual_navigation 功能测试: 34 passed
graphnav_planner GTest: 12 passed
visual_navigation 编译: passed
graphnav_planner 编译: passed
git diff --check: passed
```

`graphnav_planner` 全包 `uncrustify` 仍报告现有文件整体格式与当前规则不一致, 本次没有执行大范围机械格式化, 避免把无关历史代码纳入日志修改
