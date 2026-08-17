# 目标搜索永久完成锁

## 问题

WildOS 在近距离视觉证据短暂消失后会重新发布 `False`, 并继续生成目标导航点候选

虽然 goal mux 已有 reached latch, 但原配置会在超时后恢复搜索, 因此任务完成状态不是永久状态

## 修改

- 新增 `ObjectReachedLatch`, 连续近距离证据达到阈值后永久锁定 `completed`
- WildOS 首次完成后停止目标定位, 目标候选选择和相关日志, 后续图像空帧不能恢复搜索
- WildOS 完成后继续发布 `/spot1/object_search_reached=True`, 让下游在晚启动或短时通信中断后仍能收到完成状态
- 移除 `reached_latch_timeout_sec`, goal mux 在节点生命周期内永久保持当前位置 hold goal
- 当前没有任务重置 topic, 新任务通过重启 WildOS 和 goal mux 开始

## 状态约束

1. 未完成时, 近距离证据必须连续满足 `reached_confirm_frames`
2. 首次满足后, WildOS 将 `completed` 永久设置为 `True`
3. 完成帧不再生成新的目标候选
4. 后续帧跳过目标定位与候选链, 不再打印 `目标近距离确认=False` 和候选更新日志
5. goal mux 收到首次 `True` 后锁定当前位置, 后续 `False` 不能解除停止状态

## 验证

- `test_object_reached_latch.py` 验证连续帧门槛和永久锁定
- `test_object_search_goal_mux.py` 验证首次 reached 后永久保持当前位置 goal
