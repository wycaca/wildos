import numpy as np

from visual_navigation.object_reached_latch import ObjectReachedLatch


def _mask(pixel_count: int, height: int = 20, width: int = 20):
    """构造三相机二值 mask"""
    mask = np.zeros((3, 1, height, width), dtype=np.uint8)
    mask[0, 0].flat[:pixel_count] = 1
    return mask


def test_reached_latch_requires_consecutive_evidence():
    """中断帧必须重新累计近距离确认帧数"""
    latch = ObjectReachedLatch(40, 0.1, 2)

    assert not latch.update(_mask(40))
    assert not latch.update(None)
    assert not latch.update(_mask(40))
    assert latch.update(_mask(40))


def test_reached_latch_stays_complete_after_empty_frames():
    """首次完成后空帧和弱证据不能撤销任务完成状态"""
    latch = ObjectReachedLatch(40, 0.1, 2)

    assert not latch.update(_mask(40))
    assert latch.update(_mask(40))
    assert latch.completed
    assert latch.update(None)
    assert latch.update(_mask(0))
