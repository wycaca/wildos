import numpy as np

from visual_navigation.object_reached_evidence import VisualReachedEvidence


def _mask(pixel_count: int, height: int = 20, width: int = 20):
    """构造三相机二值 mask"""
    mask = np.zeros((3, 1, height, width), dtype=np.uint8)
    mask[0, 0].flat[:pixel_count] = 1
    return mask


def test_visual_reached_requires_consecutive_evidence():
    """中断帧必须重新累计近距离确认帧数"""
    evidence = VisualReachedEvidence(40, 0.1, 2)

    assert not evidence.update(_mask(40))
    assert not evidence.update(None)
    assert not evidence.update(_mask(40))
    assert evidence.update(_mask(40))


def test_visual_reached_is_not_a_permanent_completion_latch():
    """视觉证据消失后必须恢复 False, 最终完成由 Mux 负责"""
    evidence = VisualReachedEvidence(40, 0.1, 2)

    assert not evidence.update(_mask(40))
    assert evidence.update(_mask(40))
    assert not evidence.update(None)
    assert not evidence.update(_mask(0))
