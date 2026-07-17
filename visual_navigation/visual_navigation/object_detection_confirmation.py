from __future__ import annotations

from collections import deque
from typing import Deque


class DetectionConfirmationWindow:
    """用滑动窗口确认视觉证据, 允许窗口内出现短暂丢帧"""

    def __init__(self, required_frames: int, window_frames: int) -> None:
        self.required_frames = max(1, int(required_frames))
        self.window_frames = max(self.required_frames, int(window_frames))
        self._history: Deque[bool] = deque(maxlen=self.window_frames)

    @property
    def evidence_count(self) -> int:
        return sum(self._history)

    @property
    def sample_count(self) -> int:
        return len(self._history)

    @property
    def ready(self) -> bool:
        return self.evidence_count >= self.required_frames

    def update(self, has_evidence: bool) -> bool:
        """记录当前帧, 仅在当前帧有证据且窗口达标时确认"""
        self._history.append(bool(has_evidence))
        return bool(has_evidence) and self.ready

    def reset(self) -> None:
        self._history.clear()
