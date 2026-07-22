"""调试消息发布频率控制"""

from __future__ import annotations

import time


class PeriodicPublishGate:
    """限制非关键调试输出的最高发布频率"""

    def __init__(self, period_sec: float) -> None:
        """保存最小发布周期"""
        self.period_sec = max(float(period_sec), 0.0)
        self._last_publish_time: float | None = None

    def ready(self, now: float | None = None) -> bool:
        """到达周期时放行一次, period 为零时不限制"""
        current_time = time.monotonic() if now is None else float(now)
        if (
            self._last_publish_time is not None
            and self.period_sec > 0.0
            and current_time - self._last_publish_time < self.period_sec
        ):
            return False

        self._last_publish_time = current_time
        return True
