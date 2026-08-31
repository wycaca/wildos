from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Mapping

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


@dataclass(frozen=True)
class TimingSummary:
    count: int
    average_ms: float
    p95_ms: float
    maximum_ms: float


class TimingWindow:
    """记录固定容量耗时窗口"""

    def __init__(self, max_samples: int = 256) -> None:
        self._samples: deque[float] = deque(maxlen=max(max_samples, 1))

    def add_seconds(self, elapsed_seconds: float) -> None:
        if math.isfinite(elapsed_seconds) and elapsed_seconds >= 0.0:
            self._samples.append(elapsed_seconds * 1000.0)

    def summary(self, reset: bool = False) -> TimingSummary:
        samples = sorted(self._samples)
        if not samples:
            return TimingSummary(0, 0.0, 0.0, 0.0)
        percentile_index = min(
            math.ceil(len(samples) * 0.95) - 1,
            len(samples) - 1,
        )
        result = TimingSummary(
            len(samples),
            sum(samples) / len(samples),
            samples[percentile_index],
            samples[-1],
        )
        if reset:
            self._samples.clear()
        return result


class EventRate:
    """按 diagnostics 周期计算事件频率"""

    def __init__(self) -> None:
        self._started_at = time.monotonic()
        self._count = 0

    def tick(self) -> None:
        self._count += 1

    def sample(self, reset: bool = False) -> float:
        now = time.monotonic()
        rate = self._count / max(now - self._started_at, 1.0e-6)
        if reset:
            self._started_at = now
            self._count = 0
        return rate


def timing_metrics(prefix: str, summary: TimingSummary) -> dict[str, float | int]:
    return {
        f"{prefix}.count": summary.count,
        f"{prefix}.average_ms": summary.average_ms,
        f"{prefix}.p95_ms": summary.p95_ms,
        f"{prefix}.maximum_ms": summary.maximum_ms,
    }


def message_age_ms(now_ns: int, stamp_ns: int | None) -> float | None:
    if stamp_ns is None:
        return None
    age_ms = (now_ns - stamp_ns) / 1_000_000.0
    return age_ms if math.isfinite(age_ms) and age_ms >= 0.0 else None


def diagnostic_status(
    name: str,
    metrics: Mapping[str, object],
    level: int = DiagnosticStatus.OK,
    message: str = "OK",
) -> DiagnosticStatus:
    return DiagnosticStatus(
        level=level,
        name=name,
        message=message,
        hardware_id="wildos",
        values=[
            KeyValue(key=key, value=str(value))
            for key, value in metrics.items()
        ],
    )
