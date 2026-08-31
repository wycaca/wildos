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
    """Collect bounded timing samples for low-frequency diagnostics"""

    def __init__(self, max_samples: int = 512) -> None:
        self._samples: deque[float] = deque(maxlen=max(max_samples, 1))

    def add_seconds(self, elapsed_seconds: float) -> None:
        if math.isfinite(elapsed_seconds) and elapsed_seconds >= 0.0:
            self._samples.append(elapsed_seconds * 1000.0)

    def summary(self, reset: bool = False) -> TimingSummary:
        samples = sorted(self._samples)
        if not samples:
            return TimingSummary(0, 0.0, 0.0, 0.0)
        percentile_index = min(math.ceil(len(samples) * 0.95) - 1, len(samples) - 1)
        result = TimingSummary(
            count=len(samples),
            average_ms=sum(samples) / len(samples),
            p95_ms=samples[percentile_index],
            maximum_ms=samples[-1],
        )
        if reset:
            self._samples.clear()
        return result


class EventRate:
    """Measure event rate between low-frequency reports"""

    def __init__(self) -> None:
        self._started_at = time.monotonic()
        self._count = 0

    def tick(self) -> None:
        self._count += 1

    def sample(self, reset: bool = False) -> float:
        now = time.monotonic()
        elapsed = max(now - self._started_at, 1.0e-6)
        rate = self._count / elapsed
        if reset:
            self._started_at = now
            self._count = 0
        return rate


def publish_due(now_ns: int, last_ns: int | None, max_rate_hz: float) -> bool:
    """Return whether a rate-limited event may be published"""
    if max_rate_hz <= 0.0 or last_ns is None:
        return True
    return now_ns - last_ns >= 1.0e9 / max_rate_hz


def timing_metrics(prefix: str, summary: TimingSummary) -> dict[str, float | int]:
    """把固定窗口统计转换为稳定指标名称"""
    return {
        f"{prefix}.count": summary.count,
        f"{prefix}.average_ms": summary.average_ms,
        f"{prefix}.p95_ms": summary.p95_ms,
        f"{prefix}.maximum_ms": summary.maximum_ms,
    }


def diagnostic_status(
    name: str,
    metrics: Mapping[str, object],
    level: int = DiagnosticStatus.OK,
    message: str = "OK",
) -> DiagnosticStatus:
    """使用标准 diagnostics 消息承载轻量标量指标"""
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
