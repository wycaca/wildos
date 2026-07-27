from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time


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
