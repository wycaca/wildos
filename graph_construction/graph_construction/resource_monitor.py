from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import subprocess
import time


DEFAULT_PROCESS_PATTERNS = (
    "graph_construction",
    "visual_navigation",
    "object_target_fusion",
    "planner_node",
    "map_pub",
    "start_nav",
)


def cpu_percent(previous, current) -> float | None:
    if previous is None:
        return None
    total_delta = current[0] - previous[0]
    idle_delta = current[1] - previous[1]
    if total_delta <= 0 or idle_delta < 0:
        return None
    return max(0.0, min(100.0, 100.0 * (1.0 - idle_delta / total_delta)))


def parse_nvidia_smi(output: str) -> dict[str, float]:
    """解析 nvidia-smi 单卡标量输出"""
    line = next((line for line in output.splitlines() if line.strip()), "")
    if not line:
        return {}
    try:
        utilization, used, total, temperature = (
            float(value.strip())
            for value in line.split(",")
        )
    except (TypeError, ValueError):
        return {}
    return {
        "host.gpu.utilization_percent": utilization,
        "host.gpu.memory_used_mib": used,
        "host.gpu.memory_total_mib": total,
        "host.gpu.temperature_c": temperature,
    }


def query_nvidia_smi() -> dict[str, float]:
    executable = shutil.which("nvidia-smi")
    if executable is None:
        return {}
    try:
        result = subprocess.run(
            [
                executable,
                "--query-gpu=utilization.gpu,memory.used,memory.total,"
                "temperature.gpu",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return {}
    return parse_nvidia_smi(result.stdout)


class ResourceSampler:
    """在独立 monitor 中采集主机和核心进程资源"""

    def __init__(
        self,
        process_patterns=DEFAULT_PROCESS_PATTERNS,
        proc_root: Path = Path("/proc"),
        thermal_root: Path = Path("/sys/class/thermal"),
        clock=time.monotonic,
        gpu_query=query_nvidia_smi,
    ) -> None:
        self.process_patterns = tuple(process_patterns)
        self.proc_root = proc_root
        self.thermal_root = thermal_root
        self.clock = clock
        self.gpu_query = gpu_query
        self.clock_ticks = float(os.sysconf("SC_CLK_TCK"))
        self.page_size = float(os.sysconf("SC_PAGE_SIZE"))
        self._previous_host = None
        self._previous_process_ticks = {}
        self._previous_time = self.clock()

    def sample(self) -> dict[str, float]:
        """读取一次资源快照并计算相邻周期 CPU 使用率"""
        now = self.clock()
        elapsed = max(now - self._previous_time, 1.0e-6)
        metrics = {}
        current_host = self._host_cpu()
        usage = cpu_percent(self._previous_host, current_host)
        if usage is not None:
            metrics["host.cpu.utilization_percent"] = usage
        metrics.update(self._memory())
        metrics.update(self._temperature())
        metrics.update(self._processes(elapsed))
        metrics.update(self.gpu_query())
        self._previous_host = current_host
        self._previous_time = now
        return metrics

    def _host_cpu(self):
        values = (
            (self.proc_root / "stat")
            .read_text(encoding="utf-8")
            .splitlines()[0]
        )
        fields = [int(value) for value in values.split()[1:]]
        return sum(fields), fields[3] + (fields[4] if len(fields) > 4 else 0)

    def _memory(self):
        values = {}
        lines = (self.proc_root / "meminfo").read_text(
            encoding="utf-8"
        ).splitlines()
        for line in lines:
            key, raw = line.split(":", 1)
            values[key] = float(raw.split()[0]) / 1024.0
        total = values.get("MemTotal")
        available = values.get("MemAvailable")
        if total is None or available is None:
            return {}
        return {
            "host.memory.used_mib": total - available,
            "host.memory.available_mib": available,
            "host.memory.utilization_percent": (
                100.0 * (total - available) / total
            ),
        }

    def _temperature(self):
        temperatures = []
        for path in self.thermal_root.glob("thermal_zone*/temp"):
            try:
                value = float(path.read_text(encoding="utf-8").strip())
            except (OSError, ValueError):
                continue
            value = value / 1000.0 if value > 1000.0 else value
            if math.isfinite(value):
                temperatures.append(value)
        if not temperatures:
            return {}
        return {"host.temperature.maximum_c": max(temperatures)}

    def _processes(self, elapsed):
        totals = {}
        current_ticks = {}
        for directory in self.proc_root.iterdir():
            if not directory.name.isdigit():
                continue
            pid = int(directory.name)
            if pid == os.getpid():
                continue
            try:
                command = (
                    (directory / "cmdline")
                    .read_bytes()
                    .replace(b"\0", b" ")
                    .decode()
                )
                pattern = next(
                    (
                        name
                        for name in self.process_patterns
                        if name in command
                    ),
                    None,
                )
                if pattern is None:
                    continue
                stat_fields = (
                    (directory / "stat")
                    .read_text(encoding="utf-8")
                    .rsplit(")", 1)[1]
                    .split()
                )
                ticks = int(stat_fields[11]) + int(stat_fields[12])
                resident_pages = int(
                    (directory / "statm")
                    .read_text(encoding="utf-8")
                    .split()[1]
                )
            except (OSError, UnicodeError, ValueError, IndexError):
                continue
            current_ticks[pid] = ticks
            previous_ticks = self._previous_process_ticks.get(pid, ticks)
            cpu = (
                max(0.0, ticks - previous_ticks)
                / self.clock_ticks
                / elapsed
                * 100.0
            )
            values = totals.setdefault(pattern, [0.0, 0.0])
            values[0] += cpu
            values[1] += resident_pages * self.page_size / 1024.0**2
        self._previous_process_ticks = current_ticks
        metrics = {}
        for pattern, (cpu, rss) in totals.items():
            metrics[f"process.{pattern}.cpu_percent"] = cpu
            metrics[f"process.{pattern}.rss_mib"] = rss
        return metrics


def format_resource_metrics(metrics: dict[str, float]) -> str:
    return ", ".join(
        f"{key}={value:.1f}"
        for key, value in sorted(metrics.items())
    )
