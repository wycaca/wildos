from pathlib import Path

from graph_construction.resource_monitor import (
    ResourceSampler,
    cpu_percent,
    format_resource_metrics,
    parse_nvidia_smi,
)


def test_cpu_and_gpu_parsers_keep_stable_units():
    assert cpu_percent((100, 20), (200, 30)) == 90.0
    assert parse_nvidia_smi("25, 1024, 8192, 55\n") == {
        "host.gpu.utilization_percent": 25.0,
        "host.gpu.memory_used_mib": 1024.0,
        "host.gpu.memory_total_mib": 8192.0,
        "host.gpu.temperature_c": 55.0,
    }


def test_resource_sampler_reads_host_process_and_temperature(tmp_path: Path):
    proc_root = tmp_path / "proc"
    thermal_root = tmp_path / "thermal"
    process = proc_root / "123"
    process.mkdir(parents=True)
    zone = thermal_root / "thermal_zone0"
    zone.mkdir(parents=True)
    (proc_root / "stat").write_text("cpu  100 0 0 900 0\n", encoding="utf-8")
    (proc_root / "meminfo").write_text(
        "MemTotal: 8192000 kB\nMemAvailable: 6144000 kB\n",
        encoding="utf-8",
    )
    (process / "cmdline").write_bytes(b"python\0graph_construction\0")
    (process / "stat").write_text(
        "123 (python) S " + "0 " * 11 + "100 50 0 0\n",
        encoding="utf-8",
    )
    (process / "statm").write_text("1000 256\n", encoding="utf-8")
    (zone / "temp").write_text("55000\n", encoding="utf-8")
    times = iter((0.0, 1.0, 2.0))
    sampler = ResourceSampler(
        process_patterns=("graph_construction",),
        proc_root=proc_root,
        thermal_root=thermal_root,
        clock=lambda: next(times),
        gpu_query=lambda: {},
    )
    sampler.sample()
    (proc_root / "stat").write_text("cpu  180 0 0 920 0\n", encoding="utf-8")
    (process / "stat").write_text(
        "123 (python) S " + "0 " * 11 + "110 50 0 0\n",
        encoding="utf-8",
    )

    metrics = sampler.sample()

    assert metrics["host.cpu.utilization_percent"] == 80.0
    assert metrics["host.memory.used_mib"] == 2000.0
    assert metrics["host.temperature.maximum_c"] == 55.0
    assert metrics["process.graph_construction.cpu_percent"] > 0.0
    assert metrics["process.graph_construction.rss_mib"] > 0.0
    formatted = format_resource_metrics(metrics)
    assert "host.cpu.utilization_percent=80.0" in formatted
