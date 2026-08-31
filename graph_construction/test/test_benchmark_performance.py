import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "scripts" / "benchmark_performance.py"


def _module():
    spec = importlib.util.spec_from_file_location(
        "benchmark_performance",
        SCRIPT,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sample_summary_uses_nearest_rank_p95_and_budget():
    result = _module().summarize_samples(
        [float(value) for value in range(1, 101)],
        96.0,
    )

    assert result["median_ms"] == 50.5
    assert result["p95_ms"] == 95.0
    assert result["maximum_ms"] == 100.0
    assert result["budget_met"] is True


def test_migration_gate_only_selects_over_budget_scenarios():
    candidates = _module().migration_candidates(
        {
            "map_pub": {"budget_met": True, "p95_ms": 50.0},
            "graph_full": {
                "budget_met": False,
                "p95_ms": 600.0,
                "stages": {"edges": {"p95_ms": 250.0}},
            },
        }
    )

    assert candidates == ["graph_full"]


def test_migration_gate_rejects_distributed_overhead():
    candidates = _module().migration_candidates(
        {
            "graph_full": {
                "budget_met": False,
                "p95_ms": 600.0,
                "stages": {
                    f"stage_{index}": {"p95_ms": 100.0}
                    for index in range(6)
                },
            }
        },
        hotspot_share_threshold=0.30,
    )

    assert candidates == []
