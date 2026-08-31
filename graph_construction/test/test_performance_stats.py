from diagnostic_msgs.msg import DiagnosticStatus

from graph_construction.performance_stats import (
    TimingSummary,
    TimingWindow,
    diagnostic_status,
    timing_metrics,
)


def test_timing_window_reports_average_p95_and_maximum():
    timing = TimingWindow()
    for milliseconds in range(1, 101):
        timing.add_seconds(milliseconds / 1000.0)

    summary = timing.summary()

    assert summary.count == 100
    assert summary.average_ms == 50.5
    assert summary.p95_ms == 95.0
    assert summary.maximum_ms == 100.0


def test_timing_window_can_reset_samples():
    timing = TimingWindow()
    timing.add_seconds(0.01)

    assert timing.summary(reset=True).count == 1
    assert timing.summary().count == 0


def test_diagnostic_status_contains_only_scalar_key_values():
    metrics = timing_metrics(
        "cycle.total",
        TimingSummary(3, 10.0, 12.0, 15.0),
    )

    status = diagnostic_status("wildos/test", metrics)

    assert status.level == DiagnosticStatus.OK
    assert status.name == "wildos/test"
    assert {item.key: item.value for item in status.values} == {
        "cycle.total.count": "3",
        "cycle.total.average_ms": "10.0",
        "cycle.total.p95_ms": "12.0",
        "cycle.total.maximum_ms": "15.0",
    }
