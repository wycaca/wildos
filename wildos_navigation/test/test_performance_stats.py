import math

from diagnostic_msgs.msg import DiagnosticStatus

from wildos_navigation.performance_stats import (
    TimingSummary,
    TimingWindow,
    diagnostic_status,
    message_age_ms,
)


def test_timing_window_is_bounded_and_ignores_invalid_samples():
    window = TimingWindow(max_samples=3)
    for elapsed in (0.001, math.nan, -1.0, 0.002, 0.003, 0.004):
        window.add_seconds(elapsed)

    summary = window.summary(reset=True)

    assert summary == TimingSummary(3, 3.0, 4.0, 4.0)
    assert window.summary() == TimingSummary(0, 0.0, 0.0, 0.0)


def test_diagnostic_status_keeps_scalar_metric_contract():
    status = diagnostic_status(
        "wildos/test",
        {"cycle.total.p95_ms": 12.5, "workload.points": 200},
    )

    assert status.level == DiagnosticStatus.OK
    assert status.hardware_id == "wildos"
    assert {item.key: item.value for item in status.values} == {
        "cycle.total.p95_ms": "12.5",
        "workload.points": "200",
    }


def test_message_age_rejects_future_and_missing_stamps():
    assert message_age_ms(2_000_000, 1_000_000) == 1.0
    assert message_age_ms(1_000_000, 2_000_000) is None
    assert message_age_ms(1_000_000, None) is None
