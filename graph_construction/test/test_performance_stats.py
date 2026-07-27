from graph_construction.performance_stats import TimingWindow


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
