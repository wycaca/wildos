from graph_construction.performance_stats import TimingWindow, format_timing


def test_timing_window_reports_average_p95_and_maximum():
    timing = TimingWindow()
    for milliseconds in range(1, 101):
        timing.add_seconds(milliseconds / 1000.0)

    summary = timing.summary()

    assert summary.count == 100
    assert summary.average_ms == 50.5
    assert summary.p95_ms == 95.0
    assert summary.maximum_ms == 100.0
    assert "avg:50.5/p95:95.0/max:100.0ms" in format_timing("stage", summary)


def test_timing_window_can_reset_samples():
    timing = TimingWindow()
    timing.add_seconds(0.01)

    assert timing.summary(reset=True).count == 1
    assert timing.summary().count == 0
