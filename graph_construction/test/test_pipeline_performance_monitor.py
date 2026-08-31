from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from graph_construction.pipeline_performance_monitor import (
    PipelinePerformanceMonitor,
    format_status,
)


def test_monitor_formats_compact_diagnostics_without_payload_topics():
    status = DiagnosticStatus(
        level=DiagnosticStatus.OK,
        name="wildos/graph_construction",
        values=[
            KeyValue(key="publish.rate_hz", value="2.0"),
            KeyValue(key="cycle.total.p95_ms", value="25.0"),
            KeyValue(key="graph.frontier.p95_ms", value="5.0"),
            KeyValue(key="workload.total_node_count", value="100"),
        ],
    )

    output = format_status(status)

    assert "publish.rate_hz=2.0" in output
    assert "cycle.total.p95_ms=25.0" in output
    assert "workload.total_node_count=100" in output
    assert "graph.frontier.p95_ms" not in output
    assert not hasattr(PipelinePerformanceMonitor, "TOPICS")
