from visual_navigation.utils.publish_gate import PeriodicPublishGate


def test_periodic_publish_gate_limits_debug_rate():
    gate = PeriodicPublishGate(period_sec=2.0)

    assert gate.ready(now=10.0)
    assert not gate.ready(now=11.9)
    assert gate.ready(now=12.0)


def test_periodic_publish_gate_can_be_disabled():
    gate = PeriodicPublishGate(period_sec=0.0)

    assert gate.ready(now=10.0)
    assert gate.ready(now=10.0)
