from pathlib import Path

import pytest

from graph_construction.dlio_output_guard import HealthLease
from graph_construction.performance_stats import publish_due


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_removed_input_filter_does_not_reappear():
    source = (
        REPO_ROOT
        / "graph_construction"
        / "graph_construction"
        / "dlio_output_guard.py"
    ).read_text(encoding="utf-8")

    assert "last_pointcloud_stamp" not in source
    assert "last_imu_stamp" not in source
    assert "is_strictly_new_stamp" not in source


def test_health_lease_stops_forwarding_after_timeout():
    lease = HealthLease(timeout_sec=1.5)
    lease.update(True, now=10.0)

    assert lease.is_healthy(now=11.5)
    assert not lease.is_healthy(now=11.5001)
    assert lease.timeouts == 1
    assert lease.transitions == 2


def test_false_health_stops_immediately_without_timeout():
    lease = HealthLease(timeout_sec=1.5)
    lease.update(True, now=10.0)
    lease.update(False, now=10.1)

    assert not lease.is_healthy(now=10.1)
    assert lease.timeouts == 0


def test_new_healthy_heartbeat_recovers_expired_lease():
    lease = HealthLease(timeout_sec=1.5)
    lease.update(True, now=10.0)
    assert not lease.is_healthy(now=12.0)

    lease.update(True, now=12.1)

    assert lease.is_healthy(now=13.5)
    assert lease.transitions == 3


def test_health_lease_rejects_nonpositive_timeout():
    with pytest.raises(ValueError, match="greater than 0"):
        HealthLease(timeout_sec=0.0)


def test_output_rate_limit_keeps_dlio_internal_rate_unchanged():
    assert publish_due(1_000_000_000, None, 2.0)
    assert not publish_due(1_499_999_999, 1_000_000_000, 2.0)
    assert publish_due(1_500_000_000, 1_000_000_000, 2.0)
    assert publish_due(1_000_000_001, 1_000_000_000, 0.0)
