from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from omegaconf import OmegaConf
import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from visual_navigation.utils.buffer import MessageBuffer
from visual_navigation.utils.tf_lookup_sub import (
    TFEdge,
    TFLookupSubscriber,
    dynamic_tf_qos,
)


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_dynamic_tf_qos_matches_unity_and_dlio_publishers():
    qos = dynamic_tf_qos(7)

    assert qos.depth == 7
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.history == HistoryPolicy.KEEP_LAST


def test_message_buffer_keeps_latest_when_full():
    buffer = MessageBuffer(max_size=1, wait_for_oldest=False)
    buffer.add_msg({"frame": 1}, Time(sec=1))
    buffer.add_msg({"frame": 2}, Time(sec=2))

    assert buffer.get_oldest_msg()["frame"] == 2


def test_tf_extrapolation_detects_both_time_directions():
    assert TFLookupSubscriber._is_extrapolation(
        RuntimeError("Lookup would require extrapolation into the past")
    )
    assert TFLookupSubscriber._is_extrapolation(
        RuntimeError("Lookup would require extrapolation into the future")
    )


def test_latest_tf_fallback_is_disabled_by_default_and_robot_config():
    config = OmegaConf.load(
        REPO_ROOT / "visual_navigation" / "configs" / "wildos_nav_conf.yaml"
    )

    assert not TFLookupSubscriber.default_tflookup_config[
        "allow_latest_tf_on_past_extrapolation"
    ]
    assert not config.tf_lookup_config.allow_latest_tf_on_past_extrapolation


@pytest.mark.parametrize(
    "error",
    [
        RuntimeError("Lookup would require extrapolation into the past"),
        RuntimeError("Lookup would require extrapolation into the future"),
    ],
)
def test_disabled_fallback_never_queries_latest_tf(error):
    calls = []

    def lookup_transform(target, source, stamp, timeout):
        calls.append((target, source, stamp))
        raise error

    subscriber = SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=lookup_transform),
        lookup_timeout=0.0,
        allow_latest_tf_on_extrapolation=False,
        _is_extrapolation=TFLookupSubscriber._is_extrapolation,
    )

    with pytest.raises(RuntimeError, match="extrapolation"):
        TFLookupSubscriber._lookup_transform_with_fallback(
            subscriber,
            TFEdge(source_frame="camera", target_frame="odom"),
            Time(sec=10),
        )

    assert len(calls) == 1
    assert calls[0][2].sec == 10


def test_measurement_time_lookup_recovers_after_tf_arrives():
    error = RuntimeError("Lookup would require extrapolation into the future")
    expected = object()
    responses = iter([error, expected])

    def lookup_transform(target, source, stamp, timeout):
        response = next(responses)
        if isinstance(response, Exception):
            raise response
        return response

    subscriber = SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=lookup_transform),
        lookup_timeout=0.0,
        allow_latest_tf_on_extrapolation=False,
        _is_extrapolation=TFLookupSubscriber._is_extrapolation,
    )
    edge = TFEdge(source_frame="camera", target_frame="odom")
    stamp = Time(sec=10)

    with pytest.raises(RuntimeError, match="future"):
        TFLookupSubscriber._lookup_transform_with_fallback(
            subscriber,
            edge,
            stamp,
        )

    assert TFLookupSubscriber._lookup_transform_with_fallback(
        subscriber,
        edge,
        stamp,
    ) is expected
