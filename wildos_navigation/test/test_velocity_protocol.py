import math

import pytest

from wildos_navigation.velocity_protocol import (
    clamp_velocity,
    decode_velocity,
    encode_velocity,
)


def test_velocity_packet_round_trip():
    packet = encode_velocity(0.2, 0.0, -0.5)

    assert decode_velocity(packet) == pytest.approx((0.2, 0.0, -0.5))


def test_velocity_packet_rejects_corruption():
    packet = bytearray(encode_velocity(0.1, 0.0, 0.2))
    packet[6] ^= 0x01

    with pytest.raises(ValueError):
        decode_velocity(bytes(packet))


def test_velocity_packet_rejects_nonfinite_values():
    with pytest.raises(ValueError):
        encode_velocity(math.nan, 0.0, 0.0)


def test_gateway_applies_independent_limits():
    limited = clamp_velocity((0.8, 0.2, -1.2), 0.2, 0.0, 0.5)

    assert limited == pytest.approx((0.2, 0.0, -0.5))
