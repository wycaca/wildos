from __future__ import annotations

import math
import struct
import zlib


_MAGIC = b"WNV1"
_PAYLOAD = struct.Struct("!4sfff")
_PACKET = struct.Struct("!4sfffI")


def encode_velocity(linear_x: float, linear_y: float, angular_z: float) -> bytes:
    values = (float(linear_x), float(linear_y), float(angular_z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity values must be finite")
    payload = _PAYLOAD.pack(_MAGIC, *values)
    return _PACKET.pack(_MAGIC, *values, zlib.crc32(payload))


def decode_velocity(packet: bytes) -> tuple[float, float, float]:
    """校验固定长度、协议版本和 CRC 后解析速度"""
    if len(packet) != _PACKET.size:
        raise ValueError("invalid velocity packet length")
    magic, linear_x, linear_y, angular_z, checksum = _PACKET.unpack(packet)
    payload = _PAYLOAD.pack(magic, linear_x, linear_y, angular_z)
    if magic != _MAGIC or zlib.crc32(payload) != checksum:
        raise ValueError("invalid velocity packet")
    values = (float(linear_x), float(linear_y), float(angular_z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity values must be finite")
    return values


def clamp_velocity(
    velocity: tuple[float, float, float],
    max_linear_x: float,
    max_linear_y: float,
    max_angular_z: float,
) -> tuple[float, float, float]:
    """在 AGX 信任边界内独立执行最终速度限幅"""
    limits = (max_linear_x, max_linear_y, max_angular_z)
    if any(limit < 0.0 or not math.isfinite(limit) for limit in limits):
        raise ValueError("velocity limits must be finite and nonnegative")
    return tuple(
        max(-limit, min(limit, value))
        for value, limit in zip(velocity, limits)
    )
