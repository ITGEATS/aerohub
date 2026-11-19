"""Message-independent input conversion helpers."""
from __future__ import annotations

from array import array
import math
import sys


def depth_meters_to_millimeters(data: bytes, width: int, height: int, step: int) -> bytes:
    if width <= 0 or height <= 0 or step < width * 4 or len(data) != step * height:
        raise ValueError("invalid 32FC1 image layout")
    output = array("H")
    for row in range(height):
        values = array("f")
        values.frombytes(data[row * step:row * step + width * 4])
        if sys.byteorder != "little":
            values.byteswap()
        for value in values:
            if not math.isfinite(value) or value <= 0.0 or value >= 65.535:
                output.append(0)
            else:
                output.append(max(1, min(65534, int(round(value * 1000.0)))))
    if sys.byteorder != "little":
        output.byteswap()
    return output.tobytes()


def stamp_ns(message) -> int:
    value = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
    if value <= 0:
        raise ValueError("sensor message has a non-positive timestamp")
    return value
