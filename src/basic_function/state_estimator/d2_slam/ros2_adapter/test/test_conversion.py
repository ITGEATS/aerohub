import math
import struct

import pytest

from uav_d2_slam_ros2.conversion import depth_meters_to_millimeters
from uav_d2_slam_ros2.protocol import receive_packet, send_packet


def test_depth_conversion_preserves_rows_and_invalid_values():
    source = struct.pack("<6f", 1.0, float("nan"), 65.535, 0.25, -1.0, 2.5)
    result = struct.unpack("<6H", depth_meters_to_millimeters(source, 3, 2, 12))
    assert result == (1000, 0, 0, 250, 0, 2500)


def test_depth_conversion_rejects_bad_stride():
    with pytest.raises(ValueError):
        depth_meters_to_millimeters(b"\0" * 8, 2, 1, 4)


def test_protocol_round_trip():
    import socket
    left, right = socket.socketpair()
    try:
        send_packet(left, {"kind": "rgb", "stamp_ns": 1}, b"abc")
        assert receive_packet(right) == ({"kind": "rgb", "stamp_ns": 1}, b"abc")
    finally:
        left.close()
        right.close()
