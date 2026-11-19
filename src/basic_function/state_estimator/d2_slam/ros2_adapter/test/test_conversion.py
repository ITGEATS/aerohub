import math
import struct

import pytest
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32, PointCloud

from uav_d2_slam_ros2.conversion import depth_meters_to_millimeters
from uav_d2_slam_ros2.node import point_cloud_to_point_cloud2


def test_depth_conversion_preserves_rows_and_invalid_values():
    source = struct.pack("<6f", 1.0, float("nan"), 65.535, 0.25, -1.0, 2.5)
    result = struct.unpack("<6H", depth_meters_to_millimeters(source, 3, 2, 12))
    assert result == (1000, 0, 0, 250, 0, 2500)


def test_depth_conversion_rejects_bad_stride():
    with pytest.raises(ValueError):
        depth_meters_to_millimeters(b"\0" * 8, 2, 1, 4)


def test_point_cloud_conversion_preserves_points_and_rgb():
    source = PointCloud()
    source.header.frame_id = "native-world"
    source.points = [Point32(x=1.0, y=2.0, z=3.0), Point32(x=4.0, y=5.0, z=6.0)]
    source.channels = [ChannelFloat32(name="rgb", values=[1.25, 2.5])]

    result = point_cloud_to_point_cloud2(source, "drone0/d2_slam_world")

    assert result.header.frame_id == "drone0/d2_slam_world"
    assert result.width == 2
    assert result.point_step == 16
    assert struct.unpack("<8f", bytes(result.data)) == (
        1.0, 2.0, 3.0, 1.25, 4.0, 5.0, 6.0, 2.5)


def test_point_cloud_conversion_rejects_mismatched_channel():
    source = PointCloud()
    source.points = [Point32()]
    source.channels = [ChannelFloat32(name="rgb", values=[])]
    with pytest.raises(ValueError):
        point_cloud_to_point_cloud2(source, "world")
