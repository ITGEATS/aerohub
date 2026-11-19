#!/usr/bin/env python3
import math
import struct
import time

import rclpy
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import ChannelFloat32, Image, Imu, PointCloud, PointCloud2
from std_msgs.msg import String
from swarm_msgs.msg import VIOFrame


def set_stamp(message, stamp):
    message.header.stamp = stamp.to_msg()


def main() -> None:
    rclpy.init()
    node = Node("d2_slam_native_adapter_smoke")
    received = {}

    subscriptions = [
        node.create_subscription(
            Image, "/drone0/d2slam/input/rgb",
            lambda message: received.setdefault("rgb", message), qos_profile_sensor_data),
        node.create_subscription(
            Image, "/drone0/d2slam/input/depth",
            lambda message: received.setdefault("depth", message), qos_profile_sensor_data),
        node.create_subscription(
            Imu, "/drone0/d2slam/input/imu",
            lambda message: received.setdefault("imu", message), qos_profile_sensor_data),
        node.create_subscription(
            Odometry, "/drone0/d2_slam/odometry",
            lambda message: received.setdefault("odometry", message), qos_profile_sensor_data),
        node.create_subscription(
            PointCloud2, "/drone0/d2_slam/landmarks",
            lambda message: received.setdefault("landmarks", message), qos_profile_sensor_data),
        node.create_subscription(
            String, "/drone0/d2_slam/collaboration",
            lambda message: received.setdefault("collaboration", message), 10),
    ]

    publishers = {
        "rgb": node.create_publisher(
            Image, "/drone0/sensor_measurements/FrontCamera/rgb/image",
            qos_profile_sensor_data),
        "depth": node.create_publisher(
            Image, "/drone0/sensor_measurements/FrontCamera/depth_perspective/image",
            qos_profile_sensor_data),
        "imu": node.create_publisher(
            Imu, "/drone0/sensor_measurements/imu/data", qos_profile_sensor_data),
        "odometry": node.create_publisher(
            Odometry, "/drone0/imu_propagation", qos_profile_sensor_data),
        "landmarks": node.create_publisher(
            PointCloud, "/drone0/point_cloud", qos_profile_sensor_data),
        "remote": node.create_publisher(
            VIOFrame, "/drone0/frame_remote", qos_profile_sensor_data),
    }

    deadline = time.monotonic() + 8.0
    next_publish = 0.0
    while time.monotonic() < deadline and len(received) < 6:
        now = time.monotonic()
        if now >= next_publish:
            stamp = node.get_clock().now()

            rgb = Image()
            set_stamp(rgb, stamp)
            rgb.height, rgb.width, rgb.encoding, rgb.step = 1, 2, "rgb8", 6
            rgb.data = [255, 0, 0, 0, 255, 0]
            publishers["rgb"].publish(rgb)

            depth = Image()
            set_stamp(depth, stamp)
            depth.height, depth.width, depth.encoding, depth.step = 1, 2, "32FC1", 8
            depth.data = struct.pack("<2f", 1.0, math.nan)
            publishers["depth"].publish(depth)

            imu = Imu()
            set_stamp(imu, stamp)
            imu.orientation.w = 1.0
            publishers["imu"].publish(imu)

            odometry = Odometry()
            set_stamp(odometry, stamp)
            odometry.header.frame_id = "world"
            odometry.child_frame_id = "imu"
            odometry.pose.pose.orientation.w = 1.0
            publishers["odometry"].publish(odometry)

            landmarks = PointCloud()
            set_stamp(landmarks, stamp)
            landmarks.points = [Point32(x=1.0, y=2.0, z=3.0)]
            landmarks.channels = [ChannelFloat32(name="rgb", values=[1.25])]
            publishers["landmarks"].publish(landmarks)

            remote = VIOFrame()
            set_stamp(remote, stamp)
            remote.drone_id = 1
            remote.frame_id = 7
            remote.reference_frame_id = 5
            remote.is_keyframe = True
            publishers["remote"].publish(remote)
            next_publish = now + 0.2

        rclpy.spin_once(node, timeout_sec=0.05)

    assert len(subscriptions) == 6
    assert set(received) == {
        "rgb", "depth", "imu", "odometry", "landmarks", "collaboration"}
    assert received["depth"].encoding == "16UC1"
    assert struct.unpack("<2H", bytes(received["depth"].data)) == (1000, 0)
    assert received["odometry"].header.frame_id == "drone0/d2_slam_world"
    assert received["odometry"].child_frame_id == "drone0/d2_slam_imu"
    assert received["landmarks"].header.frame_id == "drone0/d2_slam_world"
    assert struct.unpack("<4f", bytes(received["landmarks"].data)) == (
        1.0, 2.0, 3.0, 1.25)
    assert '"drone_id":1' in received["collaboration"].data
    print("PASS: native ROS 2 D2SLAM adapter graph")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
