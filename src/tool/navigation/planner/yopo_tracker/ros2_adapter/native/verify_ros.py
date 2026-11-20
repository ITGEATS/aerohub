"""Verify the installed native node through DDS using an empty RGB-D scene."""
from array import array
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import torch
from visualization_msgs.msg import MarkerArray


NAMESPACE = '/yopo_tracker_humble_check'
PREFIX = NAMESPACE + '/yopo_net'
ORIGIN = (1.25, -2.5, .75)


def stamp(msg, value):
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(value, 10**9)


def check(node, child):
    def wait(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError(f'Native node exited with code {child.returncode}')
            rclpy.spin_once(node, timeout_sec=.02)
        return predicate()

    odom_pub = node.create_publisher(Odometry, PREFIX + '/odom', qos_profile_sensor_data)
    rgb_pub = node.create_publisher(Image, PREFIX + '/rgb', 1)
    depth_pub = node.create_publisher(Image, PREFIX + '/depth', 1)
    received, commands = [], []
    candidates_topic = PREFIX + '/diagnostics/trajs_visual'
    references_topic = PREFIX + '/diagnostics/position_command'
    node.create_subscription(MarkerArray, candidates_topic, received.append, 1)
    node.create_subscription(PositionCommand, references_topic, commands.append, 1)

    def discovered():
        return (all(pub.get_subscription_count() for pub in (odom_pub, rgb_pub, depth_pub))
                and node.count_publishers(candidates_topic) and node.count_publishers(references_topic))

    assert wait(discovered, 30), 'Native DDS endpoints did not match'
    odom = Odometry()
    odom.header.frame_id = 'world'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = ORIGIN
    odom.pose.pose.orientation.z = math.sqrt(.5)
    odom.pose.pose.orientation.w = math.sqrt(.5)
    rgb = Image(height=192, width=320, encoding='rgb8', step=960)
    rgb.data = array('B', bytes([12, 23, 34]) * (rgb.width * rgb.height))
    depth = Image(height=192, width=320, encoding='32FC1', step=1280)
    depth.data = array('B', np.full((192, 320), 10., dtype='<f4').tobytes())
    now = node.get_clock().now().nanoseconds
    stamp(odom, now)
    stamp(rgb, now)
    stamp(depth, now - 10**9)
    odom_pub.publish(odom)
    rgb_pub.publish(rgb)
    depth_pub.publish(depth)
    assert not wait(lambda: bool(received), .4), 'Unsynchronized images reached inference'

    for index in range(4):
        # Wait for the real downstream output before sending the next input.
        now = node.get_clock().now().nanoseconds
        stamp(rgb, now)
        stamp(depth, now)
        rgb_pub.publish(rgb)
        depth_pub.publish(depth)
        assert wait(lambda: len(received) > index, 5), 'No candidate output for RGB-D pair'
        msg = received[index]
        assert [marker.id for marker in msg.markers] == list(range(15))
        for marker in msg.markers:
            assert marker.header.frame_id == 'world' and len(marker.points) == 20
            assert marker.header.stamp.sec * 10**9 + marker.header.stamp.nanosec >= now
            points = np.array([(p.x, p.y, p.z) for p in marker.points])
            np.testing.assert_array_equal(points[0], ORIGIN)
            assert np.isfinite(points).all()
    assert not wait(lambda: bool(commands), .2), 'Empty scene triggered a position command'
    print(json.dumps(dict(status='pass', ros=os.environ['ROS_DISTRO'],
                          cuda_device=torch.cuda.get_device_name(),
                          paired_outputs=len(received), candidates_per_output=15,
                          samples_per_candidate=20, mismatched_pair_outputs=0,
                          position_commands=len(commands))))


def main():
    if os.environ.get('ROS_DISTRO') != 'humble' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Run this verification with Humble and ROS_LOCALHOST_ONLY=1')
    if not torch.cuda.is_available():
        raise RuntimeError('Run this verification with a CUDA GPU')
    rclpy.init(args=[])
    node = Node('probe', namespace=NAMESPACE)
    try:
        native = Path(__file__).with_name('native_node.py')
        child = subprocess.Popen([sys.executable, '-u', native, '--ros-args',
                                  '-r', '__ns:=' + NAMESPACE])
        try:
            check(node, child)
        finally:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
