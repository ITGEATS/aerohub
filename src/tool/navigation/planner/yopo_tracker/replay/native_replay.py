"""Run the original YOPO-Tracker callbacks on a finite ROS 2 recording."""
import argparse
import json
from pathlib import Path
import sys

import message_filters
from nav_msgs.msg import Odometry
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_adapter.native.tracker_model import load_tracker


def replay(node, bag, storage_id, topics, *, on_images=None, advance_reference=True, complete=None):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    expected = ['nav_msgs/msg/Odometry', 'sensor_msgs/msg/Image', 'sensor_msgs/msg/Image']
    for name, typename in zip(topics, expected):
        if types.get(name) != typename:
            raise ValueError(f'{name}: expected {typename}, found {types.get(name)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    depth, rgb = message_filters.SimpleFilter(), message_filters.SimpleFilter()
    # Same ordering, queue and slop as the pinned native subscriber.
    sync = message_filters.ApproximateTimeSynchronizer([depth, rgb], 1, 0.05)
    counts = dict(odometry=0, synchronized=0, inferred=0, confirmed=0)

    def images(depth_msg, rgb_msg):
        counts['synchronized'] += 1
        if node.odom_init:
            (on_images or node.callback_images)(depth_msg, rgb_msg)
            counts['inferred'] += 1
            counts['confirmed'] += int(node.ekf_target is not None)

    sync.registerCallback(images)
    # Drive the upstream reference clock from recording time, never CPU speed.
    next_tick = None
    tick_ns = round(node.ctrl_dt * 1e9)
    while reader.has_next() and not (complete and complete()):
        name, data, time_ns = reader.read_next()
        if next_tick is None:
            next_tick = time_ns + tick_ns
        while advance_reference and next_tick <= time_ns:
            node.control_pub()
            next_tick += tick_ns
        if name == topics[0]:
            node.callback_odometry(deserialize_message(data, Odometry))
            counts['odometry'] += 1
        else:
            stream = depth if name == topics[1] else rgb
            stream.signalMessage(deserialize_message(data, Image))
    if not counts['inferred']:
        raise ValueError(f'No synchronized RGB-D pair after odometry: {counts}')
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path, help='Pinned YOPO-Tracker checkout')
    parser.add_argument('bag', type=Path)
    parser.add_argument('--storage-id', default='sqlite3')
    parser.add_argument('--odom', default='/drone_0/odom')
    parser.add_argument('--depth', default='/drone_0/depth')
    parser.add_argument('--rgb', default='/drone_0/rgb')
    args = parser.parse_args()
    rclpy.init(args=[])
    try:
        node, digest = load_tracker(args.upstream)
        try:
            counts = replay(node, args.bag, args.storage_id, [args.odom, args.depth, args.rgb])
            print(json.dumps(dict(device=node.device, weight_sha256=digest, **counts)))
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
