"""Record synchronized, non-actuating AirSim inputs for an offline Tracker run."""
import argparse
from bisect import bisect_left
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.serialization import serialize_message
import rosbag2_py
from tf2_ros import TransformException

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_adapter.bridge.live_inputs import CAMERA_FRAME, SYNC_S, TrackingInputs, check_images, ns


class GroundCapture(TrackingInputs):
    def __init__(self, node_name='yopo_tracker_ground_capture'):
        super().__init__(node_name, record_all=True)

    def save(self, output):
        self.check_camera()
        if len(set(self.clock)) < 2 or any(a > b for a, b in zip(self.clock, self.clock[1:])):
            raise ValueError('Clock not advancing monotonically')
        stamps = sorted(self.poses.keys() & self.twists.keys())
        if not stamps or not self.pairs:
            raise ValueError(f'Inputs missing: RGB-D={len(self.pairs)}, matched state={len(stamps)}')

        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output / 'bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for name, typename in [('odom', 'nav_msgs/msg/Odometry'), ('depth', 'sensor_msgs/msg/Image'),
                               ('rgb', 'sensor_msgs/msg/Image')]:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name='/drone_0/' + name, type=typename, serialization_format='cdr'))
        rows = []
        previous_event = tf_unavailable = 0
        for depth, rgb in self.pairs:
            check_images(depth, rgb)
            depth_ns, rgb_ns = ns(depth.header.stamp), ns(rgb.header.stamp)
            index = bisect_left(stamps, depth_ns)
            stamp = min(stamps[max(0, index - 1):index + 1], key=lambda t: abs(t - depth_ns))
            if max(abs(stamp - depth_ns), abs(stamp - rgb_ns)) > round(SYNC_S * 10**9):
                continue
            pose, twist = self.poses[stamp], self.twists[stamp]
            try:
                odom = self.to_odom(pose, twist)
            except TransformException:
                tf_unavailable += 1
                continue
            # Storage ordering keeps the original measurement stamps in the payloads.
            event = max(depth_ns, rgb_ns, stamp)
            if event <= previous_event:
                continue
            previous_event = event
            for name, msg in [('odom', odom), ('depth', depth), ('rgb', rgb)]:
                writer.write('/drone_0/' + name, serialize_message(msg), event)
            rows.append(dict(depth_ns=depth_ns, rgb_ns=rgb_ns, pose_ns=stamp,
                             skew_ns=abs(depth_ns-rgb_ns), pose_skew_ns=abs(stamp-depth_ns)))
        if not rows:
            raise ValueError(f'No synchronized RGB-D and estimator observations, TF misses={tf_unavailable}')
        report = dict(clock_samples=len(set(self.clock)), paired_images=len(self.pairs),
                      state_samples=len(stamps), observations=len(rows), tf_unavailable=tf_unavailable,
                      camera_k=list(self.info['rgb'].k), rows=rows)
        (output / 'capture.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({key: value for key, value in report.items() if key != 'rows'}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--seconds', type=float, default=8.)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = GroundCapture()
    try:
        finish = time.monotonic() + args.seconds
        while time.monotonic() < finish:
            rclpy.spin_once(node, timeout_sec=.02)
        node.save(args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
