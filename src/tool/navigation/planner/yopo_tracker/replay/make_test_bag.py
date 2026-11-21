"""Make a finite empty-scene RGB-D fixture for negative-path inference."""
import argparse

from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', help='New sqlite3 bag directory')
    parser.add_argument('--frame', default='world', help='Fixed frame declared by test odometry')
    args = parser.parse_args()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.destination, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    odom = Odometry()
    odom.header.frame_id = args.frame
    odom.child_frame_id = 'base_link'
    odom.pose.pose.orientation.w = 1.
    depth = Image(height=192, width=320, encoding='32FC1', step=320*4)
    depth.data = np.full((192, 320), 10., dtype='<f4').tobytes()
    rgb = Image(height=192, width=320, encoding='bgr8', step=320*3)
    rgb.data = bytes(rgb.height * rgb.step)
    entries = [('odom', 'nav_msgs/msg/Odometry', odom),
               ('depth', 'sensor_msgs/msg/Image', depth), ('rgb', 'sensor_msgs/msg/Image', rgb)]
    for name, typename, _ in entries:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name='/drone_0/' + name, type=typename, serialization_format='cdr'))
    for index in range(8):
        t = 1_000_000_000 + index * 1_000_000_000 // 30
        for name, _, message in entries:
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(t, 1_000_000_000)
            writer.write('/drone_0/' + name, serialize_message(message), t)


if __name__ == '__main__':
    main()
