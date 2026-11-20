"""Synchronized RGB-D and AS2 state for the single-vehicle tracker."""
from bisect import bisect_left
from collections import deque
from copy import deepcopy

import message_filters
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


ODOM_FRAME = 'drone0/odom'
BODY_FRAME = 'drone0/base_link'
CAMERA_FRAME = 'drone0/FrontCamera_optical'
CAMERA_PATH = '/drone0/sensor_measurements/FrontCamera'
SYNC_S = .05  # Upstream approximate synchronization window.
SECOND = 10**9
OBSERVATION_AGE_NS = 300_000_000  # Ordinary YOPO max_observation_age_s.
SYNC_NS = round(SYNC_S * SECOND)


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class TrackingInputs(Node):
    def __init__(self, node_name, *, record_all=False):
        super().__init__(node_name, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.record_all = record_all
        self.last_snapshot = 0
        self.poses, self.twists, self.info, self.transforms = {}, {}, {}, {}
        self.tf = Buffer(cache_time=Duration(seconds=20))
        self.pairs, self.clock = ([] if record_all else deque(maxlen=1)), []
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 lambda msg: self.store_state(self.poses, msg),
                                 qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/drone0/self_localization/twist',
                                 lambda msg: self.store_state(self.twists, msg),
                                 qos_profile_sensor_data)
        if record_all:
            self.create_subscription(Clock, '/clock', lambda msg: self.clock.append(ns(msg.clock)),
                                     qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', self.store_transforms,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/tf', self.store_dynamic, qos_profile_sensor_data)
        for stream in ('depth_planar', 'rgb'):
            self.create_subscription(CameraInfo, f'{CAMERA_PATH}/{stream}/camera_info',
                                     lambda msg, key=stream: self.info.update({key: msg}), 1)
        self.depth = message_filters.Subscriber(self, Image, CAMERA_PATH + '/depth_planar/image',
                                               qos_profile=QoSProfile(depth=1))
        self.rgb = message_filters.Subscriber(self, Image, CAMERA_PATH + '/rgb/image',
                                             qos_profile=QoSProfile(depth=1))
        self.sync = message_filters.ApproximateTimeSynchronizer([self.depth, self.rgb], 1, SYNC_S)
        self.sync.registerCallback(lambda depth, rgb: self.pairs.append((depth, rgb)))

    def store_state(self, history, msg):
        stamp = ns(msg.header.stamp)
        history[stamp] = msg
        if self.record_all:
            return
        earliest = stamp - OBSERVATION_AGE_NS - SYNC_NS
        for old_stamp in tuple(history):
            if old_stamp < earliest:
                del history[old_stamp]

    def store_transforms(self, msg):
        self.transforms.update({tf.child_frame_id: tf for tf in msg.transforms})
        for tf in msg.transforms:
            self.tf.set_transform_static(tf, 'airsim_ground_capture')

    def store_dynamic(self, msg):
        for tf in msg.transforms:
            self.tf.set_transform(tf, 'airsim_ground_capture')

    def to_odom(self, pose, twist):
        message = Odometry(header=deepcopy(pose.header), child_frame_id=BODY_FRAME)
        message.header.frame_id = ODOM_FRAME
        message.pose.pose = deepcopy(pose.pose)
        message.twist.twist = deepcopy(twist.twist)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        orientation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        if pose.header.frame_id != ODOM_FRAME:
            transform = self.tf.lookup_transform(
                ODOM_FRAME, pose.header.frame_id, Time.from_msg(pose.header.stamp)).transform
            t, r = transform.translation, transform.rotation
            fixed_rotation = Rotation.from_quat([r.x, r.y, r.z, r.w])
            p.x, p.y, p.z = map(float, fixed_rotation.apply([p.x, p.y, p.z]) + [t.x, t.y, t.z])
            orientation = fixed_rotation * orientation
            q.x, q.y, q.z, q.w = map(float, orientation.as_quat())
        if twist.header.frame_id == BODY_FRAME:
            linear = message.twist.twist.linear
            linear.x, linear.y, linear.z = map(float, orientation.apply([linear.x, linear.y, linear.z]))
        elif twist.header.frame_id != ODOM_FRAME:
            raise ValueError('Unexpected twist frame')
        return message

    def check_camera(self):
        transform = self.transforms.get(CAMERA_FRAME)
        if transform is None or transform.header.frame_id != BODY_FRAME:
            raise ValueError('Missing body-to-optical static transform')
        t, q = transform.transform.translation, transform.transform.rotation
        np.testing.assert_allclose([t.x, t.y, t.z], 0., atol=1e-6)
        np.testing.assert_allclose(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                   [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]], atol=1e-6)
        for stream in ('depth_planar', 'rgb'):
            info = self.info[stream]
            assert info.header.frame_id == CAMERA_FRAME and (info.width, info.height) == (320, 192)
            assert not any(info.d)
        np.testing.assert_array_equal(self.info['depth_planar'].k, self.info['rgb'].k)

    def snapshot(self, after):
        if not self.pairs:
            return None
        depth, rgb = self.pairs[-1]
        depth_ns, rgb_ns = ns(depth.header.stamp), ns(rgb.header.stamp)
        if min(depth_ns, rgb_ns) <= max(after, self.last_snapshot):
            return None
        now = self.get_clock().now().nanoseconds
        if max(depth_ns, rgb_ns) > now or now - min(depth_ns, rgb_ns) > OBSERVATION_AGE_NS:
            return None
        stamps = sorted(self.poses.keys() & self.twists.keys())
        index = bisect_left(stamps, depth_ns)
        nearby = stamps[max(0, index - 1):index + 1]
        if not nearby:
            return None
        stamp = min(nearby, key=lambda t: abs(t - depth_ns))
        if max(abs(stamp - depth_ns), abs(stamp - rgb_ns)) > SYNC_NS:
            return None
        try:
            odom = self.to_odom(self.poses[stamp], self.twists[stamp])
        except TransformException:
            return None
        check_images(depth, rgb)
        self.last_snapshot = max(depth_ns, rgb_ns)
        return depth, rgb, odom


def check_images(depth, rgb):
    assert depth.header.frame_id == rgb.header.frame_id == CAMERA_FRAME
    assert (depth.width, depth.height, depth.step, depth.encoding) == (320, 192, 1280, '32FC1')
    assert (rgb.width, rgb.height, rgb.step, rgb.encoding) == (320, 192, 960, 'rgb8')
    assert not depth.is_bigendian
    assert len(depth.data) == depth.height * depth.step and len(rgb.data) == rgb.height * rgb.step
