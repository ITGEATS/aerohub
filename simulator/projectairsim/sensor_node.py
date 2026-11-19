#!/usr/bin/env python3
"""Normalize upstream ROS messages; SDK access remains in the native bridge."""
import argparse
from collections import deque
import time
import os
import itertools
from copy import deepcopy
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from gimbal import interpolate, stabilized_transform

from sensor_contract import streams

CALIBRATION_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SensorAdapter(Node):
    def __init__(self, inventory):
        super().__init__("projectairsim_sensor_adapter", parameter_overrides=[Parameter("use_sim_time",value=True)])
        self.timing_sequence=itertools.count()
        self.timing_publisher=(self.create_publisher(String,'/diagnostics/vision_pipeline',100)
            if os.environ.get('AEROHUB_VISION_TIMING')=='1' else None)
        if self.timing_publisher is not None:
            self.udp_publisher=self.create_publisher(String,'/diagnostics/vision_udp',20)
            self.udp_timer=self.create_timer(1.,self.publish_udp)
        self.subscriptions_owned = []
        self.broadcaster = StaticTransformBroadcaster(self)
        self.calibrations = []
        self.pointing = {}
        for stream in inventory:
            if stream.get('pointing_target') and stream['frame'] not in self.pointing:
                from camera_pointing import CameraPointing
                self.pointing[stream['frame']] = CameraPointing(self,stream)
        self.dynamic = TransformBroadcaster(self)
        self.gimbal_poses = {}
        self.gimbal_pending = deque(maxlen=32)
        for source in set(s['pose_source'] for s in inventory if s.get('gimbal_origin')):
            self.gimbal_poses[source] = deque(maxlen=1000)
            self.subscriptions_owned.append(self.create_subscription(PoseStamped, source,
                lambda msg, key=source:self.receive_gimbal_pose(key,msg), 20))
        self.gimbal_timer = self.create_timer(.01, self.drain_gimbal)
        transforms = {}
        for stream in inventory:
            frame = stream["frame"]
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = stream["parent"], frame
            xyz, quat = stream["transform"]
            (transform.transform.translation.x, transform.transform.translation.y,
             transform.transform.translation.z) = xyz
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = quat
            if not stream.get("gimbal_origin"):
                transforms[frame] = transform
            msg_type = {"image": Image, "imu": Imu, "lidar": PointCloud2}[stream["kind"]]
            publisher = self.create_publisher(msg_type, stream["target"], qos_profile_sensor_data)
            self.subscribe(msg_type, stream["source"], publisher, stream)
            if stream["kind"] == "image":
                target = stream["target"].removesuffix("/image") + "/camera_info"
                publisher = self.create_publisher(CameraInfo, target, CALIBRATION_QOS)
                # Native camera.cpp defines this static pinhole model directly from
                # capture-settings (zero distortion, square pixels). Native info
                # topics may be emitted before a late ROS subscriber exists.
                # Keep its timeless calibration available without rewriting an
                # image acquisition timestamp or fabricating a sensor sample.
                capture = stream["capture"]
                calibration = CameraInfo()
                calibration.header.frame_id = frame
                calibration.width, calibration.height = capture["width"], capture["height"]
                focal = capture["width"] / (2 * math.tan(math.radians(capture["fov-degrees"]) / 2))
                cx, cy = capture["width"] / 2, capture["height"] / 2
                calibration.k = [focal, 0., cx, 0., focal, cy, 0., 0., 1.]
                calibration.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                calibration.p = [focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.]
                calibration.d = [0.] * 5
                calibration.distortion_model = "plumb_bob"
                self.calibrations.append((publisher, calibration))
                self.subscribe(CameraInfo, stream["source"] + "/camera_info", publisher, stream,
                               calibration=True)
        self.broadcaster.sendTransform(list(transforms.values()))
        self.calibration_timer = self.create_timer(.5, self.publish_calibrations)
        self.publish_calibrations()

    def trace(self, stage, message=None, **details):
        publisher=getattr(self,'timing_publisher',None)
        if publisher is None:return
        header=getattr(message,'image_header',None)
        if header is None or (header.stamp.sec==0 and header.stamp.nanosec==0):
            header=getattr(message,'header',None)
        sample_ns=(header.stamp.sec*10**9+header.stamp.nanosec) if header is not None else None
        event=dict(stage=stage,sample_ns=sample_ns,mono_ns=time.monotonic_ns(),
            wall_ns=time.time_ns(),ros_ns=self.get_clock().now().nanoseconds,
            node=self.get_name(),pid=os.getpid(),seq=next(self.timing_sequence),**details)
        publisher.publish(String(data=json.dumps(event,separators=(',',':'))))

    def publish_udp(self):
        from udp_diagnostics import udp_snapshot
        row=udp_snapshot({'sensor_node.py','projectairsim_ros2_cpp_node'})
        row['role']='sensor_bridge'
        self.udp_publisher.publish(String(data=json.dumps(row)))

    def publish_calibrations(self):
        for publisher, calibration in self.calibrations:
            publisher.publish(calibration)

    def subscribe(self, msg_type, source, publisher, stream, calibration=False):
        def receive(message):
            if isinstance(message,Image):self.trace('sensor_receive',message,source=source)
            if not calibration and message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
                return
            msg = deepcopy(message)
            # Native camera pixels are already optical; native IMU/cloud vectors
            # already have FRD->FLU conversion. Attach the corresponding real TF.
            msg.header.frame_id = stream["frame"]
            if isinstance(msg, Imu):
                # Native orientation is in a different world convention. Do not
                # advertise it as an ENU estimate or manufacture covariance.
                msg.orientation.x = msg.orientation.y = msg.orientation.z = 0.0
                msg.orientation.w = 1.0
                msg.orientation_covariance[0] = -1.0
            elif isinstance(msg, CameraInfo):
                capture = stream["capture"]
                focal = capture["width"] / (2 * math.tan(math.radians(capture["fov-degrees"]) / 2))
                if (msg.width != capture["width"] or msg.height != capture["height"]
                    or not math.isclose(msg.k[0], focal, rel_tol=1e-5)
                    or not math.isclose(msg.k[4], focal, rel_tol=1e-5)
                    or any(msg.d) or list(msg.r) != [1, 0, 0, 0, 1, 0, 0, 0, 1]):
                    raise ValueError("native calibration differs from supported pinhole configuration")
                # Native camera.cpp emits identity P. For this verified zero-
                # distortion, unrectified monocular model P is [K | 0].
                msg.p = [float(msg.k[0]), float(msg.k[1]), float(msg.k[2]), 0.0,
                         float(msg.k[3]), float(msg.k[4]), float(msg.k[5]), 0.0,
                         float(msg.k[6]), float(msg.k[7]), float(msg.k[8]), 0.0]
                msg.distortion_model = "plumb_bob"
            elif isinstance(msg, Image) and (msg.width != stream["capture"]["width"] or msg.height != stream["capture"]["height"]):
                raise ValueError("native image dimensions differ from capture-settings calibration")
            elif isinstance(msg, Image) and msg.encoding == "bgr8":
                # Match the existing AirSim rgb8 contract, preserving row padding.
                data = bytearray(msg.data)
                for row in range(msg.height):
                    start, end = row * msg.step, row * msg.step + msg.width * 3
                    data[start:end:3], data[start+2:end:3] = data[start+2:end:3], data[start:end:3]
                msg.data, msg.encoding = bytes(data), "rgb8"
            if isinstance(msg, Image) and stream.get('gimbal_origin'):
                if len(self.gimbal_pending)==self.gimbal_pending.maxlen:
                    self.trace('sensor_drop',self.gimbal_pending[0][1],reason='pose_queue_full')
                self.trace('pose_queue',msg)
                self.gimbal_pending.append((time.monotonic(),msg,publisher,stream))
                self.drain_gimbal()
            else:
                if isinstance(msg,Image):self.trace('sensor_publish_begin',msg)
                publisher.publish(msg)
                if isinstance(msg,Image):self.trace('sensor_publish',msg)
        self.subscriptions_owned.append(self.create_subscription(
            msg_type, source, receive, qos_profile_sensor_data))


    @staticmethod
    def stamp(message):
        return message.header.stamp.sec*10**9+message.header.stamp.nanosec

    def receive_gimbal_pose(self, source, message):
        history=self.gimbal_poses[source]
        stamp=self.stamp(message)
        if history and stamp<history[-1][0]:
            history.clear()
            self.gimbal_pending.clear()
            for controller in self.pointing.values():
                controller.error='native clock reset; restart camera pointing'
        if history and stamp==history[-1][0]:return
        q=message.pose.orientation
        history.append((stamp,(q.x,q.y,q.z,q.w)))
        self.drain_gimbal()

    def drain_gimbal(self):
        for _ in range(len(self.gimbal_pending)):
            received,msg,publisher,stream=self.gimbal_pending.popleft()
            stamp=self.stamp(msg)
            history=self.gimbal_poses[stream['pose_source']]
            pair=next(((a,b) for a,b in zip(history,list(history)[1:]) if a[0]<=stamp<=b[0]),None)
            if pair is None:
                if time.monotonic()-received<.25:
                    self.gimbal_pending.append((received,msg,publisher,stream))
                else:self.trace('sensor_drop',msg,reason='pose_bracket_timeout')
                continue  # no latest-pose fallback or synthetic static extrinsic
            a,b=pair
            if b[0]-a[0]>100_000_000:
                self.trace('sensor_drop',msg,reason='pose_bracket_gap',span_ns=b[0]-a[0])
                continue
            origin=stream['gimbal_origin']
            controller=self.pointing.get(stream['frame'])
            if controller is not None:
                origin=controller.origin_at(stamp)
                if origin is None:
                    self.trace('sensor_drop',msg,reason='camera_transition_or_unknown',
                        controller_error=controller.error,before_ns=controller.before,phase=controller.phase)
                    continue
            q=interpolate(a[1],b[1],(stamp-a[0])/(b[0]-a[0]))
            xyz,rotation=stabilized_transform(q,origin)
            tf=TransformStamped()
            tf.header.stamp=msg.header.stamp
            tf.header.frame_id,tf.child_frame_id=stream['parent'],stream['frame']
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=xyz
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=rotation
            self.dynamic.sendTransform(tf)
            self.trace('sensor_publish_begin',msg)
            publisher.publish(msg)
            self.trace('sensor_publish',msg,pose_wait_ms=(time.monotonic()-received)*1000)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--topic-root", default="/ProjectAirsim")
    args = parser.parse_args()
    inventory = streams(json.loads(args.scene.read_text()), json.loads(args.runtime.read_text()),
                        args.topic_root)
    rclpy.init()
    node = SensorAdapter(inventory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
