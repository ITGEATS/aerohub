#!/usr/bin/env python3
"""Bridge local IPC packets to the unmodified ROS 1 D2SLAM process."""
from __future__ import annotations

import json
import os
import queue
import socket
import struct
import threading

import rospy
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import ChannelFloat32, Image, Imu, PointCloud
from swarm_msgs.msg import VIOFrame


PREFIX = struct.Struct("!II")
MAX_HEADER_BYTES = 1 << 20
MAX_PAYLOAD_BYTES = 64 << 20


def receive_exact(stream, size):
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.recv(size - len(chunks))
        if not chunk:
            raise EOFError("D2SLAM bridge disconnected")
        chunks.extend(chunk)
    return bytes(chunks)


def receive_packet(stream):
    header_size, payload_size = PREFIX.unpack(receive_exact(stream, PREFIX.size))
    if header_size > MAX_HEADER_BYTES or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError("invalid D2SLAM bridge packet length")
    header = json.loads(receive_exact(stream, header_size).decode("utf-8"))
    return header, receive_exact(stream, payload_size)


def send_packet(stream, header, payload=b""):
    encoded = json.dumps(header, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(encoded) > MAX_HEADER_BYTES or len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("D2SLAM bridge packet exceeds configured limit")
    stream.sendall(PREFIX.pack(len(encoded), len(payload)) + encoded + payload)


class Proxy:
    def __init__(self):
        self.socket_path = rospy.get_param("~socket_path", "/run/d2_slam/bridge.sock")
        self.rgb_pub = rospy.Publisher("/d2slam/input/rgb", Image, queue_size=2)
        self.depth_pub = rospy.Publisher("/d2slam/input/depth", Image, queue_size=2)
        self.imu_pub = rospy.Publisher("/d2slam/input/imu", Imu, queue_size=1000)
        self.connection = None
        self.connection_lock = threading.Lock()
        self.stopping = threading.Event()
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        os.makedirs(os.path.dirname(self.socket_path), exist_ok=True)
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        self.server.bind(self.socket_path)
        self.server.listen(1)
        self.server.settimeout(1.0)
        self.thread = threading.Thread(target=self.serve, name="d2-slam-proxy", daemon=False)
        self.thread.start()
        odom_topic = rospy.get_param("~odometry_topic", "/d2vins/odometry")
        landmarks_topic = rospy.get_param("~landmarks_topic", "/d2vins/point_cloud")
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odometry, queue_size=50)
        self.landmarks_sub = rospy.Subscriber(
            landmarks_topic, PointCloud, self.landmarks, queue_size=2)
        remote_frame_topic = rospy.get_param("~remote_frame_topic", "/d2vins/frame_remote")
        self.remote_frame_sub = rospy.Subscriber(
            remote_frame_topic, VIOFrame, self.remote_frame, queue_size=100)

    @staticmethod
    def stamp(stamp_ns):
        return rospy.Time(secs=int(stamp_ns // 1_000_000_000),
                          nsecs=int(stamp_ns % 1_000_000_000))

    def serve(self):
        while not self.stopping.is_set() and not rospy.is_shutdown():
            try:
                stream, _ = self.server.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                if self.stopping.is_set() or rospy.is_shutdown():
                    return
                rospy.logerr("D2SLAM bridge listener failed: %s", exc)
                return
            with self.connection_lock:
                if self.connection is not None:
                    self.connection.close()
                self.connection = stream
            try:
                while not self.stopping.is_set() and not rospy.is_shutdown():
                    header, payload = receive_packet(stream)
                    self.input(header, payload)
            except (EOFError, OSError, ValueError, KeyError, TypeError) as exc:
                rospy.logwarn("D2SLAM bridge input disconnected: %s", exc)
            finally:
                with self.connection_lock:
                    if self.connection is stream:
                        self.connection = None
                stream.close()

    def input(self, header, payload):
        kind = header["kind"]
        if kind in ("rgb", "depth"):
            expected = int(header["step"]) * int(header["height"])
            if len(payload) != expected:
                raise ValueError("image payload length does not match its layout")
            msg = Image()
            msg.header.stamp = self.stamp(header["stamp_ns"])
            msg.header.frame_id = header["frame_id"]
            msg.width, msg.height = int(header["width"]), int(header["height"])
            msg.encoding, msg.step, msg.data = header["encoding"], int(header["step"]), payload
            (self.rgb_pub if kind == "rgb" else self.depth_pub).publish(msg)
            return
        if kind == "imu":
            msg = Imu()
            msg.header.stamp = self.stamp(header["stamp_ns"])
            msg.header.frame_id = header["frame_id"]
            (msg.orientation.x, msg.orientation.y, msg.orientation.z,
             msg.orientation.w) = header["orientation"]
            (msg.angular_velocity.x, msg.angular_velocity.y,
             msg.angular_velocity.z) = header["angular_velocity"]
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z) = header["linear_acceleration"]
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0
            self.imu_pub.publish(msg)
            return
        raise ValueError("unknown D2SLAM input kind: " + str(kind))

    def output(self, header, payload=b""):
        with self.connection_lock:
            stream = self.connection
            if stream is None:
                return
            try:
                send_packet(stream, header, payload)
            except (OSError, ValueError):
                try:
                    stream.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    def odometry(self, message):
        stamp_ns = message.header.stamp.secs * 1_000_000_000 + message.header.stamp.nsecs
        pose, twist = message.pose.pose, message.twist.twist
        self.output({
            "kind": "odometry", "stamp_ns": stamp_ns,
            "pose": [pose.position.x, pose.position.y, pose.position.z,
                     pose.orientation.x, pose.orientation.y, pose.orientation.z,
                     pose.orientation.w],
            "twist": [twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z],
            "pose_covariance": list(message.pose.covariance),
            "twist_covariance": list(message.twist.covariance),
        })

    def landmarks(self, message):
        if not message.channels or message.channels[0].name != "rgb":
            rospy.logwarn_throttle(5.0, "D2SLAM point cloud lacks rgb channel")
            colors = [0.0] * len(message.points)
        else:
            colors = message.channels[0].values
        if len(colors) != len(message.points):
            rospy.logwarn_throttle(5.0, "D2SLAM point cloud channel length mismatch")
            return
        payload = bytearray()
        for point, color in zip(message.points, colors):
            payload.extend(struct.pack("<ffff", point.x, point.y, point.z, color))
        stamp_ns = message.header.stamp.secs * 1_000_000_000 + message.header.stamp.nsecs
        self.output({"kind": "landmarks", "stamp_ns": stamp_ns}, bytes(payload))

    def remote_frame(self, message):
        stamp_ns = message.header.stamp.secs * 1_000_000_000 + message.header.stamp.nsecs
        self.output({
            "kind": "remote_frame",
            "stamp_ns": stamp_ns,
            "drone_id": int(message.drone_id),
            "frame_id": int(message.frame_id),
            "reference_frame_id": int(message.reference_frame_id),
            "is_keyframe": bool(message.is_keyframe),
        })

    def close(self):
        self.stopping.set()
        self.server.close()
        with self.connection_lock:
            if self.connection is not None:
                self.connection.close()
        self.thread.join(timeout=3.0)
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass


def main():
    rospy.init_node("d2_slam_ros1_proxy")
    proxy = Proxy()
    rospy.on_shutdown(proxy.close)
    rospy.spin()


if __name__ == "__main__":
    main()
