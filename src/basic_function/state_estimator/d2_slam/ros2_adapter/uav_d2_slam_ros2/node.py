from __future__ import annotations

from array import array
import json
import os
import queue
import socket
import struct
import threading
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import String

from .conversion import depth_meters_to_millimeters, stamp_ns
from .protocol import receive_packet, send_packet


class BridgeNode(Node):
    def __init__(self):
        super().__init__("d2_slam_bridge")
        self.declare_parameter("vehicle_namespace", "drone0")
        self.declare_parameter("camera_name", "FrontCamera")
        self.declare_parameter("socket_path", "/run/d2_slam/bridge.sock")
        self.declare_parameter("output_world_frame", "d2_slam_world")
        self.declare_parameter("output_body_frame", "d2_slam_imu")
        namespace = str(self.get_parameter("vehicle_namespace").value).strip("/")
        camera = str(self.get_parameter("camera_name").value)
        self.socket_path = str(self.get_parameter("socket_path").value)
        self.output_world = namespace + "/" + str(self.get_parameter("output_world_frame").value)
        self.output_body = namespace + "/" + str(self.get_parameter("output_body_frame").value)
        prefix = f"/{namespace}/sensor_measurements"
        self.outgoing: queue.Queue[tuple[dict, bytes]] = queue.Queue(maxsize=2048)
        self.incoming: queue.Queue[tuple[dict, bytes]] = queue.Queue(maxsize=256)
        self.stopping = threading.Event()
        self.counts = {"rgb": 0, "depth": 0, "imu": 0, "odometry": 0, "landmarks": 0,
                       "remote_frames": 0, "input_drop": 0, "output_drop": 0,
                       "reconnect": 0}
        self.remote_ids: set[int] = set()
        self.odom_pub = self.create_publisher(Odometry, f"/{namespace}/d2_slam/odometry", qos_profile_sensor_data)
        self.landmarks_pub = self.create_publisher(
            PointCloud2, f"/{namespace}/d2_slam/landmarks", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, f"/{namespace}/d2_slam/status", 10)
        self.collaboration_pub = self.create_publisher(
            String, f"/{namespace}/d2_slam/collaboration", 10)
        self.create_subscription(
            Image, f"{prefix}/{camera}/rgb/image", self.rgb, qos_profile_sensor_data)
        self.create_subscription(
            Image, f"{prefix}/{camera}/depth_perspective/image", self.depth,
            qos_profile_sensor_data)
        self.create_subscription(Imu, f"{prefix}/imu/data", self.imu, qos_profile_sensor_data)
        self.worker = threading.Thread(target=self.bridge_loop, name="d2-slam-ipc", daemon=False)
        self.worker.start()
        self.create_timer(0.01, self.drain_outputs)
        self.create_timer(1.0, self.publish_status)

    def enqueue(self, header: dict, payload: bytes = b"") -> None:
        try:
            self.outgoing.put_nowait((header, payload))
        except queue.Full:
            self.counts["input_drop"] += 1

    def rgb(self, message: Image) -> None:
        if message.encoding not in ("rgb8", "bgr8", "mono8"):
            self.get_logger().error(f"unsupported RGB encoding {message.encoding}")
            return
        try:
            stamp = stamp_ns(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.enqueue({"kind": "rgb", "stamp_ns": stamp, "frame_id": message.header.frame_id,
                      "width": message.width, "height": message.height,
                      "encoding": message.encoding, "step": message.step}, bytes(message.data))
        self.counts["rgb"] += 1

    def depth(self, message: Image) -> None:
        if message.encoding != "32FC1" or message.is_bigendian:
            self.get_logger().error(
                "D2SLAM requires little-endian 32FC1 DepthPerspective input")
            return
        try:
            converted = depth_meters_to_millimeters(
                bytes(message.data), int(message.width), int(message.height), int(message.step))
            stamp = stamp_ns(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.enqueue({"kind": "depth", "stamp_ns": stamp, "frame_id": message.header.frame_id,
                      "width": message.width, "height": message.height,
                      "encoding": "16UC1", "step": int(message.width) * 2}, converted)
        self.counts["depth"] += 1

    def imu(self, message: Imu) -> None:
        try:
            stamp = stamp_ns(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        values = [message.orientation.x, message.orientation.y, message.orientation.z,
                  message.orientation.w, message.angular_velocity.x, message.angular_velocity.y,
                  message.angular_velocity.z, message.linear_acceleration.x,
                  message.linear_acceleration.y, message.linear_acceleration.z]
        if not all(float("-inf") < value < float("inf") for value in values):
            self.get_logger().error("non-finite IMU sample")
            return
        self.enqueue({"kind": "imu", "stamp_ns": stamp, "frame_id": message.header.frame_id,
                      "orientation": values[:4], "angular_velocity": values[4:7],
                      "linear_acceleration": values[7:10]})
        self.counts["imu"] += 1

    def bridge_loop(self) -> None:
        while not self.stopping.is_set():
            stream = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            stream.settimeout(1.0)
            try:
                stream.connect(self.socket_path)
                stream.settimeout(None)
                self.counts["reconnect"] += 1
                reader = threading.Thread(
                    target=self.read_outputs, args=(stream,), name="d2-slam-ipc-reader", daemon=True)
                reader.start()
                while not self.stopping.is_set() and reader.is_alive():
                    try:
                        header, payload = self.outgoing.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    send_packet(stream, header, payload)
            except (FileNotFoundError, ConnectionError, OSError, EOFError, ValueError):
                if not self.stopping.wait(0.5):
                    continue
            finally:
                try:
                    stream.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                stream.close()

    def read_outputs(self, stream: socket.socket) -> None:
        try:
            while not self.stopping.is_set():
                packet = receive_packet(stream)
                try:
                    self.incoming.put_nowait(packet)
                except queue.Full:
                    self.counts["output_drop"] += 1
        except (ConnectionError, OSError, EOFError, ValueError):
            return

    @staticmethod
    def set_stamp(target, stamp: int) -> None:
        target.sec, target.nanosec = divmod(int(stamp), 1_000_000_000)

    def drain_outputs(self) -> None:
        for _ in range(64):
            try:
                header, payload = self.incoming.get_nowait()
            except queue.Empty:
                return
            if header["kind"] == "odometry":
                msg = Odometry()
                self.set_stamp(msg.header.stamp, header["stamp_ns"])
                msg.header.frame_id, msg.child_frame_id = self.output_world, self.output_body
                pose, twist = header["pose"], header["twist"]
                (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) = pose[:3]
                (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = pose[3:]
                (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                 msg.twist.twist.linear.z) = twist[:3]
                (msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                 msg.twist.twist.angular.z) = twist[3:]
                msg.pose.covariance = header.get("pose_covariance", [0.0] * 36)
                msg.twist.covariance = header.get("twist_covariance", [0.0] * 36)
                self.odom_pub.publish(msg)
                self.counts["odometry"] += 1
            elif header["kind"] == "landmarks":
                if len(payload) % 16:
                    self.get_logger().error("invalid D2SLAM landmark payload")
                    continue
                msg = PointCloud2()
                self.set_stamp(msg.header.stamp, header["stamp_ns"])
                msg.header.frame_id = self.output_world
                msg.height, msg.width = 1, len(payload) // 16
                msg.fields = [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                msg.is_bigendian, msg.point_step, msg.row_step = False, 16, len(payload)
                msg.is_dense, msg.data = False, array("B", payload)
                self.landmarks_pub.publish(msg)
                self.counts["landmarks"] += 1
            elif header["kind"] == "remote_frame":
                remote_id = int(header["drone_id"])
                self.remote_ids.add(remote_id)
                msg = String()
                msg.data = json.dumps({
                    "drone_id": remote_id,
                    "frame_id": int(header["frame_id"]),
                    "reference_frame_id": int(header["reference_frame_id"]),
                    "is_keyframe": bool(header["is_keyframe"]),
                    "stamp_ns": int(header["stamp_ns"]),
                }, separators=(",", ":"), sort_keys=True)
                self.collaboration_pub.publish(msg)
                self.counts["remote_frames"] += 1

    def publish_status(self) -> None:
        msg = String()
        fields = dict(self.counts)
        fields["remote_ids"] = ":".join(str(value) for value in sorted(self.remote_ids))
        msg.data = ",".join(f"{key}={value}" for key, value in sorted(fields.items()))
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.stopping.set()
        self.worker.join(timeout=3.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = BridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
