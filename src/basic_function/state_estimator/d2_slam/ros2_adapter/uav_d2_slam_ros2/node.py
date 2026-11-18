from __future__ import annotations

from array import array
import json
import math
import struct

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, PointCloud, PointCloud2, PointField
from std_msgs.msg import String
from swarm_msgs.msg import VIOFrame

from .conversion import depth_meters_to_millimeters, stamp_ns


def point_cloud_to_point_cloud2(message: PointCloud, frame_id: str) -> PointCloud2:
    channel = next((item for item in message.channels if item.name == "rgb"), None)
    if channel is not None and len(channel.values) != len(message.points):
        raise ValueError("D2SLAM rgb channel length does not match point count")

    payload = bytearray()
    is_dense = True
    for index, point in enumerate(message.points):
        rgb = channel.values[index] if channel is not None else 0.0
        values = (float(point.x), float(point.y), float(point.z), float(rgb))
        is_dense = is_dense and all(math.isfinite(value) for value in values[:3])
        payload.extend(struct.pack("<ffff", *values))

    output = PointCloud2()
    output.header = message.header
    output.header.frame_id = frame_id
    output.height = 1
    output.width = len(message.points)
    output.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    output.is_bigendian = False
    output.point_step = 16
    output.row_step = len(payload)
    output.is_dense = is_dense
    output.data = array("B", payload)
    return output


class AdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("d2_slam_adapter")
        self.declare_parameter("camera_name", "FrontCamera")
        self.declare_parameter("output_world_frame", "d2_slam_world")
        self.declare_parameter("output_body_frame", "d2_slam_imu")
        self.declare_parameter("native_rgb_topic", "d2slam/input/rgb")
        self.declare_parameter("native_depth_topic", "d2slam/input/depth")
        self.declare_parameter("native_imu_topic", "d2slam/input/imu")
        self.declare_parameter("native_odometry_topic", "imu_propagation")
        self.declare_parameter("native_landmarks_topic", "point_cloud")
        self.declare_parameter("native_remote_frame_topic", "frame_remote")

        namespace = self.get_namespace().strip("/")
        camera = str(self.get_parameter("camera_name").value)
        frame_prefix = f"{namespace}/" if namespace else ""
        self.output_world = frame_prefix + str(
            self.get_parameter("output_world_frame").value)
        self.output_body = frame_prefix + str(
            self.get_parameter("output_body_frame").value)
        sensor_prefix = "sensor_measurements"

        self.counts = {
            "rgb": 0,
            "depth": 0,
            "imu": 0,
            "odometry": 0,
            "landmarks": 0,
            "remote_frames": 0,
            "rejected": 0,
        }
        self.remote_ids: set[int] = set()

        self.rgb_pub = self.create_publisher(
            Image, str(self.get_parameter("native_rgb_topic").value),
            qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(
            Image, str(self.get_parameter("native_depth_topic").value),
            qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(
            Imu, str(self.get_parameter("native_imu_topic").value),
            qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(
            Odometry, "d2_slam/odometry", qos_profile_sensor_data)
        self.landmarks_pub = self.create_publisher(
            PointCloud2, "d2_slam/landmarks", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, "d2_slam/status", 10)
        self.collaboration_pub = self.create_publisher(
            String, "d2_slam/collaboration", 10)

        self.create_subscription(
            Image, f"{sensor_prefix}/{camera}/rgb/image", self.rgb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, f"{sensor_prefix}/{camera}/depth_perspective/image", self.depth,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, f"{sensor_prefix}/imu/data", self.imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(self.get_parameter("native_odometry_topic").value),
            self.odometry, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud, str(self.get_parameter("native_landmarks_topic").value),
            self.landmarks, qos_profile_sensor_data)
        self.create_subscription(
            VIOFrame, str(self.get_parameter("native_remote_frame_topic").value),
            self.remote_frame, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_status)

    def reject(self, reason: str) -> None:
        self.counts["rejected"] += 1
        self.get_logger().error(reason)

    def rgb(self, message: Image) -> None:
        if message.encoding not in ("rgb8", "bgr8", "mono8"):
            self.reject(f"unsupported RGB encoding {message.encoding}")
            return
        try:
            stamp_ns(message)
        except ValueError as exc:
            self.reject(str(exc))
            return
        self.rgb_pub.publish(message)
        self.counts["rgb"] += 1

    def depth(self, message: Image) -> None:
        if message.encoding != "32FC1" or message.is_bigendian:
            self.reject("D2SLAM requires little-endian 32FC1 DepthPerspective input")
            return
        try:
            converted = depth_meters_to_millimeters(
                bytes(message.data), int(message.width), int(message.height),
                int(message.step))
            stamp_ns(message)
        except ValueError as exc:
            self.reject(str(exc))
            return
        output = Image()
        output.header = message.header
        output.height = message.height
        output.width = message.width
        output.encoding = "16UC1"
        output.is_bigendian = 0
        output.step = int(message.width) * 2
        output.data = array("B", converted)
        self.depth_pub.publish(output)
        self.counts["depth"] += 1

    def imu(self, message: Imu) -> None:
        values = [
            message.orientation.x, message.orientation.y, message.orientation.z,
            message.orientation.w, message.angular_velocity.x,
            message.angular_velocity.y, message.angular_velocity.z,
            message.linear_acceleration.x, message.linear_acceleration.y,
            message.linear_acceleration.z,
        ]
        try:
            stamp_ns(message)
        except ValueError as exc:
            self.reject(str(exc))
            return
        if not all(math.isfinite(value) for value in values):
            self.reject("non-finite IMU sample")
            return
        self.imu_pub.publish(message)
        self.counts["imu"] += 1

    def odometry(self, message: Odometry) -> None:
        try:
            stamp_ns(message)
        except ValueError as exc:
            self.reject(str(exc))
            return
        message.header.frame_id = self.output_world
        message.child_frame_id = self.output_body
        self.odom_pub.publish(message)
        self.counts["odometry"] += 1

    def landmarks(self, message: PointCloud) -> None:
        try:
            output = point_cloud_to_point_cloud2(message, self.output_world)
        except ValueError as exc:
            self.reject(str(exc))
            return
        self.landmarks_pub.publish(output)
        self.counts["landmarks"] += 1

    def remote_frame(self, message: VIOFrame) -> None:
        remote_id = int(message.drone_id)
        self.remote_ids.add(remote_id)
        output = String()
        output.data = json.dumps({
            "drone_id": remote_id,
            "frame_id": int(message.frame_id),
            "reference_frame_id": int(message.reference_frame_id),
            "is_keyframe": bool(message.is_keyframe),
            "stamp_ns": stamp_ns(message),
        }, separators=(",", ":"), sort_keys=True)
        self.collaboration_pub.publish(output)
        self.counts["remote_frames"] += 1

    def publish_status(self) -> None:
        output = String()
        fields = dict(self.counts)
        fields["remote_ids"] = ":".join(
            str(value) for value in sorted(self.remote_ids))
        output.data = ",".join(
            f"{key}={value}" for key, value in sorted(fields.items()))
        self.status_pub.publish(output)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = AdapterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
