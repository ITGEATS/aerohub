#!/usr/bin/env python3
"""Verify the real three-vehicle D2SLAM ground data path without commanding motion."""
from __future__ import annotations

import json
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String


class Probe(Node):
    def __init__(self, vehicles: list[str]):
        super().__init__("d2_slam_ground_probe")
        self.counts: dict[str, int] = {"clock": 0}
        self.status: dict[str, str] = {}
        self.samples: dict[str, dict] = {}
        self.handles = [
            self.create_subscription(
                Clock, "/clock", lambda _: self.bump("clock"), qos_profile_sensor_data)
        ]
        for vehicle in vehicles:
            topics = {
                "rgb": (Image, f"/{vehicle}/sensor_measurements/FrontCamera/rgb/image"),
                "depth": (
                    Image,
                    f"/{vehicle}/sensor_measurements/FrontCamera/depth_perspective/image",
                ),
                "imu": (Imu, f"/{vehicle}/sensor_measurements/imu/data"),
                "local_odom": (Odometry, f"/{vehicle}/sensor_measurements/odom"),
                "d2_odom": (Odometry, f"/{vehicle}/d2_slam/odometry"),
                "landmarks": (PointCloud2, f"/{vehicle}/d2_slam/landmarks"),
                "canonical_pose": (PoseStamped, f"/{vehicle}/self_localization/pose"),
            }
            for kind, (message_type, topic) in topics.items():
                key = f"{vehicle}.{kind}"
                self.counts[key] = 0
                self.handles.append(self.create_subscription(
                    message_type, topic,
                    lambda message, current=key: self.message(current, message),
                    qos_profile_sensor_data))
            status_key = f"{vehicle}.status"
            self.counts[status_key] = 0
            self.handles.append(self.create_subscription(
                String, f"/{vehicle}/d2_slam/status",
                lambda message, current=status_key, name=vehicle: self.status_message(
                    current, name, message), 10))

    def bump(self, key: str) -> None:
        self.counts[key] += 1

    def message(self, key: str, message) -> None:
        self.bump(key)
        header = getattr(message, "header", None)
        stamp = None
        frame = None
        if header is not None:
            stamp = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
            frame = header.frame_id
        sample = {"stamp_ns": stamp, "frame_id": frame}
        if isinstance(message, Image):
            sample.update({"width": message.width, "height": message.height,
                           "encoding": message.encoding, "bytes": len(message.data)})
        elif isinstance(message, PointCloud2):
            sample.update({"width": message.width, "point_step": message.point_step,
                           "bytes": len(message.data)})
        self.samples[key] = sample

    def status_message(self, key: str, vehicle: str, message: String) -> None:
        self.bump(key)
        self.status[vehicle] = message.data


def main() -> int:
    vehicles = os.environ.get("D2_VEHICLES", "drone0,drone1,drone2").split(",")
    duration = float(os.environ.get("D2_PROBE_SECONDS", "45"))
    output = Path(os.environ.get("D2_PROBE_OUTPUT", "/results/ground-probe.json"))
    rclpy.init()
    node = Probe(vehicles)
    started = time.monotonic()
    while time.monotonic() - started < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    required = ["clock"]
    for vehicle in vehicles:
        required.extend(f"{vehicle}.{kind}" for kind in (
            "rgb", "depth", "imu", "local_odom", "d2_odom", "landmarks",
            "canonical_pose", "status"))
    assertions = {
        "required_streams_present": all(node.counts[key] > 0 for key in required),
        "all_statuses_report_real_input": all(
            all(f"{kind}=" in node.status.get(vehicle, "") for kind in ("rgb", "depth", "imu"))
            and all(
                int(part.split("=", 1)[1]) > 0
                for part in node.status.get(vehicle, "").split(",")
                if part.split("=", 1)[0] in ("rgb", "depth", "imu")
            )
            for vehicle in vehicles),
        "rgb_contract": all(
            node.samples.get(f"{vehicle}.rgb", {}).get("encoding") == "rgb8"
            for vehicle in vehicles),
        "depth_contract": all(
            node.samples.get(f"{vehicle}.depth", {}).get("encoding") == "32FC1"
            for vehicle in vehicles),
        "timestamps_positive": all(
            (node.samples.get(key, {}).get("stamp_ns") or 0) > 0
            for key in required if key != "clock" and not key.endswith(".status")),
    }
    result = {
        "schema_version": 1,
        "duration_s": time.monotonic() - started,
        "vehicles": vehicles,
        "counts": node.counts,
        "status": node.status,
        "samples": node.samples,
        "assertions": assertions,
        "passed": all(assertions.values()),
    }
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
