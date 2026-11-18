#!/usr/bin/env python3
"""Run three-vehicle D2SLAM motion, authority, and fault acceptance."""
from __future__ import annotations

from bisect import bisect_left
import json
import math
import os
from pathlib import Path
import statistics
import struct
import threading
import time
import zlib

import rclpy
from as2_msgs.action import GoToWaypoint, Land, Takeoff
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommandAck, VehicleControlMode, VehicleStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
import yaml

from airsim_rpc import Rpc


OUTPUT = Path(os.environ.get("D2_VALIDATION_OUTPUT", "/results"))
OUTPUT.mkdir(parents=True, exist_ok=True)
SCENARIO = os.environ.get("D2_VALIDATION_SCENARIO", "shadow")
VEHICLES = os.environ.get("D2_VEHICLES", "drone0,drone1,drone2").split(",")
AIRSIM_NAMES = {vehicle: "Drone" + vehicle.removeprefix("drone") for vehicle in VEHICLES}
VALID_SCENARIOS = {"shadow", "authority", "sensor_fault", "algorithm_fault", "communication_fault"}


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("inf")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def stream_rate(stamps: list[int]) -> float:
    ordered = sorted(set(stamps))
    if len(ordered) < 2:
        return 0.0
    duration = (ordered[-1] - ordered[0]) * 1.0e-9
    return (len(ordered) - 1) / duration if duration > 0.0 else 0.0


def maximum_gap(stamps: list[int]) -> float:
    ordered = sorted(set(stamps))
    return max(((b - a) * 1.0e-9 for a, b in zip(ordered, ordered[1:])), default=float("inf"))


def nearest_stamp_deltas(left: set[int], right: set[int]) -> list[float]:
    ordered = sorted(right)
    if not ordered:
        return []
    deltas = []
    for stamp in sorted(left):
        index = bisect_left(ordered, stamp)
        candidates = ordered[max(0, index - 1):min(len(ordered), index + 1)]
        deltas.append(min(abs(stamp - candidate) for candidate in candidates) * 1.0e-9)
    return deltas


def path_extent(points: list[tuple[float, float, float]]) -> float:
    if not points:
        return 0.0
    origin = points[0]
    return max(math.dist(origin, point) for point in points)


def aligned_errors(samples: list[dict]) -> dict:
    """Score a metric trajectory after a fixed-scale planar rigid alignment."""
    candidates = [tuple(sample["candidate"]) for sample in samples]
    truths = [tuple(sample["truth"]) for sample in samples]
    if len(candidates) < 3:
        return {"horizontal_rmse_m": float("inf"), "position_rmse_m": float("inf"),
                "position_p95_m": float("inf"), "alignment_yaw_rad": None,
                "candidate_movement_m": path_extent(candidates),
                "truth_movement_m": path_extent(truths)}
    cc = tuple(statistics.fmean(point[axis] for point in candidates) for axis in range(3))
    tc = tuple(statistics.fmean(point[axis] for point in truths) for axis in range(3))
    cross = sum(
        (candidate[0] - cc[0]) * (truth[1] - tc[1])
        - (candidate[1] - cc[1]) * (truth[0] - tc[0])
        for candidate, truth in zip(candidates, truths))
    dot = sum(
        (candidate[0] - cc[0]) * (truth[0] - tc[0])
        + (candidate[1] - cc[1]) * (truth[1] - tc[1])
        for candidate, truth in zip(candidates, truths))
    yaw = math.atan2(cross, dot)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    horizontal_errors = []
    position_errors = []
    aligned = []
    for candidate, truth in zip(candidates, truths):
        x = cosine * (candidate[0] - cc[0]) - sine * (candidate[1] - cc[1]) + tc[0]
        y = sine * (candidate[0] - cc[0]) + cosine * (candidate[1] - cc[1]) + tc[1]
        z = candidate[2] - cc[2] + tc[2]
        horizontal = math.hypot(x - truth[0], y - truth[1])
        position = math.dist((x, y, z), truth)
        horizontal_errors.append(horizontal)
        position_errors.append(position)
        aligned.append([x, y, z])
    return {
        "horizontal_rmse_m": math.sqrt(statistics.fmean(value * value for value in horizontal_errors)),
        "position_rmse_m": math.sqrt(statistics.fmean(value * value for value in position_errors)),
        "position_p95_m": percentile(position_errors, 0.95),
        "alignment_yaw_rad": yaw,
        "candidate_movement_m": path_extent(candidates),
        "truth_movement_m": path_extent(truths),
        "aligned_candidate": aligned,
    }


class Validation:
    def __init__(self):
        if SCENARIO not in VALID_SCENARIOS:
            raise ValueError(f"invalid scenario {SCENARIO}")
        if len(VEHICLES) != 3:
            raise ValueError("D2SLAM acceptance requires exactly three vehicles")
        config_path = Path(os.environ.get(
            "D2_VALIDATION_CONFIG", "/opt/d2_slam/runtime/validation.yaml"))
        self.config = yaml.safe_load(config_path.read_text())
        self.gates = self.config["gates"]
        self.node = rclpy.create_node(
            "d2_slam_fleet_validation",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.rpc = Rpc("127.0.0.1", int(os.environ.get("AIRSIM_RPC_PORT", "42471")), timeout=5)
        self.poses: dict[str, PoseStamped] = {}
        self.candidates: dict[str, Odometry] = {}
        self.candidate_stamps = {vehicle: [] for vehicle in VEHICLES}
        self.candidate_arrivals = {vehicle: 0.0 for vehicle in VEHICLES}
        self.canonical_stamps = {vehicle: [] for vehicle in VEHICLES}
        self.canonical_arrivals = {vehicle: [] for vehicle in VEHICLES}
        self.landmarks = {vehicle: {"messages": 0, "nonempty": 0, "max_points": 0}
                          for vehicle in VEHICLES}
        self.collaboration = {vehicle: {"events": 0, "remote_ids": set(), "last_arrival": 0.0}
                              for vehicle in VEHICLES}
        self.bridge_status = {vehicle: {} for vehicle in VEHICLES}
        self.image_arrivals = {vehicle: 0.0 for vehicle in VEHICLES}
        self.sensor_hashes = {
            vehicle: {"rgb": set(), "depth": set()} for vehicle in VEHICLES
        }
        self.sensor_stamps = {
            vehicle: {"rgb": set(), "depth": set()} for vehicle in VEHICLES
        }
        self.sensor_first_hash = {
            vehicle: {"rgb": None, "depth": None} for vehicle in VEHICLES
        }
        self.sensor_counts = {
            vehicle: {"rgb": 0, "depth": 0} for vehicle in VEHICLES
        }
        self.sensor_sample_errors = []
        self.depth_sample_stats = {vehicle: [] for vehicle in VEHICLES}
        self.imu_stamps = {vehicle: [] for vehicle in VEHICLES}
        self.imu_acceleration_norms = {vehicle: [] for vehicle in VEHICLES}
        self.imu_angular_velocity_norms = {vehicle: [] for vehicle in VEHICLES}
        self.sensor_sample_dir = OUTPUT / "sensor-samples"
        self.sensor_sample_dir.mkdir(exist_ok=True)
        self.modes: dict[str, VehicleControlMode] = {}
        self.statuses: dict[str, VehicleStatus] = {}
        self.acks = {vehicle: {} for vehicle in VEHICLES}
        self.samples = {vehicle: [] for vehicle in VEHICLES}
        self.sampled_candidate_stamp = {vehicle: -1 for vehicle in VEHICLES}
        self.handles = []
        self.clients = []
        self.recording = False
        self.airborne_started = None
        self.fault_evidence = None
        for vehicle in VEHICLES:
            self.handles.extend([
                self.node.create_subscription(
                    PoseStamped, f"/{vehicle}/self_localization/pose",
                    lambda message, name=vehicle: self.pose(name, message), qos_profile_sensor_data),
                self.node.create_subscription(
                    Odometry, f"/{vehicle}/d2_slam/odometry",
                    lambda message, name=vehicle: self.candidate(name, message), qos_profile_sensor_data),
                self.node.create_subscription(
                    PointCloud2, f"/{vehicle}/d2_slam/landmarks",
                    lambda message, name=vehicle: self.cloud(name, message), qos_profile_sensor_data),
                self.node.create_subscription(
                    String, f"/{vehicle}/d2_slam/collaboration",
                    lambda message, name=vehicle: self.remote_frame(name, message), 100),
                self.node.create_subscription(
                    String, f"/{vehicle}/d2_slam/status",
                    lambda message, name=vehicle: self.status(name, message), 10),
                self.node.create_subscription(
                    Image, f"/{vehicle}/sensor_measurements/FrontCamera/rgb/image",
                    lambda message, name=vehicle: self.sensor_image(
                        name, "rgb", message), qos_profile_sensor_data),
                self.node.create_subscription(
                    Image, f"/{vehicle}/sensor_measurements/FrontCamera/depth_perspective/image",
                    lambda message, name=vehicle: self.sensor_image(
                        name, "depth", message), qos_profile_sensor_data),
                self.node.create_subscription(
                    Imu, f"/{vehicle}/sensor_measurements/imu/data",
                    lambda message, name=vehicle: self.sensor_imu(name, message),
                    qos_profile_sensor_data),
                self.node.create_subscription(
                    VehicleControlMode, f"/{vehicle}/fmu/out/vehicle_control_mode",
                    lambda message, name=vehicle: self.modes.__setitem__(name, message),
                    qos_profile_sensor_data),
                self.node.create_subscription(
                    VehicleStatus, f"/{vehicle}/fmu/out/vehicle_status_v4",
                    lambda message, name=vehicle: self.statuses.__setitem__(name, message),
                    qos_profile_sensor_data),
                self.node.create_subscription(
                    VehicleCommandAck, f"/{vehicle}/fmu/out/vehicle_command_ack_v1",
                    lambda message, name=vehicle: self.acks[name].__setitem__(
                        int(message.command), int(message.result)), qos_profile_sensor_data),
            ])
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin, name="d2-validation-executor", daemon=True)
        self.executor_thread.start()

    def pose(self, vehicle: str, message: PoseStamped) -> None:
        if stamp_ns(message) <= 0:
            return
        self.poses[vehicle] = message
        if self.recording:
            self.canonical_stamps[vehicle].append(stamp_ns(message))
            self.canonical_arrivals[vehicle].append(time.monotonic_ns())

    def candidate(self, vehicle: str, message: Odometry) -> None:
        if stamp_ns(message) <= 0:
            return
        self.candidates[vehicle] = message
        self.candidate_arrivals[vehicle] = time.monotonic()
        if self.recording:
            self.candidate_stamps[vehicle].append(stamp_ns(message))

    def cloud(self, vehicle: str, message: PointCloud2) -> None:
        state = self.landmarks[vehicle]
        state["messages"] += 1
        points = int(message.width) * int(message.height)
        if points > 0 and len(message.data) == points * int(message.point_step):
            state["nonempty"] += 1
            state["max_points"] = max(state["max_points"], points)

    def remote_frame(self, vehicle: str, message: String) -> None:
        event = json.loads(message.data)
        state = self.collaboration[vehicle]
        state["events"] += 1
        state["remote_ids"].add(int(event["drone_id"]))
        state["last_arrival"] = time.monotonic()

    def status(self, vehicle: str, message: String) -> None:
        fields = {}
        for field in message.data.split(","):
            key, separator, value = field.partition("=")
            if separator:
                fields[key] = value
        self.bridge_status[vehicle] = fields

    def sensor_image(self, vehicle: str, kind: str, message: Image) -> None:
        stamp = stamp_ns(message)
        if stamp <= 0:
            return
        self.sensor_counts[vehicle][kind] += 1
        self.sensor_stamps[vehicle][kind].add(stamp)
        checksum = zlib.crc32(bytes(message.data))
        self.sensor_hashes[vehicle][kind].add(checksum)
        if self.sensor_first_hash[vehicle][kind] is None:
            self.sensor_first_hash[vehicle][kind] = checksum
        if kind == "rgb":
            self.image_arrivals[vehicle] = time.monotonic()
            count = self.sensor_counts[vehicle][kind]
            if count == 1 or count % 100 == 0:
                try:
                    self.save_rgb_sample(vehicle, count, message)
                except (OSError, ValueError) as exc:
                    self.sensor_sample_errors.append(f"{vehicle} rgb {count}: {exc}")
        elif self.sensor_counts[vehicle][kind] == 1 or self.sensor_counts[vehicle][kind] % 100 == 0:
            try:
                self.record_depth_sample(vehicle, self.sensor_counts[vehicle][kind], message)
            except ValueError as exc:
                self.sensor_sample_errors.append(
                    f"{vehicle} depth {self.sensor_counts[vehicle][kind]}: {exc}")

    def save_rgb_sample(self, vehicle: str, count: int, message: Image) -> None:
        if message.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"unsupported sample encoding {message.encoding}")
        width, height, step = int(message.width), int(message.height), int(message.step)
        if width <= 0 or height <= 0 or step < width * 3:
            raise ValueError("invalid RGB sample layout")
        source = bytes(message.data)
        if len(source) != step * height:
            raise ValueError("RGB sample payload does not match its layout")
        packed = bytearray()
        for row in range(height):
            pixels = source[row * step:row * step + width * 3]
            if message.encoding == "bgr8":
                pixels = bytes(
                    value for index in range(0, len(pixels), 3)
                    for value in (pixels[index + 2], pixels[index + 1], pixels[index]))
            packed.extend(pixels)
        header = f"P6\n{width} {height}\n255\n".encode("ascii")
        (self.sensor_sample_dir / f"{vehicle}-rgb-{count:04d}.ppm").write_bytes(
            header + packed)

    def sensor_imu(self, vehicle: str, message: Imu) -> None:
        stamp = stamp_ns(message)
        if stamp <= 0:
            return
        self.imu_stamps[vehicle].append(stamp)
        acceleration = message.linear_acceleration
        angular = message.angular_velocity
        self.imu_acceleration_norms[vehicle].append(
            math.sqrt(acceleration.x ** 2 + acceleration.y ** 2 + acceleration.z ** 2))
        self.imu_angular_velocity_norms[vehicle].append(
            math.sqrt(angular.x ** 2 + angular.y ** 2 + angular.z ** 2))

    def record_depth_sample(self, vehicle: str, count: int, message: Image) -> None:
        width, height, step = int(message.width), int(message.height), int(message.step)
        if message.encoding != "32FC1" or message.is_bigendian:
            raise ValueError("expected little-endian 32FC1 depth")
        if width <= 0 or height <= 0 or step < width * 4:
            raise ValueError("invalid depth sample layout")
        source = bytes(message.data)
        if len(source) != step * height:
            raise ValueError("depth sample payload does not match its layout")
        values = []
        for row in range(height):
            packed = source[row * step:row * step + width * 4]
            values.extend(value[0] for value in struct.iter_unpack("<f", packed))
        valid = [value for value in values if math.isfinite(value) and value > 0.0]
        if not valid:
            raise ValueError("depth sample has no finite positive values")
        ordered = sorted(valid)
        self.depth_sample_stats[vehicle].append({
            "count": count,
            "valid_fraction": len(valid) / len(values),
            "minimum_m": ordered[0],
            "median_m": statistics.median(ordered),
            "p95_m": percentile(ordered, 0.95),
            "maximum_m": ordered[-1],
            "center_m": values[(height // 2) * width + width // 2],
        })

    def spin(self, seconds: float, sample: bool = True) -> None:
        deadline = time.monotonic() + seconds
        next_sample = 0.0
        while time.monotonic() < deadline:
            time.sleep(0.01)
            if sample and self.recording and time.monotonic() >= next_sample:
                self.sample_truth()
                next_sample = time.monotonic() + 0.2

    def wait(self, predicate, seconds: float, description: str) -> None:
        deadline = time.monotonic() + seconds
        next_sample = 0.0
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"timed out waiting for {description}")
            time.sleep(0.01)
            if self.recording and time.monotonic() >= next_sample:
                self.sample_truth()
                next_sample = time.monotonic() + 0.2

    def sample_truth(self) -> None:
        for vehicle in VEHICLES:
            candidate = self.candidates.get(vehicle)
            if candidate is None:
                continue
            current_stamp = stamp_ns(candidate)
            if current_stamp == self.sampled_candidate_stamp[vehicle]:
                continue
            pose = self.rpc.call("simGetVehiclePose", AIRSIM_NAMES[vehicle])
            position = pose["position"]
            truth = [float(position["y_val"]), float(position["x_val"]), -float(position["z_val"])]
            value = candidate.pose.pose.position
            self.samples[vehicle].append({
                "candidate_stamp_ns": current_stamp,
                "candidate": [value.x, value.y, value.z],
                "truth": truth,
                "wall_time": time.time(),
            })
            self.sampled_candidate_stamp[vehicle] = current_stamp

    def service(self, vehicle: str, name: str, value: bool, command: int, feedback) -> None:
        client = self.node.create_client(SetBool, f"/{vehicle}/{name}")
        self.clients.append(client)
        self.wait(client.service_is_ready, 20, f"{vehicle} {name} service")
        self.acks[vehicle].pop(command, None)
        future = client.call_async(SetBool.Request(data=value))
        self.wait(future.done, 15, f"{vehicle} {name} response")
        if not future.result().success:
            raise RuntimeError(f"{vehicle} {name} rejected by AS2")
        self.wait(lambda: command in self.acks[vehicle], 15, f"{vehicle} PX4 ack {command}")
        if self.acks[vehicle][command] != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            raise RuntimeError(f"{vehicle} PX4 rejected command {command}")
        self.wait(feedback, 15, f"{vehicle} firmware state for {name}")

    def actions(self, kind, name: str, goals: dict, success_field: str, seconds: float) -> None:
        clients = {vehicle: ActionClient(self.node, kind, f"/{vehicle}/{name}")
                   for vehicle in VEHICLES}
        self.clients.extend(clients.values())
        for vehicle, client in clients.items():
            self.wait(client.server_is_ready, 30, f"{vehicle} {name} action")
        requests = {vehicle: clients[vehicle].send_goal_async(goals[vehicle]) for vehicle in VEHICLES}
        for vehicle, future in requests.items():
            self.wait(future.done, 20, f"{vehicle} {name} acceptance")
            if not future.result() or not future.result().accepted:
                raise RuntimeError(f"{vehicle} {name} rejected")
        results = {vehicle: future.result().get_result_async() for vehicle, future in requests.items()}
        for vehicle, future in results.items():
            self.wait(future.done, seconds, f"{vehicle} {name} result")
            if not getattr(future.result().result, success_field):
                raise RuntimeError(f"{vehicle} {name} failed")

    def screenshot(self, name: str) -> None:
        request = {"camera_name": "swarm_overhead", "image_type": 0,
                   "pixels_as_float": False, "compress": True}
        response = self.rpc.call("simGetImages", [request], "", True)[0]
        data = bytes(response["image_data_uint8"])
        if len(data) < 1000:
            raise RuntimeError("AirSim overhead screenshot is empty")
        (OUTPUT / f"{name}.png").write_bytes(data)

    def initialize(self) -> None:
        self.wait(lambda: len(self.poses) == len(VEHICLES), 60, "all canonical poses")
        self.wait(lambda: len(self.candidates) == len(VEHICLES), 120, "all D2SLAM candidates")
        self.wait(lambda: all(state["nonempty"] >= 3 for state in self.landmarks.values()),
                  60, "nonempty landmark clouds")
        self.wait(lambda: all(
            state["remote_ids"] == ({0, 1, 2} - {index})
            for index, state in enumerate(self.collaboration.values())),
            90, "each estimator to receive both peers")
        self.wait(lambda: len(self.modes) == len(VEHICLES) and len(self.statuses) == len(VEHICLES),
                  30, "PX4 state")
        self.wait(lambda: all(status.pre_flight_checks_pass for status in self.statuses.values()),
                  60, "PX4 preflight checks")
        self.screenshot("ready")

    def takeoff(self) -> None:
        for vehicle in VEHICLES:
            if not self.modes[vehicle].flag_control_offboard_enabled:
                self.service(
                    vehicle, "set_offboard_mode", True, 176,
                    lambda name=vehicle: self.modes.get(name) is not None
                    and self.modes[name].flag_control_offboard_enabled)
            if not self.modes[vehicle].flag_armed:
                self.service(
                    vehicle, "set_arming_state", True, 400,
                    lambda name=vehicle: self.modes.get(name) is not None
                    and self.modes[name].flag_armed)
        goals = {vehicle: Takeoff.Goal(takeoff_height=1.5, takeoff_speed=0.4)
                 for vehicle in VEHICLES}
        self.actions(Takeoff, "TakeoffBehavior", goals, "takeoff_success", 90)
        self.airborne_started = time.monotonic()
        self.recording = True
        self.screenshot("airborne")

    def goto_relative(self, dx: float, dy: float) -> None:
        goals = {}
        for vehicle in VEHICLES:
            point = self.poses[vehicle].pose.position
            goal = GoToWaypoint.Goal()
            goal.target_pose.header.frame_id = "earth"
            goal.target_pose.point.x = point.x + dx
            goal.target_pose.point.y = point.y + dy
            goal.target_pose.point.z = point.z
            goal.max_speed = 0.5
            goals[vehicle] = goal
        self.actions(GoToWaypoint, "GoToBehavior", goals, "go_to_success", 120)
        self.spin(2)

    def land(self) -> None:
        self.recording = False
        goals = {vehicle: Land.Goal(land_speed=0.4) for vehicle in VEHICLES}
        self.actions(Land, "LandBehavior", goals, "land_success", 120)
        self.wait(lambda: all(not self.modes[vehicle].flag_armed for vehicle in VEHICLES),
                  60, "fleet disarm")
        self.screenshot("landed")

    def execute_fault(self) -> None:
        self.goto_relative(2.0, 0.0)
        marker = {
            "scenario": SCENARIO,
            "target_vehicle": "drone1",
            "candidate_counts": {key: len(value) for key, value in self.candidate_stamps.items()},
            "collaboration_counts": {
                key: value["events"] for key, value in self.collaboration.items()},
            "ready_wall_time": time.time(),
        }
        (OUTPUT / "fault-ready.json").write_text(json.dumps(marker, indent=2) + "\n")
        self.wait(lambda: (OUTPUT / "fault-injected.json").exists(), 30, "fault injection marker")
        injection = json.loads((OUTPUT / "fault-injected.json").read_text())
        started = time.monotonic()
        if SCENARIO == "sensor_fault":
            self.wait(lambda: time.monotonic() - self.image_arrivals["drone1"] >= 2.0,
                      30, "drone1 image loss")
            self.wait(lambda: time.monotonic() - self.candidate_arrivals["drone1"] >= 2.0,
                      30, "drone1 candidate loss")
            assertion = "sensor_and_candidate_stale"
        elif SCENARIO == "algorithm_fault":
            self.wait(lambda: time.monotonic() - self.candidate_arrivals["drone1"] >= 2.0,
                      30, "drone1 candidate loss")
            assertion = "candidate_stale"
        else:
            baseline_candidate = len(self.candidate_stamps["drone1"])
            self.wait(lambda: time.monotonic() - self.collaboration["drone1"]["last_arrival"] >= 3.0,
                      30, "drone1 remote-frame loss")
            self.spin(2)
            if len(self.candidate_stamps["drone1"]) - baseline_candidate < 5:
                raise RuntimeError("drone1 local estimator did not remain live after peer loss")
            assertion = "remote_frames_stale_local_candidate_live"
        self.fault_evidence = {
            "injection": injection,
            "detection_seconds": time.monotonic() - started,
            "assertion": assertion,
            "candidate_counts": {key: len(value) for key, value in self.candidate_stamps.items()},
            "collaboration_counts": {
                key: value["events"] for key, value in self.collaboration.items()},
        }

    def metrics(self) -> dict:
        reports = {}
        for index, vehicle in enumerate(VEHICLES):
            report = aligned_errors(self.samples[vehicle])
            stamps = self.candidate_stamps[vehicle]
            report.update({
                "candidate_samples": len(set(stamps)),
                "candidate_rate_hz": stream_rate(stamps),
                "candidate_maximum_gap_s": maximum_gap(stamps),
                "landmarks": self.landmarks[vehicle],
                "collaboration_events": self.collaboration[vehicle]["events"],
                "collaboration_remote_ids": sorted(self.collaboration[vehicle]["remote_ids"]),
                "canonical_maximum_gap_s": maximum_gap(self.canonical_arrivals[vehicle]),
                "canonical_header_maximum_gap_s": maximum_gap(self.canonical_stamps[vehicle]),
            })
            expected_remotes = sorted({0, 1, 2} - {index})
            checks = {
                "candidate_samples": report["candidate_samples"] >= self.gates["candidate_samples_min"],
                "candidate_rate": report["candidate_rate_hz"] >= self.gates["candidate_output_rate_min_hz"],
                "candidate_gap": report["candidate_maximum_gap_s"] <= self.gates["candidate_maximum_gap_s"],
                "truth_movement": report["truth_movement_m"] >= self.config["duration"]["minimum_truth_movement_m"],
                "candidate_movement": report["candidate_movement_m"] >= self.config["duration"]["minimum_candidate_movement_m"],
                "horizontal_ate": report["horizontal_rmse_m"] <= self.gates["horizontal_ate_rmse_max_m"],
                "position_ate": report["position_rmse_m"] <= self.gates["position_ate_rmse_max_m"],
                "position_p95": report["position_p95_m"] <= self.gates["position_error_p95_max_m"],
                "landmarks": report["landmarks"]["nonempty"] >= self.gates["nonempty_landmark_samples_min"],
                "collaboration_events": report["collaboration_events"] >= self.gates["collaboration_events_min"],
                "both_remote_vehicles": report["collaboration_remote_ids"] == expected_remotes,
            }
            if SCENARIO == "authority":
                checks["canonical_gap"] = (
                    report["canonical_maximum_gap_s"]
                    <= self.gates["authority_canonical_maximum_gap_s"])
            report["checks"] = checks
            report["passed"] = all(checks.values())
            report.pop("aligned_candidate", None)
            reports[vehicle] = report
        return reports

    def sensor_diagnostics(self) -> dict:
        report = {
            vehicle: {
                kind + "_unique_frames": len(self.sensor_hashes[vehicle][kind])
                for kind in ("rgb", "depth")
            }
            for vehicle in VEHICLES
        }
        for vehicle in VEHICLES:
            acceleration = self.imu_acceleration_norms[vehicle]
            angular = self.imu_angular_velocity_norms[vehicle]
            report[vehicle]["imu_samples"] = len(set(self.imu_stamps[vehicle]))
            report[vehicle]["imu_rate_hz"] = stream_rate(self.imu_stamps[vehicle])
            report[vehicle]["imu_maximum_gap_s"] = maximum_gap(self.imu_stamps[vehicle])
            report[vehicle]["acceleration_norm_min"] = min(acceleration, default=None)
            report[vehicle]["acceleration_norm_max"] = max(acceleration, default=None)
            report[vehicle]["angular_velocity_norm_max"] = max(angular, default=None)
            rgb_stamps = self.sensor_stamps[vehicle]["rgb"]
            depth_stamps = self.sensor_stamps[vehicle]["depth"]
            report[vehicle]["rgb_depth_exact_stamp_matches"] = len(
                rgb_stamps & depth_stamps)
            report[vehicle]["rgb_depth_exact_match_fraction"] = (
                len(rgb_stamps & depth_stamps) / max(1, len(rgb_stamps | depth_stamps)))
            deltas = nearest_stamp_deltas(rgb_stamps, depth_stamps)
            report[vehicle]["rgb_depth_nearest_delta_median_s"] = (
                statistics.median(deltas) if deltas else None)
            report[vehicle]["rgb_depth_nearest_delta_p95_s"] = (
                percentile(deltas, 0.95) if deltas else None)
            report[vehicle]["rgb_depth_nearest_delta_max_s"] = max(deltas, default=None)
            report[vehicle]["bridge_status"] = self.bridge_status[vehicle]
            report[vehicle]["depth_samples"] = self.depth_sample_stats[vehicle]
        for kind in ("rgb", "depth"):
            first = [self.sensor_first_hash[vehicle][kind] for vehicle in VEHICLES]
            report[kind + "_first_frames_distinct"] = (
                all(value is not None for value in first) and len(set(first)) == len(first)
            )
        report["sample_errors"] = self.sensor_sample_errors
        return report

    def emergency_land(self) -> None:
        if not any(mode.flag_armed for mode in self.modes.values()):
            return
        try:
            self.land()
        except Exception as exc:
            (OUTPUT / "emergency-landing-failure.txt").write_text(str(exc) + "\n")

    def run(self) -> dict:
        self.initialize()
        self.takeoff()
        if SCENARIO in {"sensor_fault", "algorithm_fault", "communication_fault"}:
            self.execute_fault()
        else:
            for delta in ((3.0, 0.0), (0.0, 3.0), (-3.0, 0.0), (0.0, -3.0)):
                self.goto_relative(*delta)
            minimum = float(self.config["duration"]["minimum_airborne_seconds"])
            if self.airborne_started is not None:
                self.spin(max(0.0, minimum - (time.monotonic() - self.airborne_started)))
        airborne = 0.0 if self.airborne_started is None else time.monotonic() - self.airborne_started
        self.land()
        reports = self.metrics() if SCENARIO in {"shadow", "authority"} else None
        passed = (
            all(report["passed"] for report in reports.values())
            if reports is not None else self.fault_evidence is not None)
        result = {
            "schema_version": 1,
            "scenario": SCENARIO,
            "airborne_seconds": airborne,
            "vehicle_reports": reports,
            "sensor_diagnostics": self.sensor_diagnostics(),
            "fault": self.fault_evidence,
            "landed_and_disarmed": all(not self.modes[vehicle].flag_armed for vehicle in VEHICLES),
            "passed": passed,
        }
        result["passed"] = result["passed"] and result["landed_and_disarmed"]
        (OUTPUT / "trajectory-samples.json").write_text(json.dumps(self.samples, indent=2) + "\n")
        (OUTPUT / "validation-result.json").write_text(json.dumps(result, indent=2) + "\n")
        return result

    def close(self) -> None:
        self.executor.shutdown(timeout_sec=5.0)
        self.executor_thread.join(timeout=5.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()


def main() -> int:
    rclpy.init()
    validation = Validation()
    try:
        result = validation.run()
        print(json.dumps(result, indent=2))
        return 0 if result["passed"] else 1
    except BaseException as exc:
        validation.emergency_land()
        failure = {"scenario": SCENARIO, "error": f"{type(exc).__name__}: {exc}"}
        (OUTPUT / "validation-failure.json").write_text(json.dumps(failure, indent=2) + "\n")
        raise
    finally:
        validation.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
