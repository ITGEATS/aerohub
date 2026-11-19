"""Workstation acceptance client for FALCON exploration."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from as2_msgs.action import Land, Takeoff
from as2_msgs.msg import PlatformInfo, TrajectorySetpoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from falcon_airsim_rpc import Rpc


class Mission(Node):
    def __init__(self):
        super().__init__(
            "falcon_acceptance",
            parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.output = Path("/results")
        self.case = os.environ.get("FALCON_CASE", "normal")
        self.odom = None
        self.platform = None
        self.adapter = {}
        self.runtime = {}
        self.last_clock = None
        self.clock_regressed = False
        self.references = 0
        self.frontier_points = 0
        self.map_counts = {"occupied": 0, "free": 0, "unknown": 0}
        self.map_first = {}
        self.route = []
        self.last_collision_check = 0.0
        self.result = {
            "success": False,
            "case": self.case,
            "mission_role": "workstation-ROS-client",
            "compute_role": "independent-compute-runtime",
        }
        self.rpc = Rpc("127.0.0.1", 43482, timeout=5)
        self.samples = (self.output / "flight.jsonl").open("w")
        self.create_subscription(
            Odometry, "/drone0/sensor_measurements/odom", self.on_odometry,
            qos_profile_sensor_data)
        self.create_subscription(
            PlatformInfo, "/drone0/platform/info",
            lambda message: setattr(self, "platform", message), qos_profile_sensor_data)
        self.create_subscription(
            String, "/drone0/falcon/adapter/state",
            lambda message: setattr(self, "adapter", json.loads(message.data)), 10)
        self.create_subscription(
            String, "/drone0/falcon/runtime_state",
            lambda message: setattr(self, "runtime", json.loads(message.data)), 10)
        self.create_subscription(
            PointCloud2, "/planning_vis/frontier_pcl", self.on_frontier, 10)
        for name in self.map_counts:
            self.create_subscription(
                PointCloud2, f"/voxel_mapping/occupancy_grid_{name}",
                lambda message, key=name: self.on_map(key, message), 10)
        self.create_subscription(
            TrajectorySetpoints, "/drone0/motion_reference/trajectory",
            lambda _message: setattr(self, "references", self.references + 1),
            qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.trigger = self.create_publisher(PoseStamped, "/move_base_simple/goal", 1)
        self.create_timer(0.1, self.record)

    def on_odometry(self, message):
        self.odom = message

    def on_frontier(self, message):
        self.frontier_points = max(self.frontier_points, int(message.width) * int(message.height))

    def on_map(self, name, message):
        count = int(message.width) * int(message.height)
        self.map_first.setdefault(name, count)
        self.map_counts[name] = max(self.map_counts[name], count)

    def on_clock(self, message):
        stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if self.last_clock is not None and stamp < self.last_clock:
            self.clock_regressed = True
        self.last_clock = stamp

    def position(self):
        point = self.odom.pose.pose.position
        return [point.x, point.y, point.z]

    def speed(self):
        vector = self.odom.twist.twist.linear
        return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)

    def record(self):
        if self.odom is None:
            return
        sample = {
            "sim_time": self.get_clock().now().nanoseconds / 1e9,
            "position": self.position(), "speed": self.speed(),
            "adapter": self.adapter, "runtime": self.runtime,
            "map_counts": self.map_counts, "frontier_points": self.frontier_points,
            "references": self.references,
        }
        self.route.append(sample)
        self.samples.write(json.dumps(sample, sort_keys=True) + "\n")
        self.samples.flush()

    def wait(self, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError("condition timeout")
            rclpy.spin_once(self, timeout_sec=0.02)

    def service(self, service_type, name, request, timeout=20):
        client = self.create_client(service_type, name)
        self.wait(client.service_is_ready, timeout)
        future = client.call_async(request)
        self.wait(future.done, timeout)
        response = future.result()
        if not response.success:
            raise RuntimeError(f"{name}: {response}")
        return response

    def action(self, action_type, name, goal, success_field, timeout=90):
        client = ActionClient(self, action_type, name)
        self.wait(client.server_is_ready, 60)
        sent = client.send_goal_async(goal)
        self.wait(sent.done)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError(name + " rejected")
        result = handle.get_result_async()
        self.wait(result.done, timeout)
        wrapped = result.result()
        if wrapped.status != 4 or not getattr(wrapped.result, success_field):
            raise RuntimeError(name + " failed")

    def capture(self, name):
        response = self.rpc.call("simGetImages", [dict(
            camera_name="swarm_overhead", image_type=0,
            pixels_as_float=False, compress=True)], "", True)[0]
        data = bytes(response["image_data_uint8"])
        if not data:
            raise RuntimeError("empty overhead screenshot")
        (self.output / f"{name}.png").write_bytes(data)

    def collision(self):
        return self.rpc.call("simGetCollisionInfo", "PX4")

    @staticmethod
    def in_bounds(position):
        return (-8.0 <= position[0] <= 8.0 and -8.0 <= position[1] <= 8.0
                and 0.0 <= position[2] <= 5.0)

    @staticmethod
    def positions_since(route, start_index):
        return [sample["position"] for sample in route[start_index:]]

    @staticmethod
    def is_planner_exit_failure(message):
        return message in {
            "FALCON replacement trajectory timeout",
            "FALCON trajectory coverage expired without replacement",
            "FollowTimedPath execution failed",
        }

    def route_gate(self, initial, collision_stamp, force=False):
        if self.clock_regressed:
            raise RuntimeError("simulation clock regressed")
        position = self.position()
        if not self.in_bounds(position):
            raise RuntimeError("vehicle left declared exploration bounds")
        wall = time.monotonic()
        if force or wall - self.last_collision_check >= 0.2:
            collision = self.collision()
            self.last_collision_check = wall
            if collision["time_stamp"] != collision_stamp:
                (self.output / "collision.json").write_text(
                    json.dumps(collision, indent=2, default=str) + "\n")
                raise RuntimeError("collision:" + str(collision.get("object_name", "unknown")))
        return math.dist(initial, position)

    def run(self):
        try:
            self.wait(lambda: self.odom is not None and self.platform is not None
                      and self.runtime.get("ready") and self.adapter.get("odometry_ready")
                      and self.adapter.get("depth_ready"), 120)
            settle = time.monotonic()
            self.wait(lambda: time.monotonic() - settle >= 3)
            authorities = {topic: len(self.get_publishers_info_by_topic(topic)) for topic in (
                "/clock", "/drone0/sensor_measurements/odom")}
            if self.clock_regressed or authorities != {
                    "/clock": 1, "/drone0/sensor_measurements/odom": 1}:
                raise RuntimeError("clock/state authority gate failed: " + str(authorities))
            self.result["authority_counts"] = authorities
            self.service(SetBool, "/drone0/set_arming_state", SetBool.Request(data=True))
            self.wait(lambda: self.platform.armed)
            self.service(SetBool, "/drone0/set_offboard_mode", SetBool.Request(data=True))
            self.wait(lambda: self.platform.offboard)
            self.action(Takeoff, "/drone0/TakeoffBehavior",
                        Takeoff.Goal(takeoff_height=2.0, takeoff_speed=0.5),
                        "takeoff_success")
            settled = time.monotonic()
            self.wait(lambda: time.monotonic() - settled > 4 and self.speed() < 0.1, 30)
            self.wait(lambda: self.map_counts["free"] > 0 and self.frontier_points > 0, 30)
            collision_baseline = self.collision()
            collision_stamp = collision_baseline["time_stamp"]
            (self.output / "collision-baseline.json").write_text(
                json.dumps(collision_baseline, indent=2, default=str) + "\n")
            self.capture("before-exploration")
            initial = self.position()
            exploration_start_index = len(self.route)
            self.service(SetBool, "/drone0/falcon/enable", SetBool.Request(data=True))
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "world"
            goal.pose.orientation.w = 1.0
            self.trigger.publish(goal)
            self.wait(lambda: self.adapter.get("last_trajectory_id", -1) >= 0
                      or bool(self.adapter.get("failed")), 60)
            if self.adapter.get("failed"):
                raise RuntimeError(self.adapter["failed"])
            self.wait(lambda: self.references > 0, 20)
            self.wait(lambda: self.route_gate(initial, collision_stamp) > 1.2, 45)

            if self.case == "depth-loss":
                self.service(SetBool, "/drone0/falcon/test/depth_enabled",
                             SetBool.Request(data=False))
                self.wait(lambda: bool(self.adapter.get("failed")), 10)
                if "depth stream expired" not in self.adapter["failed"]:
                    raise RuntimeError("unexpected depth-loss result: " + self.adapter["failed"])
            elif self.case == "planner-exit":
                self.service(Trigger, "/drone0/falcon/test/stop_planner", Trigger.Request())
                self.wait(lambda: self.runtime.get("planner_failed"), 10)
                coverage_wait = max(
                    20.0, float(self.adapter.get("coverage_remaining_s", 0.0)) + 15.0)
                self.wait(lambda: bool(self.adapter.get("failed")), coverage_wait)
                if not self.is_planner_exit_failure(self.adapter["failed"]):
                    raise RuntimeError("unexpected planner-exit result: " + self.adapter["failed"])
            elif self.case == "normal":
                def normal_terminal():
                    self.route_gate(initial, collision_stamp)
                    return (self.adapter.get("completed")
                            or bool(self.adapter.get("failed")))
                self.wait(normal_terminal, 300)
                if self.adapter.get("failed"):
                    raise RuntimeError(self.adapter["failed"])
            else:
                raise RuntimeError("unknown acceptance case")

            self.wait(lambda: self.adapter.get("stop_confirmed") and self.speed() < 0.3, 20)
            self.route_gate(initial, collision_stamp, force=True)
            positions = self.positions_since(self.route, exploration_start_index)
            if not positions or any(not self.in_bounds(position) for position in positions):
                raise RuntimeError("recorded route left declared exploration bounds")
            distance = max(math.dist(initial, position) for position in positions)
            observed_bounds = {
                "min": [min(position[index] for position in positions) for index in range(3)],
                "max": [max(position[index] for position in positions) for index in range(3)],
            }
            map_growth = self.map_counts["free"] - self.map_first.get("free", 0)
            matched_depth = int(self.adapter.get("matched_depth_count", 0))
            trajectory_ids = [int(value) for value in self.adapter.get("trajectory_ids", [])]
            if distance < 1.0 or map_growth <= 0 or not trajectory_ids or matched_depth <= 0:
                raise RuntimeError("exploration evidence gate failed")
            self.capture("after-exploration")
            self.result.update(
                expected_outcome=True, stop_confirmed=True,
                movement_m=distance, map_growth_points=map_growth,
                observed_route_bounds=observed_bounds,
                map_counts=self.map_counts, frontier_points=self.frontier_points,
                matched_depth_count=matched_depth,
                dropped_depth_count=int(self.adapter.get("dropped_depth_count", 0)),
                trajectory_ids=trajectory_ids,
                motion_reference_count=self.references)
        except BaseException as exc:
            self.result["error"] = str(exc)
        finally:
            if self.platform is not None and self.platform.armed:
                try:
                    self.service(SetBool, "/drone0/falcon/enable", SetBool.Request(data=False))
                    self.wait(lambda: self.adapter.get("stop_confirmed"), 15)
                    self.action(Land, "/drone0/LandBehavior",
                                Land.Goal(land_speed=0.5), "land_success")
                    self.wait(lambda: not self.platform.armed, 30)
                    self.result["landed_disarmed"] = True
                except BaseException as exc:
                    self.result["recovery_error"] = str(exc)
            self.result["success"] = bool(
                self.result.get("expected_outcome")
                and self.result.get("stop_confirmed")
                and self.result.get("landed_disarmed")
                and not self.result.get("error")
                and not self.result.get("recovery_error"))
            (self.output / "result.json").write_text(
                json.dumps(self.result, indent=2, sort_keys=True) + "\n")
            print(json.dumps(self.result, sort_keys=True), flush=True)

    def close(self):
        self.samples.close()


def main():
    rclpy.init()
    node = Mission()
    try:
        node.run()
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    if not node.result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
