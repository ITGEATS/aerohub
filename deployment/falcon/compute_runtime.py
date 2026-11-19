"""Own AS2, FALCON and its adapter inside the compute role."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Runtime(Node):
    def __init__(self):
        super().__init__(
            "falcon_compute_runtime",
            parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.output = Path("/results")
        self.children = {}
        self.streams = {}
        self.stopping = False
        self.planner_failed = False
        self.adapter_state = {}
        self.state_publisher = self.create_publisher(
            String, "/drone0/falcon/runtime_state", 10)
        self.create_subscription(
            String, "/drone0/falcon/adapter/state", self.on_adapter_state, 10)
        self.create_service(
            Trigger, "/drone0/falcon/test/stop_planner", self.stop_planner)
        self.create_timer(0.1, self.tick)

    def on_adapter_state(self, message):
        self.adapter_state = json.loads(message.data)

    def stop_planner(self, _request, response):
        planner = self.children.get("falcon")
        if planner is None or planner.poll() is not None:
            response.message = "planner is not running"
            return response
        # This test hook models an abrupt planner crash. SIGTERM can be
        # handled by the ros2 CLI wrapper while leaving its launched node
        # alive, so kill the complete process group deterministically.
        os.killpg(planner.pid, signal.SIGKILL)
        response.success = True
        response.message = "planner stop injected"
        return response

    def start(self, name, command):
        stream = (self.output / f"{name}.log").open("w")
        child = subprocess.Popen(
            command, stdout=stream, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.children[name] = child
        self.streams[name] = stream

    def tick(self):
        states = {name: child.poll() for name, child in self.children.items()}
        for name in ("as2", "adapter"):
            if name in states and states[name] is not None:
                raise RuntimeError(f"{name} exited {states[name]}")
        if states.get("falcon") is not None:
            self.planner_failed = True
        state = {
            "ready": all(states.get(name) is None for name in ("as2", "falcon", "adapter")),
            "children": {name: code is None for name, code in states.items()},
            "planner_failed": self.planner_failed,
            "adapter": self.adapter_state,
        }
        self.state_publisher.publish(String(data=json.dumps(state, sort_keys=True)))

    def run(self):
        parameter_file = self.output / "falcon.params.yaml"
        runtime_dir = self.output / "falcon-runtime"
        subprocess.run([
            "python3", "/opt/falcon/runtime/prepare_params.py",
            "--source", "/opt/falcon/config/upstream",
            "--runtime-dir", str(runtime_dir), "--output", str(parameter_file),
        ], check=True)
        self.start("as2", ["python3", "/opt/falcon/runtime/run_as2.py"])
        self.start("falcon", [
            "ros2", "run", "falcon_ros2", "falcon_exploration_node",
            "--ros-args", "--params-file", str(parameter_file),
            "-r", "/odom_world:=/drone0/falcon/input/odometry",
            "-r", "/voxel_mapping/depth_image:=/drone0/falcon/input/depth",
            "-r", "/transformer/sensor_pose_topic:=/drone0/falcon/input/sensor_pose",
        ])
        self.start("adapter", [
            "ros2", "run", "uav_falcon_adapter", "falcon_adapter", "--ros-args",
            "-r", "__ns:=/drone0", "-p", "use_sim_time:=true",
            "-r", "sensor_measurements/odom:=/drone0/sensor_measurements/odom",
            "-r", "sensor_measurements/depth:=/drone0/sensor_measurements/FrontCamera/depth_planar/image",
        ])
        try:
            while not self.stopping:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            for child in self.children.values():
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGTERM)
            for name, child in self.children.items():
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=5)
                self.streams[name].close()


def main():
    rclpy.init()
    node = Runtime()
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, "stopping", True))
    signal.signal(signal.SIGINT, lambda *_: setattr(node, "stopping", True))
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
