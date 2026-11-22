"""Real ROS process/lifecycle check; no inference, motion goals or flight."""
import json
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from uav_planner_interfaces.srv import ControlPlanner
from uav_planner_framework.ros_node import PlannerFrameworkNode
from uav_planner_framework.behavior import BehaviorStatus


def main():
    rclpy.init()
    framework = PlannerFrameworkNode()
    inputs = Node('tool_lifecycle_inputs')
    pose_pub = inputs.create_publisher(PoseStamped, '/drone0/self_localization/pose', qos_profile_sensor_data)
    twist_pub = inputs.create_publisher(TwistStamped, '/drone0/self_localization/twist', qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(framework)
    executor.add_node(inputs)
    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pose = PoseStamped()
            pose.header.stamp = inputs.get_clock().now().to_msg()
            pose.header.frame_id = 'earth'
            pose.pose.position.z = 2.0
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            twist = TwistStamped()
            twist.header = pose.header
            twist_pub.publish(twist)
            executor.spin_once(timeout_sec=0.02)
    def control(command):
        request = ControlPlanner.Request()
        request.command = command
        response = framework._on_control(request, ControlPlanner.Response())
        assert response.accepted, response.message
    try:
        with tempfile.TemporaryDirectory(prefix='tool-lifecycle-') as directory:
            root = Path(directory)
            task = {
                'task': {'name': 'tool-lifecycle'},
                'planner': {'id': 'aerovla', 'version': 'b87aaa7c9338ae878d39300a1ef930c753e1ff90',
                            'parameters': {'prompt_table': ''}},
                'vehicles': ['drone0'], 'targets': {'drone0': {'x': 0., 'y': 0., 'z': -2.}},
                'completion': {'position_tolerance': 0.2, 'hold_duration': 0.3, 'task_timeout': 30.}}
            deployment = {'drone0': {'namespace': '/drone0', 'px4_origin_in_fleet_ned': [0., 0., 0.],
                                     'topics': {'pose': '/drone0/self_localization/pose'}}}
            for name, value in (('task', task), ('deployment', deployment)):
                (root / (name + '.yaml')).write_text(yaml.safe_dump(value))
            framework._start(str(root / 'task.yaml'), str(root / 'deployment.yaml'))
            pump(2.0)
            assert framework.execution is None and framework.gateway is None
            assert framework.tool_execution.status, 'no task-scoped heartbeat'
            assert framework.lifecycle.snapshot.status == BehaviorStatus.RUNNING
            assert framework.planner_process.poll() is None
            # Proximity to the oracle must not complete this Tool; it owns no trajectory session.
            control(ControlPlanner.Request.PAUSE)
            pump(1.5)
            assert framework.tool_execution.quiescent, framework.tool_execution.status
            assert framework.planner_process.poll() is None, 'pause killed the Tool'
            control(ControlPlanner.Request.RESUME)
            pump(1.0)
            assert framework.lifecycle.snapshot.status == BehaviorStatus.RUNNING
            assert framework.tool_execution.status['state'] == 'RUNNING'
            control(ControlPlanner.Request.CANCEL)
            pump(1.5)
            assert framework.tool_execution.quiescent
            assert framework.planner_process is None
            assert framework.lifecycle.snapshot.status == BehaviorStatus.CANCELED
            print('TOOL_RUNTIME_PASS: registry GoTo spawn, no trajectory session, pause/stop confirmation, resume, cancel')
    finally:
        framework._stop_runtime()
        executor.shutdown()
        inputs.destroy_node()
        framework.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
