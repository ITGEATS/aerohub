"""Real native GoTo with synthetic state; no controller, inference or flight.

Run in the Humble/AS2 image. Exercises early yaw success and the Tool's measured
convergence gate. The control-mode service is a stub, explicitly not a plant.
"""
import math
import subprocess
import time

import rclpy
from as2_msgs.action import GoToWaypoint
from as2_msgs.msg import PlatformInfo, PlatformStatus
from as2_msgs.srv import SetControlMode
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from uav_aerovla_planner.goto_steps import Step
from uav_aerovla_planner.goto_node import AerovlaGoToNode


def main():
    rclpy.init()
    node = Node('native_goto_check', namespace='drone0')
    tool = AerovlaGoToNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(tool)
    actual_yaw = 0.0
    process = subprocess.Popen([
        '/root/aerostack2_ws/install/lib/as2_behaviors_motion/go_to_behavior_node',
        '--ros-args', '-r', '__ns:=/drone0',
        '--params-file', '/root/aerostack2_ws/install/share/as2_behaviors_motion/go_to_behavior/config/config_default.yaml',
        '-p', 'plugin_name:=go_to_plugin_position', '-p', 'use_sim_time:=false'])
    try:
        tf = TransformBroadcaster(node)
        info_pub = node.create_publisher(PlatformInfo, 'platform/info', 10)
        twist_pub = node.create_publisher(TwistStamped, 'self_localization/twist', qos_profile_sensor_data)
        references = []
        node.create_subscription(PoseStamped, 'motion_reference/pose', references.append, qos_profile_sensor_data)
        def set_mode(request, response):
            response.success = True
            return response
        node.create_service(SetControlMode, 'controller/set_control_mode', set_mode)
        client = ActionClient(node, GoToWaypoint, 'GoToBehavior')
        def pump(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                assert process.poll() is None, 'native process exited'
                stamp = node.get_clock().now().to_msg()
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = 'earth'
                transform.child_frame_id = 'drone0/base_link'
                transform.transform.translation.z = 2.0
                transform.transform.rotation.w = math.cos(actual_yaw / 2)
                transform.transform.rotation.z = math.sin(actual_yaw / 2)
                tf.sendTransform(transform)
                info = PlatformInfo()
                info.status.state = PlatformStatus.FLYING
                info_pub.publish(info)
                twist = TwistStamped()
                twist.header.stamp = stamp
                twist.header.frame_id = 'earth'
                twist_pub.publish(twist)
                pose = PoseStamped()
                pose.header = transform.header
                pose.pose.position.z = 2.0
                pose.pose.orientation = transform.transform.rotation
                tool.pose = pose
                tool.on_twist(twist)
                executor.spin_once(timeout_sec=0.02)
        pump(2.0)
        assert client.server_is_ready(), 'native action unavailable'
        goal = GoToWaypoint.Goal()
        goal.target_pose.header.frame_id = 'earth'
        goal.target_pose.point.z = 2.0
        goal.yaw.mode = goal.yaw.FIXED_YAW
        goal.yaw.angle = 1.0
        goal.max_speed = 1.0
        accepted = client.send_goal_async(goal)
        end = time.monotonic() + 8
        while not accepted.done() and time.monotonic() < end:
            pump(0.05)
        assert accepted.done() and accepted.result().accepted, 'native goal rejected'
        result = accepted.result().get_result_async()
        while not result.done() and time.monotonic() < end:
            pump(0.05)
        assert result.done() and result.result().result.go_to_success, 'no native success'
        pump(0.3)
        assert references, 'no final position reference'
        reference = references[-1]
        yaw = 2 * math.atan2(reference.pose.orientation.z, reference.pose.orientation.w)
        assert abs(yaw - 1.0) < 1e-5, yaw
        # Exercise the actual Tool callbacks against the same native server.
        tool.goal_frame = 'earth'
        tool.pending_steps = [Step((0.0, 0.0, 2.0), 1.0, 1.0, 'rotate')]
        tool.state = 'RUNNING'
        tool.phase = 'READY'
        tool.deadline = time.monotonic() + 10
        pump(1.0)
        assert tool.native_success and tool.phase == 'EXECUTING', (tool.phase, tool.last_error)
        assert tool.step == 0, 'Tool advanced before yaw convergence'
        actual_yaw = 1.0
        pump(1.5)
        assert tool.phase == 'IDLE' and tool.step == 1, (tool.phase, tool.last_error)
        print('NATIVE_GOTO_PASS: early success at yaw=0; final yaw reference=1; actual Tool waits and advances after convergence')
    finally:
        process.terminate()
        process.wait(timeout=5)
        executor.shutdown()
        tool.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
