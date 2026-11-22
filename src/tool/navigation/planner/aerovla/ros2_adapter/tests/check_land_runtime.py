"""Real Tool + ROS Land Action type with a synthetic server; no flight.

Exercises model-output dispatch, acceptance/rejection and terminal results.
The server is a test double, not the native AS2 Land implementation.
"""
import time
from concurrent.futures import Future
from types import SimpleNamespace

import rclpy
from as2_msgs.action import Land
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from uav_aerovla_planner.goto_node import AerovlaGoToNode


def main():
    rclpy.init()
    tool = AerovlaGoToNode()
    server_node = Node('synthetic_land_server', namespace='drone0')
    executor = SingleThreadedExecutor()
    executor.add_node(tool)
    executor.add_node(server_node)
    mode = 'success'
    calls = []
    def execute(handle):
        calls.append(handle.request.land_speed)
        result = Land.Result()
        result.land_success = mode == 'success'
        if mode == 'success':
            handle.succeed()
        else:
            handle.abort()
        return result
    server = ActionServer(server_node, Land, 'LandBehavior', execute,
                          goal_callback=lambda _: GoalResponse.REJECT if mode == 'reject' else GoalResponse.ACCEPT)
    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            tool.pose = PoseStamped()
            tool.pose.header.stamp = tool.get_clock().now().to_msg()
            tool.pose.header.frame_id = 'earth'
            tool.pose.pose.orientation.w = 1.0
            tool.twist = TwistStamped()
            tool.twist.header = tool.pose.header
            executor.spin_once(timeout_sec=0.02)
    def output(text):
        tool.phase = 'IDLE'
        tool.state = 'RUNNING'
        tool.last_error = ''
        tool.land_success = False
        context = {'generation': tool.generation, 'sent': time.monotonic()}
        tool.in_flight = context
        future = Future()
        future.set_result(SimpleNamespace(success=True, total_ms=1., gen_ids=[], text=text))
        tool.on_response(future, context)
    try:
        pump(0.5)
        output('Action: 44 49')
        pump(0.2)
        assert not calls and tool.last_error.startswith('INVALID_ACTION:'), tool.last_error
        output('Action: LAND')
        pump(1.0)
        assert calls == [0.0], calls  # AS2 default landing speed requested
        assert tool.state == 'SUCCEEDED' and tool.land_success, (tool.phase, tool.last_error)
        assert tool.in_flight is None
        mode = 'reject'
        output('Action: 0 49 49')
        pump(0.2)
        assert tool.state == 'PAUSED' and tool.last_error == 'LAND_REJECTED', (tool.phase, tool.last_error)
        assert not tool.land_success
        pump(0.5)
        mode = 'failure'
        output('<LAND>')
        pump(0.2)
        assert tool.state == 'PAUSED' and tool.last_error == 'LAND_FAILED', (tool.phase, tool.last_error)
        assert not tool.land_success
        print('LAND_RUNTIME_PASS: invalid output sends no goal; LAND dispatches native action type; success/rejection/failure propagate')
    finally:
        server.destroy()
        executor.shutdown()
        tool.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
