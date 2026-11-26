"""Real Humble Action readiness/rejection checks; no sensors or motion servers."""
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from uav_flowpilot_planner.node import FlowPilotPlanner
from uav_planner_interfaces.action import NavigateToPose


def main():
    rclpy.init()
    planner = FlowPilotPlanner(namespace='/flowpilot_local_smoke')
    client_node = rclpy.create_node('navigation_smoke_client', namespace='/flowpilot_local_smoke')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(planner)
    executor.add_node(client_node)
    client = ActionClient(client_node, NavigateToPose, 'navigate_to_pose')

    def wait(future):
        executor.spin_until_future_complete(future, timeout_sec=5.)
        assert future.done(), 'ROS operation did not complete'
        return future.result()

    try:
        assert planner.navigation is not None
        assert planner.pose is None and not planner.ready(), 'Use an isolated ROS domain with no execution servers'
        assert client.wait_for_server(timeout_sec=5.)
        for label, expected in [('wrong-frame', 'INVALID_GOAL'), ('yaw', 'UNSUPPORTED'),
                                ('hold', 'UNSUPPORTED'), ('not-ready', 'NOT_READY')]:
            goal = NavigateToPose.Goal()
            goal.target.header.frame_id = 'wrong' if label == 'wrong-frame' else planner.frame
            goal.target.pose.position.x = 12.
            if label == 'yaw':
                goal.orientation_mode = goal.KEEP_YAW
            if label == 'hold':
                goal.arrival_behavior = goal.HOLD
            handle = wait(client.send_goal_async(goal))
            assert handle.accepted, label
            result = wait(handle.get_result_async()).result
            assert not result.success and result.code == expected, (label, result)
            assert planner.state == 'IDLE' and planner.goal is None
            print(label, result.code, flush=True)
        cancel = client_node.create_client(Trigger, 'flowpilot/cancel')
        assert cancel.wait_for_service(timeout_sec=5.)
        assert not wait(cancel.call_async(Trigger.Request())).success
        print('Public Navigation Action and private cancel rejection passed; motion untested.', flush=True)
    finally:
        client.destroy()
        executor.shutdown(timeout_sec=3.)
        planner.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
