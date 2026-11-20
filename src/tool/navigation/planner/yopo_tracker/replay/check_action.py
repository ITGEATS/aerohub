"""Exercise a frozen, target-independent two-segment fixture with the ROS test rig."""
import argparse
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal

from timed_points import timed_goal


SECOND = 10**9


def ns(stamp):
    return stamp.sec * SECOND + stamp.nanosec


def boundary(axes, t):
    return [np.array([getattr(axis, method)(t) for axis in axes])
            for method in ('get_position', 'get_velocity', 'get_acceleration')]


def build_curve(set_trajectory, initial, terminal, duration):
    planner = SimpleNamespace(desire_acc=initial[2])
    set_trajectory(planner, *initial[:2], *terminal, duration)
    return (planner.optimal_poly_x, planner.optimal_poly_y, planner.optimal_poly_z)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.upstream.resolve() / 'YOPO'))
    sys.path.insert(0, str(Path(__file__).resolve().parents[6] / 'src/behavior/follow_path/tests'))
    from test_yopo_ros import YopoNet
    from ros_smoke import Test

    rclpy.init(args=[])
    node = Test()
    try:
        assert node.action.wait_for_server(timeout_sec=5)
        node.spin(.5)
        assert node.modify.wait_for_service(timeout_sec=5)
        task_id = 'yopo-replay'
        origin = [node.x, 0., 0.]
        initial = [origin, np.zeros(3), np.zeros(3)]
        terminal = [[node.x + .4, .15, -.05], [.04, -.03, .02], [.02, .04, -.01]]
        duration = 2.
        curve = build_curve(YopoNet.set_trajectory, initial, terminal, duration)
        metadata = dict(task_id=task_id, vehicle_id='drone0', max_speed=1.)
        generated_ns = node.get_clock().now().nanoseconds
        first_ns = generated_ns + SECOND // 2
        start_ns = first_ns + SECOND
        yaw0, yaw1, yaw2 = 3.0, 3.1, -3.08
        first = timed_goal(curve, duration, [(0, yaw0), (SECOND, yaw1), (2 * SECOND, yaw1)],
                           revision=1, generated_ns=generated_ns, start_ns=first_ns, **metadata)
        feedback = []
        handle = node.wait(node.action.send_goal_async(
            first, feedback_callback=lambda message: feedback.append(message.feedback)))
        assert handle.accepted, 'First segment rejected'
        continuation = boundary(curve, 1.)
        next_curve = build_curve(YopoNet.set_trajectory, continuation,
                                 [[node.x + .1, -.2, .3], np.zeros(3), np.zeros(3)], duration)
        knots = [(0, yaw1), (2 * SECOND, yaw2)]
        generated_ns = node.get_clock().now().nanoseconds
        second = timed_goal(next_curve, duration, knots, revision=2,
                            generated_ns=generated_ns, start_ns=start_ns, **metadata)
        request = FollowTimedPath_SendGoal.Request()
        # Use the real server's continuity check. A rejected update must not take revision 2.
        wrong_start = next_curve[0].get_position(0.) + 1.
        request.goal = timed_goal(next_curve, duration, knots, revision=2,
                                  generated_ns=generated_ns, start_ns=start_ns, **metadata)
        request.goal.path.points[0].position.x = float(wrong_start)
        assert not node.wait(node.modify.call_async(request)).accepted, 'Discontinuous update accepted'
        request.goal = second
        assert node.wait(node.modify.call_async(request)).accepted, 'Modify rejected'
        assert not node.wait(node.modify.call_async(request)).accepted, 'Duplicate accepted'
        deadline = time.monotonic() + 4.
        while not (any(ns(msg.header.stamp) >= start_ns + SECOND for msg in node.trajectory_references)
                   and any(msg.revision == 2 and not msg.waiting_for_start for msg in feedback)):
            assert time.monotonic() < deadline, 'No post-handoff reference or feedback'
            rclpy.spin_once(node, timeout_sec=.02)
        revisions = {1: 0, 2: 0}
        for msg in node.trajectory_references:
            t_ns = ns(msg.header.stamp)
            assert t_ns >= first_ns, 'Trajectory reference emitted before its epoch'
            revision = 1 if t_ns < start_ns else 2
            revisions[revision] += 1
            axes, epoch = (curve, first_ns) if revision == 1 else (next_curve, start_ns)
            t = (t_ns - epoch) / SECOND
            point = msg.setpoints[0]
            sample = [[getattr(field, axis) for axis in ('x', 'y', 'z')]
                      for field in (point.position, point.twist, point.acceleration)]
            np.testing.assert_allclose(sample, boundary(axes, t), atol=1e-5)
            expected_yaw = (yaw0 + (yaw1 - yaw0) * t if revision == 1
                            else yaw1 + math.remainder(yaw2 - yaw1, math.tau) * t / duration)
            assert abs(math.remainder(point.yaw_angle - expected_yaw, math.tau)) < 1e-5
        assert all(revisions.values())
        assert {msg.revision for msg in feedback
                if msg.task_id == task_id and not msg.waiting_for_start} == {1, 2}
        result = handle.get_result_async()
        node.wait(handle.cancel_goal_async())
        response = node.wait(result)
        assert response.status == GoalStatus.STATUS_CANCELED and response.result.revision == 2
        stopped_ns = node.get_clock().now().nanoseconds
        node.spin(.2)
        assert all(ns(msg.header.stamp) <= stopped_ns for msg in node.trajectory_references)
        print(f'PASS: native quintics, Goal/Modify, rejection, reference samples {revisions}, cancel')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
