"""Exercise a short handoff through the live adapter and the real ROS behavior."""

import argparse
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from as2_msgs.msg import ControllerInfo, ControlMode, PlatformInfo
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

from check_action import build_curve
from ros2_adapter.bridge.live_inputs import ns
from ros2_adapter.bridge.live_node import LiveTracker
from ros2_adapter.bridge.protected_trajectory import BrakedAxis, CONTROL_STEP_NS, HOLD_NS, ProtectedTrajectory
from verify_live_handoff import image_at, MS


SHORT_SPLICE_NS = 240 * MS
SECOND = 10**9


class HandoffTracker(LiveTracker):
    def __init__(self, set_trajectory):
        self.set_trajectory = set_trajectory
        self.events = {}
        self.curves = {}
        self.goal_handle = None
        self.short_frame_ns = None
        self.frame_ns = 0
        self.frame_active = 0
        tracker = SimpleNamespace(device='fixture', reset_state=lambda: None)
        super().__init__(tracker, None, 'fixture')
        result, = self.set_parameters([Parameter('use_sim_time', value=False)])
        assert result.successful, result.reason

    def check_camera(self):
        pass  # Supply already-paired observations for this handoff fixture.

    def record(self, event, **fields):
        self.events[event, fields.get('revision')] = self.now_ns(), fields
        super().record(event, **fields)

    def receive(self):
        if self.pending and self.pending[1].path.revision == 1 and self.pending[0].done():
            self.goal_handle = self.pending[0].result()
        super().receive()

    def snapshot(self, _):
        now = self.now_ns()
        first_active_frame = self.active == 1 and self.frame_active == 0
        if not first_active_frame and now - self.frame_ns < 60 * MS:
            return None
        if self.revision >= 2:
            epoch = self.events['submit', 2][1]['start_ns']
            if self.short_frame_ns is None:
                if now < epoch - 140 * MS:
                    return None
                self.short_frame_ns = now
            elif now < epoch + 100 * MS:
                return None
        elif self.pending and self.pending[1].path.revision == 2:
            return None
        self.frame_ns = now
        self.frame_active = self.active
        frame = image_at(now)
        return frame, frame, frame

    def select(self, depth, rgb, odom, anchor):
        revision = self.revision + 1
        if revision > 3:
            return None, None, None
        self.record('worker_started', revision=revision)
        if revision == 3:
            time.sleep(.070)  # Reproduce a worker that cannot fit in the 40 ms after activation.
        initial = anchor[:3] if anchor is not None else (np.zeros(3),) * 3
        terminal = ([float(initial[0][0]) + .5, 0., 0.], [.25, 0., 0.], np.zeros(3))
        axes = build_curve(self.set_trajectory, initial, terminal, 2.)
        cut_ns = {1: 400 * MS, 2: SHORT_SPLICE_NS, 3: 800 * MS}[revision]
        brake_ns = 400 * MS
        end_ns = cut_ns + brake_ns
        protected = tuple(BrakedAxis(axis, cut_ns / SECOND, brake_ns / SECOND) for axis in axes)
        knots = [(t, 0.) for t in range(0, end_ns + 1, CONTROL_STEP_NS)] + [(end_ns + HOLD_NS, 0.)]
        curve = ProtectedTrajectory(protected, knots, cut_ns, end_ns, .1)
        self.curves[revision] = curve
        return curve, [1., 0., 0.], None


def run(node, rig, executor):
    def wait_until(predicate, failure):
        deadline = time.monotonic() + 6.
        while not predicate():
            assert time.monotonic() < deadline, failure
            executor.spin_once(timeout_sec=.02)

    wait_until(lambda: node.action.server_is_ready() and node.modify.service_is_ready()
               and node.poses, 'FollowTimedPath or state input unavailable')
    # Seed the live start check with the airborne hover state of the fixture.
    node.platform = PlatformInfo(armed=True, offboard=True)
    node.controller = ControllerInfo(input_control_mode=ControlMode(control_mode=ControlMode.HOVER))
    reply = node.start(Trigger.Request(), Trigger.Response())
    assert reply.success, reply.message
    wait_until(lambda: node.active == 3 or node.brake_reason or ('finished', 0) in node.events,
               'Third revision never activated')
    assert node.active == 3 and node.brake_reason is None, f'Handoff stopped: {node.brake_reason}'

    activated_2, _ = node.events['active', 2]
    activated_3, _ = node.events['active', 3]
    planned_3, _ = node.events['worker_started', 3]
    submitted_3, submission = node.events['submit', 3]
    accepted_3, _ = node.events['accepted', 3]
    _, parent = node.events['submit', 2]
    splice_ns = submission['start_ns']
    assert planned_3 < activated_2 <= submitted_3 <= accepted_3 < splice_ns <= activated_3
    assert submission['anchor_revision'] == 2 and splice_ns == parent['start_ns'] + SHORT_SPLICE_NS
    assert submission['rgb_ns'] == node.short_frame_ns < activated_2

    def post_splice_reference():
        return next((msg for msg in rig.trajectory_references if ns(msg.header.stamp) >= splice_ns + 100 * MS), None)

    wait_until(post_splice_reference, 'No trajectory reference after the splice')
    msg = post_splice_reference()
    elapsed_s = (ns(msg.header.stamp) - splice_ns) / SECOND
    sample = msg.setpoints[0]
    next_axis = node.curves[3].axes[0]
    fallback_axis = node.curves[2].axes[0]
    np.testing.assert_allclose([sample.position.x, sample.twist.x, sample.acceleration.x],
                               [next_axis.get_position(elapsed_s), next_axis.get_velocity(elapsed_s),
                                next_axis.get_acceleration(elapsed_s)], atol=1e-4)
    assert abs(sample.twist.x - fallback_axis.get_velocity(.24 + elapsed_s)) > .005
    print(f'PASS: revision 3 worker started {(activated_2 - planned_3) / MS:.1f} ms before revision 2 '
          f'activated, update accepted {(splice_ns - accepted_3) / MS:.1f} ms before the splice, '
          'and controller-facing reference matches revision 3')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.upstream.resolve() / 'YOPO'))
    sys.path.insert(0, str(Path(__file__).resolve().parents[6] / 'src/behavior/follow_path/tests'))
    from ros_smoke import Test
    from test_yopo_ros import YopoNet

    rclpy.init(args=[])
    node, rig = HandoffTracker(YopoNet.set_trajectory), Test()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(rig)
    try:
        run(node, rig, executor)
    finally:
        if node.goal_handle and node.goal_handle.accepted and node.result and not node.result.done():
            node.stop(Trigger.Request(), Trigger.Response())
            node.goal_handle.cancel_goal_async()
            executor.spin_until_future_complete(node.result, timeout_sec=3.)
        executor.shutdown()
        node.worker.shutdown()
        node.destroy_node()
        rig.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
