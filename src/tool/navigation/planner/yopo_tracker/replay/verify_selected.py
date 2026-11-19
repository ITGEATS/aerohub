"""Replay a finite bag through native planning and the isolated FollowTimedPath rig."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
from threading import Condition, Event, Thread

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.executors import SingleThreadedExecutor
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal

from check_action import SECOND, boundary, ns
from native_replay import load_tracker, replay
from timed_points import timed_goal
from ros2_adapter.native.tracker_model import frozen_yaw, sample_yaw


class ReplayHandoff:
    def __init__(self, rig, tracker):
        self.rig = rig
        self.tracker = tracker
        self.handle = None
        self.accepted = []
        self.feedback = []
        self.frame = 'drone0/odom'
        self.task = 'yopo-selected-replay'

    def images(self, depth, rgb):
        tracker = self.tracker
        if tracker.odom.header.frame_id != self.frame:
            raise ValueError(f'Replay odometry must be expressed in {self.frame}')
        generated = self.rig.get_clock().now().nanoseconds
        if self.accepted:
            epoch, duration_ns, axes, knots = self.accepted[-1]
            start = epoch + duration_ns // 2
            if start <= generated:
                raise RuntimeError('Replay missed the fixed overlap; no trajectory re-stamping')
            tracker.desire_pos, tracker.desire_vel, tracker.desire_acc = boundary(
                axes, (start - epoch) / SECOND)
            tracker.last_yaw = sample_yaw(knots, start - epoch)
        else:
            # Same first-goal lead as the existing isolated execution fixture.
            start = generated + SECOND // 2

        previous = tracker.optimal_poly_x
        tracker.callback_images(depth, rgb)
        if (tracker.optimal_poly_x is previous or tracker.ekf_target is None or
                not tracker.last_trajectory_tracking or tracker.searching or tracker.hovering):
            return

        axes = (tracker.optimal_poly_x, tracker.optimal_poly_y, tracker.optimal_poly_z)
        duration = tracker.traj_time
        duration_ns = round(duration * SECOND)
        knots = frozen_yaw(tracker, axes, duration_ns)
        revision = len(self.accepted) + 1
        goal = timed_goal(axes, duration, knots, task_id=self.task, vehicle_id='drone0',
                          revision=revision, generated_ns=generated, start_ns=start, max_speed=1.)
        if start <= self.rig.get_clock().now().nanoseconds:
            raise RuntimeError('Planner output arrived after the proposed start')
        if self.handle is None:
            handle = self.rig.wait(self.rig.action.send_goal_async(
                goal, feedback_callback=self.record_feedback))
            if not handle.accepted:
                raise RuntimeError('Behavior rejected native Goal')
            self.handle = handle
        else:
            request = FollowTimedPath_SendGoal.Request()
            request.goal = goal
            if not self.rig.wait(self.rig.modify.call_async(request)).accepted:
                raise RuntimeError('Behavior rejected native Modify')
        self.accepted.append((start, duration_ns, axes, knots))
        print(json.dumps(dict(event='accepted', revision=revision, valid_from_ns=start,
                              duration_ns=duration_ns, rgb_ns=ns(rgb.header.stamp),
                              depth_ns=ns(depth.header.stamp), odom_ns=ns(tracker.odom.header.stamp))))

    def record_feedback(self, event):
        with self.rig.changed:
            self.feedback.append(event.feedback)
            self.rig.changed.notify_all()

    def observe(self):
        start, duration_ns, _, _ = self.accepted[-1]

        def ready():
            active = {msg.revision for msg in self.feedback
                      if msg.task_id == self.task and not msg.waiting_for_start}
            return (active == {1, 2} and self.rig.trajectory_references and
                    ns(self.rig.trajectory_references[-1].header.stamp) >= start + duration_ns // 2)

        with self.rig.changed:
            if not self.rig.changed.wait_for(ready, timeout=5.):
                raise RuntimeError('Both revisions must produce real execution feedback and references')
            references = tuple(self.rig.trajectory_references)
        samples = [0, 0]
        for msg in references:
            stamp = ns(msg.header.stamp)
            assert stamp >= self.accepted[0][0]
            index = int(stamp >= self.accepted[1][0])
            epoch, _, axes, knots = self.accepted[index]
            point = msg.setpoints[0]
            actual = [[getattr(field, axis) for axis in ('x', 'y', 'z')]
                      for field in (point.position, point.twist, point.acceleration)]
            np.testing.assert_allclose(actual, boundary(axes, (stamp - epoch) / SECOND), atol=1e-5)
            expected_yaw = sample_yaw(knots, stamp - epoch)
            assert abs(math.remainder(point.yaw_angle - expected_yaw, math.tau)) < 1e-5
            samples[index] += 1
        assert all(samples)
        return samples

    def cancel(self):
        if self.handle is None:
            return
        handle, self.handle = self.handle, None
        result = handle.get_result_async()
        self.rig.wait(handle.cancel_goal_async())
        response = self.rig.wait(result)
        cutoff = self.rig.get_clock().now().nanoseconds
        with self.rig.changed:
            published = self.rig.changed.wait_for(
                lambda: (self.rig.trajectory_references and
                         ns(self.rig.trajectory_references[-1].header.stamp) > cutoff), timeout=.2)
        return response.status, response.result.revision, not published


def use_fixture_heads(tracker):
    """Fixed positive tensors exercise original postprocessing; not a model result."""
    import torch

    lattice = tracker.lattice_primitive
    shape = (1, lattice.vertical_num, lattice.horizon_num)
    center = (0, lattice.vertical_num // 2, lattice.horizon_num // 2)

    def predict(image, _state):
        end = torch.zeros((1, 9, *shape[1:]), device=image.device)
        target = torch.zeros((1, 3, *shape[1:]), device=image.device)
        score = torch.zeros(shape, device=image.device)
        cost = torch.ones_like(score)
        duration = torch.full_like(score, .6)
        end[:, 2] = -.9
        target[:, 0:2] = .5
        target[:, 2] = 2.
        score[center] = .99
        return end, target, cost, score, duration

    tracker.policy = predict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    parser.add_argument('bag', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--expect-empty', action='store_true', help='Real-model negative control')
    mode.add_argument('--fixture-heads', action='store_true', help='Synthetic positive tensors')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Run in the isolated ROS fixture')
    sys.path.insert(0, str(Path(__file__).resolve().parents[6] / 'src/behavior/follow_path/tests'))
    from ros_smoke import Test

    class ReplayRig(Test):
        def __init__(self):
            self.changed = Condition()
            super().__init__()

        def trajectory_reference(self, msg):
            with self.changed:
                super().trajectory_reference(msg)
                self.changed.notify_all()

        def wait(self, future, timeout=6):
            done = Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout):
                raise TimeoutError('Execution response timed out')
            return future.result()

    rclpy.init(args=[])
    rig = ReplayRig()
    executor = SingleThreadedExecutor()
    executor.add_node(rig)
    spinner = Thread(target=executor.spin, name='simulation-state')
    spinner.start()
    tracker = None
    try:
        tracker, digest = load_tracker(args.upstream)
        tracker.timer_ctrl.cancel()
        assert tracker.device == 'cuda', 'The replay baseline expects the real CUDA runtime'
        if args.fixture_heads:
            use_fixture_heads(tracker)
        assert rig.action.wait_for_server(timeout_sec=5)
        assert rig.modify.wait_for_service(timeout_sec=5)
        handoff = ReplayHandoff(rig, tracker)
        try:
            counts = replay(tracker, args.bag, 'sqlite3',
                            ['/drone_0/odom', '/drone_0/depth', '/drone_0/rgb'],
                            on_images=handoff.images, advance_reference=False,
                            complete=lambda: len(handoff.accepted) == 2)
            if args.expect_empty:
                assert not handoff.accepted and not counts['confirmed']
                with rig.changed:
                    assert not rig.trajectory_references
                samples = []
            else:
                assert len(handoff.accepted) == 2, 'Bag did not provide two accepted tracking revisions'
                samples = handoff.observe()
        finally:
            canceled = handoff.cancel()
        if not args.expect_empty:
            assert canceled == (GoalStatus.STATUS_CANCELED, 2, True), canceled
        print(json.dumps(dict(result='pass', heads='fixture' if args.fixture_heads else 'native',
                              model_device=tracker.device, weight_sha256=digest,
                              accepted=len(handoff.accepted), samples=samples, **counts)))
    finally:
        if tracker is not None:
            tracker.destroy_node()
        executor.shutdown()
        spinner.join()
        rig.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
