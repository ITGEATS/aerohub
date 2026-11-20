"""Exercise two live YOPO-Tracker revisions in the dedicated AirSim PX4 SITL test."""
import argparse
from bisect import bisect_left
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from as2_msgs.msg import TrajectorySetpoints
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformException
from uav_as2_sensor_adapter.rpc import Rpc
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal

from check_action import SECOND, boundary, ns
from collect_airsim_ground import CAMERA_FRAME, GroundCapture, SYNC_S, check_images
from native_replay import load_tracker
from sitl_fixed_reference import Check
from timed_points import timed_goal
from verify_selected import frozen_yaw, sample_yaw


OBSERVATION_AGE_NS = 300_000_000  # Existing ordinary YOPO max_observation_age_s.
SYNC_NS = round(SYNC_S * SECOND)
TASK = 'yopo-tracker-live-sitl'


class LiveCheck(Check):
    def __init__(self, capture, tracker, trace):
        super().__init__()
        self.capture = capture
        self.tracker = tracker
        self.trace = trace
        self.reference_samples = []
        self.activation = {}
        self.last_snapshot = 0
        self.first_images = None
        self.create_subscription(TrajectorySetpoints, '/drone0/motion_reference/trajectory',
                                 self.reference_samples.append, qos_profile_sensor_data)

    def record(self, event, **fields):
        self.trace.write(json.dumps(dict(event=event, sim_ns=self.get_clock().now().nanoseconds,
                                         **fields)) + '\n')
        self.trace.flush()

    def snapshot(self, after):
        capture = self.capture
        if not capture.pairs:
            return None
        depth, rgb = capture.pairs[-1]
        depth_ns, rgb_ns = ns(depth.header.stamp), ns(rgb.header.stamp)
        if min(depth_ns, rgb_ns) <= max(after, self.last_snapshot):
            return None
        now = self.get_clock().now().nanoseconds
        if max(depth_ns, rgb_ns) > now or now - min(depth_ns, rgb_ns) > OBSERVATION_AGE_NS:
            return None
        for history in (capture.poses, capture.twists):
            for stamp in tuple(history):
                if now - stamp > OBSERVATION_AGE_NS + SYNC_NS:
                    del history[stamp]
        stamps = sorted(capture.poses.keys() & capture.twists.keys())
        index = bisect_left(stamps, depth_ns)
        nearby = stamps[max(0, index - 1):index + 1]
        if not nearby:
            return None
        stamp = min(nearby, key=lambda t: abs(t - depth_ns))
        if max(abs(stamp - depth_ns), abs(stamp - rgb_ns)) > SYNC_NS:
            return None
        try:
            odom = capture.to_odom(capture.poses[stamp], capture.twists[stamp])
        except TransformException:
            return None
        check_images(depth, rgb)
        self.last_snapshot = max(depth_ns, rgb_ns)
        return depth, rgb, odom

    def next_snapshot(self, after, seconds=10.):
        data = None

        def ready():
            nonlocal data
            data = self.snapshot(after)
            return data is not None

        self.wait(ready, seconds)
        return data

    def infer(self, depth, rgb, odom, continuation=None):
        if self.first_images is None:
            self.first_images = depth, rgb
        tracker = self.tracker
        tracker.callback_odometry(odom)
        generated = self.get_clock().now().nanoseconds
        if continuation:
            epoch, duration_ns, axes, knots = continuation
            overlap = duration_ns // 2
            start = epoch + overlap
            if generated >= start:
                raise RuntimeError('Live observation missed the prior trajectory midpoint')
            tracker.desire_pos, tracker.desire_vel, tracker.desire_acc = boundary(axes, overlap / SECOND)
            tracker.last_yaw = sample_yaw(knots, overlap)
        else:
            start = generated + SECOND // 2

        previous = tracker.optimal_poly_x
        process_output = tracker.process_output
        max_confidence = None

        def observe_heads(endstates, targets, costs, scores, durations, **kwargs):
            nonlocal max_confidence
            max_confidence = float(np.max(scores))
            return process_output(endstates, targets, costs, scores, durations, **kwargs)

        wall_start = time.monotonic_ns()
        tracker.process_output = observe_heads
        try:
            tracker.callback_images(depth, rgb)
        finally:
            tracker.process_output = process_output
        selected = (tracker.optimal_poly_x is not previous and tracker.ekf_target is not None
                    and tracker.last_trajectory_tracking and not tracker.searching and not tracker.hovering)
        source = dict(rgb_ns=ns(rgb.header.stamp), depth_ns=ns(depth.header.stamp),
                      odom_ns=ns(odom.header.stamp))
        self.record('inference', **source, generated_ns=generated, selected=selected,
                    confirmed=tracker.ekf_target is not None, max_confidence=max_confidence,
                    inference_wall_ns=time.monotonic_ns() - wall_start)
        if not selected:
            return None
        axes = (tracker.optimal_poly_x, tracker.optimal_poly_y, tracker.optimal_poly_z)
        duration_ns = round(tracker.traj_time * SECOND)
        knots = frozen_yaw(tracker, axes, duration_ns)
        return (start, duration_ns, axes, knots), generated, source

    def wait_selected(self, after, continuation=None):
        finish = time.monotonic() + 10.
        while (remaining := finish - time.monotonic()) > 0:
            plan = self.infer(*self.next_snapshot(after, remaining), continuation)
            if plan is not None:
                return plan
        raise TimeoutError(f'No native tracking trajectory during {self.phase}')

    def save_first_images(self, output):
        if self.first_images is None:
            return
        depth, rgb = self.first_images
        color = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, 3)
        cv2.imwrite(str(output / 'first-rgb.png'), cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
        np.save(output / 'first-depth-m.npy',
                np.frombuffer(depth.data, np.float32).reshape(depth.height, depth.width))

    def observe(self, event):
        message = event.feedback
        if message.task_id == TASK and not message.waiting_for_start:
            revision = message.revision
            self.feedback[revision] = message
            if revision not in self.activation:
                self.activation[revision] = self.get_clock().now().nanoseconds
                self.record('active', revision=revision, elapsed_s=message.elapsed)

    def submit(self, selection, revision):
        curve, generated, source = selection
        start, duration_ns, axes, knots = curve
        if self.get_clock().now().nanoseconds >= start:
            raise RuntimeError(f'Revision {revision} finished after the proposed epoch')
        goal = timed_goal(axes, self.tracker.traj_time, knots, task_id=TASK,
                          vehicle_id='drone0', revision=revision, generated_ns=generated,
                          start_ns=start, max_speed=1.)
        self.record('submit', revision=revision, start_ns=start, duration_ns=duration_ns,
                    start_pva=[pva.tolist() for pva in boundary(axes, 0.)],
                    terminal_pva=[pva.tolist() for pva in boundary(axes, self.tracker.traj_time)],
                    **source)
        if revision == 1:
            self.reference_handle = self.run_action(self.action, goal, self.observe)
        else:
            request = FollowTimedPath_SendGoal.Request(goal=goal)
            if not self.response(self.modify.call_async(request)).accepted:
                raise RuntimeError('Default FollowTimedPath rejected native Modify')
        self.record('accepted', revision=revision)
        return curve

    def check_cancel_hold(self):
        last = None

        def observe(mode, odom_ns, age_ns, speed, good):
            nonlocal last
            row = (mode, odom_ns, good)
            if row != last:
                self.record('cancel_hold', control_mode=int(mode), odom_ns=odom_ns,
                            odom_age_ns=age_ns, speed_m_s=speed, eligible=good)
                last = row

        self.phase = 'hover after cancellation'
        self.settled(observe)
        self.evidence['settled_after_cancel'] = True

    def run_live(self, rpc, ground_only):
        self.preflight(rpc)
        self.phase = 'RGB-D calibration and optical TF'
        self.wait(lambda: len(self.capture.info) == 2 and CAMERA_FRAME in self.capture.transforms)
        self.capture.check_camera()
        self.evidence['camera_k'] = list(self.capture.info['rgb'].k)
        if ground_only:
            self.phase = 'ground inference'
            self.wait_selected(self.get_clock().now().nanoseconds)
            self.evidence['ground_selected'] = True
            return

        self.takeoff_and_settle()
        origin = self.odom.pose.pose.position
        self.evidence['origin_enu_m'] = [origin.x, origin.y, origin.z]
        collision_before = rpc.call('simGetCollisionInfo', 'PX4')['time_stamp']

        self.phase = 'first live trajectory'
        first = self.submit(self.wait_selected(self.get_clock().now().nanoseconds), 1)
        self.wait(lambda: 1 in self.activation and self.reference_samples)
        self.phase = 'rolling live trajectory'
        second = self.submit(self.wait_selected(self.activation[1], first), 2)
        self.wait(lambda: 2 in self.activation and self.feedback[2].elapsed >= .5)
        depth, rgb, _ = self.next_snapshot(self.activation[2])
        self.evidence['post_update_images'] = dict(rgb_ns=ns(rgb.header.stamp),
                                                   depth_ns=ns(depth.header.stamp))
        collision = rpc.call('simGetCollisionInfo', 'PX4')
        if collision['has_collided'] and collision['time_stamp'] != collision_before:
            raise RuntimeError('AirSim reported a collision during live execution')
        reference_counts = [
            sum(start <= ns(msg.header.stamp) < end and len(msg.setpoints) == 1
                for msg in self.reference_samples)
            for start, end in ((first[0], second[0]), (second[0], second[0] + second[1]))
        ]
        if not all(reference_counts):
            raise RuntimeError('No trajectory motion reference in one of the actual execution intervals')
        position = self.odom.pose.pose.position
        self.evidence.update(active_revisions=sorted(self.activation),
                             reference_samples=reference_counts,
                             last_enu_m=[position.x, position.y, position.z])
        self.phase = 'live trajectory cancellation'
        self.stop_reference()
        if self.evidence['reference_terminal'] != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('FollowTimedPath did not return CANCELED')
        self.check_cancel_hold()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--ground-only', action='store_true')
    args = parser.parse_args()
    args.output.mkdir()
    rclpy.init(args=[])
    capture = GroundCapture('yopo_tracker_live_observations')
    capture.pairs = deque(maxlen=1)
    capture.clock = deque(maxlen=1)
    executor = rclpy.get_global_executor()
    executor.add_node(capture)
    try:
        tracker, digest = load_tracker(Path('/opt/yopo-tracker/upstream'))
        tracker.timer_ctrl.cancel()
        try:
            if str(tracker.device) != 'cuda':
                raise RuntimeError('Live validation requires the native CUDA model')
            with (args.output / 'timeline.jsonl').open('w') as trace:
                node = LiveCheck(capture, tracker, trace)
                node.evidence.update(weight_sha256=digest, device=str(tracker.device))
                rpc = Rpc(args.host, args.port, 3)
                try:
                    node.run_live(rpc, args.ground_only)
                except Exception as error:
                    node.evidence['error'] = f'{node.phase}: {error}'
                    node.evidence['inputs'] = dict(clock=len(set(node.clock)), images=node.images,
                                                   odom=node.odom is not None,
                                                   controller=node.controller is not None,
                                                   camera_info=sorted(capture.info),
                                                   static_frames=sorted(capture.transforms),
                                                   pose_samples=len(capture.poses),
                                                   twist_samples=len(capture.twists))
                finally:
                    rpc.close()
                    try:
                        node.recover()
                    except Exception as error:
                        node.evidence['recovery_error'] = str(error)
                data = node.evidence
                data['passed'] = bool(data.get('disarmed') and not data.get('error') and
                                      not data.get('recovery_error') and
                                      (data.get('ground_selected') if args.ground_only else
                                       data.get('settled_after_cancel') and data.get('land')))
                (args.output / 'result.json').write_text(json.dumps(data, indent=2) + '\n')
                node.save_first_images(args.output)
                print(json.dumps(data))
                node.destroy_node()
                return 0 if data['passed'] else 1
        finally:
            tracker.destroy_node()
    finally:
        executor.remove_node(capture)
        capture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
