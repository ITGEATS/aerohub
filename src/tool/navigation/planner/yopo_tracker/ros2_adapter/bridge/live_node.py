"""Use native YOPO-Tracker RGB-D planning with one default FollowTimedPath session."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import math
from pathlib import Path
from uuid import uuid4

from action_msgs.msg import GoalStatus
from as2_msgs.msg import ControllerInfo, ControlMode, PlatformInfo
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal

from .live_inputs import OBSERVATION_AGE_NS, ODOM_FRAME, TrackingInputs, ns
from .protected_trajectory import BrakeLimits, CONTROL_STEP_NS, HANDOFF_LEAD_NS, protect_curve
from .timed_points import timed_goal
from ..native.tracker_model import SECOND, boundary, frozen_yaw, load_tracker, sample_yaw


class LiveTracker(TrackingInputs):
    def __init__(self, tracker, limits, weight_sha256):
        super().__init__('yopo_tracker_live')
        self.tracker = tracker
        self.limits = limits
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.inference = None
        self.platform = self.controller = self.odom = None
        self.status = self.create_publisher(String, '~/status', 20)
        self.action = ActionClient(self, FollowTimedPath, '/drone0/follow_timed_path')
        self.modify = self.create_client(FollowTimedPath_SendGoal, '/drone0/follow_timed_path/_behavior/modify')
        self.create_service(Trigger, '~/start', self.start)
        self.create_service(Trigger, '~/stop', self.stop)
        for kind, topic, field in (
                (PlatformInfo, 'platform/info', 'platform'),
                (ControllerInfo, 'controller/info', 'controller'),
                (Odometry, 'sensor_measurements/odom', 'odom')):
            self.create_subscription(kind, f'/drone0/{topic}', lambda msg, f=field: setattr(self, f, msg),
                                     qos_profile_sensor_data)
        self.task = None
        self.brake_reason = None
        self.accepted = self.pending = self.result = None
        self.revision = self.active = 0
        self.last_input_ns = self.low_speed_since = None
        self.finalizing = False
        self.create_timer(CONTROL_STEP_NS / SECOND, self.tick)
        self.record('ready', device=str(tracker.device), weight_sha256=weight_sha256)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def record(self, event, **fields):
        record = json.dumps(dict(event=event, sim_ns=self.now_ns(), task_id=self.task, **fields),
                            allow_nan=False)
        self.get_logger().info(record)
        self.status.publish(String(data=record))

    def start(self, _, reply):
        if self.task or self.inference:
            reply.message = 'A tracking or stopping session is still present'
            return reply
        if (not self.platform or not self.platform.armed or not self.platform.offboard or
                not self.controller or self.controller.input_control_mode.control_mode not in
                (ControlMode.HOVER, ControlMode.POSITION) or
                not self.action.server_is_ready() or not self.modify.service_is_ready()):
            reply.message = 'Require airborne hover, offboard and FollowTimedPath'
            return reply
        try:
            self.check_camera()
        except (AssertionError, ValueError, KeyError) as error:
            reply.message = f'RGB-D calibration unavailable: {error}'
            return reply
        self.tracker.reset_state()
        self.task = f'yopo-tracker-{uuid4().hex}'
        self.revision = self.active = 0
        self.accepted = self.pending = self.result = None
        self.brake_reason = self.low_speed_since = None
        self.finalizing = False
        self.last_input_ns = self.now_ns()
        self.last_snapshot = self.last_input_ns
        self.record('started')
        reply.success, reply.message = True, self.task
        return reply

    def stop(self, _, reply):
        self.brake('REQUESTED_STOP')
        reply.success, reply.message = True, 'Stop latched'
        return reply

    def brake(self, reason):
        if self.task and self.brake_reason is None:
            self.brake_reason = reason
            self.record('brake_latched', reason=reason, revision=self.revision)

    def feedback(self, event):
        msg = event.feedback
        if msg.task_id == self.task and not msg.waiting_for_start and msg.revision > self.active:
            self.active = msg.revision
            self.record('active', revision=self.active, elapsed_s=msg.elapsed)

    def select(self, depth, rgb, odom, anchor):
        tracker = self.tracker
        tracker.desire_init = False
        tracker.callback_odometry(odom)
        if anchor:
            tracker.desire_pos, tracker.desire_vel, tracker.desire_acc, tracker.last_yaw = anchor
            tracker.desire_init = True
        previous = tracker.optimal_poly_x
        tracker.callback_images(depth, rgb)
        if tracker.ekf_target is None or tracker.searching or tracker.hovering:
            return None, None, 'NATIVE_HOVER' if tracker.hovering else 'TARGET_LOST'
        target = tracker.ekf_target.tolist()
        if tracker.optimal_poly_x is previous or not tracker.last_trajectory_tracking:
            return None, target, None
        axes = (tracker.optimal_poly_x, tracker.optimal_poly_y, tracker.optimal_poly_z)
        duration_ns = round(tracker.traj_time * SECOND)
        curve = protect_curve(axes, frozen_yaw(tracker, axes, duration_ns), duration_ns, self.limits)
        return curve, target, None

    def submit(self, now, start, curve, source):
        revision = self.revision + 1
        goal = timed_goal(curve.axes, curve.duration_ns / SECOND, curve.knots, task_id=self.task,
                          vehicle_id='drone0', revision=revision, generated_ns=now,
                          start_ns=start, max_speed=1.)
        self.record('submit', revision=revision, start_ns=start, brake_from_ns=start + curve.brake_from_ns,
                    brake_end_ns=start + curve.brake_end_ns, reserve_m=curve.reserve_m, **source)
        future = (self.action.send_goal_async(goal, feedback_callback=self.feedback) if revision == 1 else
                  self.modify.call_async(FollowTimedPath_SendGoal.Request(goal=goal)))
        self.pending = future, goal, start, curve

    def receive(self):
        if self.pending is None or not self.pending[0].done():
            return
        future, goal, start, curve = self.pending
        self.pending = None
        reply = future.result()
        if not reply.accepted:
            self.record('rejected', revision=goal.path.revision)
            self.brake('EXECUTION_REJECTED')
            return
        self.revision = goal.path.revision
        if self.revision == 1:
            self.result = reply.get_result_async()
        self.accepted = goal, start, curve
        self.record('accepted', revision=self.revision, final=goal.path.is_final)

    def finish(self, now):
        if self.result and self.result.done():
            answer = self.result.result()
            self.record('finished', success=bool(answer.status == GoalStatus.STATUS_SUCCEEDED and
                                                answer.result.success and self.finalizing and
                                                answer.result.revision == self.active == self.revision),
                        reason=answer.result.reason, revision=answer.result.revision)
            self.task = self.accepted = self.pending = self.result = None
            return
        if self.accepted is None:
            if self.pending is None:
                self.record('finished', success=False, reason=self.brake_reason, revision=0)
                self.task = None
            return
        if self.pending or self.finalizing:
            return
        goal, start, curve = self.accepted
        if self.active != self.revision or now < start + curve.brake_end_ns:
            return
        velocity = self.odom.twist.twist.linear if self.odom else None
        speed = math.hypot(velocity.x, velocity.y, velocity.z) if velocity else math.inf
        if (not self.odom or self.odom.header.frame_id != ODOM_FRAME or
                not 0 <= now - ns(self.odom.header.stamp) < SECOND // 2 or speed >= .2 or
                not self.controller or self.controller.input_control_mode.control_mode != ControlMode.TRAJECTORY):
            self.low_speed_since = None
            return
        if self.low_speed_since is None:
            self.low_speed_since = now
        if now - self.low_speed_since < SECOND:
            return
        epoch = now + HANDOFF_LEAD_NS
        if epoch > start + curve.duration_ns:
            return  # The accepted goal will report its coverage failure.
        final = deepcopy(goal)
        path = final.path
        path.revision += 1
        path.is_final = True
        path.generated_at.sec, path.generated_at.nanosec = divmod(now, SECOND)
        path.valid_from.sec, path.valid_from.nanosec = divmod(epoch, SECOND)
        first = deepcopy(path.points[-1])
        first.time_from_start.sec = first.time_from_start.nanosec = 0
        last = deepcopy(first)
        last.time_from_start.sec = 1
        path.points = [first, last]
        self.finalizing = True
        self.record('brake_settled', revision=self.revision, speed_m_s=speed)
        self.pending = self.modify.call_async(FollowTimedPath_SendGoal.Request(goal=final)), final, epoch, curve

    def tick(self):
        now = self.now_ns()
        self.receive()
        if self.task and self.result and self.result.done():
            self.finish(now)
        if not self.task:
            if self.inference and self.inference[0].done():
                self.inference = None
            return
        snapshot = None if self.brake_reason else self.snapshot(0)
        if snapshot:
            depth, rgb, odom = snapshot
            observed = dict(depth_ns=ns(depth.header.stamp), rgb_ns=ns(rgb.header.stamp),
                            odom_ns=ns(odom.header.stamp))
            self.last_input_ns = min(observed['depth_ns'], observed['rgb_ns'])
        if self.accepted:
            _, start, curve = self.accepted
            if now - self.last_input_ns >= OBSERVATION_AGE_NS:
                self.brake('RGBD_TIMEOUT')
            elif now >= start + curve.brake_from_ns:
                self.brake('BRAKE_RESERVE')
        if self.brake_reason:
            self.finish(now)
            return
        if self.inference and self.inference[0].done():
            future, start, source = self.inference
            self.inference = None
            try:
                curve, target, reason = future.result()
            except Exception as error:
                self.record('inference_error', error=str(error))
                self.brake('INFERENCE_FAILED')
                return
            self.record('inference', selected=curve is not None, target_enu=target, reason=reason, **source)
            if reason and self.accepted:
                self.brake(reason)
                return
            if curve and now < start and now - min(source['depth_ns'], source['rgb_ns']) <= OBSERVATION_AGE_NS:
                self.submit(now, start, curve, source)
                return
        if snapshot is None:
            return
        if self.pending or self.inference or self.active != self.revision:
            return
        start, anchor = now + SECOND // 2, None
        if self.accepted:
            _, epoch, current = self.accepted
            # The splice already belongs to the accepted curve. Plan toward
            # that fixed epoch so inference and DDS share its full lead.
            start = epoch + current.brake_from_ns
            if start - now < HANDOFF_LEAD_NS:
                return  # The already accepted brake covers this handoff.
            anchor = (*boundary(current.axes, current.brake_from_ns / SECOND),
                      sample_yaw(current.knots, current.brake_from_ns))
        self.inference = self.worker.submit(self.select, depth, rgb, odom, anchor), start, observed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--brake-calibration', required=True, type=Path)
    parser.add_argument('--upstream', type=Path, default=Path('/opt/yopo-tracker/upstream'))
    args = parser.parse_args()
    limits = BrakeLimits.from_calibration(json.loads(args.brake_calibration.read_text()))
    rclpy.init(args=[])
    tracker, digest = load_tracker(args.upstream)
    tracker.timer_ctrl.cancel()
    node = LiveTracker(tracker, limits, digest)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.worker.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        tracker.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
