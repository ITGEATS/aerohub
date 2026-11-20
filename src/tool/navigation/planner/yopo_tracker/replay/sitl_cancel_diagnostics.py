"""Compare immediate Cancel and a final braking path on a dedicated AirSim PX4 SITL."""
import argparse
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from as2_msgs.msg import ControllerInfo, ControlMode
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from uav_as2_sensor_adapter.rpc import Rpc
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal

from check_action import SECOND, boundary, ns
from forward_brake_fixture import (
    CONTROL_PERIOD_NS, FIXTURE_DISTANCE_M, HANDOFF_LEAD_NS, CollisionWatch,
    NoBrakingPath, check_front_clearance, forecast_stop, select_measured_brake,
)
from sitl_fixed_reference import Check
from timed_points import timed_goal


TASK = 'sitl-fixed-cancel-diagnostics'
MATCH_SPEED = 1.0752578498097947  # First post-cancel odometry speed in the live comparison.
STOP_BUDGET_S = 10.  # Existing stop confirmation budget, measured in wall time.


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def orientation(pose):
    q = pose.orientation
    return Rotation.from_quat([q.x, q.y, q.z, q.w])


@dataclass
class PreparedBrake:
    generated_ns: int
    start_ns: int
    duration_ns: int
    terminal: np.ndarray
    details: dict


class CancelCheck(Check):
    def __init__(self, log):
        super().__init__()
        self.log = log
        self.recording = False
        self.commands = set()
        self.motion_history = deque(maxlen=200)
        self.collision_watch = None
        self.reserve_axis = None
        self.reserve_peak_m = 0.
        self.create_subscription(Odometry, '/drone0/sensor_measurements/odom',
                                 self.record_odometry, qos_profile_sensor_data)
        self.create_subscription(ControllerInfo, '/drone0/controller/info',
                                 self.record_mode, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/drone0/actuator_command/twist',
                                 self.record_twist, qos_profile_sensor_data)
        self.create_subscription(TrajectorySetpoint, '/drone0/fmu/in/trajectory_setpoint',
                                 self.record_px4_dds, qos_profile_sensor_data)
        self.create_subscription(OffboardControlMode, '/drone0/fmu/in/offboard_control_mode',
                                 self.record_offboard, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, '/drone0/fmu/out/vehicle_local_position_v1',
                                 self.record_px4_state, qos_profile_sensor_data)

    def record(self, event, **fields):
        data = dict(event=event, sim_ns=self.get_clock().now().nanoseconds,
                    wall_monotonic_ns=time.monotonic_ns(), **fields)
        self.log.write(json.dumps(data, allow_nan=False) + '\n')

    def record_odometry(self, msg):
        if self.recording:
            pose = msg.pose.pose
            stamp = ns(msg.header.stamp)
            velocity = orientation(pose).apply(xyz(msg.twist.twist.linear))
            if not self.motion_history or stamp > self.motion_history[-1][0]:
                self.motion_history.append((stamp, velocity))
            if self.reserve_axis is not None:
                direction, origin_m = self.reserve_axis
                progress = float(np.asarray(xyz(pose.position)) @ direction) - origin_m
                self.reserve_peak_m = max(self.reserve_peak_m, progress)
            self.record('odometry', stamp_ns=stamp, position_enu_m=xyz(pose.position),
                        velocity_enu_m_s=velocity.tolist())

    def wait(self, predicate, seconds=10):
        def ready():
            if self.collision_watch and self.reference_handle is not None:
                if self.collision_watch.error:
                    raise RuntimeError('AirSim contact monitor stopped during motion')
                if self.collision_watch.contacts:
                    raise RuntimeError('AirSim reported contact during the motion case')
            return predicate()

        super().wait(ready, seconds)

    def record_mode(self, msg):
        if self.recording:
            self.record('controller_mode', stamp_ns=ns(msg.header.stamp),
                        input_mode=msg.input_control_mode.control_mode,
                        output_mode=msg.output_control_mode.control_mode)

    def record_twist(self, msg):
        if self.recording:
            self.commands.add('as2_twist')
            self.record('as2_twist', stamp_ns=ns(msg.header.stamp), frame=msg.header.frame_id,
                        velocity_m_s=xyz(msg.twist.linear))

    def record_px4_dds(self, msg):
        if self.recording:
            self.commands.add('px4_dds_setpoint')

            def optional(vector):
                return [float(value) if math.isfinite(value) else None for value in vector]

            self.record('px4_dds_setpoint', source_us=int(msg.timestamp),
                        position_ned_m=optional(msg.position), velocity_ned_m_s=optional(msg.velocity),
                        acceleration_ned_m_s2=optional(msg.acceleration))

    def record_offboard(self, msg):
        if self.recording:
            self.record('px4_dds_mode', source_us=int(msg.timestamp),
                        position=msg.position, velocity=msg.velocity)

    def record_px4_state(self, msg):
        if self.recording and msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid:
            self.record('px4_dds_local_position', source_us=int(msg.timestamp),
                        position_ned_m=[msg.x, msg.y, msg.z], velocity_ned_m_s=[msg.vx, msg.vy, msg.vz])

    def submit_update(self, goal, deadline):
        request = FollowTimedPath_SendGoal.Request(goal=goal)
        reply = self.response(self.modify.call_async(request), deadline - time.monotonic())
        if not reply.accepted:
            raise RuntimeError(f'FollowTimedPath rejected revision {goal.path.revision}')
        self.record('brake_accepted', revision=goal.path.revision)

    def wait_before_final(self, goal, original_end, deadline, result_future):
        self.phase = 'holding the zero-velocity trajectory'
        self.wait(lambda: (2 in self.feedback and self.get_clock().now().nanoseconds >= original_end)
                  or result_future.done(), deadline - time.monotonic())

        def observe(mode, stamp, age, speed, good):
            if result_future.done():
                raise RuntimeError('Braking goal terminated before low speed was confirmed')
            self.record('brake_stop_check', controller_input_mode=mode, odom_ns=stamp,
                        odom_age_ns=age, speed_m_s=speed, eligible=good)

        self.settled(observe, seconds=deadline - time.monotonic(), modes=(ControlMode.TRAJECTORY,))
        self.evidence['brake_low_speed_confirmed'] = True
        self.record('brake_low_speed_confirmed',
                    speed_m_s=math.hypot(*xyz(self.odom.twist.twist.linear)))
        self.phase = 'final hold Modify'
        # The pending non-final segment is already stationary at the handoff.
        # A separate final revision allows the Behavior to exit at low speed.
        final = deepcopy(goal)
        path = final.path
        generated = self.get_clock().now().nanoseconds
        start = generated + HANDOFF_LEAD_NS
        coverage_end = ns(path.valid_from) + ns(path.points[-1].time_from_start)
        if start > coverage_end:
            raise RuntimeError('Stationary reference no longer covers the final handoff')
        path.generated_at.sec, path.generated_at.nanosec = divmod(generated, SECOND)
        path.valid_from.sec, path.valid_from.nanosec = divmod(start, SECOND)
        path.revision = 3
        path.is_final = True
        first = deepcopy(path.points[-1])
        first.time_from_start.sec = first.time_from_start.nanosec = 0
        last = deepcopy(first)
        last.time_from_start.sec = 1
        path.points = [first, last]
        self.record('brake_finalize', revision=3, start_ns=start)
        self.submit_update(final, deadline)

    def brake_measurement(self, now, direction):
        measured = self.odom
        stamp = ns(measured.header.stamp)
        earlier = [sample for sample in self.motion_history if sample[0] < stamp]
        if not earlier or not 0 <= now - stamp < SECOND // 2:
            return None
        prior_stamp, prior_velocity = min(earlier, key=lambda item: abs(item[0] - stamp + HANDOFF_LEAD_NS))
        speed = float(orientation(measured.pose.pose).apply(xyz(measured.twist.twist.linear)) @ direction)
        acc = (speed - float(prior_velocity @ direction)) * SECOND / (stamp - prior_stamp)
        position = float(np.asarray(xyz(measured.pose.pose.position)) @ direction)
        return stamp, position, speed, acc

    def reserve_brake(self, axes, epoch, coverage_end, direction, reserve_end, calibration):
        self.phase = 'forward brake reserve'
        bound = float(reserve_end @ direction)
        last_stamp = None
        backup = selected = None

        def check():
            nonlocal last_stamp, backup, selected
            now = self.get_clock().now().nanoseconds
            if 1 not in self.feedback:
                return False
            measurement = self.brake_measurement(now, direction)
            if measurement is None or measurement[0] == last_stamp:
                return False
            stamp, position, speed, acc = measurement
            last_stamp = stamp

            def prepare(start):
                duration, terminal, details = select_measured_brake(
                    *boundary(axes, (start - epoch) / SECOND), reserve_end, direction,
                    coverage_end - start, position, speed, acc, (start - stamp) / SECOND, calibration)
                details.update(measured_position_m=position, measured_speed_m_s=speed,
                               measured_acceleration_m_s2=acc, measured_odom_ns=stamp)
                return PreparedBrake(now, start, duration, terminal, details)

            start = now + HANDOFF_LEAD_NS
            next_start = start + CONTROL_PERIOD_NS
            _, next_stop = forecast_stop(
                position, speed, acc, (next_start - stamp) / SECOND, calibration['deceleration_m_s2'])
            next_feasible = False
            try:
                backup = prepare(start)
            except NoBrakingPath:
                if backup is None or backup.start_ns <= now:
                    raise NoBrakingPath('No prepared braking path remains valid for handoff')
                reason = 'current_brake_infeasible_using_previous'
            else:
                try:
                    prepare(next_start)
                except NoBrakingPath:
                    reason = 'next_brake_infeasible'
                else:
                    next_feasible = True
                    if next_stop < bound:
                        return False
                    reason = 'predicted_stop_reaches_reserved_bound'
            selected = backup
            _, origin_m = self.reserve_axis
            self.record('brake_reserve_trigger', reason=reason, selected_start_ns=selected.start_ns,
                        odom_ns=stamp, measured_progress_m=position - origin_m,
                        measured_speed_m_s=speed, next_brake_feasible=next_feasible,
                        projected_next_stop_m=next_stop - origin_m, reference_limit_m=bound - origin_m)
            self.evidence['reserve_trigger'] = reason
            return True

        self.wait(check)
        return selected

    def complete_brake(self, solver, axes, first, yaw, deadline, wait_for_low_speed,
                       direction, calibration, prepared=None):
        self.phase = 'braking Modify'
        original_start = ns(first.path.valid_from)
        original_end = original_start + ns(first.path.points[-1].time_from_start)
        now = self.get_clock().now().nanoseconds
        generated, start = (prepared.generated_ns, prepared.start_ns) if prepared else (now, now + HANDOFF_LEAD_NS)
        braking_ns = original_end - start
        if start <= now or braking_ns <= 0:
            raise RuntimeError('The current reference has no remaining brake coverage')
        position, velocity, acceleration = boundary(axes, (start - original_start) / SECOND)
        end_bound = np.array(xyz(first.path.points[-1].position))
        details = {}
        if prepared:
            braking_ns, end, details = prepared.duration_ns, prepared.terminal, prepared.details
        elif calibration:
            measurement = self.brake_measurement(generated, direction)
            if measurement is None:
                raise RuntimeError('No fresh velocity history for the braking decision')
            stamp, actual_progress, speed, actual_acc = measurement
            braking_ns, end, details = select_measured_brake(
                position, velocity, acceleration, end_bound, direction, braking_ns,
                actual_progress, speed, actual_acc, (start - stamp) / SECOND, calibration)
            details.update(measured_position_m=actual_progress, measured_speed_m_s=speed,
                           measured_acceleration_m_s2=actual_acc)
        else:
            # Integrate a cubic Hermite velocity ending at zero speed and acceleration.
            duration = braking_ns / SECOND
            end = position + duration * velocity / 2 + duration**2 * acceleration / 12
        duration = braking_ns / SECOND
        along = float(velocity @ direction)
        if (not float(position @ direction) <= float(end @ direction) <= float(end_bound @ direction) or
                along < 0. or 3 * along + float(acceleration @ direction) * duration < 0.):
            raise RuntimeError('The calculated braking path leaves the fixed test segment')
        brake_end = start + braking_ns
        braking_axes = tuple(solver(p, v, a, target, 0., 0., duration)
                             for p, v, a, target in zip(position, velocity, acceleration, end))
        goal = timed_goal(braking_axes, duration, [(0, yaw), (braking_ns, yaw)],
                          task_id=TASK, vehicle_id='drone0', revision=2, generated_ns=generated,
                          start_ns=start, max_speed=first.path.max_speed)
        goal.path.is_final = not wait_for_low_speed
        stopped = goal.path.points[-1]
        stopped.position.x, stopped.position.y, stopped.position.z = (float(value) for value in end)
        stopped.velocity.x = stopped.velocity.y = stopped.velocity.z = 0.
        stopped.acceleration.x = stopped.acceleration.y = stopped.acceleration.z = 0.
        hold = deepcopy(stopped)
        hold_ns = round(STOP_BUDGET_S * SECOND) if wait_for_low_speed else SECOND
        hold.time_from_start.sec, hold.time_from_start.nanosec = divmod(braking_ns + hold_ns, SECOND)
        goal.path.points.append(hold)
        self.record('brake_submit', revision=2, start_ns=start, brake_end_ns=brake_end,
                    coverage_end_ns=brake_end + hold_ns, terminal_enu_m=end.tolist(),
                    boundary_pva=[state.tolist() for state in (position, velocity, acceleration)], **details)
        result_future = self.reference_handle.get_result_async()
        self.submit_update(goal, deadline)
        revision = 2
        if wait_for_low_speed:
            self.wait_before_final(goal, brake_end, deadline, result_future)
            revision = 3
        self.phase = 'braking reference execution'
        terminal = self.response(result_future, deadline - time.monotonic())
        self.reference_handle = None
        self.evidence.update(reference_terminal=terminal.status,
                             reference_reason=terminal.result.reason,
                             reference_revision=int(terminal.result.revision))
        self.record('brake_result', status=terminal.status, reason=terminal.result.reason,
                    revision=int(terminal.result.revision))
        if (terminal.status != GoalStatus.STATUS_SUCCEEDED or not terminal.result.success or
                terminal.result.revision != revision or not {2, revision} <= self.feedback.keys()):
            raise RuntimeError('FollowTimedPath did not execute and complete the braking revision')

    def run_cancel(self, rpc, solver, stop_method, forward, calibration):
        self.preflight(rpc)
        self.takeoff_and_settle()
        pose = self.odom.pose.pose
        origin = xyz(pose.position)
        yaw = float(orientation(pose).as_euler('ZYX')[0])
        direction = np.array([math.cos(yaw), math.sin(yaw), 0.] if forward else [1., 0., 0.])
        if forward:
            self.phase = 'forward corridor clearance'
            clearance = check_front_clearance(rpc, direction)
            self.evidence.update(clearance)
            self.record('corridor_clearance', **clearance)
            if clearance['measured_clearance_m'] < clearance['required_clearance_m']:
                raise RuntimeError('Forward test corridor does not cover the planned and braking motion')
        collision_before = rpc.call('simGetCollisionInfo', 'PX4')
        duration = 2.8
        end = (np.array(origin) + FIXTURE_DISTANCE_M * direction).tolist()
        reserve_end = None
        if stop_method == 'reserved-brake':
            # Clearance already covers the entire fixed test segment. Keep the
            # previously observed forward overrun inside that segment.
            margin = calibration['observed_terminal_overrun_m']
            reserve_end = np.asarray(end) - margin * direction
            self.reserve_axis = (direction, float(np.asarray(origin) @ direction))
            self.evidence.update(reserve_limit_m=FIXTURE_DISTANCE_M, reserve_margin_m=margin)
        axes = tuple(solver(start, 0., 0., target, 0., 0., duration)
                     for start, target in zip(origin, end))
        generated = self.get_clock().now().nanoseconds
        epoch = generated + SECOND // 2
        knots = [(0, yaw), (round(duration * SECOND), yaw)]
        goal = timed_goal(axes, duration, knots, task_id=TASK, vehicle_id='drone0', revision=1,
                          generated_ns=generated, start_ns=epoch, max_speed=1.)
        self.recording = True
        self.record('fixture', epoch_ns=epoch, duration_s=duration, start_enu_m=origin,
                    end_enu_m=end, yaw_rad=yaw, direction_enu=direction.tolist(),
                    match_speed_m_s=MATCH_SPEED)
        self.record('collision_snapshot', phase='before_reference',
                    stamp=collision_before['time_stamp'], flag=collision_before['has_collided'],
                    object_name=collision_before['object_name'], object_id=collision_before['object_id'])
        if forward:
            self.collision_watch = CollisionWatch(*rpc.address, collision_before['time_stamp'])
        self.phase = 'fixed reference execution'

        def feedback(event):
            message = event.feedback
            if message.task_id == TASK and not message.waiting_for_start:
                if message.revision > 1 and message.revision not in self.feedback:
                    self.record('brake_active', revision=message.revision, elapsed_s=message.elapsed)
                self.feedback[message.revision] = message

        self.reference_handle = self.run_action(self.action, goal, feedback)

        def reached_trigger():
            if not self.feedback:
                return False
            speed = math.hypot(*xyz(self.odom.twist.twist.linear))
            # Allow a late response around peak speed. Bound the test stimulus by
            # the fixed curve even if the prior observed cancellation speed is missed.
            return speed >= MATCH_SPEED or self.feedback[1].elapsed >= 2.

        prepared = None
        if reserve_end is None:
            self.wait(reached_trigger)
        else:
            prepared = self.reserve_brake(axes, epoch, epoch + knots[-1][0], direction, reserve_end, calibration)
        stop_started = time.monotonic()
        deadline = stop_started + STOP_BUDGET_S
        at_cancel = self.odom
        speed = math.hypot(*xyz(at_cancel.twist.twist.linear))
        self.evidence.update(cancel_ns=self.get_clock().now().nanoseconds,
                             cancel_speed_m_s=speed, matched_speed=speed >= MATCH_SPEED,
                             cancel_elapsed_s=self.feedback[1].elapsed)
        event = 'cancel_request' if stop_method == 'cancel' else 'controlled_stop_request'
        self.record(event, **self.evidence)
        self.log.flush()
        last = None

        def observe(mode, stamp, age, speed, good):
            nonlocal last
            sample = (mode, stamp, good)
            if sample != last:
                last = sample
                self.record('stop_check', controller_input_mode=mode, odom_ns=stamp,
                            odom_age_ns=age, speed_m_s=speed, eligible=good)

        try:
            if stop_method != 'cancel':
                self.complete_brake(solver, axes, goal, yaw, deadline,
                                    stop_method != 'brake', direction, calibration, prepared)
            else:
                self.phase = 'fixed reference cancellation'
                self.stop_reference()
                self.record('cancel_result', status=self.evidence['reference_terminal'],
                            reason=self.evidence['reference_reason'])
                if self.evidence['reference_terminal'] != GoalStatus.STATUS_CANCELED:
                    raise RuntimeError('FollowTimedPath did not return CANCELED')
            self.phase = 'hover after stop'
            # Retain the original immediate-cancel case. For a controlled stop,
            # the braking revision and the actual hold share the old budget.
            budget = STOP_BUDGET_S if stop_method == 'cancel' else deadline - time.monotonic()
            self.settled(observe, seconds=budget)
            self.evidence['settled_after_cancel'] = True
        finally:
            self.evidence['stop_wall_elapsed_s'] = time.monotonic() - stop_started
            try:
                collision = rpc.call('simGetCollisionInfo', 'PX4')
                # This RPC clears has_collided on read. The timestamp of the
                # most recent contact persists even when another reader cleared the flag.
                self.evidence['new_collision'] = collision['time_stamp'] != collision_before['time_stamp']
                self.record('collision_snapshot', phase='after_stop',
                            stamp=collision['time_stamp'], flag=collision['has_collided'],
                            object_name=collision['object_name'], object_id=collision['object_id'])
            except Exception as error:
                self.evidence['collision_check_error'] = str(error)
            self.record('hover_end', settled=self.evidence.get('settled_after_cancel', False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--stop-method',
                        choices=('cancel', 'brake', 'brake-gated', 'measured-brake', 'reserved-brake'),
                        default='cancel')
    parser.add_argument('--forward-fixture', action='store_true')
    parser.add_argument('--calibration', type=Path)
    args = parser.parse_args()
    calibration = None
    if args.stop_method in ('measured-brake', 'reserved-brake'):
        if not args.forward_fixture or args.calibration is None:
            parser.error('Measured braking requires the forward fixture and its stopping calibration')
        calibration = json.loads(args.calibration.read_text())
        for field in ('deceleration_m_s2', 'reference_acceleration_m_s2', 'reference_jerk_m_s3'):
            if not math.isfinite(calibration[field]) or calibration[field] <= 0.:
                parser.error(f'Invalid braking calibration field {field}')
        if args.stop_method == 'reserved-brake':
            margin = calibration.get('observed_terminal_overrun_m', math.nan)
            if not 0. < margin < FIXTURE_DISTANCE_M:
                parser.error('The observed overrun must fit inside the fixed forward segment')
    args.output.mkdir()
    sys.path.insert(0, '/opt/yopo-tracker/upstream/YOPO')
    from policy.poly_solver import Poly5Solver

    rclpy.init(args=[])
    with (args.output / 'timeline.jsonl').open('w') as log:
        node = CancelCheck(log)
        node.evidence['stop_method'] = args.stop_method
        rpc = Rpc(args.host, args.port, 3)
        try:
            try:
                node.run_cancel(rpc, Poly5Solver, args.stop_method, args.forward_fixture, calibration)
            except Exception as error:
                node.evidence['error'] = f'{node.phase}: {error}'
            finally:
                if node.collision_watch:
                    watch = node.collision_watch
                    node.collision_watch = None
                    watch.close()
                    node.evidence['collision_watch_count'] = len(watch.contacts)
                    if watch.contacts:
                        node.evidence['new_collision'] = True
                    for contact in watch.contacts:
                        node.record('collision_event', stamp=contact['time_stamp'],
                                    object_name=contact['object_name'], object_id=contact['object_id'])
                    if watch.error:
                        node.evidence['collision_watch_error'] = watch.error
                node.recording = False
                log.flush()
                rpc.close()
                try:
                    node.recover()
                except Exception as error:
                    node.evidence['recovery_error'] = str(error)
            data = node.evidence
            data['command_topics'] = sorted(node.commands)
            if node.reserve_axis is not None:
                data['reserve_observed_peak_progress_m'] = node.reserve_peak_m
                data['reserve_contained'] = node.reserve_peak_m <= data['reserve_limit_m']
            data['passed'] = bool(data.get('settled_after_cancel') and data.get('land')
                                  and data.get('disarmed') and data.get('new_collision') is False
                                  and (args.stop_method != 'reserved-brake' or data.get('reserve_contained'))
                                  and not data.get('error') and not data.get('collision_watch_error'))
            (args.output / 'result.json').write_text(json.dumps(data, indent=2) + '\n')
            print(json.dumps(data))
            return 0 if data['passed'] else 1
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
