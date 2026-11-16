#!/usr/bin/env python3
"""Workstation orchestration example. No flight acceptance claim or platform setup."""
import argparse
import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time


def follow_point(own, target, distance):
    """Horizontal standoff; preserve measured own altitude, including inside radius."""
    if not all(math.isfinite(v) for v in (*own, *target, distance)) or distance < 0:
        raise ValueError('nonfinite geometry or negative distance')
    dx, dy = target[0] - own[0], target[1] - own[1]
    length = math.hypot(dx, dy)
    scale = max(0., length - distance) / length if length else 0.
    return own[0] + scale * dx, own[1] + scale * dy, own[2]


def fresh(stamp, received, now, wall, max_age):
    return (all(math.isfinite(v) for v in (stamp, received, now, wall, max_age))
            and stamp > 0 and 0 <= now - stamp <= max_age
            and 0 <= wall - received <= max_age)


def tracking_valid(state, received, now, wall, frame, max_age):
    return all(tracking_diagnostics(state, received, now, wall, frame, max_age)['predicates'].values())


def tracking_diagnostics(state, received, now, wall, frame, max_age):
    header, observation, until = map(seconds, (state.header.stamp,
                                             state.observation_stamp, state.valid_until))
    return dict(header_stamp=header, observation_stamp=observation, valid_until=until,
                ros_time=now, monotonic_time=wall, received_monotonic=received,
                receive_age=wall-received, header_age=now-header,
                observation_age=now-observation, frame=state.header.frame_id,
                status=state.status, capture_id=state.capture_id,
                predicates=dict(valid=bool(state.valid), frame=state.header.frame_id == frame,
                    header_fresh=fresh(header, received, now, wall, max_age),
                    observation_not_future=observation <= now, not_expired=now < until,
                    observation_positive=observation > 0))


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class Mailbox:
    def __init__(self):
        self.lock = threading.Lock()
        self.items = {}
        self.last_clock = None
        self.regressed = False

    def put(self, key, value):
        with self.lock:
            self.items[key] = (copy.deepcopy(value), time.monotonic())

    def clock(self, message):
        stamp = seconds(message.clock)
        with self.lock:
            if self.last_clock is not None and stamp < self.last_clock:
                self.regressed = True
            self.last_clock = stamp

    def snapshot(self):
        with self.lock:
            return dict(self.items), self.regressed


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute-navigation', action='store_true')
    p.add_argument('--drone-id', default='drone0')
    p.add_argument('--frame', default='earth')
    p.add_argument('--category', default='person')
    p.add_argument('--use-wall-time', action='store_true', help='default uses ROS simulation clock')
    for name, default in [('rate', 2.), ('distance', 5.), ('duration', 60.),
                          ('initial-wait', 15.), ('max-age', 1.),
                          ('target-timeout', 3.), ('response-timeout', 2.),
                          ('cleanup-timeout', 10.)]:
        p.add_argument('--' + name, type=float, default=default)
    p.add_argument('--output', type=Path)
    p.add_argument('--stop-file', type=Path, help='Local supervisor requests a normal observation stop')
    return p


def arguments(argv=None):
    p = parser()
    a = p.parse_args(argv)
    for name in ('rate', 'distance', 'duration', 'initial_wait', 'max_age',
                 'target_timeout', 'response_timeout', 'cleanup_timeout'):
        value = getattr(a, name)
        if not math.isfinite(value) or value <= 0:
            p.error(name + ' must be finite and positive')
    if not a.frame.strip() or not a.category.strip():
        p.error('frame and category required')
    if a.execute_navigation and 1 / a.rate >= a.target_timeout:
        p.error('update period must be shorter than target timeout')
    return a


def wait_future(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            raise TimeoutError('action response/result timeout')
        time.sleep(.01)
    return future.result()


def run(a, log, summary):
    # ROS imports stay local so geometry, freshness and argument tests need no ROS.
    import rclpy
    from as2_python_api.drone_interface_base import DroneInterfaceBase
    from as2_msgs.msg import PlatformInfo, PlatformStatus
    from geometry_msgs.msg import PoseStamped
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data

    drone = None
    track_submission = track_handle = track_result = None
    nav_attempted = False
    box = Mailbox()
    rclpy.init()
    try:
        # The AS2 auto-spin processes one callback per interval. Clock, duplicate
        # state subscriptions and tracking feedback exceed its default 20 Hz.
        drone = DroneInterfaceBase(a.drone_id, use_sim_time=not a.use_wall_time,
                                   spin_rate=1000.)
        drone.load_module('target_tracking')
        if a.execute_navigation:
            drone.load_module('navigation')
        log('modules_loaded', navigation_loaded=hasattr(drone, 'navigation'))
        drone.create_subscription(PoseStamped, 'self_localization/pose',
                                  lambda m: box.put('pose', m), qos_profile_sensor_data)
        drone.create_subscription(PlatformInfo, 'platform/info',
                                  lambda m: box.put('platform', m), qos_profile_sensor_data)
        if not a.use_wall_time:
            drone.create_subscription(Clock, '/clock', box.clock, qos_profile_sensor_data)
        ready_deadline = time.monotonic() + a.initial_wait
        while True:
            items, regressed = box.snapshot()
            if regressed:
                raise RuntimeError('ROS clock regression during readiness')
            pose, received = items.get('pose', (None, 0.))
            now = drone.get_clock().now().nanoseconds / 1e9
            if (drone.target_tracking.ready and pose is not None
                    and pose.header.frame_id == a.frame
                    and fresh(seconds(pose.header.stamp), received, now,
                              time.monotonic(), a.max_age)):
                log('inputs_ready', frame=a.frame, own_stamp=seconds(pose.header.stamp),
                    ros_time=now)
                break
            if time.monotonic() >= ready_deadline:
                raise TimeoutError('tracking server or fresh own pose unavailable')
            time.sleep(.05)
        def on_tracking(message):
            state = message.feedback.state
            box.put('tracking', state)
            log('tracking', capture_id=state.capture_id,
                observation_stamp=seconds(state.observation_stamp),
                stamp=seconds(state.header.stamp), frame=state.header.frame_id,
                valid_until=seconds(state.valid_until), received_monotonic=time.monotonic(),
                valid=state.valid, status=state.status, reason=state.reason,
                position=[state.position.x, state.position.y, state.position.z])

        track_submission = drone.target_tracking.track(
            a.category, timeout=a.duration,
            feedback_callback=on_tracking)
        track_handle = wait_future(track_submission, a.response_timeout)
        if not track_handle.accepted:
            raise RuntimeError('tracking goal rejected')
        track_result = track_handle.get_result_async()
        start = time.monotonic()
        acquired = False
        previous_now = None
        while drone.context.ok():
            items, regressed = box.snapshot()
            wall = time.monotonic()
            now = drone.get_clock().now().nanoseconds / 1e9
            if regressed or (previous_now is not None and now < previous_now):
                raise RuntimeError('ROS clock regression')
            previous_now = now
            if a.stop_file is not None and a.stop_file.exists():
                summary['reason'] = 'supervisor observation stop'
                break
            if track_result.done():
                result = track_result.result()
                if result.result.code == 'TIMEOUT' and wall - start >= a.duration - .05:
                    summary['reason'] = 'expected tracking duration elapsed'
                    break
                raise RuntimeError('tracking unexpectedly finished: ' + result.result.code + ': ' + result.result.reason)
            if wall - start >= a.duration:
                summary['reason'] = 'requested duration elapsed'
                break
            if nav_attempted and drone.navigation.done:
                raise RuntimeError('navigation unexpectedly finished: ' + drone.navigation.result.reason)
            tracking = items.get('tracking')
            valid = tracking is not None and tracking_valid(
                *tracking, now, wall, a.frame, a.max_age)
            if not valid:
                if tracking is not None:
                    log('tracking_check', **tracking_diagnostics(
                        *tracking, now, wall, a.frame, a.max_age))
                if acquired:
                    raise RuntimeError('target lost, invalid, expired, wrong frame or stale feedback')
                if wall - start >= a.initial_wait:
                    raise TimeoutError('no valid initial target')
                time.sleep(min(.05, 1 / a.rate))
                continue
            acquired = True
            pose, received = items.get('pose', (None, 0.))
            if (pose is None or pose.header.frame_id != a.frame
                    or not fresh(seconds(pose.header.stamp), received, now, wall, a.max_age)):
                log('pose_rejected', ros_time=now,
                    stamp=seconds(pose.header.stamp) if pose else None,
                    frame=pose.header.frame_id if pose else None,
                    receive_age=wall-received)
                raise RuntimeError('own pose missing, stale or wrong fixed frame')
            state = tracking[0]
            own = pose.pose.position
            point = follow_point((own.x, own.y, own.z),
                                 (state.position.x, state.position.y, state.position.z), a.distance)
            log('follow_point', point=point, ros_time=now, tracker_status=state.status,
                capture_id=state.capture_id, observation_stamp=seconds(state.observation_stamp),
                frame=a.frame, own=[own.x, own.y, own.z],
                own_stamp=seconds(pose.header.stamp),
                target=[state.position.x, state.position.y, state.position.z],
                raw_range=math.dist((own.x, own.y, own.z),
                                    (state.position.x, state.position.y, state.position.z)),
                distance=a.distance,
                tracking_check=tracking_diagnostics(*tracking, now, wall, a.frame, a.max_age),
                own_received_monotonic=received, own_receive_age=wall-received)
            if a.execute_navigation:
                platform, received = items.get('platform', (None, 0.))
                if (platform is None or not fresh(seconds(platform.header.stamp), received, now, wall, a.max_age)
                        or not (platform.connected and platform.armed and platform.offboard)
                        or platform.status.state != PlatformStatus.FLYING):
                    raise RuntimeError('fresh connected/armed/offboard/FLYING platform required')
                # Set before submission: even an uncertain response must enter stop cleanup.
                nav_attempted = True
                if not drone.navigation.to_point(
                        *point, frame_id=a.frame, wait=False, arrival_behavior='hold',
                        timeout=a.duration, target_timeout=a.target_timeout,
                        response_timeout=a.response_timeout):
                    raise RuntimeError('navigation update rejected')
                summary['motion_requested'] = True
                log('navigation_target_accepted', point=point)
            time.sleep(1 / a.rate)
        else:
            raise RuntimeError('ROS context stopped')
    except KeyboardInterrupt:
        summary['reason'] = 'interrupted'
        summary['exit_code'] = 130
    except Exception as error:
        summary['reason'] = str(error)
        summary['exit_code'] = 1
    finally:
        # Confirm navigation's terminal stop before cancelling the tracking producer.
        if drone is not None and nav_attempted:
            try:
                if not drone.navigation.cancel(timeout=a.cleanup_timeout):
                    raise RuntimeError('navigation cancel returned false')
                # A submission timeout may leave no owned handle. Public API cannot
                # confirm the late-goal stop in that case; never claim confirmation.
                if not drone.navigation.done or not drone.navigation.result.stop_confirmed:
                    raise RuntimeError('navigation stop unconfirmed (including uncertain submission)')
                summary['navigation_stop_confirmed'] = True
            except Exception as error:
                summary['navigation_stop_confirmed'] = False
                summary['cleanup_errors'].append(str(error))
                summary['exit_code'] = 1
        try:
            if track_handle is None and track_submission is not None:
                track_handle = wait_future(track_submission, a.cleanup_timeout)
            if track_handle is not None and track_handle.accepted:
                if track_result is None:
                    track_result = track_handle.get_result_async()
                if not track_result.done():
                    wait_future(track_handle.cancel_goal_async(), a.cleanup_timeout)
                result = wait_future(track_result, a.cleanup_timeout)
                summary['tracking_result'] = dict(code=result.result.code, reason=result.result.reason,
                                                  status=result.status)
        except Exception as error:
            summary['cleanup_errors'].append('tracking drain: ' + str(error))
            summary['exit_code'] = 1
        if drone is not None:
            drone.shutdown()
            drone.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    a = arguments(argv)
    output = a.output or Path(__file__).resolve().parents[4] / 'log' / 'target_tracking' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output.mkdir(parents=True, exist_ok=True)
    summary = dict(mode='execute' if a.execute_navigation else 'observe-only',
                   scope='observational/orchestration example; not flight acceptance',
                   motion_requested=False, navigation_stop_confirmed=None,
                   exit_code=0, cleanup_errors=[], reason='not started')
    with (output / 'mission.jsonl').open('w') as stream:
        log_lock = threading.Lock()
        def log(event, **fields):
            with log_lock:
                stream.write(json.dumps(dict(event=event, wall_time=time.time(), **fields), allow_nan=False) + '\n')
                stream.flush()
        log('start', mode=summary['mode'])
        run(a, log, summary)
        log('summary', **summary)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary))
    return summary['exit_code']


if __name__ == '__main__':
    raise SystemExit(main())
