#!/usr/bin/env python3
"""Workstation mission: System + Tracking + Navigation Tool calls only.

Requires separately deployed Tools and sensors. Scene/runner RPC belongs to the
simulator fixture. This initial pursuit uses fixed yaw (Tracking follow=False).
"""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time

from follow_policy import destination, follow_offset, recovery_destination
from observation_policy import select_observation_point
from tangent_policy import TangentReference
from observation_recovery import ObservationRecovery


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--drone-id', default='drone0')
    parser.add_argument('--duration', type=float, default=20., help='0 runs until canceled')
    parser.add_argument('--observation-grace',type=float,default=2.)
    parser.add_argument('--recovery-window',type=float,default=.5)
    parser.add_argument('--tangent-lead',type=float,default=.35)
    parser.add_argument('--tangent-follow',action='store_true')
    parser.add_argument('--predictive-observation',action='store_true')
    parser.add_argument('--continuous-yaw',action='store_true')
    parser.add_argument('--target-yaw',action='store_true')
    parser.add_argument('--height', type=float, default=2.)
    parser.add_argument('--distance', type=float, default=8.)
    parser.add_argument('--update-period', type=float, default=.25)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('duration must be finite and nonnegative')
    # Tool Actions require finite positive timeouts; keep them beyond any practical run.
    tool_timeout = args.duration+30. if args.duration else 1.e12
    for name in ('height', 'distance', 'update_period'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(name+' must be finite and positive')
    if args.update_period >= 1.:
        parser.error('update-period must be below the 1s Navigation target timeout')
    if not math.isfinite(args.recovery_window) or not .5<=args.recovery_window<=2.:
        parser.error('recovery-window must be within .5..2 seconds')
    if args.tangent_follow and args.predictive_observation:
        parser.error('Choose one experimental follow policy')
    if not math.isfinite(args.observation_grace) or not 0<args.observation_grace<=5.:
        parser.error('observation-grace must be within (0,5] seconds')
    args.output.mkdir(parents=True, exist_ok=False)

    import rclpy
    from as2_python_api.drone_interface_base import DroneInterfaceBase
    from geometry_msgs.msg import PoseStamped
    from uav_planner_interfaces.action import NavigateToPose

    rclpy.init()
    drone = DroneInterfaceBase(args.drone_id, use_sim_time=True, spin_rate=500.)
    for name in ('system', 'navigation', 'target_tracking'):
        drone.load_module(name)
    evidence = dict(observation_grace_s=args.observation_grace,success=False, events=[], cleanup_errors=[], yaw='continuous yaw' if args.continuous_yaw else ('target yaw' if args.target_yaw else 'takeover yaw'),
                    topology='workstation -> Tools -> Behavior -> PX4')
    canceled = threading.Event()
    lock = threading.Lock()
    latest = None
    feedback_stream = (args.output/'tracking-states.jsonl').open('w')
    tracking_submit = tracking_handle = tracking_result = None
    takeoff_attempted = navigation_attempted = False
    navigation_stopped = tracking_stopped = True
    initial_navigation_accepted = False
    stream = (args.output/'targets.jsonl').open('w')

    def event(name, **data):
        entry = dict(event=name, wall_time=time.time(), **data)
        evidence['events'].append(entry)
        print(json.dumps(entry), flush=True)
        (args.output/'events.json').write_text(json.dumps(evidence['events'], indent=2))

    def wait(future, timeout, interruptible=True):
        deadline = time.monotonic()+timeout
        while not future.done():
            if interruptible and canceled.is_set():
                raise InterruptedError('Mission interrupted')
            if time.monotonic() > deadline:
                raise TimeoutError('Tool response deadline exceeded')
            time.sleep(.01)
        return future.result()

    def system_command(operation):
        future = (drone.system.takeoff(height=args.height) if operation == 'takeoff'
                  else drone.system.land())
        # Always settle submission outcome before handing motion to another Tool.
        try:
            handle = wait(future, 10, interruptible=False)
        except BaseException:
            def cancel_late(completed):
                handle = completed.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            future.add_done_callback(cancel_late)
            raise
        if not handle.accepted:
            raise RuntimeError(operation+' rejected')
        result_future = handle.get_result_async()
        try:
            wrapped = wait(result_future, 110, interruptible=operation == 'takeoff')
        except BaseException:
            wait(handle.cancel_goal_async(), 10, interruptible=False)
            result = wait(result_future, 15, interruptible=False).result
            if not result.stop_confirmed:
                raise RuntimeError(operation+' stop unconfirmed')
            raise
        result = wrapped.result
        event(operation, code=result.code, reason=result.reason,
              stop_confirmed=result.stop_confirmed,
              landed=result.landed, disarmed=result.disarmed)
        if not result.success or not result.stop_confirmed:
            raise RuntimeError(operation+' failed: '+result.reason)
        if operation == 'land' and not (result.landed and result.disarmed):
            raise RuntimeError('Land did not confirm landed and disarmed')

    def feedback(message):
        nonlocal latest
        with lock:
            latest = message.feedback.state
            state = latest
            seconds = lambda stamp:stamp.sec+stamp.nanosec/1e9
            feedback_stream.write(json.dumps(dict(wall_time=time.time(),
                ros_now=drone.get_clock().now().nanoseconds/1e9,
                status=state.status, valid=state.valid, reason=state.reason,
                observation_stamp=seconds(state.observation_stamp),
                valid_until=seconds(state.valid_until), stamp=seconds(state.header.stamp),
                velocity=[state.velocity.x,state.velocity.y,state.velocity.z], capture_id=state.capture_id, position=[state.position.x,state.position.y,state.position.z]))+'\n')
            feedback_stream.flush()

    old_signals = {sig: signal.signal(sig, lambda *_: canceled.set())
                   for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        deadline = time.monotonic()+60
        while not all((drone.system.ready, drone.navigation.ready, drone.target_tracking.ready)):
            if canceled.is_set() or time.monotonic() > deadline:
                raise TimeoutError('System/Navigation/Tracking Tool not ready')
            time.sleep(.1)
        state = wait(drone.system.get_state(), 5)
        state_deadline = time.monotonic()+15
        while not state.fresh and time.monotonic()<state_deadline:
            time.sleep(.05)
            state = wait(drone.system.get_state(), 5)
        if not state.fresh or state.armed or state.busy:
            raise RuntimeError('Fresh idle disarmed System state required')
        takeoff_attempted = True
        system_command('takeoff')
        state = wait(drone.system.get_state(), 5)
        if not state.fresh or not state.armed or not state.offboard or state.busy:
            raise RuntimeError('System not ready for navigation after takeoff')
        if not state.navigation_pose_fresh:
            raise RuntimeError("Fresh local navigation pose required")
        own = state.navigation_pose.pose.position
        frame, altitude = state.navigation_pose.header.frame_id, own.z
        # Simulator fixture can start the runner after this event. No RPC here.
        event('airborne_ready', frame=frame, altitude=altitude)
        tracking_stopped = False
        tracking_submit = drone.target_tracking.track('person', tool_timeout, feedback,
                                                       follow=False)
        tracking_handle = wait(tracking_submit, 10)
        if not tracking_handle.accepted:
            tracking_stopped = True
            raise RuntimeError('Tracking rejected')
        tracking_result = tracking_handle.get_result_async()
        deadline = time.monotonic()+15
        while True:
            if canceled.is_set():
                raise InterruptedError('Mission interrupted')
            with lock:
                target = latest
            now = drone.get_clock().now().nanoseconds/1e9
            if target is not None:
                try:
                    destination(target, now=now, frame=frame, altitude=altitude, offset=(0., 0.))
                    break
                except ValueError:
                    pass
            if tracking_result.done() or time.monotonic() > deadline:
                raise RuntimeError('No fresh valid person observation after takeoff')
            time.sleep(.02)
        offset = follow_offset((own.x, own.y), (target.position.x, target.position.y), args.distance)
        event('target_acquired', session_id=target.session_id, offset=list(offset))
        session = target.session_id
        deadline = time.monotonic()+args.duration if args.duration else math.inf
        tangent=TangentReference(offset,(own.x,own.y),args.distance,lead_seconds=args.tangent_lead)
        tangent_time=drone.get_clock().now().nanoseconds/1e9
        previous_own=None
        previous_point=None
        previous_yaw=None
        observation_recovery=ObservationRecovery(frame,target.generation,
            target.observation_stamp.sec+target.observation_stamp.nanosec/1e9,args.observation_grace)
        was_waiting=False
        was_recovering=False
        while time.monotonic() < deadline:
            if canceled.is_set():
                raise InterruptedError('Mission interrupted')
            if tracking_result.done():
                raise RuntimeError('Tracking ended during pursuit')
            with lock:
                target = latest
            if target.session_id != session:
                raise RuntimeError('Tracking session changed')
            now = drone.get_clock().now().nanoseconds/1e9
            observation_wait,wait_elapsed=observation_recovery.update(target,now,time.monotonic())
            if observation_wait!=was_waiting:
                event('observation_wait_started' if observation_wait else 'observation_wait_recovered',
                    wait_seconds=wait_elapsed,source_valid=target.valid,source_status=target.status,
                    observation_age=now-(target.observation_stamp.sec+target.observation_stamp.nanosec/1e9))
                was_waiting=observation_wait
            observation_audit=None
            own_state=None
            active_offset=offset
            if args.predictive_observation and not observation_wait:
                own_state = wait(drone.system.get_state(), 3)
                if not own_state.fresh or not own_state.navigation_pose_fresh:
                    raise RuntimeError('Fresh Tool navigation pose required for observation policy')
                p=own_state.navigation_pose.pose.position
                q=own_state.navigation_pose.pose.orientation
                h=own_state.navigation_pose.header.stamp
                own_time=h.sec+h.nanosec/1e9
                v=(0.,0.)
                if previous_own and 0<own_time-previous_own[0]<1.:
                    v=tuple((a-b)/(own_time-previous_own[0]) for a,b in zip((p.x,p.y),previous_own[1]))
                previous_own=(own_time,(p.x,p.y))
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                goal,observation_audit=select_observation_point((p.x,p.y),v,yaw,
                    (target.position.x,target.position.y),(target.velocity.x,target.velocity.y),
                    distance=args.distance,previous=previous_point[:2] if previous_point else None)
                active_offset=(goal[0]-target.position.x,goal[1]-target.position.y)
            if observation_wait:
                if previous_point is None:
                    own_state=wait(drone.system.get_state(),3)
                    if not own_state.fresh or not own_state.navigation_pose_fresh:
                        raise RuntimeError('Fresh Tool pose required for initial observation hold')
                    p=own_state.navigation_pose.pose.position
                    point=(p.x,p.y,p.z)
                else:point=previous_point
                recovering=True
                observation_audit=dict(mode='wait_for_observation',elapsed_s=wait_elapsed)
            else:
                point,recovering = recovery_destination(target, now=now, frame=frame,
                    altitude=altitude, offset=active_offset, previous=previous_point, window=args.recovery_window,
                    freeze_after=.6 if args.tangent_follow and args.recovery_window>.6 else .3)
            if args.tangent_follow and not recovering and not observation_wait:
                goal,observation_audit=tangent.update((target.position.x,target.position.y),
                    (target.velocity.x,target.velocity.y),max(.001,min(1.,now-tangent_time)))
                point=(*goal,altitude)
            tangent_time=now
            if recovering!=was_recovering:
                event('recovery_started' if recovering else 'recovered',
                    observation_age=now-(target.observation_stamp.sec+target.observation_stamp.nanosec/1e9))
                was_recovering=recovering
            if navigation_attempted and drone.navigation.done:
                raise RuntimeError('Navigation ended: '+drone.navigation.result.reason)
            navigation_attempted, navigation_stopped = True, False
            yaw_command = None
            if args.target_yaw or args.continuous_yaw:
                if own_state is None:own_state = wait(drone.system.get_state(), 3)
                if not own_state.fresh or not own_state.navigation_pose_fresh:
                    raise RuntimeError('Fresh Tool navigation pose required for target yaw')
                own_now = own_state.navigation_pose.pose.position
                if observation_wait:
                    q=own_state.navigation_pose.pose.orientation
                    yaw_command=previous_yaw if previous_yaw is not None else math.atan2(
                        2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                else:
                    # TargetState position is already predicted at its header stamp.
                    target_time = target.header.stamp.sec+target.header.stamp.nanosec/1e9
                    lead = max(0., min(.3, now-target_time+.1))
                    tx = target.position.x+target.velocity.x*lead
                    ty = target.position.y+target.velocity.y*lead
                    yaw_command = math.atan2(ty-own_now.y,tx-own_now.x)
                pose = PoseStamped();pose.header.frame_id=frame
                pose.header.stamp=drone.get_clock().now().to_msg()
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point
                pose.pose.orientation.z=math.sin(yaw_command/2)
                pose.pose.orientation.w=math.cos(yaw_command/2)
                drone.navigation.to_pose(pose, orientation_mode=(NavigateToPose.Goal.CONTINUOUS_YAW if args.continuous_yaw else NavigateToPose.Goal.TARGET_YAW),
                    wait=False, arrival_behavior='hold', target_timeout=1.,
                    timeout=tool_timeout, response_timeout=3.)
            else:
                drone.navigation.to_point(*point, frame_id=frame, wait=False,
                    arrival_behavior='hold', target_timeout=1., timeout=tool_timeout,
                    response_timeout=3.)
            initial_navigation_accepted = True
            previous_point=point
            previous_yaw=yaw_command
            stream.write(json.dumps(dict(sim_time=now, capture_id=target.capture_id,
                session_id=session, observation_wait=observation_wait, observation_policy=observation_audit, recovering=recovering, yaw_command=yaw_command, destination=point, target=[target.position.x,
                target.position.y, target.position.z]))+'\n')
            stream.flush()
            canceled.wait(args.update_period)
        if was_waiting:
            raise RuntimeError('Pursuit window ended before observation recovery')
        evidence['pursuit_window_complete'] = True
    except BaseException as exc:
        evidence['error'] = str(exc)
    finally:
        if navigation_attempted:
            try:
                navigation_stopped = drone.navigation.cancel(timeout=15.)
                if not initial_navigation_accepted:
                    navigation_stopped = False
                    raise RuntimeError("Initial Navigation submission outcome uncertain")
            except BaseException as exc:
                evidence['cleanup_errors'].append('navigation: '+str(exc))
        if tracking_submit is not None:
            try:
                if tracking_handle is None:
                    tracking_handle = wait(tracking_submit, 10, interruptible=False)
                if not tracking_handle.accepted:
                    tracking_stopped = True
                else:
                    if tracking_result is None:
                        tracking_result = tracking_handle.get_result_async()
                    if not tracking_result.done():
                        wait(tracking_handle.cancel_goal_async(), 10, interruptible=False)
                    tracking_stopped = bool(wait(tracking_result, 15, interruptible=False).result.stop_confirmed)
            except BaseException as exc:
                evidence['cleanup_errors'].append('tracking: '+str(exc))
                if tracking_handle is None:
                    tracking_submit.add_done_callback(lambda f: f.result().cancel_goal_async()
                        if f.result().accepted else None)
        if takeoff_attempted and navigation_stopped and tracking_stopped:
            try:
                system_command('land')
                state = wait(drone.system.get_state(), 5, interruptible=False)
                evidence['landed_disarmed_idle'] = bool(state.fresh and not state.armed and not state.busy)
            except BaseException as exc:
                evidence['cleanup_errors'].append('land: '+str(exc))
        elif takeoff_attempted:
            evidence['cleanup_errors'].append('Motion stop unconfirmed; retain onboard execution for recovery')
        evidence['success'] = bool(evidence.get('pursuit_window_complete') and
            evidence.get('landed_disarmed_idle') and not evidence['cleanup_errors'] and 'error' not in evidence)
        stream.close()
        (args.output/'result.json').write_text(json.dumps(evidence, indent=2))
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
        drone.shutdown()
        feedback_stream.close()
        drone.destroy_node()
        rclpy.try_shutdown()
    if not evidence['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
