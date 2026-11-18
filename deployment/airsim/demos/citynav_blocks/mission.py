"""CityNav language-navigation workstation mission (four-part topology).

Readiness -> teleport initial pose (official RPC, Blocks ignores settings X/Y)
-> native Takeoff -> citynav/goal (String JSON {instruction, max_steps,
use_memory}) -> monitor citynav/status + citynav/events + the FollowTimedPath
action status topic -> gates -> native Land -> disarm. Ground truth is used
for scoring only (per validation skill); the planner never consumes it.

Waypoint-segment gate (locked before formal1): the planner sends one
FollowTimedPath goal per executed exploration cycle (revision++). A cycle's
decomposition `steps` count (15 deg / 5 m / 2 m primitives, the upstream
calculate_movement_steps semantics) is the number of timed waypoint segments
in that goal. The gate requires sum(steps of the first `revision` cycles) >= 3
goals-worth of executed segments, with goal acceptance evidenced by the
/drone0/follow_timed_path/_action/status stream (SPF lesson: score during
navigation, never after Land; collision via time_stamp comparison because this
Blocks build has no resetLastCollision).

Reserved switches for later rounds (not part of formal1):
--cancel-after N injects citynav/cancel N seconds after the goal;
--use-memory toggles the goal payload memory flag (exploit reuse round).
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from as2_msgs.action import Takeoff, Land
from as2_msgs.msg import PlatformInfo
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from uav_as2_sensor_adapter.rpc import Rpc


class Mission(Node):
    def __init__(self, args):
        super().__init__('citynav_mission')
        self.args = args
        self.clock = None
        self.pose = None
        self.camera_info = None
        self.status = None
        self.platform = None
        self.events = []
        self.goal_statuses = {}  # goal_id hex -> max status code seen
        self.samples = []
        # The 0.5 s heartbeat re-publishes the bare snapshot (no reason/detail
        # keys), so the finish reason must be latched from the status that
        # first carried it.
        self.terminal_detail = None
        # The clock bridge publishes best-effort; a reliable subscription is
        # rejected by DDS (SPF phase-2 note), so match best-effort here.
        self.create_subscription(Clock, '/clock', lambda m: setattr(
            self, 'clock', m.clock.sec + 1e-9 * m.clock.nanosec),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 lambda m: setattr(self, 'pose', m), 10)
        self.create_subscription(CameraInfo,
                                 '/drone0/sensor_measurements/CityFront/rgb/camera_info',
                                 lambda m: setattr(self, 'camera_info', m), qos_profile_sensor_data)
        self.create_subscription(String, '/drone0/citynav/status',
                                 lambda m: self._on_status(m), 10)
        self.create_subscription(String, '/drone0/citynav/events',
                                 lambda m: self._on_event(m), 10)
        # Direct downstream-consumer evidence: which FollowTimedPath goals the
        # Behavior accepted/executed/succeeded (action_msgs GoalStatusArray).
        self.create_subscription(GoalStatusArray,
                                 '/drone0/follow_timed_path/_action/status',
                                 self._on_goal_statuses, qos_profile_sensor_data)
        # AS2 motion behaviors expose their action servers under the behavior
        # node name (TakeoffBehavior/LandBehavior), matching the spf demo.
        self.takeoff = ActionClient(self, Takeoff, '/drone0/TakeoffBehavior')
        self.land = ActionClient(self, Land, '/drone0/LandBehavior')
        self.goal_pub = self.create_publisher(String, '/drone0/citynav/goal', 10)
        self.cancel_cli = self.create_client(Trigger, '/drone0/citynav/cancel')
        self.create_subscription(PlatformInfo, '/drone0/platform/info',
                                 lambda m: setattr(self, 'platform', m), qos_profile_sensor_data)
        self.arm_cli = self.create_client(SetBool, '/drone0/set_arming_state')
        self.offboard_cli = self.create_client(SetBool, '/drone0/set_offboard_mode')

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            return
        if self.status.get('state') in ('STOPPED', 'FAULT') and \
                ('detail' in self.status or 'reason' in self.status):
            self.terminal_detail = self.status.get('detail',
                                                   self.status.get('reason'))

    def _on_event(self, msg):
        try:
            self.events.append(json.loads(msg.data))
        except ValueError:
            pass

    def _on_goal_statuses(self, msg):
        for status in msg.status_list:
            key = bytes(status.goal_info.goal_id.uuid).hex()
            # Status codes: 0 UNKNOWN 1 ACCEPTED 2 EXECUTING 3 CANCELING
            # 4 CANCELED 5 SUCCEEDED 6 ABORTED; keep the highest observed.
            self.goal_statuses[key] = max(self.goal_statuses.get(key, -1),
                                          int(status.status))

    def call_bool(self, client, value, timeout):
        if not client.wait_for_service(timeout):
            raise TimeoutError('service %s unavailable' % client.srv_name)
        fut = client.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise TimeoutError('service %s timeout' % client.srv_name)
        return fut.result()

    def run_action(self, client, GoalType, goal, timeout):
        """Send an AS2 action goal with a bounded wait (never hangs the mission)."""
        if not client.wait_for_server(timeout):
            raise TimeoutError('action %s server unavailable' % client._action_name)
        sf = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=timeout)
        gh = sf.result()
        if gh is None or not gh.accepted:
            raise RuntimeError('action %s goal rejected' % client._action_name)
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        if rf.result() is None:
            raise TimeoutError('action %s result timeout' % client._action_name)
        return rf.result()

    def spin_until(self, predicate, timeout, message):
        deadline = time.time() + timeout
        while time.time() < deadline and not predicate():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not predicate():
            raise TimeoutError(message)

    def fresh(self, stamp, now, limit):
        if stamp is None:
            return False
        return abs(now - stamp) <= limit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--instruction', required=True)
    parser.add_argument('--max-steps', type=int, default=60)
    parser.add_argument('--use-memory', type=int, default=1,
                        help='1: goal payload use_memory=true (exploit reuse '
                             'allowed); 0: explore-only control')
    parser.add_argument('--cancel-after', type=float, default=0.0,
                        help='inject citynav/cancel this many seconds after '
                             'the goal (0 = disabled; formal1 keeps it off)')
    parser.add_argument('--target', type=float, nargs=3, required=True,
                        help='ground-truth target x y z (odom ENU frame, scoring only)')
    parser.add_argument('--spawn', type=float, nargs=3, default=[10.0, 10.0, 45.0],
                        help='teleport spawn x y yaw_deg (ground level; run.py '
                             'passes the measured default; see README deviation '
                             'on the 50 m depth gate)')
    parser.add_argument('--rpc-host', default=None)
    parser.add_argument('--rpc-port', type=int, default=45481)
    parser.add_argument('--timeout', type=float, default=900.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    rpc = Rpc(args.rpc_host, args.rpc_port) if args.rpc_host else None
    rclpy.init()
    mission = Mission(args)
    result = {'instruction': args.instruction, 'max_steps': args.max_steps,
              'use_memory': bool(args.use_memory), 'gates': {}, 'samples': []}
    stage = 'readiness'
    t_start = time.time()
    try:
        mission.spin_until(lambda: mission.clock is not None, 60, 'no /clock')
        now0 = mission.clock
        mission.spin_until(
            lambda: (mission.pose and mission.camera_info
                     and mission.fresh(mission.pose.header.stamp.sec
                                       + 1e-9 * mission.pose.header.stamp.nanosec,
                                       now0, 30)
                     and mission.status and mission.status.get('ready')),
            120, 'compute role not ready')
        result['gates']['readiness'] = 'PASS'

        # Teleport initial condition (Blocks ignores settings X/Y; official RPC
        # mechanism, same as the spf demo). Must run before takeoff.
        if rpc:
            x, y, yaw = args.spawn
            qz, qw = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
            rpc.call('simSetVehiclePose', {
                'position': {'x_val': x, 'y_val': y, 'z_val': 0.0},
                'orientation': {'x_val': 0.0, 'y_val': 0.0, 'z_val': qz, 'w_val': qw}}, True, 'PX4')
            time.sleep(0.5)
            # The EKF needs seconds to re-converge onto the teleported GPS fix
            # (SPF run-20261116v lesson: arming before convergence starts the
            # flight from a stale estimate).
            mission.spin_until(
                lambda: (mission.pose
                         and abs(mission.pose.pose.position.x - x) < 1.5
                         and abs(mission.pose.pose.position.y - y) < 1.5),
                60, 'EKF did not converge onto teleported pose')
        result['gates']['teleport'] = 'PASS' if rpc else 'NOT-EXECUTED'

        stage = 'takeoff'
        # The pixhawk platform requires explicit arming + offboard before the
        # TakeoffBehavior climbs. Retry arming: preflight checks may reject the
        # first ARM after a teleport (SPF one-mission-per-session rule).
        time.sleep(3.0)
        armed = False
        for attempt in range(8):
            mission.call_bool(mission.arm_cli, True, 15)
            try:
                mission.spin_until(lambda: mission.platform and mission.platform.armed,
                                   10, 'not armed')
                armed = True
                break
            except TimeoutError:
                mission.get_logger().warn(
                    'arm attempt %d rejected (preflight not ready); retrying' % (attempt + 1))
                time.sleep(5)
        if not armed:
            raise RuntimeError('platform never armed after retries')
        mission.call_bool(mission.offboard_cli, True, 15)
        mission.spin_until(lambda: mission.platform and mission.platform.offboard,
                           20, 'not offboard')
        goal = Takeoff.Goal()
        goal.takeoff_height = 1.5
        goal.takeoff_speed = 0.5
        handle = mission.run_action(mission.takeoff, Takeoff, goal, 60)
        if handle.status != 4 or not handle.result.takeoff_success:
            raise RuntimeError('native Takeoff failed (status=%d)' % handle.status)
        result['gates']['takeoff'] = 'PASS'

        # Collision baseline AFTER takeoff: the flag is sticky for the whole
        # Blocks session and spawn/teleport residue sets it early; new flight
        # contacts are detected by the time_stamp changing (this build lacks
        # resetLastCollision).
        collision_events = []
        collision_baseline = None
        if rpc:
            info0 = rpc.call('simGetCollisionInfo', 'PX4')
            collision_baseline = {'has_collided': info0.get('has_collided'),
                                  'time_stamp': info0.get('time_stamp')}

        stage = 'navigation'
        tx, ty, tz = args.target
        start = mission.pose.pose.position
        d0 = math.dist((start.x, start.y, start.z), (tx, ty, tz))
        goal_msg = String(data=json.dumps({
            'instruction': args.instruction, 'max_steps': args.max_steps,
            'use_memory': bool(args.use_memory)}))
        mission.goal_pub.publish(goal_msg)
        goal_wall = time.time()

        cancelled = False
        deadline = goal_wall + args.timeout
        last_sample = 0.0
        while time.time() < deadline:
            rclpy.spin_once(mission, timeout_sec=0.1)
            if args.cancel_after > 0 and not cancelled \
                    and time.time() - goal_wall >= args.cancel_after:
                cancelled = True
                if not mission.cancel_cli.wait_for_service(10):
                    raise TimeoutError('citynav/cancel unavailable')
                mission.cancel_cli.call_async(Trigger.Request())
                (args.output / 'fault-injection.json').write_text(json.dumps(
                    {'fault': 'cancel', 'wall': time.time(),
                     'seconds_after_goal': args.cancel_after}, indent=2) + '\n')
            if mission.pose and time.time() - last_sample > 1.0:
                last_sample = time.time()
                p = mission.pose.pose.position
                mission.samples.append(
                    {'t': mission.clock, 'pos': [p.x, p.y, p.z],
                     'status': mission.status, 'events': len(mission.events)})
                if rpc:
                    info = rpc.call('simGetCollisionInfo', 'PX4')
                    if (info.get('has_collided') and
                            info.get('time_stamp') !=
                            collision_baseline['time_stamp']):
                        collision_events.append(
                            {'t': mission.clock, 'pos': [p.x, p.y, p.z],
                             'object_name': info.get('object_name'),
                             'collision_normal': info.get('normal'),
                             'impact_point': info.get('impact_point'),
                             'time_stamp': info.get('time_stamp')})
                        collision_baseline['time_stamp'] = info.get('time_stamp')
            status = mission.status or {}
            if status.get('state') in ('STOPPED', 'FAULT'):
                break
        else:
            result['navigation_timeout'] = True

        end = mission.pose.pose.position
        d1 = math.dist((end.x, end.y, end.z), (tx, ty, tz))
        status = mission.status or {}
        result['samples'] = mission.samples
        result['events'] = mission.events
        result['distance'] = {'start': round(d0, 2), 'end': round(d1, 2),
                              'net_approach_ratio': round((d0 - d1) / d0, 4) if d0 > 0.1 else None}
        result['final_status'] = status
        result['goal_statuses'] = {k: v for k, v in mission.goal_statuses.items()}

        # Waypoint-segment accounting (pre-locked interpretation, README):
        # revision == number of FollowTimedPath goals the planner sent; the
        # first `revision` cycle events are the executed ones (the planner
        # blocks until each goal result before the next cycle).
        cycles = [e for e in mission.events if e.get('event') == 'cycle']
        revision = int(status.get('revision', 0) or 0)
        executed = cycles[:revision] if revision else []
        steps_executed = sum(int(c.get('steps', 0)) for c in executed)
        accepted_goals = sum(1 for v in mission.goal_statuses.values() if v >= 2)
        # action_msgs/GoalStatus enum (ROS 2 Humble): UNKNOWN=0 ACCEPTED=1
        # EXECUTING=2 CANCELING=3 SUCCEEDED=4 CANCELED=5 ABORTED=6. The
        # original gate compared ==5 (CANCELED) by mistake, failing run c
        # whose single goal legitimately reached SUCCEEDED=4 (consistent
        # with the planner's own `status == 4` success check).
        succeeded_goals = sum(1 for v in mission.goal_statuses.values() if v == 4)
        result['waypoints'] = {
            'cycles_seen': len(cycles), 'goals_sent_by_planner': revision,
            'steps_executed': steps_executed,
            'goals_observed_on_action_status': len(mission.goal_statuses),
            'goals_executing_or_better': accepted_goals,
            'goals_succeeded': succeeded_goals,
            'landmarks': status.get('landmarks'),
            'next_landmark_idx': status.get('next_landmark_idx'),
            'step_size': status.get('step_size'),
            'exploit_events': sum(1 for e in mission.events
                                  if e.get('event') in ('exploit_hit', 'exploit_executed'))}
        if args.cancel_after > 0:
            result['gates']['waypoints'] = 'NOT-EXECUTED'
        else:
            result['gates']['waypoints'] = ('PASS' if steps_executed >= 3 and revision >= 1
                                            else 'FAIL')
        # revision match: every planner-sent goal must appear on the action
        # status stream and reach SUCCEEDED for a clean task_complete.
        if args.cancel_after > 0:
            result['gates']['revision_match'] = 'NOT-EXECUTED'
        else:
            result['gates']['revision_match'] = (
                'PASS' if revision >= 1 and succeeded_goals >= revision
                else 'FAIL')
        result['gates']['stop_state'] = ('PASS' if status.get('state') in
                                         ('STOPPED', 'FAULT') else 'FAIL')
        result['gates']['final_reason'] = (
            'NOT-EXECUTED' if args.cancel_after > 0 else
            ('PASS' if mission.terminal_detail == 'task_complete' else 'FAIL'))
        result['terminal_detail'] = mission.terminal_detail

        stage = 'land'
        land_goal = Land.Goal()
        land_goal.land_speed = 0.5
        land_handle = mission.run_action(mission.land, Land, land_goal, 60)
        result['gates']['land'] = 'PASS' if (land_handle.status == 4
                                             and land_handle.result.land_success) else 'FAIL'
        mission.spin_until(lambda: mission.platform and mission.platform.armed == 0,
                           30, 'not disarmed')
        result['disarmed'] = True
        result['gates']['disarm'] = 'PASS'

        if rpc:
            result['collision'] = {'baseline': collision_baseline,
                                   'events': collision_events}
            result['gates']['collision'] = 'PASS' if not collision_events else 'FAIL'
        result['duration_s'] = round(time.time() - t_start, 1)
        # NOT-EXECUTED marks a structurally inapplicable gate (e.g. waypoints
        # for a cancel case); it is neither a pass nor a failure.
        result['success'] = all(v in ('PASS', 'NOT-EXECUTED')
                                for v in result['gates'].values())
    except Exception as exc:
        result['error'] = f'{type(exc).__name__} at {stage}: {exc}'
        result['success'] = False
        result['duration_s'] = round(time.time() - t_start, 1)
        # Bounded recovery: native Land even after task failure.
        try:
            mission.run_action(mission.land, Land, Land.Goal(land_speed=0.5), 30)
        except Exception:
            pass
    finally:
        (args.output / 'result.json').write_text(json.dumps(result, indent=2))
        rclpy.shutdown()
    print(json.dumps({'success': result['success'], 'gates': result['gates'],
                      'waypoints': result.get('waypoints'),
                      'distance': result.get('distance')}, indent=2))
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
