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
from geometry_msgs.msg import PoseStamped, TwistStamped
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
        self.failure_statuses = []
        self.goal_statuses = {}  # goal_id hex -> max status code seen
        self.samples = []
        # The 0.5 s heartbeat re-publishes the bare snapshot (no reason/detail
        # keys), so the finish reason must be latched from the status that
        # first carried it.
        self.terminal_detail = None
        self.terminal_stop_confirmed = None
        self._settle_quiet_since = None  # velocity-gate quiet window start
        self.twist = None
        # The clock bridge publishes best-effort; a reliable subscription is
        # rejected by DDS (SPF phase-2 note), so match best-effort here.
        self.create_subscription(Clock, '/clock', lambda m: setattr(
            self, 'clock', m.clock.sec + 1e-9 * m.clock.nanosec),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 lambda m: setattr(self, 'pose', m), 10)
        self.create_subscription(TwistStamped, '/drone0/self_localization/twist',
                                 lambda m: setattr(self, 'twist', m),
                                 qos_profile_sensor_data)
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
        if str(self.status.get('state', '')).endswith('_failure'):
            # instantaneous *_failure statuses are overwritten by the 0.5 s
            # heartbeat before the 1 s sampler can catch them (fault1 run:
            # counter recorded 0 despite 3 failures) - latch them here.
            self.failure_statuses.append(self.status)
        if self.status.get('state') in ('STOPPED', 'FAULT', 'fault') and \
                ('detail' in self.status or 'reason' in self.status):
            # 'fault' (lowercase) is the instantaneous _do_fault status state;
            # the heartbeat snapshot carries the uppercase FAULT without
            # detail/reason, so the terminal detail must latch from it too.
            self.terminal_detail = self.status.get('detail',
                                                   self.status.get('reason'))
            self.terminal_stop_confirmed = self.status.get('stop_confirmed')

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

    def _settled(self):
        """Velocity-based hover-stability gate (2026-11-18 z-oscillation
        attribution, log/citynav/z-limcycle): the post-takeoff z swing is
        PX4-EKF-driven and gain-independent (p2p up to ~1.9 m at ~0.1 Hz),
        so a position-range window cannot be met; but the FollowTimedPath
        continuity check only needs the path's first point (the pose at
        decision time) to still match the vehicle ~0.5-1.5 s later. At an
        oscillation extremum the short-horizon drift is minimal, so the
        gate waits for |vz| to stay below 0.06 m/s for 2 s - exactly the
        quantity the Behavior compares."""
        if not self.twist:
            return False
        now = time.monotonic()
        vz = self.twist.twist.linear.z
        quiet = abs(vz) < 0.06
        if not quiet:
            self._settle_quiet_since = None
            return False
        if self._settle_quiet_since is None:
            self._settle_quiet_since = now
            return False
        return now - self._settle_quiet_since >= 2.0


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
    parser.add_argument('--expect-fault', action='store_true',
                        help='M6 fault1 case: the LLM path is persistently '
                             'broken (proxy points at a dead endpoint, set in '
                             '<output>/planner.yaml before restart-compute). '
                             'Gates switch to FAULT convergence: state FAULT '
                             'with detail llm_failures/service_failures, no '
                             'FollowTimedPath goal ever sent, airborne at '
                             'FAULT, then clean land/disarm without collision.')
    parser.add_argument('--expect-stuck', action='store_true',
                        help='M6 stuck1 case: the grounding service was made '
                             'to always answer seg_success=false (stub on the '
                             'host, grounding_url repointed in <output>/ '
                             'planner.yaml + restart-compute) with a 5-'
                             'landmark instruction, so every cycle plans zero '
                             'displacement. Gates: FAULT with detail=stuck '
                             'from the upgraded progress criterion, >=4 zero-'
                             'progress cycles, no goal ever sent, airborne, '
                             'clean land/disarm.')
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
        # 2.5 m (was 1.5): the EKF/lockstep z transient observed once per
        # gain family (run l kp1.0, run v kp0.05, both coincident with a
        # grounding GPU burst, ~1 m drop in ~1 s) needs headroom so a
        # transient cannot reach the ground before the next decision.
        goal.takeoff_height = 2.5
        goal.takeoff_speed = 0.5
        handle = mission.run_action(mission.takeoff, Takeoff, goal, 60)
        if handle.status != 4 or not handle.result.takeoff_success:
            raise RuntimeError('native Takeoff failed (status=%d)' % handle.status)
        result['gates']['takeoff'] = 'PASS'

        # Hover-stability gate before publishing the goal (run-20261118h:
        # with a warm grounding service the planner's first FollowTimedPath
        # goal landed ~17 s after takeoff, inside the post-takeoff settle;
        # the Behavior rejected it because the path's first point (decision
        # pose) was >0.25 m from the actual position at valid_from). The
        # mission waits until the pose holds still, so segment-1 continuity
        # holds regardless of how fast the compute role decides.
        mission.spin_until(lambda: mission._settled(), 90,
                           'drone did not settle after takeoff')
        result['gates']['hover_settle'] = 'PASS'

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
        # Wait for the planner to enter RUNNING before monitoring: when a
        # second mission reuses the same session, the planner heartbeat still
        # publishes the PREVIOUS task's STOPPED for up to 0.5 s, which would
        # break the loop instantly and mis-score the new task
        # (run-20261118q/expand-building2).
        mission.spin_until(lambda: (mission.status or {}).get('state') == 'RUNNING',
                           20, 'planner never entered RUNNING (busy/rejected?)')
        goal_wall = time.time()

        cancelled = False
        first_goal_wall = None
        deadline = goal_wall + args.timeout
        last_sample = 0.0
        while time.time() < deadline:
            rclpy.spin_once(mission, timeout_sec=0.1)
            if args.cancel_after > 0 and not cancelled:
                # Event-driven injection (run-20261118k: with a single
                # landmark the LLM can answer is_found=true and the task
                # finishes ~8 s after the goal before any timed path is
                # sent, so a wall-clock-after-goal cancel would fire on an
                # idle planner). revision>=1 means the planner just sent
                # its first FollowTimedPath goal; cancel_after counts from
                # that instant, so the handoff always interrupts an
                # executing trajectory (>=2 segments by then: yaw 0.5 s +
                # 5 m steps at 1 m/s).
                if int((mission.status or {}).get('revision', 0) or 0) >= 1:
                    if first_goal_wall is None:
                        first_goal_wall = time.time()
                    elif time.time() - first_goal_wall >= args.cancel_after:
                        cancelled = True
                        if not mission.cancel_cli.wait_for_service(10):
                            raise TimeoutError('citynav/cancel unavailable')
                        mission.cancel_cli.call_async(Trigger.Request())
                        (args.output / 'fault-injection.json').write_text(json.dumps(
                            {'fault': 'cancel', 'wall': time.time(),
                             'seconds_after_first_goal_sent': round(
                                 time.time() - first_goal_wall, 1),
                             'revision_at_injection': int(
                                 (mission.status or {}).get('revision', 0))},
                            indent=2) + '\n')
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
        # first `revision` segment events are the executed ones (the planner
        # blocks until each goal result before the next cycle). Exploit
        # goals publish `exploit_executed` instead of `cycle` (decision-6
        # reuse round), so both event kinds count as executed segments.
        cycles = [e for e in mission.events
                  if e.get('event') in ('cycle', 'exploit_executed')]
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
        if args.expect_fault or args.expect_stuck:
            # M6 fault-path gates (locked): a persistent service outage /
            # zero-progress loop must drive the planner to FAULT BEFORE any
            # trajectory execution, with the drone still airborne; the
            # waypoint/revision/final-reason formal gates are structurally
            # inapplicable (NOT-EXECUTED, never silently passed).
            result['gates']['waypoints'] = 'NOT-EXECUTED'
            result['gates']['revision_match'] = 'NOT-EXECUTED'
            result['gates']['final_reason'] = 'NOT-EXECUTED'
            if args.expect_stuck:
                result['gates']['stop_state'] = (
                    'PASS' if status.get('state') == 'FAULT'
                    and mission.terminal_detail == 'stuck' else 'FAIL')
                cyc = [e for e in mission.events if e.get('event') == 'cycle']
                result['gates']['stuck_zero_progress'] = (
                    'PASS' if len(cyc) >= 4 and all(
                        int(e.get('progress', 1)) == 0 for e in cyc)
                    else 'FAIL')
                result['gates']['stuck_before_execution'] = (
                    'PASS' if revision == 0 and accepted_goals == 0 else 'FAIL')
                (args.output / 'fault-injection.json').write_text(json.dumps(
                    {'fault': 'grounding_always_empty_persistent',
                     'injection': 'grounding_url repointed to the host stub '
                                  '(validation/grounding_stub.py, always '
                                  'seg_success=false) in <output>/planner.yaml '
                                  '+ restart-compute; 5-landmark instruction '
                                  'so task completion cannot precede the '
                                  '4-cycle window',
                     'wall': time.time(),
                     'cycles': len(cyc),
                     'progress_bits': [int(e.get('progress', 1)) for e in cyc],
                     'failure_statuses': mission.failure_statuses},
                    indent=2) + '\n')
            else:
                result['gates']['stop_state'] = (
                    'PASS' if status.get('state') == 'FAULT'
                    and mission.terminal_detail in ('llm_failures', 'service_failures')
                    else 'FAIL')
                (args.output / 'fault-injection.json').write_text(json.dumps(
                    {'fault': 'llm_unreachable_persistent',
                     'injection': 'proxy base_url set to a dead endpoint in '
                                  '<output>/planner.yaml before restart-compute; '
                                  'every citynav/llm query returns error_code=http',
                     'wall': time.time(),
                     'landmark_extract_failures': len(mission.failure_statuses),
                     'failure_statuses': mission.failure_statuses},
                    indent=2) + '\n')
            result['gates']['fault_no_execution'] = (
                'PASS' if revision == 0 and accepted_goals == 0 else 'FAIL')
            zs = [s['pos'][2] for s in mission.samples]
            result['gates']['airborne_at_fault'] = (
                'PASS' if zs and zs[-1] > 1.0 else 'FAIL')
            result['fault_detail'] = mission.terminal_detail
        elif args.cancel_after > 0:
            result['gates']['waypoints'] = 'NOT-EXECUTED'
            result['gates']['revision_match'] = 'NOT-EXECUTED'
            result['gates']['final_reason'] = 'NOT-EXECUTED'
            # M6 gate (locked): the cancel handoff must converge through the
            # speed-gated STOPPED path with detail=cancelled; reaching FAULT
            # (the 10 s stop-gate timeout) is a FAIL, not a pass.
            result['gates']['stop_state'] = (
                'PASS' if status.get('state') == 'STOPPED'
                and mission.terminal_detail == 'cancelled'
                and mission.terminal_stop_confirmed is True else 'FAIL')
        else:
            result['gates']['waypoints'] = ('PASS' if steps_executed >= 3 and revision >= 1
                                            else 'FAIL')
            # revision match: every planner-sent goal must appear on the action
            # status stream and reach SUCCEEDED for a clean task_complete.
            result['gates']['revision_match'] = (
                'PASS' if revision >= 1 and succeeded_goals >= revision
                else 'FAIL')
            result['gates']['stop_state'] = ('PASS' if status.get('state') in
                                             ('STOPPED', 'FAULT') else 'FAIL')
            result['gates']['final_reason'] = (
                'PASS' if mission.terminal_detail == 'task_complete' else 'FAIL')
        result['terminal_detail'] = mission.terminal_detail
        # Decision-6 exploit evidence summary (events stream is the primary
        # record; these are the load/hit/walk markers the reuse round needs).
        result['exploit'] = [e for e in mission.events if e.get('event') in
                             ('memory_loaded', 'exploit_hit', 'exploit_executed')]

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
