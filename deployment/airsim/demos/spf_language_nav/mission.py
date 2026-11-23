"""SPF language-navigation workstation mission.

Readiness -> teleport initial pose (official RPC, Blocks ignores settings X/Y)
-> native Takeoff -> to_semantic_goal (language instruction) -> monitor spf/status and
ground-truth pose -> gates -> native Land -> disarmed. Ground truth is used
for scoring only (per validation skill); the planner never consumes it.

Fault cases (orchestrated by run.py): stale (sensor container paused),
vlm-down (proxy container stopped), cancel (mid-task Action cancellation).
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
from as2_msgs.action import Takeoff, Land
from uav_planner_interfaces.action import NavigateToSemanticGoal
from as2_msgs.msg import PlatformInfo
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_as2_sensor_adapter.rpc import Rpc


class Mission(Node):
    def __init__(self, args):
        super().__init__('spf_mission')
        self.args = args
        self.clock = None
        self.pose = None
        self.camera_info = None
        self.status = None
        self.platform = None
        self.samples = []
        # The clock bridge publishes best-effort; a reliable subscription is
        # rejected by DDS (see phase-2 note), so match best-effort here.
        self.create_subscription(Clock, '/clock', lambda m: setattr(
            self, 'clock', m.clock.sec + 1e-9 * m.clock.nanosec),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 lambda m: setattr(self, 'pose', m), 10)
        self.create_subscription(CameraInfo,
                                 '/drone0/sensor_measurements/FrontCamera/rgb/camera_info',
                                 lambda m: setattr(self, 'camera_info', m), qos_profile_sensor_data)
        self.create_subscription(String, '/drone0/spf/status',
                                 lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        # AS2 motion behaviors expose their action servers under the behavior
        # node name (TakeoffBehavior/LandBehavior), matching the yopo demo.
        self.takeoff = ActionClient(self, Takeoff, '/drone0/TakeoffBehavior')
        self.land = ActionClient(self, Land, '/drone0/LandBehavior')
        self.navigation = ActionClient(
            self, NavigateToSemanticGoal, '/drone0/navigate_to_semantic_goal')
        self.create_subscription(PlatformInfo, '/drone0/platform/info',
                                 lambda m: setattr(self, 'platform', m), qos_profile_sensor_data)
        self.arm_cli = self.create_client(SetBool, '/drone0/set_arming_state')
        self.offboard_cli = self.create_client(SetBool, '/drone0/set_offboard_mode')

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

    @staticmethod
    def stamp_sec(stamp):
        # Accept either a float (already seconds) or a ROS Time/Builtin Time
        # message and return wall seconds.
        if hasattr(stamp, 'sec'):
            return stamp.sec + 1e-9 * stamp.nanosec
        return stamp

    def fresh(self, stamp, now, limit):
        if stamp is None:
            return False
        return abs(now - self.stamp_sec(stamp)) <= limit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--instruction', required=True)
    parser.add_argument('--max-duration-s', type=float, default=180.0)
    parser.add_argument('--target', type=float, nargs=3, required=True,
                        help='ground-truth target x y z (odom frame, scoring only)')
    parser.add_argument('--spawn', type=float, nargs=3, default=[10.0, 10.0, 45.0],
                        help='teleport spawn x y yaw_deg')
    parser.add_argument('--rpc-host', default=None)
    parser.add_argument('--rpc-port', type=int, default=None)
    parser.add_argument('--fault', choices=['none', 'cancel', 'stale', 'vlm-down'], default='none')
    parser.add_argument('--timeout', type=float, default=420.0)
    args = parser.parse_args()
    if args.rpc_port is None:
        args.rpc_port = json.loads(Path(__file__).with_name('settings.json').read_text())['ApiServerPort']
    args.output.mkdir(parents=True, exist_ok=True)
    # exist_ok=True: the observer container may legitimately create the case
    # directory first; overwrite protection stays in run.py, which refuses an
    # existing case name before launching anything.

    rpc = Rpc(args.rpc_host, args.rpc_port) if args.rpc_host else None
    rclpy.init()
    mission = Mission(args)
    result = {'instruction': args.instruction, 'fault': args.fault,
              'navigation_timeout_s': args.max_duration_s, 'gates': {}, 'samples': []}
    stage = 'readiness'
    try:
        mission.spin_until(lambda: mission.clock is not None, 60, 'no /clock')
        now0 = mission.clock
        mission.spin_until(
            lambda: (mission.pose and mission.camera_info
                     and mission.fresh(mission.pose.header.stamp, now0, 30)
                     and mission.status and mission.status.get('ready')),
            90, 'compute role not ready')
        result['gates']['readiness'] = 'PASS'

        # Teleport initial condition (Blocks ignores settings X/Y; official RPC
        # mechanism, same as upstream tooling). Must run before takeoff.
        if rpc:
            x, y, yaw = args.spawn
            qz, qw = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
            rpc.call('simSetVehiclePose', {
                'position': {'x_val': x, 'y_val': y, 'z_val': 0.0},
                'orientation': {'x_val': 0.0, 'y_val': 0.0, 'z_val': qz, 'w_val': qw}}, True, 'PX4')
            time.sleep(0.5)
            # The EKF needs seconds to re-converge onto the teleported GPS
            # fix. Arming/taking off before that starts the flight from the
            # stale (0,0) estimate and the planner then commands motion in a
            # confused frame (observed in run-20261116v: pose jumped to the
            # spawn only mid-flight and the segment hovered in place).
            mission.spin_until(
                lambda: (mission.pose
                         and abs(mission.pose.pose.position.x - x) < 1.5
                         and abs(mission.pose.pose.position.y - y) < 1.5),
                60, 'EKF did not converge onto teleported pose')
        result['gates']['teleport'] = 'PASS' if rpc else 'NOT-EXECUTED'

        stage = 'takeoff'
        # The pixhawk platform requires explicit arming + offboard before the
        # TakeoffBehavior climbs (same sequence as the yopo demo). On a fresh
        # PX4 start the preflight checks (EKF/GPS/sensor calibration) may still
        # reject the first ARM command ("PX4 command 400 result 1"), so settle
        # briefly and retry arming until the platform reports armed.
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
                # After a teleport the EKF needs extra seconds to damp the
                # GPS drift the jump introduces (run y: second mission in one
                # Blocks session was denied arming for minutes).
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

        # Collision must be sampled DURING navigation, not after Land: the
        # flag is sticky for the whole Blocks session and normal touchdown
        # sets it. This Blocks build has no resetLastCollision (run z4), so
        # new flight contacts are detected by the collision time_stamp
        # changing from the baseline captured after takeoff (spawn/teleport
        # residue sets has_collided early but is excluded by timestamp).
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
        if not mission.navigation.wait_for_server(timeout_sec=15.0):
            raise TimeoutError('to_semantic_goal unavailable')
        goal = NavigateToSemanticGoal.Goal()
        goal.description = args.instruction
        goal.timeout = args.max_duration_s
        accepted = mission.navigation.send_goal_async(goal)
        rclpy.spin_until_future_complete(mission, accepted, timeout_sec=15.0)
        if not accepted.done():
            accepted.add_done_callback(
                lambda f: f.result().cancel_goal_async() if f.result().accepted else None)
            raise TimeoutError('to_semantic_goal acceptance timeout')
        navigation_handle = accepted.result()
        if not navigation_handle.accepted:
            raise RuntimeError('to_semantic_goal rejected')
        navigation_result = navigation_handle.get_result_async()

        injected = False
        deadline = time.time() + args.timeout
        last_sample = 0.0
        while time.time() < deadline:
            rclpy.spin_once(mission, timeout_sec=0.1)
            if mission.args.fault == 'cancel' and not injected and len(mission.samples) >= 3:
                injected = True
                navigation_handle.cancel_goal_async()
                (args.output / 'fault-injection.json').write_text(json.dumps(
                    {'fault': 'cancel', 'wall': time.time(), 'cycles_seen': len(mission.samples)}))
            elif args.fault in ('stale', 'vlm-down') and not injected and len(mission.samples) >= 3:
                injected = True
                (args.output / 'fault-request.json').write_text(json.dumps(
                    {'fault': args.fault, 'wall': time.time()}))
            if mission.pose and time.time() - last_sample > 1.0:
                last_sample = time.time()
                p = mission.pose.pose.position
                sample = {'t': mission.clock, 'pos': [p.x, p.y, p.z],
                          'status': mission.status}
                mission.samples.append(sample)
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
            if navigation_result.done():
                break

        if not navigation_result.done():
            navigation_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(mission, navigation_result, timeout_sec=15.0)
        if not navigation_result.done():
            raise TimeoutError('navigation stop unconfirmed')
        terminal = navigation_result.result().result
        result['navigation_result'] = dict(
            success=terminal.success, code=terminal.code, reason=terminal.reason,
            stop_confirmed=terminal.stop_confirmed)
        end = mission.pose.pose.position
        d1 = math.dist((end.x, end.y, end.z), (tx, ty, tz))
        status = mission.status or {}
        result['samples'] = mission.samples
        result['distance'] = {'start': round(d0, 2), 'end': round(d1, 2),
                              'net_approach_ratio': round((d0 - d1) / d0, 4) if d0 > 0.1 else None}
        result['final_status'] = status
        cycles = status.get('cycles_done', 0)
        attempts = status.get('vlm_attempts', 0)
        result['metrics'] = {'cycles_done': cycles, 'vlm_attempts': attempts,
                             'cycle_completion_ratio': round(cycles / attempts, 4) if attempts else None}
        if args.fault == 'none':
            result['gates']['trajectory_execution'] = 'PASS' if cycles > 0 else 'FAIL'
        # This gate verifies the timeout-only contract, NOT goal attainment.
        expected_codes = {'none': ('TIMEOUT',), 'cancel': ('CANCELED',),
                          'stale': ('NOT_READY',),
                          'vlm-down': ('EXECUTION_FAILED', 'TIMEOUT')}[args.fault]
        result['gates']['navigation_contract'] = (
            'PASS' if terminal.code in expected_codes and not terminal.success else 'FAIL')
        if args.fault != 'none':
            result['gates']['fault_injected'] = 'PASS' if injected else 'FAIL'
        result['gates']['stop_state'] = 'PASS' if terminal.stop_confirmed else 'FAIL'
        if not terminal.stop_confirmed:
            raise RuntimeError('navigation stop unconfirmed; recovery required')

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
        # NOT-EXECUTED marks a structurally inapplicable gate (e.g. cycles for a
        # fault case); it is neither a pass nor a failure. Any FAIL fails the run.
        result['success'] = all(v in ('PASS', 'NOT-EXECUTED')
                                for v in result['gates'].values())
    except Exception as exc:
        result['error'] = f'{type(exc).__name__} at {stage}: {exc}'
        result['success'] = False
        # Bounded recovery: native Land even after task failure.
        try:
            mission.run_action(mission.land, Land, Land.Goal(land_speed=0.5), 30)
        except Exception:
            pass
    finally:
        (args.output / 'result.json').write_text(json.dumps(result, indent=2))
        rclpy.shutdown()
    print(json.dumps({'success': result['success'], 'gates': result['gates'],
                      'distance': result.get('distance')}, indent=2))
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
