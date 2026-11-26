"""Workstation ROS task client for the FlowPilot SITL closed loop.

Adapted from yopo_single/mission.py: same four-role discipline (AirSim RPC is
used exclusively for collision scoring and the pre-goal depth snapshot), with
the FlowPilot planner topics and stop reasons.
"""
import argparse
import json
import math
import struct
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.action import Land, Takeoff
from as2_msgs.msg import PlatformInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool
from as2_python_api.drone_interface_base import DroneInterfaceBase
from uav_as2_sensor_adapter.rpc import Rpc
from result_checks import cancel_case_passed
try:
    from px4_msgs.msg import VehicleCommand, VehicleCommandAck
except ImportError:  # without px4_msgs the exit can only be judged by the AS2 service
    VehicleCommand = VehicleCommandAck = None

PX4_TARGET_SYSTEM = 71  # fleet.yaml target_system_id for drone0


class Mission(Node):
    def __init__(self, output, goal, rpc_host, rpc_port, cancel_after_revision=0):
        super().__init__('flowpilot_mission',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.output, self.goal = output, list(goal)
        self.cancel_after_revision = cancel_after_revision
        self.platform = None
        self.platform_at = self.pose_at = self.twist_at = 0.
        self.drone = DroneInterfaceBase('drone0', use_sim_time=True)
        self.drone.load_module('navigation')
        self.navigation = self.drone.navigation
        self.navigation_started = False
        self.status = {}
        self.pose = self.twist = None
        self.status_at = 0.
        self.samples = (output / 'mission.jsonl').open('w')
        self.events = (output / 'plans.jsonl').open('w')
        self.rpc = Rpc(rpc_host, rpc_port, 3)
        self.result = dict(success=False, motion_requested=False, arrived_5m=False,
                           stopped=False, land_success=False, disarmed=False, collision=False)
        self.create_subscription(PlatformInfo, '/drone0/platform/info',
                                 self.on_platform, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 self.on_pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/drone0/self_localization/twist',
                                 self.on_twist, qos_profile_sensor_data)
        self.create_subscription(String, '/drone0/flowpilot/status', self.on_status, 10)
        self.create_subscription(String, '/drone0/flowpilot/events', self.on_event, 100)
        self.command_acks = []
        self.vehicle_command_pub = None
        if VehicleCommand is not None:
            # Same PX4 interface the AS2 pixhawk platform uses, for the
            # operator-side exit after a landed, disarmed flight.
            self.vehicle_command_pub = self.create_publisher(
                VehicleCommand, '/drone0/fmu/in/vehicle_command', qos_profile_sensor_data)
            self.create_subscription(VehicleCommandAck, '/drone0/fmu/out/vehicle_command_ack',
                                     self.on_command_ack, qos_profile_sensor_data)
        self.task_clients = []
        self.last_clock_ns = None
        self.clock_samples = 0
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.last_collision = 0.
        self.initial_collision = None

    def on_platform(self, msg):
        self.platform, self.platform_at = msg, time.monotonic()

    def on_pose(self, msg):
        self.pose, self.pose_at = msg, time.monotonic()

    def on_twist(self, msg):
        self.twist, self.twist_at = msg, time.monotonic()

    def fresh_platform(self):
        return self.platform is not None and time.monotonic() - self.platform_at < 1.

    def fresh_pose(self):
        return self.pose is not None and time.monotonic() - self.pose_at < .3 and 0 <= (
            self.get_clock().now().nanoseconds / 1e9 -
            (self.pose.header.stamp.sec + self.pose.header.stamp.nanosec / 1e9)) <= .3

    def on_clock(self, msg):
        stamp = msg.clock.sec * 1000000000 + msg.clock.nanosec
        if self.last_clock_ns is not None and stamp < self.last_clock_ns:
            raise RuntimeError('simulation clock moved backwards; stop flight validation')
        self.last_clock_ns = stamp
        self.clock_samples += 1

    def graph_ready(self):
        expected = ['/clock', '/drone0/self_localization/pose',
                    '/drone0/self_localization/twist', '/drone0/flowpilot/status']
        return self.clock_samples >= 25 and all(
            self.count_publishers(topic) == 1 for topic in expected)

    def on_event(self, msg):
        self.events.write(msg.data + '\n')
        self.events.flush()

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        self.status_at = time.monotonic()
        sample = dict(wall=time.time(), sim=self.get_clock().now().nanoseconds / 1e9,
                      **self.status)
        if self.pose is not None:
            sample['pose_frame'] = self.pose.header.frame_id
            sample['pose'] = [self.pose.pose.position.x, self.pose.pose.position.y,
                              self.pose.pose.position.z]
        if self.twist is not None:
            sample['velocity_frame'] = self.twist.header.frame_id
            sample['velocity'] = [self.twist.twist.linear.x, self.twist.twist.linear.y,
                                  self.twist.twist.linear.z]
        self.samples.write(json.dumps(sample) + '\n')
        self.samples.flush()

    def wait(self, predicate, timeout=20, score=False):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('condition timeout: ' + str(self.status))
            rclpy.spin_once(self, timeout_sec=.02)
            if score and time.monotonic() - self.last_collision > .2:
                collision = self.rpc.call('simGetCollisionInfo', 'PX4')
                self.last_collision = time.monotonic()
                if collision['has_collided'] and collision['time_stamp'] != self.initial_collision:
                    self.result['collision'] = collision
                    raise RuntimeError('AirSim collision')
            if score and time.monotonic() - self.status_at > 1:
                raise RuntimeError('planner status lost')

    def service(self, kind, name, request, timeout=10):
        client = self.create_client(kind, name)
        self.task_clients.append(client)
        self.wait(client.service_is_ready, timeout)
        future = client.call_async(request)
        self.wait(future.done, timeout)
        response = future.result()
        if not response.success:
            raise RuntimeError(name + ' rejected: ' + str(response))

    def on_command_ack(self, msg):
        self.command_acks.append(dict(command=msg.command, result=msg.result,
                                      target_system=msg.target_system))

    def offboard_state_released(self, timeout):
        try:
            self.wait(lambda: self.fresh_platform() and not self.platform.offboard, timeout)
        except TimeoutError:
            return False
        return True

    def command_px4_mode(self, main_mode, attempts=5):
        # Same VehicleCommand the AS2 pixhawk platform issues for OFFBOARD
        # (DO_SET_MODE with the custom-mode flag and a PX4 main mode), aimed
        # at the fleet's PX4 system id. Returns the acks PX4 sent for it.
        if self.vehicle_command_pub is None:
            return dict(error='px4_msgs unavailable in this client')
        self.command_acks.clear()
        for _ in range(attempts):
            message = VehicleCommand()
            message.timestamp = self.get_clock().now().nanoseconds // 1000
            message.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            message.param1 = 1.  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            message.param2 = float(main_mode)  # 3 = PX4_CUSTOM_MAIN_MODE_POSCTL
            message.param3 = 0.
            message.target_system = PX4_TARGET_SYSTEM
            message.target_component = 1
            message.source_system = 1
            message.source_component = 1
            message.from_external = True
            self.vehicle_command_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=.1)
        return dict(main_mode=main_mode, acks=list(self.command_acks))

    def action(self, kind, name, goal):
        client = ActionClient(self, kind, name)
        self.task_clients.append(client)
        self.wait(client.server_is_ready)
        accepted = client.send_goal_async(goal)
        self.wait(accepted.done)
        handle = accepted.result()
        if not handle.accepted:
            raise RuntimeError(name + ' goal rejected')
        future = handle.get_result_async()
        self.wait(future.done, 60)
        result = future.result()
        if result.status != 4 or not getattr(
                result.result, 'takeoff_success' if kind is Takeoff else 'land_success'):
            raise RuntimeError(name + ' failed: ' + str(result))

    def snapshot_depth(self):
        responses = self.rpc.call('simGetImages', [
            dict(camera_name='flowpilot_front', image_type=1,
                 pixels_as_float=True, compress=False)], 'PX4', False)
        image = responses[0]
        depth = image['image_data_float']
        width, height = image['width'], image['height']
        if not width or not height or len(depth) != width * height:
            raise RuntimeError('invalid actual AirSim depth snapshot')
        (self.output / 'depth-before-goal.f32').write_bytes(
            struct.pack('<' + str(len(depth)) + 'f', *depth))
        valid = [v for v in depth if 0. < v < 100.]
        meta = dict(width=width, height=height, stamp=image['time_stamp'],
                    encoding='little-endian float32 meters axial',
                    valid_pixels=len(valid),
                    min_m=round(min(valid), 3) if valid else None,
                    max_m=round(max(valid), 3) if valid else None)
        (self.output / 'depth-before-goal.json').write_text(json.dumps(meta, indent=2) + '\n')
        self.result['depth_snapshot'] = meta

    def run(self, ground_only=False, timeout_s=90.):
        self.result.update(goal_enu_m=self.goal, navigation_timeout_s=timeout_s)
        try:
            self.wait(lambda: self.fresh_platform() and self.status.get('ready')
                      and self.pose is not None and self.graph_ready(), 45)
            if self.platform.armed:
                raise RuntimeError('requires disarmed platform')
            collision = self.rpc.call('simGetCollisionInfo', 'PX4')
            self.initial_collision = collision['time_stamp']
            self.result['ground_ready'] = True
            self.result['motion_reference_publishers'] = {
                topic: [dict(node=p.node_name, namespace=p.node_namespace, type=p.topic_type)
                        for p in self.get_publishers_info_by_topic(topic)]
                for topic in ['/drone0/motion_reference/trajectory',
                              '/drone0/motion_reference/pose']}
            if ground_only:
                self.result.update(success=True, disarmed=True)
                return
            self.result['motion_requested'] = True
            # PX4 health checks re-converge for tens of seconds after each
            # disarm; a denied arm is temporary, so retry instead of aborting.
            for attempt in range(3):
                try:
                    self.service(SetBool, '/drone0/set_arming_state', SetBool.Request(data=True))
                    self.wait(lambda: self.fresh_platform() and self.platform.armed, 8)
                    break
                except TimeoutError:
                    if attempt == 2:
                        raise
                    self.result['arming_retry'] = attempt + 1
                    time.sleep(5)
            self.service(SetBool, '/drone0/set_offboard_mode', SetBool.Request(data=True))
            self.wait(lambda: self.fresh_platform() and self.platform.offboard)
            self.action(Takeoff, '/drone0/TakeoffBehavior',
                        Takeoff.Goal(takeoff_height=2., takeoff_speed=.5))
            # Let measured motion settle; the planner uses a future reference epoch.
            until = time.monotonic() + 3
            self.wait(lambda: time.monotonic() >= until)
            self.snapshot_depth()
            previous_task = self.status.get('task_id')
            self.navigation_started = self.navigation.to_point(*self.goal, wait=False, timeout=timeout_s)
            if not self.navigation_started:
                raise RuntimeError('public navigation goal rejected')
            self.wait(lambda: self.status.get('task_id') != previous_task
                      and self.status.get('state') == 'RUNNING' or self.navigation.done, 5, score=True)
            if self.cancel_after_revision:
                self.wait(lambda: self.status.get('effective_revision', 0) >= self.cancel_after_revision,
                          timeout_s, score=True)
                cancel_future = self.navigation.cancel_async()
                if cancel_future is None:
                    raise RuntimeError('no active owned task at cancellation injection')
                self.result['fault_injected'] = True
                self.wait(cancel_future.done, 5)
                self.result['cancel_ack'] = bool(cancel_future.result().goals_canceling)
            self.wait(lambda: self.navigation.done, timeout_s + 12, score=True)
            public_result = self.navigation.result
            self.result['navigation_result'] = dict(success=public_result.success, code=public_result.code,
                reason=public_result.reason, stop_confirmed=public_result.stop_confirmed)
            # Action result and diagnostic status travel independently over DDS.
            if public_result.code in ('SUCCEEDED', 'CANCELED') or self.status.get('task_id') != previous_task:
                self.wait(lambda: self.status.get('task_id') != previous_task
                          and self.status.get('state') in ('STOPPED', 'FAULT'), 2, score=True)
            self.result['arrived_5m'] = 'ARRIVED_5M' in self.status.get('reason', '')
            self.result['stopped'] = self.status.get('state') == 'STOPPED'
            self.result['stop_reason'] = self.status.get('reason')
            task_id = self.status.get('task_id')
            self.result['task_id'] = task_id
            rows = [json.loads(line) for line in (self.output / 'mission.jsonl').read_text().splitlines()]
            rows = [row for row in rows if task_id and row.get('task_id') == task_id]
            self.result['max_revision'] = max((row.get('revision', 0) for row in rows), default=0)
            self.result['max_effective_revision'] = max((row.get('effective_revision', 0) for row in rows), default=0)
            if self.fresh_pose():
                self.result['final_distance_raw_m'] = math.dist(
                    [self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z], self.goal)
                self.result['final_distance_m'] = round(math.dist(
                    [self.pose.pose.position.x, self.pose.pose.position.y,
                     self.pose.pose.position.z], self.goal), 3)
            self.result['final_within_5m'] = bool(
                self.result.get('final_distance_raw_m') is not None
                and self.result['final_distance_raw_m'] <= 5.)
            if not self.cancel_after_revision and (not public_result.success or not public_result.stop_confirmed
                    or self.status.get('state') != 'STOPPED'
                    or not self.status.get('reason', '').startswith('ARRIVED_5M')):
                raise RuntimeError('planner ended: ' + str(self.status))
            self.result['expected_stop'] = not bool(self.cancel_after_revision)
            if self.cancel_after_revision:
                self.result['expected_cancel'] = (public_result.code == 'CANCELED' and public_result.stop_confirmed
                    and self.status.get('state') == 'STOPPED' and self.status.get('reason') == 'CANCELED')
        except Exception as error:
            self.result['error'] = str(error)
        finally:
            if self.platform is not None and self.platform.armed:
                try:
                    try:
                        if self.status.get('state') not in ('STOPPED', 'FAULT', 'IDLE'):
                            self.navigation.cancel()
                    except Exception as error:
                        self.result['cancel_error'] = str(error)
                    try:
                        self.wait(lambda: self.status.get('state') in ('STOPPED', 'FAULT'), 12)
                    except Exception as error:
                        self.result['stop_error'] = str(error)
                    try:
                        self.action(Land, '/drone0/LandBehavior', Land.Goal(land_speed=.5))
                        self.result['land_success'] = True
                    except Exception as error:
                        # Keep going: the disarm check below must still run.
                        self.result['land_error'] = str(error)
                    # Native Land publishes its result before its own disarm cleanup.
                    try:
                        self.wait(lambda: self.fresh_platform() and not self.platform.armed, 5)
                    except TimeoutError:
                        if self.fresh_platform() and self.result.get('land_success'):
                            try:
                                self.service(SetBool, '/drone0/set_arming_state',
                                              SetBool.Request(data=False))
                            except RuntimeError:
                                self.wait(lambda: self.fresh_platform() and not self.platform.armed, 3)
                            self.wait(lambda: self.fresh_platform() and not self.platform.armed, 15)
                        else:
                            # Never disarm in the air after a failed Land; leave it armed
                            # so the operator (and the round driver's assert) sees it.
                            self.result['disarm_skipped'] = 'Land failed and fresh armed state persists; ground not independently proven'
                except Exception as error:
                    self.result['recovery_error'] = str(error)
            self.result['disarmed'] = self.fresh_platform() and not self.platform.armed
            if self.result['motion_requested'] and self.result['disarmed']:
                # as2_platform_pixhawk refuses an onboard-initiated offboard
                # exit by design (Turning into MANUAL Mode is not allowed
                # from the onboard computer), and PX4 stays in OFFBOARD after
                # disarm. The service response is kept as a diagnostic; the
                # release verdict is the platform state mirrored from PX4.
                try:
                    self.service(SetBool, '/drone0/set_offboard_mode',
                                  SetBool.Request(data=False))
                except Exception as error:
                    self.result['offboard_release_error'] = str(error)
                released = self.offboard_state_released(2.)
                if not released:
                    # Operator-side exit, as a GCS or RC pilot would do after a
                    # landed, disarmed Offboard flight: command PX4 to POSCTL
                    # directly. Only reachable once the platform is disarmed.
                    self.result['offboard_exit_command'] = self.command_px4_mode(3)
                    released = self.offboard_state_released(4.)
                self.result['offboard_released'] = released
                if not released:
                    self.result['offboard_release_state_error'] = (
                        'platform still reports offboard=true after exit attempts')
            if not ground_only:
                self.result['success'] = bool(
                    self.result.get('expected_stop') and self.result['stopped']
                    and self.result['land_success'] and self.result['disarmed']
                    and self.result.get('final_within_5m')
                    and self.result.get('offboard_released')
                    and not self.result.get('error'))
            self.samples.close()
            self.events.close()
            self.result['success'] = bool(self.result['success'] and not self.result['collision'] and (ground_only or self.result.get('max_effective_revision', 0) >= 2))
            if self.cancel_after_revision:
                self.result['fault_passed'] = cancel_case_passed(self.result)
                (self.output / 'fault-result.json').write_text(json.dumps(dict(kind='cancel', passed=self.result['fault_passed'],
                    expected_reason='CANCELED', normal_mission_success=self.result['success']), indent=2)+'\n')
            (self.output / 'result.json').write_text(json.dumps(self.result, indent=2) + '\n')
            print(json.dumps(self.result), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ground-only', action='store_true')
    parser.add_argument('--cancel-after-revision', type=int, default=0)
    parser.add_argument('--goal', type=float, nargs=3, required=True)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--rpc-host', required=True)
    parser.add_argument('--rpc-port', type=int, required=True)
    args = parser.parse_args()
    if args.cancel_after_revision < 0 or (args.ground_only and args.cancel_after_revision):
        parser.error('cancel injection requires a flight and a positive revision')
    if math.isfinite(args.timeout) is False or args.timeout <= 0 or not all(
            math.isfinite(v) for v in args.goal):
        parser.error('timeout must be finite positive; goal must be finite')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Mission(args.output, args.goal, args.rpc_host, args.rpc_port, args.cancel_after_revision)
    try:
        node.run(args.ground_only, args.timeout)
        return 0 if node.result.get('fault_passed' if args.cancel_after_revision else 'success') else 1
    finally:
        node.drone.shutdown()
        node.drone.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
