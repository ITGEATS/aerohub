"""Validate a fixed two-segment reference on a dedicated AirSim PX4 SITL instance."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from as2_msgs.action import Land, Takeoff
from as2_msgs.msg import ControllerInfo, ControlMode, PlatformInfo
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from uav_as2_sensor_adapter.rpc import Rpc
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal

from check_action import SECOND, boundary, ns
from timed_points import timed_goal


class Check(Node):
    def __init__(self):
        super().__init__('yopo_fixed_reference_check',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.platform = self.odom = self.controller = None
        self.clock = []
        self.images = {}
        self.feedback = {}
        self.reference_handle = None
        self.arm_requested = False
        self.evidence = {}
        self.phase = 'preflight'
        self.takeoff = ActionClient(self, Takeoff, '/drone0/TakeoffBehavior')
        self.land = ActionClient(self, Land, '/drone0/LandBehavior')
        self.action = ActionClient(self, FollowTimedPath, '/drone0/follow_timed_path')
        self.modify = self.create_client(FollowTimedPath_SendGoal, '/drone0/follow_timed_path/_behavior/modify')
        self.arming = self.create_client(SetBool, '/drone0/set_arming_state')
        self.offboard = self.create_client(SetBool, '/drone0/set_offboard_mode')
        for kind, topic, field in (
                (PlatformInfo, '/drone0/platform/info', 'platform'),
                (Odometry, '/drone0/sensor_measurements/odom', 'odom'),
                (ControllerInfo, '/drone0/controller/info', 'controller')):
            self.create_subscription(kind, topic, lambda msg, key=field: setattr(self, key, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', lambda msg: self.clock.append(ns(msg.clock)),
                                 qos_profile_sensor_data)
        for stream in ('rgb', 'depth_planar'):
            topic = f'/drone0/sensor_measurements/FrontCamera/{stream}/image'
            self.create_subscription(Image, topic, lambda msg, key=stream: self.images.update(
                {key: ns(msg.header.stamp)}), qos_profile_sensor_data)

    def wait(self, predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'SITL condition not reached during {self.phase}')
            rclpy.spin_once(self, timeout_sec=.02)

    def response(self, future, seconds=10):
        self.wait(future.done, seconds)
        return future.result()

    def toggle(self, client, value):
        self.wait(client.service_is_ready)
        if not self.response(client.call_async(SetBool.Request(data=value))).success:
            raise RuntimeError(f'{client.srv_name} rejected')

    def run_action(self, client, goal, feedback=None):
        self.wait(client.server_is_ready)
        handle = self.response(client.send_goal_async(goal, feedback_callback=feedback))
        if not handle.accepted:
            raise RuntimeError(f'{type(goal).__qualname__} rejected')
        return handle

    def stop_reference(self):
        if self.reference_handle is None:
            return
        handle = self.reference_handle
        result = handle.get_result_async()
        self.response(handle.cancel_goal_async())
        terminal = self.response(result)
        self.evidence.update(reference_terminal=terminal.status,
                             reference_reason=terminal.result.reason,
                             reference_revision=int(terminal.result.revision))
        self.reference_handle = None

    def settled(self, observe=None, *, seconds=10.,
                modes=(ControlMode.HOVER, ControlMode.POSITION)):
        # Reuse the ordinary YOPO SITL demo's speed and hold criteria.
        held_since = None

        def stopped():
            nonlocal held_since
            stamp = self.get_clock().now().nanoseconds
            mode = self.controller.input_control_mode.control_mode
            velocity = self.odom.twist.twist.linear
            odom_stamp = ns(self.odom.header.stamp)
            speed = math.hypot(velocity.x, velocity.y, velocity.z)
            good = (mode in modes
                    and speed < .2 and 0 <= stamp - odom_stamp < SECOND // 2)
            if observe:
                observe(mode, odom_stamp, stamp - odom_stamp, speed, good)
            if not good:
                held_since = None
                return False
            if held_since is None:
                held_since = stamp
            return stamp - held_since >= SECOND

        self.wait(stopped, seconds)

    def preflight(self, rpc):
        self.phase = 'platform, localization, clock and images'
        self.wait(lambda: (self.platform and self.platform.connected and self.odom and self.controller
                           and len(self.images) == 2 and len(set(self.clock)) >= 25), 30)
        if (self.platform.armed or self.platform.offboard or
                self.odom.header.frame_id != 'drone0/odom' or
                any(a > b for a, b in zip(self.clock, self.clock[1:]))):
            raise RuntimeError('Unexpected initial SITL state')
        rpc.call('getMultirotorState', 'PX4')

        self.phase = 'FollowTimedPath endpoint discovery'
        self.wait(self.action.server_is_ready)
        self.wait(self.modify.service_is_ready)

    def takeoff_and_settle(self):
        self.phase = 'arming and offboard'
        self.arm_requested = True
        self.toggle(self.arming, True)
        self.wait(lambda: self.platform.armed)
        self.toggle(self.offboard, True)
        self.wait(lambda: self.platform.offboard)
        self.phase = 'takeoff'
        takeoff = self.run_action(self.takeoff, Takeoff.Goal(takeoff_height=2., takeoff_speed=.5))
        takeoff_result = self.response(takeoff.get_result_async(), 60)
        if takeoff_result.status != GoalStatus.STATUS_SUCCEEDED or not takeoff_result.result.takeoff_success:
            raise RuntimeError('SITL takeoff failed')
        self.evidence['takeoff'] = True
        self.phase = 'hover after takeoff'
        self.settled()

    def run(self, rpc, solver):
        self.preflight(rpc)
        self.takeoff_and_settle()
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        origin = np.array([p.x, p.y, p.z])
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')[0]
        image_before = self.images.copy()
        collision_before = rpc.call('simGetCollisionInfo', 'PX4')['time_stamp']
        self.evidence['origin_enu_m'] = origin.tolist()
        duration = 2.
        first_axes = tuple(solver(start, 0., 0., end, 0., 0., duration)
                           for start, end in zip(origin, origin + [.4, 0., 0.]))
        start_ns = self.get_clock().now().nanoseconds
        first_epoch = start_ns + SECOND // 2
        continuation = boundary(first_axes, 1.)
        second_axes = tuple(solver(p, v, a, end, 0., 0., duration)
                            for p, v, a, end in zip(*continuation, origin + [.1, 0., 0.]))
        knots = [(0, yaw), (2 * SECOND, yaw)]
        common = dict(task_id='sitl-fixed-reference', vehicle_id='drone0', max_speed=1.)

        def observe(event):
            msg = event.feedback
            if msg.task_id == 'sitl-fixed-reference' and not msg.waiting_for_start:
                self.feedback[msg.revision] = msg

        self.phase = 'reference goal and modify'
        self.reference_handle = self.run_action(self.action, timed_goal(
            first_axes, duration, knots, revision=1, generated_ns=start_ns,
            start_ns=first_epoch, **common), observe)
        request = FollowTimedPath_SendGoal.Request()
        request.goal = timed_goal(second_axes, duration, knots, revision=2,
                                  generated_ns=self.get_clock().now().nanoseconds,
                                  start_ns=first_epoch + SECOND, **common)
        if not self.response(self.modify.call_async(request)).accepted:
            raise RuntimeError('SITL rolling Modify rejected')
        self.phase = 'reference execution'
        self.wait(lambda: (1 in self.feedback and 2 in self.feedback and
                           self.feedback[2].elapsed >= .5))
        if any(self.images[key] <= image_before[key] for key in image_before):
            raise RuntimeError('Camera streams did not advance during execution')
        collision = rpc.call('simGetCollisionInfo', 'PX4')
        if collision['has_collided'] and collision['time_stamp'] != collision_before:
            raise RuntimeError('AirSim reported a collision during reference execution')
        last = self.odom.pose.pose.position
        self.evidence.update(revisions=sorted(self.feedback), image_ns_before=image_before,
                             image_ns_after=dict(self.images), last_enu_m=[last.x, last.y, last.z],
                             second_elapsed_s=self.feedback[2].elapsed)
        self.phase = 'reference cancellation'
        self.stop_reference()
        if self.evidence['reference_terminal'] != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('FollowTimedPath did not finish in CANCELED')
        self.phase = 'hover after cancellation'
        self.settled()
        self.evidence['settled_after_cancel'] = True

    def recover(self):
        self.phase = 'recovery'
        self.stop_reference()
        if not self.arm_requested or self.platform is None or not self.platform.armed:
            self.evidence['disarmed'] = bool(self.platform and not self.platform.armed)
            return
        land = self.run_action(self.land, Land.Goal(land_speed=.5))
        result = self.response(land.get_result_async(), 60)
        self.evidence['land'] = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.land_success
        self.wait(lambda: not self.platform.armed)
        self.evidence['disarmed'] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, required=True)
    args = parser.parse_args()
    sys.path.insert(0, '/opt/yopo-tracker/upstream/YOPO')
    from policy.poly_solver import Poly5Solver

    rclpy.init(args=[])
    node = Check()
    try:
        try:
            node.run(Rpc(args.host, args.port, 3), Poly5Solver)
        except Exception as error:
            node.evidence['error'] = str(error)
        finally:
            try:
                node.recover()
            except Exception as error:
                node.evidence['recovery_error'] = str(error)
        data = node.evidence
        data['passed'] = bool(data.get('settled_after_cancel') and data.get('land') and
                              data.get('disarmed') and not data.get('error'))
        args.output.write_text(json.dumps(data, indent=2) + '\n')
        print(json.dumps(data))
        return 0 if data['passed'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
