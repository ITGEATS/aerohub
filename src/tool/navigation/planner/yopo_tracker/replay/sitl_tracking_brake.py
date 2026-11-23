"""Fly the live Adapter against a moving AirSim fixture, then interrupt RGB-D."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_as2_sensor_adapter.rpc import Rpc

from forward_brake_fixture import CollisionWatch
from sitl_fixed_reference import Check


ACTOR = 'NativeYoloPerson_20261117'
HORIZONTAL = ('x_val', 'y_val')


class MovingTarget:
    def __init__(self, rpc):
        self.rpc = rpc
        vehicle = rpc.call('simGetObjectPose', 'PX4')['position']
        initial = rpc.call('simGetObjectPose', ACTOR)
        offset = [initial['position'][key] - vehicle[key] for key in HORIZONTAL]
        distance = math.hypot(*offset)
        self.base = deepcopy(initial)
        self.direction = [component / distance for component in offset]
        for key, delta in zip(HORIZONTAL, offset):
            self.base['position'][key] = vehicle[key] + 5. * delta / distance
        self.epoch = None
        self.last = 0
        self.positions = []

    def move(self, stamp, tracking):
        if stamp - self.last < 10**9 // 15:
            return
        self.last = stamp
        if tracking and self.epoch is None:
            self.epoch = stamp
        # Acquire at the previously verified centered range, then walk toward
        # the camera while it tracks. This keeps the fixture in view.
        offset = 0. if self.epoch is None else max(-1.5, -1.7 * (stamp - self.epoch) / 1e9)
        pose = deepcopy(self.base)
        for key, direction in zip(HORIZONTAL, self.direction):
            pose['position'][key] += offset * direction
        if not self.rpc.call('simSetObjectPose', ACTOR, pose, True):
            raise RuntimeError('AirSim could not move the scene actor')
        self.positions.append(dict(sim_ns=stamp, position_ned=pose['position']))


class TrackingMission(Check):
    def __init__(self, output, tracking_revisions):
        super().__init__()
        self.output = output
        self.tracking_revisions = tracking_revisions
        self.start_tracker = self.create_client(Trigger, '/yopo_tracker_live/start')
        self.stop_tracker = self.create_client(Trigger, '/yopo_tracker_live/stop')
        self.stop_behavior = self.create_client(Trigger, '/drone0/follow_timed_path/_behavior/stop')
        self.create_subscription(String, '/yopo_tracker_live/status', self.observe, 20)
        self.events = []
        self.task = None
        self.target = None
        self.watch = None
        self.drive_actor = False

    def now(self):
        return self.get_clock().now().nanoseconds

    def observe(self, msg):
        data = json.loads(msg.data)
        self.events.append(data)
        if data['event'] != 'inference':
            print(json.dumps(data), flush=True)

    def records(self, event):
        return [data for data in self.events if data['event'] == event and data['task_id'] == self.task]

    def wait(self, predicate, seconds=10):
        def check():
            if self.watch and (self.watch.contacts or self.watch.error):
                raise RuntimeError('AirSim collision or collision monitor failure')
            if self.drive_actor:
                self.target.move(self.now(), bool(self.records('submit')))
            return predicate()

        super().wait(check, seconds)

    def run(self, rpc):
        self.preflight(rpc)
        self.wait(self.start_tracker.service_is_ready)
        self.takeoff_and_settle()
        position = self.odom.pose.pose.position
        self.evidence['origin_enu_m'] = [position.x, position.y, position.z]
        impact = rpc.call('simGetCollisionInfo', 'PX4')
        self.watch = CollisionWatch(*rpc.address, impact['time_stamp'])
        self.target = MovingTarget(rpc)
        self.drive_actor = True
        self.target.move(self.now(), False)
        self.phase = 'starting the tracker session'
        started = self.response(self.start_tracker.call_async(Trigger.Request()))
        if not started.success:
            raise RuntimeError(started.message)
        self.task = started.message
        self.phase = 'rolling revisions on the moving target'
        self.wait(lambda: len(self.records('active')) >= self.tracking_revisions or
                  self.records('brake_latched'), 15)
        active = self.records('active')
        if len(active) < self.tracking_revisions or self.records('brake_latched'):
            raise RuntimeError(f'The Adapter stopped before {self.tracking_revisions} tracking revisions took effect')
        self.evidence['tracking_revisions'] = [event['revision'] for event in active]
        position = self.odom.pose.pose.position
        reached = [position.x, position.y, position.z]
        self.evidence['tracking_displacement_m'] = math.dist(self.evidence['origin_enu_m'], reached)
        self.evidence['target_positions'] = len(self.target.positions)
        initial = self.target.positions[0]['position_ned']
        moved = rpc.call('simGetObjectPose', ACTOR)['position']
        self.evidence['target_travel_m'] = math.dist([initial[k] for k in HORIZONTAL],
                                                     [moved[k] for k in HORIZONTAL])
        if not self.evidence['target_travel_m'] > 0:
            raise RuntimeError('The target did not move before the camera interruption')
        self.drive_actor = False
        self.phase = 'RGB-D interruption'
        (self.output / 'pause-camera').touch()
        self.wait(lambda: self.records('brake_latched'))
        brake = self.records('brake_latched')[0]
        self.evidence['brake_reason'] = brake['reason']
        if brake['reason'] != 'RGBD_TIMEOUT':
            raise RuntimeError(f'Expected RGB-D interruption, got {brake["reason"]}')
        self.evidence['last_images'] = self.images.copy()
        self.phase = 'protected brake reaches low speed'
        self.wait(lambda: self.records('brake_settled') or self.records('finished'))
        self.phase = 'final FollowTimedPath result'
        self.wait(lambda: self.records('finished'))
        result = self.records('finished')[-1]
        self.evidence['execution_result'] = result
        if not result['success'] or not self.records('brake_settled'):
            raise RuntimeError('Protected braking did not produce a confirmed FollowTimedPath completion')
        self.phase = 'hover after the braking action completed'
        hover = []
        self.settled(lambda mode, stamp, age, speed, good: hover.append(
            dict(sim_ns=self.now(), mode=int(mode), odom_ns=stamp, age_ns=age, speed_m_s=speed, eligible=good)))
        self.evidence['settled'] = True
        self.evidence['hover_window'] = dict(first=hover[0], last=hover[-1])
        if self.images != self.evidence['last_images']:
            raise RuntimeError('Camera messages advanced during the interrupted segment')

    def recover_tracker(self):
        self.drive_actor = False
        if self.watch:
            self.watch.close()
            self.evidence['new_collision'] = bool(self.watch.contacts)
            self.evidence['collision_check_error'] = self.watch.error
            self.watch = None
        if self.task and not self.records('finished'):
            try:
                self.response(self.stop_tracker.call_async(Trigger.Request()))
                self.wait(lambda: self.records('finished'))
            except RuntimeError:
                raise
            except TimeoutError:
                reply = self.response(self.stop_behavior.call_async(Trigger.Request()))
                if not reply.success:
                    raise RuntimeError(f'FollowTimedPath recovery stop was rejected: {reply.message}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', required=True, type=int)
    parser.add_argument('--tracking-revisions', type=int, choices=(2, 3), default=3)
    args = parser.parse_args()
    args.output.mkdir()
    rclpy.init(args=[])
    node = TrackingMission(args.output, args.tracking_revisions)
    node.evidence['required_tracking_revisions'] = args.tracking_revisions
    rpc = Rpc(args.host, args.port, 3)
    try:
        try:
            node.run(rpc)
        except Exception as error:
            node.evidence['error'] = f'{node.phase}: {error}'
        try:
            node.recover_tracker()
        except Exception as error:
            node.evidence['tracker_recovery_error'] = str(error)
        try:
            node.recover()
        except Exception as error:
            node.evidence['landing_error'] = str(error)
        data = node.evidence
        data['passed'] = bool(data.get('settled') and data.get('land') and data.get('disarmed') and
                              data.get('new_collision') is False and not data.get('collision_check_error') and
                              not data.get('error'))
        (args.output / 'result.json').write_text(json.dumps(data, indent=2) + '\n')
        (args.output / 'tracker-events.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in node.events))
        if node.target:
            (args.output / 'target-motion.json').write_text(json.dumps(node.target.positions, indent=2) + '\n')
        print(json.dumps(data), flush=True)
        return 0 if data['passed'] else 1
    finally:
        rpc.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
