"""Passive FlowPilot timing gate; never arms, publishes goals or controls motion."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener
from as2_msgs.msg import PlatformInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from uav_inference_interfaces.srv import InferFlowPilot
from uav_flowpilot_planner.lifecycle import FlowPilotLifecycle, seconds
from uav_flowpilot_planner.node import FlowPilotPlanner
import yaml


def assess(response, context, now, elapsed, timeout, age, reserve):
    """Use the frozen request and the same temporal bounds as the planner."""
    req = context['request']
    valid = bool(response is not None and response.success)
    if valid:
        wp = np.asarray(response.waypoints, dtype=float)
        cp = np.asarray(response.control_points, dtype=float)
        valid = bool(response.request_id == req.request_id
                     and response.observation_header == req.observation_header
                     and wp.shape == (576,) and cp.shape == (24,)
                     and np.isfinite(wp).all() and np.isfinite(cp).all())
    ages = [now-seconds(req.depth.header.stamp), now-context['observation']]
    return dict(response_valid=valid, deadline_met=elapsed <= timeout,
                observation_age_met=all(0 <= value <= age for value in ages),
                submission_reserve_met=context['epoch']-now >= reserve,
                elapsed_s=elapsed, observation_ages_s=ages,
                submission_remaining_s=context['epoch']-now,
                message=response.message if response is not None else 'ROS response timeout')


class Preflight(Node):
    # Reuse this model's actual snapshot/TF/geometry logic, without its task,
    # goal publishers, Action clients, or execution timer.
    fixed_transform = FlowPilotLifecycle.fixed_transform
    on_pose = FlowPilotLifecycle.on_pose
    on_twist = FlowPilotLifecycle.on_twist
    convert_twist = FlowPilotLifecycle.convert_twist
    on_depth = FlowPilotLifecycle.on_depth
    on_info = FlowPilotLifecycle.on_info
    geometry_ready = FlowPilotLifecycle.geometry_ready
    observation_pair = FlowPilotLifecycle.observation_pair
    inputs_fresh = FlowPilotLifecycle.inputs_fresh
    observations_ready = FlowPilotLifecycle.observations_ready

    def __init__(self, params, namespace, goal):
        super().__init__('flowpilot_preflight', namespace=namespace,
                         parameter_overrides=[Parameter('use_sim_time', value=params['use_sim_time'])])
        self.vehicle = params['vehicle_id']
        self.frame = params['fixed_frame']
        self.optical_frame = params['optical_frame']
        self.depth_model = params['depth_model']
        self.age = float(params['max_observation_age_s'])
        self.skew = float(params['max_observation_skew_s'])
        self.timeout = float(params['inference_timeout_s'])
        self.lead = float(params['handoff_lead_s'])
        self.reserve = float(params['submission_reserve_s'])
        self.command_speed = float(params.get('command_speed_mps', params['max_speed']))
        if (not all(math.isfinite(x) and x > 0 for x in
                    (self.age, self.skew, self.timeout, self.lead, self.reserve, self.command_speed))
                or self.lead < self.timeout+self.reserve or self.lead > .3):
            raise ValueError('invalid planner time budget; check the actual deployment parameters')
        self.rotation_bc = Rotation.from_euler('Y', params['camera_pitch_deg'], degrees=True).as_matrix()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose = self.twist = self.depth = self.info = None
        self.pose_history, self.twist_history, self.pending_twists = (deque(maxlen=200) for _ in range(3))
        self.goal = np.asarray(goal, dtype=float)
        self.task = 'preflight-'+uuid.uuid4().hex
        self.generation = 0
        self.inference = None
        self.platform = None
        self.platform_at = 0.
        self.planner_state = None
        self.planner_at = 0.
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, 'self_localization/pose', self.on_pose, reliable)
        self.create_subscription(TwistStamped, 'self_localization/twist', self.on_twist, reliable)
        self.create_subscription(Image, 'sensor_measurements/flowpilot_front/depth_planar/image', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'sensor_measurements/flowpilot_front/depth_planar/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(PlatformInfo, 'platform/info', self.on_platform, reliable)
        self.create_subscription(String, 'flowpilot/status', self.on_status, 10)
        self.infer_client = self.create_client(InferFlowPilot, 'inference/flowpilot/infer')

    def verify_live_parameters(self, expected):
        client = self.create_client(GetParameters, 'flowpilot_planner/get_parameters')
        if not client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('cannot verify the running planner parameters')
        names = list(expected)
        future = client.call_async(GetParameters.Request(names=names))
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic() < deadline:
            self.spin()
        if not future.done():
            client.remove_pending_request(future)
            future.cancel()
            raise TimeoutError('running planner parameter query timed out')
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                  6: 'bool_array_value', 7: 'integer_array_value', 8: 'double_array_value',
                  9: 'string_array_value'}
        values = future.result().values
        actual, different = {}, []
        for name, value in zip(names, values):
            live = getattr(value, fields[value.type]) if value.type in fields else None
            if live is not None and not isinstance(live, (bool, int, float, str)):
                live = [float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) else x for x in live]
            actual[name] = live
            want = expected[name]
            if isinstance(want, list) and live is not None and not isinstance(live, (str, bytes)) \
                    and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in want):
                # e.g. plan_world_box: compare as numbers, not by container type
                try:
                    same = [float(x) for x in live] == [float(x) for x in want]
                except TypeError:
                    same = False
            else:
                same = live == want
            if not same:
                different.append(name)
        if different:
            raise ValueError('running planner differs from the selected parameter file: '+', '.join(different))
        self.destroy_client(client)
        return actual

    def on_platform(self, msg):
        self.platform, self.platform_at = msg, time.monotonic()

    def on_status(self, msg):
        self.planner_state, self.planner_at = json.loads(msg.data).get('state'), time.monotonic()

    def now_s(self):
        return self.get_clock().now().nanoseconds/1e9

    def failure(self, reason):
        raise ValueError(reason)

    def spin(self):
        rclpy.spin_once(self, timeout_sec=.01)
        pending, self.pending_twists = self.pending_twists, deque(maxlen=200)
        for msg in pending:
            if abs(self.now_s()-seconds(msg.header.stamp)) <= self.age and not self.convert_twist(msg):
                self.pending_twists.append(msg)

    def stationary_platform(self):
        return (self.platform is not None and self.platform.connected and not self.platform.armed
                and time.monotonic()-self.platform_at < 1.
                and self.planner_state in ('IDLE', 'STOPPED')
                and time.monotonic()-self.planner_at < 1.)

    def wait_ready(self, timeout=20., previous_depth=None):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.spin()
            if (self.stationary_platform() and self.infer_client.service_is_ready()
                    and self.observations_ready(self.now_s())
                    and (previous_depth is None or seconds(self.depth.header.stamp) > previous_depth)):
                return
        raise TimeoutError('fresh synchronized inputs, idle planner and connected disarmed platform are required')

    def call(self, response_timeout):
        if not self.stationary_platform():
            raise RuntimeError('platform or planner is not idle and disarmed')
        FlowPilotPlanner.request_inference(self, self.now_s())
        context = self.inference
        future = context['future']
        deadline = time.monotonic()+response_timeout
        while not future.done() and time.monotonic() < deadline:
            self.spin()
            if not self.stationary_platform():
                self.infer_client.remove_pending_request(future)
                future.cancel()
                raise RuntimeError('platform or planner became active during preflight')
        elapsed = time.monotonic()-context['sent']
        if future.done():
            response = future.result()
        else:
            self.infer_client.remove_pending_request(future)
            future.cancel()
            response = None
        row = assess(response, context, self.now_s(), elapsed, self.timeout, self.age, self.reserve)
        row['request_id'] = context['request'].request_id
        row['input_age_at_submit_s'] = context['epoch']-self.lead-seconds(context['request'].depth.header.stamp)
        row['current_inputs_fresh'] = bool(self.inputs_fresh(self.now_s()))
        row['passed'] = all(row[key] for key in ['response_valid', 'deadline_met', 'observation_age_met', 'submission_reserve_met', 'current_inputs_fresh'])
        self.inference = None
        return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', type=Path, default=Path(__file__).with_name('planner.yaml'))
    parser.add_argument('--namespace', default='/drone0')
    parser.add_argument('--goal', type=float, nargs=3, default=[0., 12., 2.])
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--idle-s', type=float, default=15.)
    parser.add_argument('--response-timeout-s', type=float, default=2.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (args.samples < 1 or not math.isfinite(args.idle_s) or args.idle_s < 0
            or not math.isfinite(args.response_timeout_s) or args.response_timeout_s <= 0
            or not all(math.isfinite(x) for x in args.goal)):
        parser.error('finite positive samples/response timeout, nonnegative idle and finite goal required')
    params = yaml.safe_load(args.params.read_text())[args.namespace.rstrip('/')+'/flowpilot_planner']['ros__parameters']
    report = dict(passed=False, motion_requested=False, params_file=str(args.params), parameters=params,
                  idle_s=args.idle_s, samples_required=args.samples, responses=[])
    rclpy.init()
    node = None
    try:
        node = Preflight(params, args.namespace, args.goal)
        if args.response_timeout_s < node.timeout:
            raise ValueError('diagnostic response timeout must cover the planner inference timeout')
        report['running_parameters'] = node.verify_live_parameters(params)
        node.wait_ready()
        report['depth'] = dict(width=node.depth.width, height=node.depth.height,
                               encoding=node.depth.encoding, frame=node.depth.header.frame_id)
        report['publishers'] = {topic: [dict(node=p.node_name, namespace=p.node_namespace)
            for p in node.get_publishers_info_by_topic(topic)] for topic in (
            '/clock', args.namespace+'/self_localization/pose',
            args.namespace+'/self_localization/twist',
            args.namespace+'/sensor_measurements/flowpilot_front/depth_planar/image')}
        if any(len(publishers) != 1 for publishers in report['publishers'].values()):
            raise RuntimeError('expected a unique clock, localization and depth publisher')
        deadline = time.monotonic()+args.idle_s
        while time.monotonic() < deadline:
            node.spin()
            if not node.stationary_platform():
                raise RuntimeError('platform or planner changed during idle test')
        previous_depth = None
        for _ in range(args.samples):
            node.wait_ready(previous_depth=previous_depth)
            previous_depth = seconds(node.depth.header.stamp)
            row = node.call(args.response_timeout_s)
            report['responses'].append(row)
            print(json.dumps(row), flush=True)
            if not row['response_valid']:
                # Do not queue additional work behind a timed out or busy GPU.
                break
        report['passed'] = (len(report['responses']) == args.samples
                            and all(row['passed'] for row in report['responses']))
    except Exception as error:
        report['error'] = str(error)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print('PREFLIGHT_PASS' if report['passed'] else 'PREFLIGHT_FAIL; do not start the flight task', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
