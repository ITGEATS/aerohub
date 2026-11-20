"""aerovla_planner: ROS 2 Tool-layer planner node (board side, stage 3).

Observe -> infer -> timed trajectory loop:

    ~/front, ~/down (sensor_msgs/Image, bgr8) + ~/pose (PoseStamped)
      -> prompts (hint from GT target_position + obj_desc) -> input_ids
      -> service InferAeroVla (uav_inference_interfaces, delivered by
         src/basic_function/inference) -> text
      -> policy.dequantize_bins / is_landing / action_to_setpoints
      -> ~/trajectory (uav_planner_interfaces/TimedTrajectorySegment)

Entry-point choice (documented per task): nav2 NavigateToPose does not fit
a policy Tool whose goal is a GT object position plus a text description.
The established in-repo paradigm for planner Tools is the yopo adapter
(``src/tool/navigation/planner/yopo/ros2_adapter/uav_yopo_planner/node.py``
L104-106): a plain PoseStamped ``goal`` topic starts the loop and a
``std_srvs/Trigger`` ``cancel`` service stops it. AeroVLA follows the same
lightweight convention (``~/goal`` + ``~/cancel``); ``obj_desc`` cannot ride
a PoseStamped, so it is a node parameter, and the capability card declares
the oracle ``target_position`` dependency honestly.

All numeric/prompt semantics live in the ROS-free ``policy``/``prompts``
modules (unit-tested on the host); this file only wires ROS messages.

FRAME ADAPTATION (declared, per AGENTS "compat via Adapter" rule): the
upstream policy is NED-native (AirSim: body +y = right, yaw clockwise+,
"down" positive = descend; hint bands and move_path both assume it). The
ROS world here is ENU. ``policy`` stays verbatim NED; this node converts
ENU pose/target -> NED before calling it and converts the resulting
timed points back to ENU (yaw sign negated) before publishing, so the
published trajectory preserves the exact physical behavior upstream would
have executed.
"""

import copy
import json
import math
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_inference_interfaces.srv import InferAeroVla
from uav_planner_interfaces.msg import (TimedTrajectoryPoint,
                                        TimedTrajectorySegment,
                                        VehicleTrajectory)

from . import policy, prompts


def stamp_to_s(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def duration_msg(seconds):
    ns = int(round(seconds * 1e9))
    sec, nanosec = divmod(ns, 1000000000)
    return Duration(seconds=int(sec), nanoseconds=nanosec).to_msg()


# ENU<->NED frame change: swap x/y and negate z (involutory for vectors).
_ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def enu_ned_vector(v):
    """Applies the involutory ENU<->NED vector map (x,y,z)_enu=(y,x,-z)_ned."""
    return [float(v[1]), float(v[0]), -float(v[2])]


def enu_to_ned_quat(orientation):
    """Attitude quaternion re-expressed in the NED basis: R_ned = W R_enu W^T."""
    q = orientation
    if isinstance(q, dict):
        quat = [q['x'], q['y'], q['z'], q['w']]
    else:
        quat = list(map(float, q))
    matrix = R.from_quat(quat).as_matrix()
    return R.from_matrix(_ENU_TO_NED @ matrix @ _ENU_TO_NED.T).as_quat().tolist()


class AerovlaPlannerNode(Node):
    def __init__(self):
        super().__init__('aerovla_planner')

        def param(name, value):
            return self.declare_parameter(name, value).value

        self.service_name = param('service_name', 'inference/aerovla/infer')
        # Oracle GT target (ENU, m). Declared in the capability card: the
        # fuzzy hint upstream is computed from target coordinates
        # (aerovla_wrapper_ui.py L46-76), not from onboard detection.
        self.target_position = param('target_position', [0.0, 0.0, 0.0])
        self.obj_desc = param('obj_desc', '')
        # Input ids come from a pre-tokenized prompt table (board contract:
        # no transformers on the board) when set; otherwise the openvla-7b
        # tokenizer is used via prompts.prompt_token_ids.
        prompt_table_path = param('prompt_table', '')
        rate_hz = param('rate_hz', 0.5)
        param('enabled', True)
        self.vehicle_id = param('vehicle_id', 'drone0')
        # Same-tick check: front/down/pose header stamps must agree within
        # this skew (task contract: >0.1 s -> skip the tick).
        self.max_skew = param('max_input_skew_s', 0.1)
        self.inference_timeout = param('inference_timeout_s', 30.0)
        # TimedTrajectorySegment.valid_from = now + this lead.
        self.valid_from_lead = param('valid_from_lead_s', 0.05)
        self.max_failures = param('max_consecutive_failures', 3)
        if not (isinstance(rate_hz, float) and math.isfinite(rate_hz) and rate_hz > 0.0):
            raise ValueError('rate_hz must be a positive finite number')
        if len(self.target_position) != 3 or not all(
                math.isfinite(float(v)) for v in self.target_position):
            raise ValueError('target_position must be three finite doubles')
        self.prompt_table = None
        if prompt_table_path:
            self.prompt_table = prompts.load_prompt_table(prompt_table_path)

        # Loop state.
        self.front = None
        self.down = None
        self.pose = None
        self.state = 'IDLE'          # IDLE|RUNNING|PAUSED|STOPPED
        self.goal_seq = 0
        self.task_id = ''
        self.generation = 0          # bumped on goal/cancel; discards in-flight
        self.step = 0
        self.in_flight = None        # dict(future, generation, sent)
        self.failures = 0
        self.last_error = ''
        self.last_gen_ids = []
        self.total_ms = 0.0

        self.create_subscription(Image, 'front', self.on_front, qos_profile_sensor_data)
        self.create_subscription(Image, 'down', self.on_down, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'goal', self.on_goal, 1)
        self.create_service(Trigger, 'cancel', self.on_cancel)
        self.trajectory_pub = self.create_publisher(TimedTrajectorySegment, 'trajectory', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.infer_client = self.create_client(InferAeroVla, self.service_name)
        self.create_timer(1.0 / float(rate_hz), self.tick)

    # ---------------- inputs ----------------
    def on_front(self, msg):
        self.front = msg

    def on_down(self, msg):
        self.down = msg

    def on_pose(self, msg):
        self.pose = msg

    def on_goal(self, msg):
        position = msg.pose.position
        if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
            self.get_logger().error('goal position must be finite')
            return
        self.goal_seq += 1
        self.task_id = 'aerovla-%d' % self.goal_seq
        self.generation += 1
        self.step = 0
        self.failures = 0
        self.last_error = ''
        self.state = 'RUNNING'
        self.get_logger().info('goal #%d accepted, task %s' % (self.goal_seq, self.task_id))
        self.publish_status()

    def on_cancel(self, request, response):
        self.generation += 1
        self.in_flight = None
        self.publish_empty_trajectory()
        self.state = 'STOPPED'
        response.success = True
        response.message = 'aerovla planner stopped; empty trajectory published'
        self.publish_status()
        return response

    # ---------------- loop ----------------
    def tick(self):
        if not self.get_parameter('enabled').value or self.state != 'RUNNING':
            return
        if self.in_flight is not None:
            if self.in_flight['future'].done():
                return  # the response callback drains it and publishes
            if time.monotonic() - self.in_flight['sent'] > self.inference_timeout:
                ctx, self.in_flight = self.in_flight, None
                self.count_failure('INFERENCE_TIMEOUT')
            # else: exactly one request may be in flight; drop this tick.
            return
        if not self.infer_client.service_is_ready():
            self.count_failure('SERVICE_UNAVAILABLE')
            return
        self.send_inference()

    def same_tick_inputs(self):
        """Latest front/down/pose must carry stamps within max_skew of each
        other (task contract: timestamp diff > 0.1 s -> skip the tick)."""
        if self.front is None or self.down is None or self.pose is None:
            return False
        stamps = [stamp_to_s(self.front.header.stamp),
                  stamp_to_s(self.down.header.stamp),
                  stamp_to_s(self.pose.header.stamp)]
        return max(stamps) - min(stamps) <= self.max_skew

    def send_inference(self):
        if not self.same_tick_inputs():
            return  # observation not synchronized this tick: skip
        pose = self.pose.pose
        # NED view of this tick's state; the same snapshot feeds the hint
        # and the action execution, as upstream does with ep[-1] state.
        ned_state = {
            'position': enu_ned_vector([pose.position.x, pose.position.y,
                                        pose.position.z]),
            'orientation': enu_to_ned_quat({'x': pose.orientation.x,
                                            'y': pose.orientation.y,
                                            'z': pose.orientation.z,
                                            'w': pose.orientation.w}),
        }
        hint = policy.semantic_hint(ned_state['position'], ned_state['orientation'],
                                    enu_ned_vector(self.target_position))
        try:
            if self.prompt_table is not None:
                input_ids = prompts.table_prompt_ids(self.prompt_table, hint,
                                                     self.obj_desc)
            else:
                input_ids = prompts.prompt_token_ids(hint, self.obj_desc)
        except Exception as error:
            self.count_failure('PROMPT_IDS:' + str(error))
            return
        request = InferAeroVla.Request()
        request.request_id = '%s:%d:%s' % (self.task_id, self.step, uuid.uuid4().hex)
        request.front = copy.deepcopy(self.front)
        request.down = copy.deepcopy(self.down)
        request.input_ids = input_ids
        ctx = {'generation': self.generation, 'sent': time.monotonic(),
               'ned_state': ned_state}
        future = self.infer_client.call_async(request)
        future.add_done_callback(lambda f: self.on_response(f, ctx))
        self.in_flight = ctx
        self.in_flight['future'] = future

    def on_response(self, future, ctx):
        if ctx['generation'] != self.generation or self.state != 'RUNNING':
            return  # superseded by a new goal or a cancel
        if self.in_flight is ctx:
            self.in_flight = None
        try:
            response = future.result()
        except Exception as error:
            self.count_failure('SERVICE:' + str(error))
            return
        self.total_ms = float(response.total_ms)
        self.last_gen_ids = [int(v) for v in response.gen_ids]
        if not response.success:
            if response.error_code == 'BUSY':
                # Worker already busy: discard this tick, not a failure.
                self.get_logger().warning('inference BUSY, tick dropped')
                return
            self.count_failure(response.error_code or 'SERVICE_FAILURE')
            return
        self.failures = 0
        fwd, down, yaw = policy.dequantize_bins(response.text)
        if policy.is_landing(response.text, fwd, down, yaw):
            self.publish_final()
            self.state = 'STOPPED'
            self.get_logger().info('LAND signal; final trajectory published')
            self.publish_status()
            return
        ned_points = policy.action_to_setpoints(ctx['ned_state'], fwd, down, yaw)
        # Back to ENU for the published trajectory (yaw negated: NED yaw is
        # clockwise-positive, ENU counterclockwise-positive).
        points = [{'t': p['t'],
                   'position': enu_ned_vector(p['position']),
                   'velocity': enu_ned_vector(p['velocity']),
                   'yaw': -float(p['yaw'])} for p in ned_points]
        self.publish_trajectory(points, is_final=False)
        self.step += 1
        self.publish_status()

    def count_failure(self, code):
        self.last_error = code
        self.failures += 1
        self.get_logger().warning('failure %d: %s' % (self.failures, code))
        if self.failures >= self.max_failures:
            self.state = 'PAUSED'
            self.get_logger().error('loop paused after %d consecutive failures' % self.failures)
        self.publish_status()

    # ---------------- outputs ----------------
    def _make_segment(self, points, is_final):
        now = self.get_clock().now()
        segment = TimedTrajectorySegment()
        segment.interface_version = 1
        segment.task_id = self.task_id
        segment.generated_at = now.to_msg()
        segment.valid_from = (now + Duration(seconds=self.valid_from_lead)).to_msg()
        segment.is_final = bool(is_final)
        trajectory = VehicleTrajectory()
        trajectory.vehicle_id = self.vehicle_id
        # VELOCITY|ACCELERATION|YAW bits (VehicleTrajectory.msg L1-3);
        # positions are always present, acceleration stays zero.
        trajectory.valid_fields = 7
        for point in points:
            tp = TimedTrajectoryPoint()
            tp.time_from_start = duration_msg(point['t'])
            tp.position.x, tp.position.y, tp.position.z = map(float, point['position'])
            tp.velocity.x, tp.velocity.y, tp.velocity.z = map(float, point['velocity'])
            tp.acceleration.x = tp.acceleration.y = tp.acceleration.z = 0.0
            tp.yaw = float(point['yaw'])
            trajectory.points.append(tp)
        segment.vehicle_trajectories.append(trajectory)
        return segment

    def publish_trajectory(self, points, is_final):
        self.trajectory_pub.publish(self._make_segment(points, is_final))

    def publish_final(self):
        """Landing: one point at the current pose, is_final=true (task
        contract); the loop stops after this publication."""
        pose = self.pose.pose
        yaw = policy.yaw_from_quat({'x': pose.orientation.x, 'y': pose.orientation.y,
                                    'z': pose.orientation.z, 'w': pose.orientation.w})
        point = {'t': 0.0,
                 'position': [pose.position.x, pose.position.y, pose.position.z],
                 'velocity': [0.0, 0.0, 0.0], 'yaw': yaw}
        self.publish_trajectory([point], is_final=True)

    def publish_empty_trajectory(self):
        """Cancel: an empty TimedTrajectorySegment (no vehicle_trajectories)."""
        now = self.get_clock().now()
        segment = TimedTrajectorySegment()
        segment.interface_version = 1
        segment.task_id = self.task_id
        segment.generated_at = now.to_msg()
        segment.valid_from = (now + Duration(seconds=self.valid_from_lead)).to_msg()
        segment.is_final = True
        self.trajectory_pub.publish(segment)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'step': self.step,
            'state': self.state,
            'task_id': self.task_id,
            'last_error': self.last_error,
            'last_gen_ids': self.last_gen_ids,
            'total_ms': self.total_ms,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = AerovlaPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
