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
ENU pose/target -> NED before calling it and publishes the resulting timed
points in the fleet NED contract (the framework's make_goal performs the
single NED -> vehicle ENU/odom conversion). It must NOT convert back to ENU
here, or the goal's first point is mirrored to the wrong altitude/heading.

ROLLING EXECUTION SHAPING (adapter layer, ``policy`` stays verbatim): NPU
inference takes ~1.8 s while a policy step's trajectory is ~1.5 s, so a
published non-final segment would expire before the next one arrives. This
node rebuilds the handoff point from the CURRENT pose at response time and
extends the trailing hold to ``roll_horizon_s`` so the next trajectory arrives
inside the current coverage window and the behavior's rolling continuity and
late-trajectory checks pass. The move endpoint velocity is zeroed so the hold
is a static quintic-Hermite segment (no overshoot, v=0 continuity).
"""

import copy
import json
import math
import time
import uuid

from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_inference_interfaces.srv import InferAeroVla
from uav_planner_interfaces.msg import (PlannerFailure, TimedTrajectoryPoint,
                                        TimedTrajectorySegment,
                                        VehicleTrajectory)

from . import policy, prompts, resolved as resolved_config_module
from .frames import enu_ned_vector, enu_to_ned_quat, enu_to_ned_yaw


def stamp_to_s(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def duration_msg(seconds):
    ns = int(round(seconds * 1e9))
    sec, nanosec = divmod(ns, 1000000000)
    return Duration(seconds=int(sec), nanoseconds=nanosec).to_msg()




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
        # Adapter-layer rolling horizon (seconds). Must exceed the gap between
        # successfully published trajectories: one inference is ~1.8 s, and a
        # transient INVALID_INPUT retry adds one dropped cycle (~3.5 s gap).
        # Must stay below the framework trajectory_timeout (8 s). A longer hold
        # only keeps the vehicle at the step endpoint; it also makes the
        # behavior's start<=previous.end() update check easier to satisfy.
        self.roll_horizon_s = param('roll_horizon_s', 6.0)
        self.max_failures = param('max_consecutive_failures', 3)
        # When spawned by the planner framework (execution_backend=follow_path)
        # the resolved task config overrides the ROS-parameter defaults so the
        # published segments carry the framework task_id and vehicle set that
        # OutputGateway.accept validates (uav_planner_framework/trajectory.py).
        resolved_path = param('resolved_config', '')
        if resolved_path:
            view = resolved_config_module.load_resolved(resolved_path)
            settings = view['parameters']
            self.service_name = settings['service_name']
            self.obj_desc = settings['obj_desc']
            self.target_position = [float(v) for v in settings['target_position_enu']]
            rate_hz = float(settings['rate_hz'])
            self.max_skew = float(settings['max_input_skew_s'])
            self.inference_timeout = float(settings['inference_timeout_s'])
            self.valid_from_lead = float(settings['valid_from_lead_s'])
            self.roll_horizon_s = float(settings.get('roll_horizon_s',
                                                    self.roll_horizon_s))
            self.max_failures = int(settings['max_consecutive_failures'])
            self.vehicle_id = view['vehicle_id']
            self.resolved_task_id = view['task_id']
            # Board contract: no transformers on the board, so the prompt
            # table must ride the resolved parameters (manifest defaults or
            # task override), not only the ROS-parameter standalone default.
            prompt_table_path = str(settings.get('prompt_table') or prompt_table_path)
            front_topic = str(settings.get('front_topic') or front_topic)
            down_topic = str(settings.get('down_topic') or down_topic)
            pose_topic = str(settings.get('pose_topic') or pose_topic)
        else:
            self.resolved_task_id = ''
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
        # Under the framework the task_id is fixed by the resolved config
        # (gateway validation); standalone goals fall back to aerovla-<seq>.
        self.task_id = self.resolved_task_id
        self.generation = 0          # bumped on goal/cancel; discards in-flight
        self.step = 0
        self.in_flight = None        # dict(future, generation, sent)
        self.failures = 0
        self.last_error = ''
        self.last_gen_ids = []
        self.total_ms = 0.0
        # Reference (not measured) handoff from the last published non-final
        # segment. The behavior checks rolling continuity against its own
        # sampled reference at new.start, so the next segment must start at
        # this cached endpoint rather than the noisy actual pose.
        self.ref_handoff = None      # {'position': [x,y,z]_ned, 'yaw': yaw_ned}

        self.create_subscription(Image, front_topic, self.on_front, qos_profile_sensor_data)
        self.create_subscription(Image, down_topic, self.on_down, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'goal', self.on_goal, 1)
        self.create_service(Trigger, 'cancel', self.on_cancel)
        # The framework subscribes planner/trajectory with RELIABLE +
        # TRANSIENT_LOCAL (ros_node.py L82-92); offer the same durability so
        # the match is QoS-compatible.
        self.trajectory_topic = param('trajectory_topic', 'trajectory')
        self.failure_topic = param('failure_topic', '')
        # Observation inputs default to the standalone relative topics; the
        # resolved config (manifest defaults) points them at the deployed
        # flight_board topics (front/down cameras and fleet localization).
        front_topic = param('front_topic', 'front')
        down_topic = param('down_topic', 'down')
        pose_topic = param('pose_topic', 'pose')
        trajectory_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trajectory_pub = self.create_publisher(TimedTrajectorySegment,
                                                    self.trajectory_topic, trajectory_qos)
        self.failure_pub = (self.create_publisher(PlannerFailure, self.failure_topic, 10)
                            if self.failure_topic else None)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.infer_client = self.create_client(InferAeroVla, self.service_name)
        self.create_timer(1.0 / float(rate_hz), self.tick)
        # Framework mode: the resolved config IS the task trigger — the
        # framework spawns this process on START/RESUME, so waiting for a
        # goal topic message would deadlock the chain (the planner is not
        # on the DDS graph when the client's one-shot goal was published).
        # Standalone mode keeps the goal-topic trigger (yopo paradigm).
        if self.resolved_task_id:
            self.state = 'RUNNING'
            self.get_logger().info('framework mode: task %s running from spawn'
                                   % self.resolved_task_id)
            self.publish_status()

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
        if not self.task_id:
            self.task_id = 'aerovla-%d' % self.goal_seq
        self.generation += 1
        self.step = 0
        self.failures = 0
        self.last_error = ''
        self.ref_handoff = None
        self.state = 'RUNNING'
        self.get_logger().info('goal #%d accepted, task %s' % (self.goal_seq, self.task_id))
        self.publish_status()

    def on_cancel(self, request, response):
        self.generation += 1
        self.in_flight = None
        self.ref_handoff = None
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

    def _fresh_pose_ned_state(self):
        """NED view of the latest measured pose (used for the first segment)."""
        pose = self.pose.pose
        return {
            'position': enu_ned_vector([pose.position.x, pose.position.y,
                                        pose.position.z]),
            'orientation': enu_to_ned_quat({'x': pose.orientation.x,
                                            'y': pose.orientation.y,
                                            'z': pose.orientation.z,
                                            'w': pose.orientation.w}),
        }

    def _handoff_state(self):
        """NED start state for the next trajectory.

        Rolling continuity is checked against the behavior's REFERENCE at
        new.start, not the noisy measured pose. During the hold tail the
        previous reference is its endpoint, so a continuing sequence must
        start from the cached reference endpoint (position + yaw), not the
        measured pose (~0.37 m tracking error would be rejected). For the
        first segment there is no cached handoff, so use the fresh pose.
        """
        if self.ref_handoff is None:
            return self._fresh_pose_ned_state()
        yaw_ned = float(self.ref_handoff['yaw'])
        # A NED z-rotation quaternion whose yaw_from_quat() == yaw_ned.
        q = R.from_euler('z', yaw_ned).as_quat().tolist()
        return {'position': list(self.ref_handoff['position']),
                'orientation': q}

    def _shape_for_rolling(self, points):
        """Adapter-layer execution shaping; ``policy`` stays verbatim.

        Zero the move endpoint velocity so the trailing hold is a static
        quintic-Hermite segment (no overshoot, v=0 handoff), and extend the
        hold to roll_horizon_s so the next trajectory is published inside the
        current coverage window instead of after TRAJECTORY_EXPIRED.
        """
        shaped = [dict(p) for p in points]
        if len(shaped) < 2:
            return shaped
        shaped[-2]['velocity'] = (0.0, 0.0, 0.0)
        tail = shaped[-1]
        tail['velocity'] = (0.0, 0.0, 0.0)
        tail['t'] = max(float(tail['t']), float(self.roll_horizon_s))
        return shaped

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
        # Build from the cached reference handoff (or fresh pose for the first
        # segment) so the t=0 point matches the behavior's reference sample at
        # new.start; policy stays verbatim NED, then adapter-only rolling
        # shaping.
        state = self._handoff_state()
        ned_points = policy.action_to_setpoints(state, fwd, down, yaw)
        # TimedTrajectorySegment uses the fleet/shared NED contract (see
        # docs/项目简介.md and framework make_goal). The policy emits NED
        # points; publish them directly so the framework applies its single
        # NED->vehicle ENU conversion. _shape_for_rolling adds the hold tail.
        shaped = self._shape_for_rolling(ned_points)
        self.publish_trajectory(shaped, is_final=False)
        # Cache the published endpoint as the next segment's reference handoff.
        tail = shaped[-1]
        self.ref_handoff = {'position': list(tail['position']),
                            'yaw': float(tail['yaw'])}
        self.step += 1
        self.publish_status()

    def count_failure(self, code):
        self.last_error = code
        self.failures += 1
        self.get_logger().warning('failure %d: %s' % (self.failures, code))
        if self.failures >= self.max_failures:
            self.state = 'PAUSED'
            self.get_logger().error('loop paused after %d consecutive failures' % self.failures)
            if self.failure_pub is not None:
                failure = PlannerFailure()
                failure.task_id = self.task_id
                failure.reason = 'PLANNER_ERROR'
                failure.detail = code
                self.failure_pub.publish(failure)
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
        yaw_enu = policy.yaw_from_quat({'x': pose.orientation.x, 'y': pose.orientation.y,
                                        'z': pose.orientation.z, 'w': pose.orientation.w})
        # Publish the final handoff point in fleet NED like every segment.
        # enu_ned_vector maps (x,y,z)_enu -> (y,x,-z)_ned; NED yaw is
        # clockwise-positive: yaw_ned = pi/2 - yaw_enu.
        point = {'t': 0.0,
                 'position': enu_ned_vector([pose.position.x, pose.position.y,
                                             pose.position.z]),
                 'velocity': [0.0, 0.0, 0.0],
                 'yaw': enu_to_ned_yaw(yaw_enu)}
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
    except Exception:
        # Teardown races (signal shutdown mid-spin_once) surface as RCLError;
        # only swallow them when the context is already gone (same guard as
        # uav_planner_framework.ros_node.main).
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
