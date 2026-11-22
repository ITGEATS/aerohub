"""Shared observation, prompt and inference loop; execution belongs to subclasses.

This node owns sensor snapshots, NED/FRD adaptation, prompt construction,
inference requests and task bookkeeping. It creates no trajectory publisher
and defines no motion execution parameters.
"""
import copy
import json
import math
import time
import uuid

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_inference_interfaces.srv import InferAeroVla
from uav_planner_interfaces.msg import PlannerFailure

from . import policy, prompts, resolved as resolved_config_module
from .frames import enu_ned_vector, enu_to_ned_quat


def stamp_to_s(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class AerovlaObservationNode(Node):
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
        front_topic = param('front_topic', 'front')
        down_topic = param('down_topic', 'down')
        pose_topic = param('pose_topic', 'pose')
        rate_hz = param('rate_hz', 0.5)
        param('enabled', True)
        self.vehicle_id = param('vehicle_id', 'drone0')
        # Same-tick check: front/down/pose header stamps must agree within
        # this skew (task contract: >0.1 s -> skip the tick).
        self.max_skew = param('max_input_skew_s', 0.1)
        self.inference_timeout = param('inference_timeout_s', 30.0)
        self.max_failures = param('max_consecutive_failures', 3)
        # Resolved task settings override standalone observation defaults.
        resolved_path = param('resolved_config', '')
        self.resolved_view = None
        if resolved_path:
            view = resolved_config_module.load_resolved(resolved_path)
            self.resolved_view = view
            settings = view['parameters']
            self.service_name = settings['service_name']
            self.obj_desc = settings['obj_desc']
            self.target_position = [float(v) for v in settings['target_position_enu']]
            rate_hz = float(settings['rate_hz'])
            self.max_skew = float(settings['max_input_skew_s'])
            self.inference_timeout = float(settings['inference_timeout_s'])
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
        # standalone goals fall back to aerovla-<seq>.
        self.task_id = self.resolved_task_id
        self.generation = 0          # bumped on goal/cancel; discards in-flight
        self.step = 0
        self.in_flight = None        # dict(future, generation, sent)
        self.failures = 0
        self.last_error = ''
        self.last_gen_ids = []
        self.total_ms = 0.0
        self.create_subscription(Image, front_topic, self.on_front, qos_profile_sensor_data)
        self.create_subscription(Image, down_topic, self.on_down, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'goal', self.on_goal, 1)
        self.create_service(Trigger, 'cancel', self.on_cancel)
        self.failure_topic = param('failure_topic', '')
        self.failure_pub = (self.create_publisher(PlannerFailure, self.failure_topic, 10)
                            if self.failure_topic else None)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.infer_client = self.create_client(InferAeroVla, self.service_name)
        self.create_timer(1.0 / float(rate_hz), self.tick)
        self.configure_execution()
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

    def configure_execution(self):
        raise NotImplementedError

    def on_cancel(self, request, response):
        raise NotImplementedError

    def on_response(self, future, ctx):
        raise NotImplementedError

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
        self.state = 'RUNNING'
        self.get_logger().info('goal #%d accepted, task %s' % (self.goal_seq, self.task_id))
        self.publish_status()

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

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'step': self.step,
            'state': self.state,
            'task_id': self.task_id,
            'last_error': self.last_error,
            'last_gen_ids': self.last_gen_ids,
            'total_ms': self.total_ms,
        })))
