"""Bridge AirSim/AS2 inputs and FALCON B-splines to FollowTimedPath."""

from __future__ import annotations

import copy
import json
import math
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from falcon_ros2.msg import Bspline
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String
from std_srvs.srv import SetBool, Trigger
from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.action._follow_timed_path import FollowTimedPath_SendGoal
from uav_planner_interfaces.msg import TimedTrajectoryPoint

from .bspline import (
    InvalidSpline, continuity_deltas, exceeds_continuity_limits,
    interpolate_samples, sample_message)
from .sensor_pose import compose, interpolate, normalize, rotate


def replan_deadline(current: float, message_type: int, now: float,
                    static_timeout: float) -> float:
    if message_type == 1:
        return now + static_timeout
    if message_type == 0 and not current:
        return now + 0.5
    return current


def time_seconds(value) -> float:
    return float(value.sec) + float(value.nanosec) / 1_000_000_000.0


def fill_duration(target, value: float) -> None:
    target.sec = int(value)
    target.nanosec = int(round((value - target.sec) * 1_000_000_000.0))
    if target.nanosec == 1_000_000_000:
        target.sec += 1
        target.nanosec = 0


class FalconAdapter(Node):
    def __init__(self):
        super().__init__("falcon_adapter")
        defaults = {
            "vehicle_id": "drone0",
            "frame_id": "drone0/odom",
            "odometry_frame": "drone0/odom",
            "depth_frame": "drone0/FrontCamera_optical",
            "task_id": "falcon-exploration",
            "sample_period": 0.02,
            "max_speed": 2.0,
            "source_timeout": 1.0,
            "static_replan_timeout": 15.0,
            "initial_plan_timeout": 30.0,
            "pose_tolerance": 0.05,
            "continuity_position": 0.25,
            "continuity_velocity": 0.5,
            "continuity_acceleration": 1.4,
            "continuity_yaw": 0.20,
            "mount_position": [0.0, 0.0, 0.0],
            "mount_orientation": [-0.5, 0.5, -0.5, 0.5],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.vehicle = str(self.get_parameter("vehicle_id").value)
        self.frame = str(self.get_parameter("frame_id").value)
        self.odometry_frame = str(self.get_parameter("odometry_frame").value)
        self.depth_frame = str(self.get_parameter("depth_frame").value)
        self.task = str(self.get_parameter("task_id").value)
        self.sample_period = float(self.get_parameter("sample_period").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.source_timeout = float(self.get_parameter("source_timeout").value)
        self.static_replan_timeout = float(
            self.get_parameter("static_replan_timeout").value)
        self.plan_timeout = float(self.get_parameter("initial_plan_timeout").value)
        self.pose_tolerance = float(self.get_parameter("pose_tolerance").value)
        self.continuity_limits = tuple(float(self.get_parameter(name).value) for name in (
            "continuity_position", "continuity_velocity",
            "continuity_acceleration", "continuity_yaw"))
        self.mount_position = tuple(float(v) for v in self.get_parameter("mount_position").value)
        self.mount_orientation = normalize(self.get_parameter("mount_orientation").value)
        if (len(self.mount_position) != 3 or not all(math.isfinite(v) for v in self.mount_position)
                or not all(math.isfinite(v) and v > 0 for v in (
                self.sample_period, self.max_speed, self.source_timeout,
                self.static_replan_timeout,
                    self.plan_timeout, self.pose_tolerance, *self.continuity_limits))):
            raise ValueError("invalid FALCON adapter parameters")

        self.odom_history = deque(maxlen=500)
        self.pending_depth = deque(maxlen=20)
        self.latest_odom_wall = 0.0
        self.latest_depth_wall = 0.0
        self.matched_depth_count = 0
        self.dropped_depth_count = 0
        self.started_wall = time.monotonic()
        self.last_traj_id = -1
        self.trajectory_ids = []
        self.last_plan_wall = 0.0
        self.revision = 0
        self.execution_handle = None
        self.pending_request = False
        self.completed = False
        self.failed = ""
        self.generation = 0
        self.stop_future = None
        self.queued_bspline = None
        self.replacement_deadline = 0.0
        self.trajectory_end = 0.0
        self.terminal_stop_future = None
        self.stop_confirmed = False
        self.depth_enabled = True
        self.enabled = False
        self.active_samples = None
        self.active_start = 0.0

        self.odom_publisher = self.create_publisher(
            Odometry, "falcon/input/odometry", qos_profile_sensor_data)
        self.depth_publisher = self.create_publisher(
            Image, "falcon/input/depth", qos_profile_sensor_data)
        self.pose_publisher = self.create_publisher(
            TransformStamped, "falcon/input/sensor_pose", qos_profile_sensor_data)
        self.state_publisher = self.create_publisher(String, "falcon/adapter/state", 10)
        self.stop_confirm_publisher = self.create_publisher(
            Empty, "/planning/execution_stopped", 10)
        self.create_subscription(
            Odometry, "sensor_measurements/odom", self.on_odometry,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, "sensor_measurements/depth", self.on_depth,
            qos_profile_sensor_data)
        self.create_subscription(Bspline, "/planning/bspline", self.on_bspline, 10)
        self.create_subscription(Int32, "/planning/replan", self.on_replan, 10)
        action_name = f"/{self.vehicle}/follow_timed_path"
        self.action = ActionClient(self, FollowTimedPath, action_name)
        self.modifier = self.create_client(
            FollowTimedPath_SendGoal, action_name + "/_behavior/modify")
        self.stopper = self.create_client(Trigger, action_name + "/_behavior/stop")
        self.create_service(SetBool, "falcon/enable", self.set_enabled)
        self.create_service(SetBool, "falcon/test/depth_enabled", self.set_depth_enabled)
        self.create_timer(0.05, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def on_odometry(self, message: Odometry) -> None:
        stamp = time_seconds(message.header.stamp)
        if message.header.frame_id != self.odometry_frame:
            self.fail("odometry frame does not match configured fixed ENU frame")
            return
        values = (
            message.pose.pose.position.x, message.pose.pose.position.y,
            message.pose.pose.position.z, message.pose.pose.orientation.x,
            message.pose.pose.orientation.y, message.pose.pose.orientation.z,
            message.pose.pose.orientation.w)
        if stamp <= 0 or not all(math.isfinite(value) for value in values):
            return
        if self.odom_history and stamp <= self.odom_history[-1][0]:
            if stamp == self.odom_history[-1][0]:
                return
            self.fail("out-of-order odometry")
            return
        position = tuple(float(value) for value in values[:3])
        orientation = normalize(values[3:])
        self.odom_history.append((stamp, position, orientation))
        self.latest_odom_wall = time.monotonic()
        adapted = copy.deepcopy(message)
        adapted.header.frame_id = "world"
        adapted.child_frame_id = "base_link"
        velocity = adapted.twist.twist.linear
        velocity.x, velocity.y, velocity.z = rotate(
            orientation, (velocity.x, velocity.y, velocity.z))
        self.odom_publisher.publish(adapted)
        self.flush_depth()

    def on_depth(self, message: Image) -> None:
        if not self.depth_enabled:
            return
        stamp = time_seconds(message.header.stamp)
        if (stamp <= 0 or message.header.frame_id != self.depth_frame
                or message.encoding != "32FC1" or message.is_bigendian
                or message.height <= 0 or message.width <= 0
                or message.step < message.width * 4
                or len(message.data) < message.step * message.height):
            self.fail("invalid axial depth image")
            return
        self.latest_depth_wall = time.monotonic()
        self.pending_depth.append((stamp, time.monotonic(), copy.deepcopy(message)))
        self.flush_depth()

    def set_depth_enabled(self, request, response):
        self.depth_enabled = bool(request.data)
        response.success = True
        response.message = "depth input enabled" if self.depth_enabled else "depth input disabled"
        return response

    def set_enabled(self, request, response):
        if not request.data:
            self.enabled = False
            self.stop_execution()
            response.success = True
            response.message = "FALCON execution disabled"
            return response
        wall = time.monotonic()
        if self.failed or self.completed:
            response.message = "adapter is terminal; restart the component"
        elif (not self.latest_odom_wall or not self.latest_depth_wall
              or wall - self.latest_odom_wall > self.source_timeout
              or wall - self.latest_depth_wall > self.source_timeout):
            response.message = "fresh depth and odometry are required"
        elif not self.action.server_is_ready() or not self.stopper.service_is_ready():
            response.message = "FollowTimedPath is not ready"
        else:
            self.enabled = True
            self.started_wall = wall
            response.success = True
            response.message = "FALCON execution enabled"
        return response

    def pose_at(self, stamp):
        if not self.odom_history:
            return None
        for index, sample in enumerate(self.odom_history):
            if sample[0] == stamp:
                return sample[1], sample[2]
            if sample[0] > stamp:
                if index == 0:
                    return None
                before = self.odom_history[index - 1]
                if stamp - before[0] > self.pose_tolerance or sample[0] - stamp > self.pose_tolerance:
                    return None
                return interpolate(before, sample, stamp)
        return None

    def flush_depth(self) -> None:
        while self.pending_depth:
            stamp, received, image = self.pending_depth[0]
            pose = self.pose_at(stamp)
            if pose is None:
                if self.odom_history and stamp < self.odom_history[0][0]:
                    # A retained camera sample can predate this process's first
                    # odometry sample at startup and can never be bracketed.
                    self.pending_depth.popleft()
                    self.dropped_depth_count += 1
                    continue
                if time.monotonic() - received > self.source_timeout:
                    self.pending_depth.popleft()
                    self.dropped_depth_count += 1
                    self.get_logger().warning(
                        "dropping depth image without timestamp-matched odometry",
                        throttle_duration_sec=5.0)
                    continue
                return
            self.pending_depth.popleft()
            position, orientation = compose(
                pose[0], pose[1], self.mount_position, self.mount_orientation)
            transform = TransformStamped()
            transform.header.stamp = copy.deepcopy(image.header.stamp)
            transform.header.frame_id = "world"
            transform.child_frame_id = "camera"
            transform.transform.translation.x = position[0]
            transform.transform.translation.y = position[1]
            transform.transform.translation.z = position[2]
            transform.transform.rotation.x = orientation[0]
            transform.transform.rotation.y = orientation[1]
            transform.transform.rotation.z = orientation[2]
            transform.transform.rotation.w = orientation[3]
            image.header.frame_id = "camera"
            self.pose_publisher.publish(transform)
            self.depth_publisher.publish(image)
            self.matched_depth_count += 1

    def goal_from_bspline(self, message: Bspline):
        samples = sample_message(message, self.sample_period)
        goal = FollowTimedPath.Goal()
        path = goal.path
        path.interface_version = 1
        path.task_id = self.task
        path.vehicle_id = self.vehicle
        path.revision = self.revision + 1
        path.frame_id = self.frame
        path.generated_at = self.get_clock().now().to_msg()
        path.valid_from = copy.deepcopy(message.start_time)
        path.is_final = False
        path.valid_fields = 1 | 2 | 4
        path.max_speed = self.max_speed
        for sample in samples:
            point = TimedTrajectoryPoint()
            fill_duration(point.time_from_start, sample.time_from_start)
            for field, source in (
                    (point.position, sample.position),
                    (point.velocity, sample.velocity),
                    (point.acceleration, sample.acceleration)):
                field.x, field.y, field.z = source
            point.yaw = sample.yaw
            path.points.append(point)
        return goal, samples

    def on_bspline(self, message: Bspline) -> None:
        if not self.enabled or self.failed or self.completed:
            return
        newest = self.queued_bspline.traj_id if self.queued_bspline is not None else self.last_traj_id
        if message.traj_id <= newest:
            self.fail("out-of-order FALCON trajectory")
            return
        if self.stop_future is not None:
            self.queued_bspline = copy.deepcopy(message)
            return
        self.submit_bspline(message)

    def submit_bspline(self, message: Bspline) -> None:
        start = time_seconds(message.start_time)
        if start <= self.now():
            self.fail("late FALCON trajectory")
            return
        try:
            goal, samples = self.goal_from_bspline(message)
            if self.execution_handle is not None and self.active_samples is not None:
                deltas = continuity_deltas(
                    self.active_samples, self.active_start, samples, start)
                if exceeds_continuity_limits(deltas, self.continuity_limits):
                    expected = interpolate_samples(
                        self.active_samples, start - self.active_start)
                    actual = samples[0]
                    raise InvalidSpline(
                        "discontinuous replacement PVA/yaw: "
                        + ",".join(f"{value:.6f}" for value in deltas)
                        + "; expected_acc="
                        + ",".join(f"{value:.6f}" for value in expected.acceleration)
                        + "; actual_acc="
                        + ",".join(f"{value:.6f}" for value in actual.acceleration)
                        + f"; previous_offset={start - self.active_start:.6f}")
        except (InvalidSpline, OverflowError, ValueError) as exc:
            self.fail("invalid FALCON B-spline: " + str(exc))
            return
        if self.pending_request:
            self.fail("FALCON trajectory arrived before previous update acknowledgement")
            return
        self.last_traj_id = int(message.traj_id)
        self.trajectory_ids.append(self.last_traj_id)
        self.last_plan_wall = time.monotonic()
        self.replacement_deadline = 0.0
        self.trajectory_end = start + time_seconds(goal.path.points[-1].time_from_start)
        self.active_samples = samples
        self.active_start = start
        self.revision += 1
        generated = time_seconds(goal.path.generated_at)
        now = self.now()
        self.get_logger().info(
            "submitting FALCON trajectory "
            f"traj_id={message.traj_id} revision={self.revision} "
            f"generated={generated:.9f} start={start:.9f} now={now:.9f} "
            f"lead={start - now:.6f}s points={len(goal.path.points)} "
            f"duration={time_seconds(goal.path.points[-1].time_from_start):.6f}s")
        self.pending_request = True
        generation = self.generation
        if self.execution_handle is None:
            if not self.action.server_is_ready():
                self.fail("FollowTimedPath action is unavailable")
                return
            future = self.action.send_goal_async(
                goal, feedback_callback=lambda feedback, g=generation: self.on_feedback(g, feedback))
            future.add_done_callback(lambda result, g=generation: self.on_accepted(g, result))
        else:
            if not self.modifier.service_is_ready():
                self.fail("FollowTimedPath modify service is unavailable")
                return
            request = FollowTimedPath_SendGoal.Request()
            request.goal = goal
            future = self.modifier.call_async(request)
            future.add_done_callback(lambda result, g=generation: self.on_modified(g, result))

    def on_accepted(self, generation, future):
        if generation != self.generation:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self.fail("FollowTimedPath rejected FALCON trajectory")
                return
            self.execution_handle = handle
            self.pending_request = False
            handle.get_result_async().add_done_callback(
                lambda result, g=generation: self.on_result(g, result))
        except Exception as exc:
            self.fail("FollowTimedPath goal error: " + str(exc))

    def on_modified(self, generation, future):
        if generation != self.generation:
            return
        try:
            if not future.result().accepted:
                self.fail("FollowTimedPath rejected FALCON update")
                return
            self.pending_request = False
        except Exception as exc:
            self.fail("FollowTimedPath update error: " + str(exc))

    def on_feedback(self, generation, message):
        if generation != self.generation:
            return
        # FALCON trajectory IDs and FollowTimedPath execution revisions are
        # separate sequences. Feedback reports the latter.
        if message.feedback.revision > self.revision:
            self.fail("FollowTimedPath reported an unknown revision")

    def on_result(self, generation, future):
        if generation != self.generation or self.completed:
            return
        try:
            result = future.result()
            if result.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                self.fail("FollowTimedPath execution failed")
        except Exception as exc:
            self.fail("FollowTimedPath result error: " + str(exc))

    def on_replan(self, message: Int32) -> None:
        if self.failed or self.completed:
            return
        if message.data == 2:
            self.completed = True
            self.enabled = False
            self.stop_execution()
        elif message.data == 1:
            self.get_logger().warning("FALCON reported an unsafe active trajectory; stopping before replan")
            self.replacement_deadline = replan_deadline(
                self.replacement_deadline, message.data, time.monotonic(),
                self.static_replan_timeout)
            self.begin_replan_stop()
        elif message.data == 0:
            # Upstream traj_server allows 0.5 s for the replacement plan.
            # A type-0 planning notification follows a type-1 emergency stop;
            # it must not shorten the bounded wait for the vehicle to settle.
            self.replacement_deadline = replan_deadline(
                self.replacement_deadline, message.data, time.monotonic(),
                self.static_replan_timeout)
        elif message.data != 0:
            self.fail("unknown FALCON replan type")

    def begin_replan_stop(self) -> None:
        if self.stop_future is not None:
            return
        self.trajectory_end = 0.0
        self.active_samples = None
        self.active_start = 0.0
        self.generation += 1
        self.pending_request = False
        if self.execution_handle is not None:
            self.execution_handle.cancel_goal_async()
            self.execution_handle = None
        if not self.stopper.service_is_ready():
            self.fail("FollowTimedPath stop service is unavailable")
            return
        self.stop_future = self.stopper.call_async(Trigger.Request())

    def stop_execution(self) -> None:
        self.generation += 1
        self.pending_request = False
        if self.execution_handle is not None:
            self.execution_handle.cancel_goal_async()
            self.execution_handle = None
        if self.stopper.service_is_ready() and self.terminal_stop_future is None:
            self.terminal_stop_future = self.stopper.call_async(Trigger.Request())

    def fail(self, reason: str) -> None:
        if self.failed:
            return
        self.failed = reason
        self.enabled = False
        self.get_logger().error(reason)
        self.stop_execution()

    def tick(self) -> None:
        wall = time.monotonic()
        simulation_now = self.now()
        if self.terminal_stop_future is not None and self.terminal_stop_future.done():
            future, self.terminal_stop_future = self.terminal_stop_future, None
            try:
                self.stop_confirmed = bool(future.result().success)
                if not self.stop_confirmed:
                    self.get_logger().error("FollowTimedPath did not confirm terminal stop")
            except Exception as exc:
                self.get_logger().error("FollowTimedPath terminal stop error: " + str(exc))
        if self.stop_future is not None and self.stop_future.done():
            future, self.stop_future = self.stop_future, None
            try:
                if not future.result().success:
                    self.fail("FollowTimedPath did not confirm stop")
                elif not self.failed and not self.completed:
                    if self.queued_bspline is not None:
                        self.get_logger().warning(
                            "discarding trajectory produced before execution stop confirmation")
                        self.queued_bspline = None
                    self.stop_confirm_publisher.publish(Empty())
            except Exception as exc:
                self.fail("FollowTimedPath stop error: " + str(exc))
        if self.enabled and not self.failed and not self.completed:
            if self.latest_odom_wall and wall - self.latest_odom_wall > self.source_timeout:
                self.fail("odometry stream expired")
            elif self.latest_depth_wall and wall - self.latest_depth_wall > self.source_timeout:
                self.fail("depth stream expired")
            elif not self.last_plan_wall and wall - self.started_wall > self.plan_timeout:
                self.fail("initial FALCON planning timeout")
            elif self.replacement_deadline and wall > self.replacement_deadline:
                self.fail("FALCON replacement trajectory timeout")
            elif self.trajectory_end and simulation_now > self.trajectory_end + 0.2:
                self.fail("FALCON trajectory coverage expired without replacement")
        state = {
            "task_id": self.task,
            "last_trajectory_id": self.last_traj_id,
            "trajectory_ids": self.trajectory_ids,
            "revision_count": self.revision,
            "execution_active": self.execution_handle is not None,
            "enabled": self.enabled,
            "request_pending": self.pending_request,
            "stop_pending": self.stop_future is not None,
            "stop_confirmed": self.stop_confirmed,
            "completed": self.completed,
            "failed": self.failed,
            "odometry_ready": bool(self.latest_odom_wall),
            "depth_received": bool(self.latest_depth_wall),
            "depth_ready": self.matched_depth_count > 0,
            "matched_depth_count": self.matched_depth_count,
            "dropped_depth_count": self.dropped_depth_count,
            "coverage_remaining_s": max(0.0, self.trajectory_end - simulation_now)
            if self.trajectory_end else 0.0,
            "pending_depth": len(self.pending_depth),
        }
        self.state_publisher.publish(String(data=json.dumps(state, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = FalconAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok(context=node.context):
            raise
    finally:
        if rclpy.ok(context=node.context):
            node.stop_execution()
        node.destroy_node()
        rclpy.try_shutdown()
