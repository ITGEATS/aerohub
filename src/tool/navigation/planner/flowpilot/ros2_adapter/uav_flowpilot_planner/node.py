import copy
import json
import time
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from uav_inference_interfaces.srv import InferFlowPilot
from std_msgs.msg import String
from .lifecycle import FlowPilotLifecycle, seconds, vector
from .curve import Segment, state_derivatives, handoff_errors, plan_dynamics, splice_onto


class FlowPilotPlanner(FlowPilotLifecycle):
    def request_inference(self, now):
        pose, twist = self.observation_pair()
        observation = seconds(pose.header.stamp)
        velocity_time = seconds(twist.header.stamp)
        self.last_image = seconds(self.depth.header.stamp)
        history = list(self.twist_history)
        try:
            acc, jerk = state_derivatives([seconds(m.header.stamp) for m in history],
                                          [vector(m.twist.linear) for m in history], velocity_time)
        except ValueError as exc:
            self.failure(str(exc))
            return
        q = pose.pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        quaternion = rotation.as_quat()[[3,0,1,2]]
        if quaternion[0] < 0:
            quaternion *= -1
        origin = vector(pose.pose.position)
        local = rotation.inv().apply
        state = np.r_[np.zeros(3), local(vector(twist.twist.linear)), local(acc), local(jerk),
                      quaternion, local(self.goal-origin), np.zeros(6)]
        req = InferFlowPilot.Request()
        req.request_id = self.task + ":" + uuid.uuid4().hex
        req.depth = copy.deepcopy(self.depth)
        req.observation_header = copy.deepcopy(pose.header)
        req.state_goal, req.speed_mps = state.tolist(), float(self.command_speed)
        self.inference = dict(future=self.infer_client.call_async(req), sent=time.monotonic(),
                              generation=self.generation, expired=False, request=req,
                              epoch=round((now+self.lead)*1e9)/1e9, observation=observation, origin=origin,
                              rotation=rotation.as_matrix(), yaw=rotation.as_euler("ZYX")[0],
                              velocity=vector(twist.twist.linear), acceleration=acc)

    def warp_scale(self, world):
        """Stretch a plan in time until it fits the platform's control envelope.

        The model emits ~1.6 s plans with up to several m/s^2; the executed
        evidence (acceptance normal-v2) shows the platform tracks ~0.75-1
        m/s^2. Time-stretching by k divides speed by k and acceleration by k^2
        while preserving the route, so the drone can actually follow the plan
        and the rolling handoff residual stays inside the gate limits.
        """
        v_peak, a_peak = plan_dynamics(world)
        scale = max(1., (a_peak / self.plan_acceleration_cap) ** .5, v_peak / self.max_speed)
        return float(min(scale, 4.)), v_peak, a_peak
    def track_altitude(self, world, scale):
        """Shape the plan's vertical profile toward the goal altitude.

        The model commands descent even with the goal above, so with only the
        box clamp the 0.5 m floor became the cruise altitude. Points 3..7 must
        stay at or above a reference that eases from the spliced start altitude
        toward the goal z with a bounded vertical rate; the model may climb up
        to `goal_altitude_climb_margin_m` above it for obstacle avoidance.
        Points 0..2 are untouched: they hold the handoff-continuous start PVA,
        and a Bezier start PVA depends only on them, so the handoff gate is
        unaffected. The box floor stays as the crash guard.
        """
        z_start = float(world[0, 2])
        info = dict(enabled=bool(self.track_altitude_enabled), z_start=round(z_start, 3),
                    goal_z=round(float(self.goal[2]), 3))
        if not self.track_altitude_enabled:
            return info
        delta = float(self.goal[2]) - z_start
        times = np.arange(3, 8) * (1.6 * scale) / 7
        reference = z_start + np.sign(delta) * np.minimum(
            abs(delta) * (1 - np.exp(-times / self.altitude_time_constant)),
            self.altitude_rate * times)
        model_min = float(world[3:, 2].min())
        # Toward the goal the plan may not lag the reference; away from it the
        # model keeps a margin for obstacle avoidance (climb when the goal is
        # above, dip when it is below), so the cruise converges either way.
        if delta >= 0:
            world[3:, 2] = np.clip(world[3:, 2], reference, reference + self.altitude_climb_margin)
        else:
            world[3:, 2] = np.clip(world[3:, 2], reference - self.altitude_climb_margin, reference)
        info.update(reference_end=round(float(reference[-1]), 3),
                    model_min=round(model_min, 3), raised=bool(model_min < float(reference.min())))
        return info

    def inferred(self, context, now):
        try:
            res = context["future"].result()
            req = context["request"]
            if not res.success or res.request_id != req.request_id or res.observation_header != req.observation_header:
                raise ValueError(res.message or "invalid response identity")
            if (not self.inputs_fresh(now) or now-seconds(req.depth.header.stamp) > self.age or
                    now-context["observation"] > self.age or context["epoch"]-now < self.reserve or
                    time.monotonic()-context["sent"] > self.timeout):
                raise ValueError("expired FlowPilot result")
            reference = self.timeline.reference(context["epoch"]) if self.timeline.latest else None
            yaw = reference[1] if reference else context["yaw"]
            world = np.asarray(res.control_points, dtype=float).reshape(8, 3) \
                @ context["rotation"].T + context["origin"]
            if not np.isfinite(world).all():
                raise ValueError("invalid controls")
            scale, v_peak, a_peak = self.warp_scale(world)
            diagnostics = dict(scale=round(scale, 3), plan_speed=round(v_peak, 2),
                               plan_acceleration=round(a_peak, 2))
            if reference is not None:
                # Gate the model's unmodified candidate before splicing:
                # splicing must never conceal a route that ignores the state
                # it was planned from. Every component is an anchor check
                # against that measured state (the origin/velocity/
                # acceleration the request was built from): the candidate's
                # own implied start PVA, read from the raw control polygon
                # on its native 1.6 s time axis. Measuring against the plan
                # prediction at the takeover epoch instead mixes in two
                # structural artifacts that are not model defects: the
                # replan lag v*(lead + age) (0.26-0.28 m at ~1.1 m/s in the
                # 2026-11-24 4090 flight) for position, and the warp's
                # start-velocity scaling v/scale for velocity (0.62-0.75
                # m/s in the same flight's rev-3) — the warp stretches the
                # time axis of the whole polygon including its start, and
                # the splice re-anchors the executed start anyway. The
                # displacement the splice absorbs is published as splice
                # diagnostics; command continuity itself is verified after
                # splicing against the reference.
                raw_errors = dict(
                    position=float(np.linalg.norm(world[0] - context["origin"])),
                    velocity=float(np.linalg.norm((world[1] - world[0]) * 7 / 1.6
                                                  - context["velocity"])),
                    acceleration=float(np.linalg.norm((world[2] - 2 * world[1] + world[0]) * 42 / 1.6 ** 2
                                                      - context["acceleration"])),
                    yaw=abs(float(np.arctan2(np.sin(yaw - reference[1]),
                                             np.cos(yaw - reference[1])))))
                self.events.publish(String(data=json.dumps(dict(
                    event="raw_handoff", task=self.task,
                    revision=self.timeline.revision+1, errors=raw_errors,
                    limits=self.handoff_limits))))
                if any(error > self.handoff_limits[field] for field, error in raw_errors.items()):
                    raise ValueError("discontinuous model handoff: " + json.dumps(raw_errors))
                # Rolling revision: rebase the model's action onto the takeover
                # epoch and splice its start onto the executing plan's
                # predicted PVA there, so the commanded stream stays continuous.
                world, splice = splice_onto(world, reference, scale)
                diagnostics["splice"] = {key: round(value, 3) for key, value in splice.items()}
                # Keep the model's route inside the deployment safety box.
                # The reference implementation (upstream run_rviz_sim) clamps
                # its rollout into the world box; the model commands descent
                # even with the goal above, so the floor is load-bearing here.
                # The first three control points are left as spliced: they hold
                # the handoff-continuous start PVA (whose z is inside the box).
                diagnostics["altitude"] = self.track_altitude(world, scale)
                world[3:] = np.clip(world[3:], self.plan_lo, self.plan_hi)
                observation = context["epoch"]
            else:
                diagnostics["altitude"] = self.track_altitude(world, scale)
                world = np.clip(world, self.plan_lo, self.plan_hi)
                observation = context["observation"]
            curve = Segment(world, np.zeros(3), np.eye(3), observation, context["epoch"],
                            yaw, self.max_yaw_rate, scale)
            if reference is None:
                errors = dict(position=float(np.linalg.norm(curve.sample(curve.start)[0][0] -
                                                           vector(self.pose.pose.position))))
            else:
                errors = handoff_errors(reference, curve)
            self.events.publish(String(data=json.dumps(dict(event="handoff", task=self.task,
                revision=self.timeline.revision+1, errors=errors, limits=self.handoff_limits,
                **diagnostics))))
            if any(error > self.handoff_limits[field] for field, error in errors.items()):
                raise ValueError("discontinuous handoff: " + json.dumps(errors))
            self.submit(curve, now)
        except Exception as exc:
            self.failure("INFERENCE_RESULT:" + str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = FlowPilotPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.queued_goal = None
        node.stop('SHUTDOWN')
        deadline = time.monotonic()+11
        while rclpy.ok() and node.state == 'STOPPING' and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
