"""Legacy rolling timed-trajectory entry; Registry uses goto_node by default."""
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from uav_planner_interfaces.msg import TimedTrajectoryPoint, TimedTrajectorySegment, VehicleTrajectory

from . import policy
from .frames import enu_ned_vector, enu_to_ned_yaw
from .observation_node import AerovlaObservationNode


def duration_msg(seconds):
    ns = int(round(seconds * 1e9))
    sec, nanosec = divmod(ns, 1000000000)
    return Duration(seconds=int(sec), nanoseconds=nanosec).to_msg()


class AerovlaPlannerNode(AerovlaObservationNode):
    def configure_execution(self):
        param = lambda name, default: self.declare_parameter(name, default).value
        self.valid_from_lead = param('valid_from_lead_s', 0.05)
        self.roll_horizon_s = param('roll_horizon_s', 6.0)
        if self.resolved_view:
            settings = self.resolved_view['parameters']
            self.valid_from_lead = float(settings['valid_from_lead_s'])
            self.roll_horizon_s = float(settings.get('roll_horizon_s', self.roll_horizon_s))
        self.ref_handoff = None
        self.trajectory_topic = param('trajectory_topic', 'trajectory')
        trajectory_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trajectory_pub = self.create_publisher(TimedTrajectorySegment,
                                                   self.trajectory_topic, trajectory_qos)

    def on_goal(self, message):
        self.ref_handoff = None
        super().on_goal(message)

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
