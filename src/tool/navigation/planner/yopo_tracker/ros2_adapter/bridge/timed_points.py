"""Encode a frozen ENU quintic segment at recorded yaw knots for TimedPath."""
import math

from uav_planner_interfaces.action import FollowTimedPath
from uav_planner_interfaces.msg import TimedTrajectoryPoint


def timed_points(axes, duration, yaw_knots):
    """axes are three upstream Poly5Solver instances. Knot times are integer ns."""
    if len(axes) != 3 or not math.isfinite(duration) or duration <= 0:
        raise ValueError('Expected three ENU polynomials and positive finite duration')
    knots = list(yaw_knots)
    end_ns = round(duration * 1e9)
    if len(knots) < 2 or knots[0][0] != 0 or knots[-1][0] != end_ns:
        raise ValueError('Yaw knots must include both segment endpoints')
    points = []
    previous = -1
    for time_ns, yaw in knots:
        if type(time_ns) is not int or time_ns <= previous or not math.isfinite(yaw):
            raise ValueError('Expected strictly increasing nanoseconds and finite yaw')
        previous = time_ns
        t = time_ns / 1e9
        point = TimedTrajectoryPoint()
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(time_ns, 10**9)
        for attr, axis in zip(('x', 'y', 'z'), axes):
            for field, value in ((point.position, axis.get_position(t)),
                                 (point.velocity, axis.get_velocity(t)),
                                 (point.acceleration, axis.get_acceleration(t))):
                if not math.isfinite(value):
                    raise ValueError('Non-finite polynomial sample')
                setattr(field, attr, float(value))
        point.yaw = float(yaw)
        points.append(point)
    return points


def timed_goal(axes, duration, yaw_knots, *, task_id, vehicle_id, revision,
               generated_ns, start_ns, max_speed):
    """Encode a frozen single-vehicle ENU segment with caller-owned epochs."""
    if (not task_id or not vehicle_id or revision <= 0 or
            not 0 <= generated_ns < start_ns or
            not math.isfinite(max_speed) or max_speed <= 0):
        raise ValueError('Invalid segment identity, execution times, or speed')
    goal = FollowTimedPath.Goal()
    path = goal.path
    path.interface_version = 1
    path.task_id = task_id
    path.vehicle_id = vehicle_id
    path.frame_id = f'{vehicle_id}/odom'
    path.revision = revision
    path.generated_at.sec, path.generated_at.nanosec = divmod(generated_ns, 10**9)
    path.valid_from.sec, path.valid_from.nanosec = divmod(start_ns, 10**9)
    path.valid_fields = 7
    path.is_final = False
    path.max_speed = float(max_speed)
    path.points = timed_points(axes, duration, yaw_knots)
    return goal
