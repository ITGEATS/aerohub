"""Convert original polynomial samples to an ordinary timed trajectory."""
import math
from uav_planner_interfaces.msg import TimedTrajectoryPoint, TimedTrajectorySegment, VehicleTrajectory
from .core import enu_to_ned, split_ns


def fill_time(target, ns):
    target.sec, target.nanosec = split_ns(ns)


def message(task_id, vehicle_id, generated_ns, valid_ns, rows):
    segment = TimedTrajectorySegment()
    segment.interface_version = 1
    segment.task_id = task_id
    fill_time(segment.generated_at, generated_ns)
    fill_time(segment.valid_from, valid_ns)
    segment.is_final = False  # continuous tracking has no declared arrival
    trajectory = VehicleTrajectory()
    trajectory.vehicle_id = vehicle_id
    trajectory.valid_fields = trajectory.VELOCITY | trajectory.ACCELERATION | trajectory.YAW
    for row in rows:
        point = TimedTrajectoryPoint()
        fill_time(point.time_from_start, row['time_ns'])
        for field, values in zip(('position', 'velocity', 'acceleration'), row['pva']):
            target = getattr(point, field)
            target.x, target.y, target.z = enu_to_ned(values)
        point.yaw = math.pi / 2 - row['yaw']
        trajectory.points.append(point)
    segment.vehicle_trajectories.append(trajectory)
    return segment
