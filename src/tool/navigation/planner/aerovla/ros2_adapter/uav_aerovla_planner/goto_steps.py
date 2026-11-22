"""ROS-free mapping of AeroVLA actions to sequential native GoTo goals.

No timing or P/V/A is invented here. The caller waits for each native result
AND measured convergence before proceeding. This is position-control execution,
not a claim of equivalence to upstream constant-velocity dynamics.
"""

from dataclasses import dataclass
import math

from . import policy
from .frames import enu_ned_vector, enu_to_ned_yaw


@dataclass(frozen=True)
class Step:
    position_enu: tuple
    yaw_enu: float
    max_speed: float
    kind: str


def action_steps(state_ned, forward, down, yaw):
    """Preserve upstream branch order, including dropped large-turn XY motion."""
    if not all(math.isfinite(v) for v in (forward, down, yaw)):
        raise ValueError('action must be finite')
    position = tuple(float(v) for v in state_ned['position'])
    current_yaw = policy.yaw_from_quat(state_ned['orientation'])
    target_yaw = current_yaw + yaw
    dx, dy = forward * math.cos(target_yaw), forward * math.sin(target_yaw)
    distance = math.sqrt(dx * dx + dy * dy + down * down)
    if not (abs(forward) > policy.EPSILON or abs(down) > policy.EPSILON
            or abs(yaw) > policy.EPSILON):
        return []
    yaw_enu = enu_to_ned_yaw(target_yaw)  # involutory planar heading map
    if abs(yaw) >= policy.YAW_THRESHOLD or distance <= policy.EPSILON:
        steps = [Step(tuple(enu_ned_vector(position)), yaw_enu,
                      policy.CRUISE_SPEED, 'rotate')]
        if abs(down) > policy.EPSILON:
            target = (position[0], position[1], position[2] + down)
            steps.append(Step(tuple(enu_ned_vector(target)), yaw_enu,
                              policy.VERTICAL_SPEED, 'vertical'))
        return steps
    target = (position[0] + dx, position[1] + dy, position[2] + down)
    # Upstream micro move duration is 1 s; larger moves use cruise speed.
    speed = distance if distance < policy.MICRO_MOVE_THRESHOLD else policy.CRUISE_SPEED
    return [Step(tuple(enu_ned_vector(target)), yaw_enu, speed, 'move')]


class Convergence:
    """A native position-only SUCCESS does not finish the Tool's step."""

    def __init__(self, position_tolerance, yaw_tolerance, speed_tolerance, hold_s):
        values = (position_tolerance, yaw_tolerance, speed_tolerance, hold_s)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('convergence tolerances and hold must be positive finite')
        self.position_tolerance, self.yaw_tolerance, self.speed_tolerance, self.hold_s = values
        self.since = None
        self.last_stamp = None

    def update(self, step, position, yaw, speed, stamp, fresh):
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.since = None
        valid = (fresh and all(math.isfinite(v) for v in (*position, yaw, speed, stamp))
                 and speed >= 0
                 and math.dist(step.position_enu, position) <= self.position_tolerance
                 and abs(math.remainder(yaw - step.yaw_enu, 2 * math.pi)) <= self.yaw_tolerance
                 and speed <= self.speed_tolerance)
        if not valid:
            self.since = None
        elif self.since is None:
            self.since = stamp
        self.last_stamp = stamp
        return valid and stamp - self.since >= self.hold_s
