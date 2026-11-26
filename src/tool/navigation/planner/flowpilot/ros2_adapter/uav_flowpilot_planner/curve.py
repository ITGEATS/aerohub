"""Rebase a FlowPilot segment at a future handoff on its original time axis."""
from math import comb, pi
import numpy as np


def bernstein(points, t, domain=1.6):
    u = t / domain
    n = len(points) - 1
    return sum(comb(n, k)*u**k*(1-u)**(n-k)*p for k, p in enumerate(points))


def state_derivatives(times, velocities, epoch):
    times = np.asarray(times, dtype=float) - epoch
    values = np.asarray(velocities, dtype=float)
    if (times.ndim != 1 or values.shape != (len(times), 3) or
            not np.isfinite(times).all() or not np.isfinite(values).all() or
            np.any(np.diff(times) <= 0)):
        raise ValueError("velocity history must be finite, aligned and strictly increasing")
    mask = (times <= 1.e-7) & (times >= -.16)
    t, v = times[mask], values[mask]
    if len(t) < 3 or np.ptp(t) < .035 or not np.isfinite(v).all():
        raise ValueError("insufficient velocity history for acceleration and jerk")
    terms = np.column_stack((np.ones_like(t), t, .5*t*t))
    coeff, _, rank, _ = np.linalg.lstsq(terms, v, rcond=None)
    if rank != 3:
        raise ValueError("degenerate velocity history")
    return coeff[1], coeff[2]


def handoff_errors(reference, candidate):
    pva, yaw = reference
    new_pva, new_yaw = candidate.sample(candidate.start)
    return dict(position=float(np.linalg.norm(pva[0] - new_pva[0])),
                velocity=float(np.linalg.norm(pva[1] - new_pva[1])),
                acceleration=float(np.linalg.norm(pva[2] - new_pva[2])),
                yaw=abs(float(np.arctan2(np.sin(yaw - new_yaw), np.cos(yaw - new_yaw)))))


def plan_dynamics(world):
    """Peak speed and acceleration magnitudes of a raw world-frame control plan."""
    vel = np.diff(world, axis=0) * 7 / 1.6
    acc = np.diff(vel, axis=0) * 6 / 1.6
    ts = np.arange(16) * 1.6 / 16
    v_peak = max(float(np.linalg.norm(bernstein(vel, t))) for t in ts)
    a_peak = max(float(np.linalg.norm(bernstein(acc, t))) for t in ts)
    return v_peak, a_peak


def splice_onto(world, reference, scale):
    """Continuation splice: make a plan start exactly at a reference PVA.

    The rolling handoff gate and the execution behavior both compare the new
    plan's first emitted PVA against the executing plan's predicted PVA at the
    takeover epoch. The model re-plans from the measured (lagging) state, so
    its raw output disagrees with that prediction by the tracking lag plus its
    own re-ramp. Translating the plan and re-fixing the first two control
    points sets the start position, velocity and acceleration exactly, while
    the remaining control points keep the model's route and avoidance shape.
    """
    p_des, v_des, a_des = (np.asarray(value, dtype=float) for value in reference[0])
    T = 1.6 * scale
    delta_p = p_des - world[0]
    delta_v = v_des - (world[1] - world[0]) * 7 / T
    delta_a = a_des - (world[2] - 2 * world[1] + world[0]) * 42 / T ** 2
    spliced = world + delta_p
    spliced[1] = spliced[0] + v_des * T / 7
    spliced[2] = 2 * spliced[1] - spliced[0] + a_des * T ** 2 / 42
    return spliced, dict(position=float(np.linalg.norm(delta_p)),
                         velocity=float(np.linalg.norm(delta_v)),
                         acceleration=float(np.linalg.norm(delta_a)))


def degree_elevate(points, degree):
    """Exact Bézier degree elevation of a control polygon up to `degree`."""
    pts = np.asarray(points, dtype=float)
    while len(pts) - 1 < degree:
        n = len(pts) - 1
        elevated = np.empty((n + 2, pts.shape[1]))
        elevated[0], elevated[-1] = pts[0], pts[-1]
        w = np.arange(1, n + 1) / (n + 1)
        elevated[1:-1] = w[:, None] * pts[:-1] + (1 - w[:, None]) * pts[1:]
        pts = elevated
    return pts


def brake_segment(reference, epoch, acceleration, yaw_rate_limit=pi / 2,
                  min_duration=.6, max_duration=2.4):
    """Final braking segment from an executing plan's full PVA.

    The behavior's terminal path for a final segment switches to streaming
    position commands once the curve ends, which moves the controller out of
    trajectory mode and lets the planner observe a real stop; the stop service
    alone sends a single hover reference that does not.

    The segment pins its start to the complete reference PVA: position,
    velocity AND acceleration. The behavior gates replacements for C2
    continuity against the active plan at the takeover epoch, and a brake
    that silently drops the plan's acceleration to zero is rejected as a
    discontinuous replacement (measured 0.91 and 1.84 m/s^2 in the
    2026-11-24 4090 flights) before it can ever run. A quintic Hermite from
    the reference PVA to rest keeps the replacement continuous while the
    remaining control points hold the constant-deceleration stopping shape.
    """
    pva, yaw = reference
    p0, v0, a0 = (np.asarray(value, dtype=float) for value in pva[:3])
    speed = float(np.linalg.norm(v0))
    duration = float(np.clip(speed / acceleration, min_duration, max_duration)) \
        if speed > 1e-3 else min_duration
    end = p0 + v0 * duration / 2
    quintic = np.stack((p0,
                        p0 + v0 * duration / 5,
                        p0 + 2 * v0 * duration / 5 + a0 * duration ** 2 / 20,
                        end, end, end))
    world = degree_elevate(quintic, 7)
    return Segment(world, np.zeros(3), np.eye(3), epoch, epoch, yaw,
                   yaw_rate_limit, duration / 1.6), duration


class Segment:
    def __init__(self, controls, origin, rotation, observation, handoff, yaw,
                 yaw_rate_limit=pi / 2, scale=1.):
        self.scale = float(scale)
        self.start = handoff
        self.observation = observation
        self.end = observation + 47 / 30 * self.scale
        if not (np.isfinite([observation, handoff, yaw, yaw_rate_limit]).all() and
                yaw_rate_limit > 0 and self.scale > 0 and
                observation <= handoff < self.end - self.scale / 30):
            raise ValueError("handoff outside sampled trajectory")
        world = np.asarray(controls).reshape(8, 3) @ rotation.T + origin
        if not np.isfinite(world).all():
            raise ValueError("invalid controls")
        vel = np.diff(world, axis=0) * 7 / (1.6 * self.scale)
        self.coefficients = (world, vel, np.diff(vel, axis=0) * 6 / (1.6 * self.scale))
        sample_times = np.arange(48) / 30 * self.scale - (handoff - observation)
        # The ROS Duration grid is also the grid used by the execution sampler.
        ns = np.unique(np.rint(sample_times * 1e9).astype(np.int64))
        self.times = np.r_[0., ns[ns > 0] / 1e9]
        self.end = self.start + self.times[-1]
        self.initial_yaw = yaw
        self.points = np.array([self._pva(self.start + t) for t in self.times])
        self.yaws = np.empty(len(self.times))
        self.yaws[0] = yaw
        for index in range(1, len(self.times)):
            velocity = self.points[index, 1, :2]
            previous = self.yaws[index - 1]
            target = np.arctan2(velocity[1], velocity[0]) if np.linalg.norm(velocity) > .1 else previous
            delta = np.arctan2(np.sin(target - previous), np.cos(target - previous))
            limit = yaw_rate_limit * (self.times[index] - self.times[index - 1])
            self.yaws[index] = previous + np.clip(delta, -limit, limit)

    def _pva(self, epoch):
        t = np.clip(epoch, self.start, self.end) - self.observation
        return np.array([bernstein(c, t, 1.6 * self.scale) for c in self.coefficients])

    def sample(self, epoch):
        yaw = np.interp(epoch - self.start, self.times, self.yaws)
        return self._pva(epoch), float(np.arctan2(np.sin(yaw), np.cos(yaw)))

    def execution_sample(self, epoch):
        """Match FollowTimedPath's quintic interpolation between emitted points."""
        elapsed = np.clip(epoch - self.start, 0., self.times[-1])
        index = min(max(np.searchsorted(self.times, elapsed, side="right") - 1, 0), len(self.times) - 2)
        duration = self.times[index + 1] - self.times[index]
        u = (elapsed - self.times[index]) / duration
        left, right = self.points[index:index + 2]
        c0 = left[0]
        c1 = duration * left[1]
        c2 = duration ** 2 * left[2] / 2
        delta_p = right[0] - c0 - c1 - c2
        delta_v = duration * right[1] - c1 - 2 * c2
        delta_a = duration ** 2 * right[2] - 2 * c2
        coefficients = (c0, c1, c2, 10 * delta_p - 4 * delta_v + delta_a / 2,
                        -15 * delta_p + 7 * delta_v - delta_a,
                        6 * delta_p - 3 * delta_v + delta_a / 2)
        position = sum(c * u ** k for k, c in enumerate(coefficients))
        velocity = sum(k * coefficients[k] * u ** (k - 1) for k in range(1, 6)) / duration
        acceleration = sum(k * (k - 1) * coefficients[k] * u ** (k - 2) for k in range(2, 6)) / duration ** 2
        return np.array([position, velocity, acceleration]), self.sample(epoch)[1]
