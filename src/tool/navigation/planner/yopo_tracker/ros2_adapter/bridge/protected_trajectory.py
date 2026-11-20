"""Attach a continuous brake to the native curve before handing it to AS2."""
from dataclasses import dataclass
from itertools import islice
import math

import numpy as np

from ..native.tracker_model import SECOND


HANDOFF_LEAD_NS = SECOND // 5  # Existing ordinary YOPO execution lead.
CONTROL_STEP_NS = SECOND // 50  # Native reference sampling interval.
HOLD_NS = 10 * SECOND  # Keep the brake endpoint covered while verifying low speed.
BRAKE_BATCH_SIZE = 96


@dataclass(frozen=True)
class BrakeLimits:
    reserve_m: float
    acceleration_m_s2: float
    jerk_m_s3: float

    @classmethod
    def from_calibration(cls, values):
        limits = cls(float(values['observed_terminal_overrun_m']),
                     float(values['reference_acceleration_m_s2']),
                     float(values['reference_jerk_m_s3']))
        if not all(math.isfinite(x) and x > 0 for x in
                   (limits.reserve_m, limits.acceleration_m_s2, limits.jerk_m_s3)):
            raise ValueError('Brake calibration values must be finite and positive')
        return limits


def brake_clock(u, duration):
    # Integrate the cubic smoothstep slowdown. Native time advances by T/2,
    # with unit rate at the splice and zero rate/acceleration at the endpoint.
    return (duration * (u - u**3 + u**4 / 2),
            1 - 3 * u**2 + 2 * u**3,
            (-6 * u + 6 * u**2) / duration,
            (-6 + 12 * u) / duration**2)


@dataclass(frozen=True)
class BrakedAxis:
    axis: object
    cut_s: float
    brake_s: float

    def clock(self, t):
        if t <= self.cut_s:
            return t, 1., 0.
        u = min(1., (t - self.cut_s) / self.brake_s)
        advance, rate, accel, _ = brake_clock(u, self.brake_s)
        return self.cut_s + advance, rate, accel

    def get_position(self, t):
        native_t, _, _ = self.clock(t)
        return self.axis.get_position(native_t)

    def get_velocity(self, t):
        native_t, rate, _ = self.clock(t)
        return self.axis.get_velocity(native_t) * rate

    def get_acceleration(self, t):
        native_t, rate, accel = self.clock(t)
        return self.axis.get_acceleration(native_t) * rate**2 + self.axis.get_velocity(native_t) * accel


@dataclass(frozen=True)
class ProtectedTrajectory:
    axes: tuple
    knots: list
    brake_from_ns: int
    brake_end_ns: int
    reserve_m: float

    @property
    def duration_ns(self):
        return self.brake_end_ns + HOLD_NS


def _exceeds_limits(axes, cuts_ns, brake_steps, sample_indices, limits):
    # Match the original linspace grid, including its exact final sample.
    u = np.where(sample_indices >= brake_steps, 1., sample_indices * (1. / brake_steps))
    advance, rate, accel, jerk = brake_clock(u, brake_steps * CONTROL_STEP_NS / SECOND)
    t = cuts_ns / SECOND + advance
    v = np.array([axis.get_velocity(t) for axis in axes])
    a = np.array([axis.get_acceleration(t) for axis in axes])
    j = np.array([axis.get_jerk(t) for axis in axes])
    acceleration = np.linalg.norm(a * rate**2 + v * accel, axis=0)
    jerk = np.linalg.norm(j * rate**3 + 3 * a * rate * accel + v * jerk, axis=0)
    return (acceleration > limits.acceleration_m_s2) | (jerk > limits.jerk_m_s3)


def protect_curve(axes, knots, duration_ns, limits):
    """Keep the brake on the selected path, with measured overrun room along it."""
    ticks = np.array([*range(0, duration_ns, CONTROL_STEP_NS), duration_ns], dtype=np.int64)
    sampled = np.array([axis.get_position(ticks / SECOND) for axis in axes])
    remaining = np.r_[np.cumsum(np.linalg.norm(np.diff(sampled, axis=1), axis=0)[::-1])[::-1], 0.]
    usable = ticks[remaining >= limits.reserve_m]
    if not len(usable):
        return None
    # Prefer the latest usable splice up to the overlap used by the live Tracker.
    cut_candidates = usable[(HANDOFF_LEAD_NS < usable) & (usable <= duration_ns // 2)]
    candidates = (
        (cut_ns, steps)
        for cut_ns in reversed(cut_candidates)
        for steps in range(2 * int(usable[-1] - cut_ns) // CONTROL_STEP_NS, 0, -1)
    )
    while batch := list(islice(candidates, BRAKE_BATCH_SIZE)):
        cuts_ns, brake_steps = np.asarray(batch, dtype=np.int64).T[:, :, None]
        # Screen on a subset of the same grid. All survivors still see every sample.
        screen_indices = brake_steps * np.arange(9) // 8
        survives = ~np.any(_exceeds_limits(axes, cuts_ns, brake_steps, screen_indices, limits), axis=1)
        if not np.any(survives):
            continue
        cuts_ns, brake_steps = cuts_ns[survives], brake_steps[survives]
        indices = np.arange(int(brake_steps.max()) + 1)
        exceeds = _exceeds_limits(axes, cuts_ns, brake_steps, indices, limits)
        allowed = np.flatnonzero(~np.any((indices <= brake_steps) & exceeds, axis=1))
        if not allowed.size:
            continue

        first = allowed[0]
        cut_ns = int(cuts_ns[first, 0])
        brake_ns = int(brake_steps[first, 0]) * CONTROL_STEP_NS
        end_ns = cut_ns + brake_ns
        wrapped = tuple(BrakedAxis(axis, cut_ns / SECOND, brake_ns / SECOND) for axis in axes)
        times = [*range(0, end_ns, CONTROL_STEP_NS), end_ns]
        native_times = [round(wrapped[0].clock(stamp / SECOND)[0] * SECOND) for stamp in times]
        # Use the same wrap-aware yaw interpolation as rolling handoffs.
        knot_times, knot_yaw = zip(*knots)
        yaw = np.interp(native_times, knot_times, np.unwrap(knot_yaw))
        held = list(zip(times, yaw))
        held.append((end_ns + HOLD_NS, held[-1][1]))
        reserve = float(np.interp(cut_ns + brake_ns / 2, ticks, remaining))
        return ProtectedTrajectory(wrapped, held, cut_ns, end_ns, reserve)
    return None
