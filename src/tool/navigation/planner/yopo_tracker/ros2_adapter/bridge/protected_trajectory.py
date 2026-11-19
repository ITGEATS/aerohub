"""Attach a continuous brake to the native curve before handing it to AS2."""
from dataclasses import dataclass
import math

import numpy as np

from ..native.tracker_model import SECOND


HANDOFF_LEAD_NS = SECOND // 5  # Existing ordinary YOPO execution lead.
CONTROL_STEP_NS = SECOND // 50  # Native reference sampling interval.
HOLD_NS = 10 * SECOND  # Keep the brake endpoint covered while verifying low speed.


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


def protect_curve(axes, knots, duration_ns, limits):
    """Keep the brake on the selected path, with measured overrun room along it."""
    ticks = np.array([*range(0, duration_ns, CONTROL_STEP_NS), duration_ns], dtype=np.int64)
    sampled = np.array([axis.get_position(ticks / SECOND) for axis in axes])
    remaining = np.r_[np.cumsum(np.linalg.norm(np.diff(sampled, axis=1), axis=0)[::-1])[::-1], 0.]
    usable = ticks[remaining >= limits.reserve_m]
    if not len(usable):
        return None
    # Prefer the latest usable splice up to the overlap used by the live Tracker.
    # Leave at least the normal handoff lead before the splice so a healthy
    # stream can roll onto a fresh candidate.
    for cut_ns in reversed(usable[(HANDOFF_LEAD_NS < usable) & (usable <= duration_ns // 2)]):
        cut_s = cut_ns / SECOND
        max_brake_ns = 2 * int(usable[-1] - cut_ns)
        for brake_ns in range(max_brake_ns // CONTROL_STEP_NS * CONTROL_STEP_NS, 0, -CONTROL_STEP_NS):
            brake_s = brake_ns / SECOND
            u = np.linspace(0., 1., brake_ns // CONTROL_STEP_NS + 1)
            advance, rate, accel, jerk = brake_clock(u, brake_s)
            t = cut_s + advance
            v = np.array([axis.get_velocity(t) for axis in axes])
            a = np.array([axis.get_acceleration(t) for axis in axes])
            j = np.array([axis.get_jerk(t) for axis in axes])
            if (np.any(np.linalg.norm(a * rate**2 + v * accel, axis=0) > limits.acceleration_m_s2)
                    or np.any(np.linalg.norm(j * rate**3 + 3 * a * rate * accel + v * jerk, axis=0) > limits.jerk_m_s3)):
                continue
            end_ns = int(cut_ns) + brake_ns
            wrapped = tuple(BrakedAxis(axis, cut_s, brake_s) for axis in axes)
            times = [*range(0, end_ns, CONTROL_STEP_NS), end_ns]
            native_times = [round(wrapped[0].clock(stamp / SECOND)[0] * SECOND) for stamp in times]
            # Use the same wrap-aware yaw interpolation as rolling handoffs.
            knot_times, knot_yaw = zip(*knots)
            yaw = np.interp(native_times, knot_times, np.unwrap(knot_yaw))
            held = list(zip(times, yaw))
            held.append((end_ns + HOLD_NS, held[-1][1]))
            reserve = float(np.interp(int(cut_ns) + brake_ns / 2, ticks, remaining))
            return ProtectedTrajectory(wrapped, held, int(cut_ns), end_ns, reserve)
    return None
