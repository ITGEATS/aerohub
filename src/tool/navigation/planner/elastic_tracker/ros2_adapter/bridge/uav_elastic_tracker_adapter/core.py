"""ROS-independent quintic conversion. Units: seconds, metres, ENU radians."""
import math
from bisect import bisect_right

NANOSECONDS = 1_000_000_000


class InsufficientCoverage(ValueError):
    """A valid source curve has no remaining coverage at the requested handoff."""


def stamp_ns(stamp):
    return int(stamp[0]) * NANOSECONDS + int(stamp[1])


def split_ns(value):
    return list(divmod(int(value), NANOSECONDS))


def evaluate(coefficients, order, piece, t, derivative):
    block = coefficients[piece * (order + 1):(piece + 1) * (order + 1)]
    result = 0.0
    for index, coefficient in enumerate(block):
        power = order - index
        if power >= derivative:
            factor = math.factorial(power) / math.factorial(power - derivative)
            result += coefficient * factor * t ** (power - derivative)
    return result


def unwrap(value, reference):
    return reference + math.atan2(math.sin(value - reference), math.cos(value - reference))


def yaw_at(rows, valid_from_ns, absolute_ns):
    """Sample the previously published yaw exactly as executor linear yaw does.

    This is a candidate reference, not proof of Action acceptance. The framework
    terminates this task if that candidate is rejected; no acknowledgement is
    invented here. Endpoint clamping follows unchanged Timeline::sample.
    """
    relative = min(max(absolute_ns-valid_from_ns, 0), rows[-1]['time_ns'])
    upper = bisect_right([row['time_ns'] for row in rows], relative)
    upper = min(max(upper, 1), len(rows)-1)
    left, right = rows[upper-1], rows[upper]
    fraction = (relative-left['time_ns'])/(right['time_ns']-left['time_ns'])
    return left['yaw'] + fraction*(unwrap(right['yaw'], left['yaw'])-left['yaw'])


def suffix(native, valid_ns, sample_dt, yaw_reference):
    """Evaluate the original epoch; preserve every remaining piece boundary.

    Yaw slew uses traj_server.cpp's 0.02 rad per 0.01 s, expressed as a rate.
    No time stretching, endpoint extension or past-trajectory relabelling.
    """
    start_ns = stamp_ns(native['start_time'])
    if native['hover']:
        raise ValueError('native hover is a stop intent, not a new finite trajectory')
    if native['order'] != 5 or not native['duration']:
        raise ValueError('expected upstream nonempty quintic trajectory')
    durations = native['duration']
    if any(not math.isfinite(d) or d <= 0 for d in durations):
        raise ValueError('invalid native piece duration')
    expected = len(durations) * (native['order'] + 1)
    if any(len(native[k]) != expected or not all(map(math.isfinite, native[k]))
           for k in ('coef_x', 'coef_y', 'coef_z')):
        raise ValueError('invalid polynomial coefficients')
    offset = (valid_ns - start_ns) / NANOSECONDS
    end = math.fsum(durations)
    if offset < 0 or offset >= end:
        raise InsufficientCoverage('no native coverage at requested future handoff')
    # Quantize once onto the ROS Duration grid before sorting/deduplication.
    end_ns = round(end * NANOSECONDS)
    offset_ns = valid_ns - start_ns
    if end_ns <= offset_ns:
        raise InsufficientCoverage('native future coverage collapsed on the ROS Duration grid')
    step_ns = round(sample_dt * NANOSECONDS)
    if step_ns <= 0:
        raise ValueError('sample_dt must represent a positive ROS Duration')
    samples = set(range(offset_ns, end_ns, step_ns)) | {end_ns, offset_ns}
    boundaries = [math.fsum(durations[:i + 1]) for i in range(len(durations))]
    samples.update(round(t * NANOSECONDS) for t in boundaries
                   if round(t * NANOSECONDS) >= offset_ns)
    desired = unwrap(native['yaw'], yaw_reference)
    rate = 0.02 / 0.01  # upstream traj_server.cpp, not a new acceptance threshold
    rows = []
    for absolute_ns in sorted(samples):
        t = absolute_ns / NANOSECONDS
        piece = 0
        while piece + 1 < len(durations) and t > boundaries[piece]:
            piece += 1
        local = t - (boundaries[piece - 1] if piece else 0.0)
        pva = [[evaluate(native[key], native['order'], piece, local, d)
                for key in ('coef_x', 'coef_y', 'coef_z')] for d in range(3)]
        elapsed = (absolute_ns - offset_ns) / NANOSECONDS
        delta = desired - yaw_reference
        yaw = yaw_reference + max(-rate * elapsed, min(rate * elapsed, delta))
        rows.append({'time_ns': absolute_ns - offset_ns, 'pva': pva, 'yaw': yaw})
    return rows


def enu_to_ned(vector):
    return vector[1], vector[0], -vector[2]
