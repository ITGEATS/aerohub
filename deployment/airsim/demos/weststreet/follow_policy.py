"""Mission policy over Tool outputs; no ROS, platform or simulator dependencies."""
import math


def destination(state, *, now, frame, altitude, offset, max_age=.6):
    stamp = lambda value: value.sec + value.nanosec / 1e9
    if not state.valid or state.status not in ('tracked', 'predicted'):
        raise ValueError('Tracking target unavailable: ' + state.reason)
    if state.header.frame_id != frame:
        raise ValueError('Tracking and System localization frames differ')
    if not 0 <= now - stamp(state.observation_stamp) <= max_age:
        raise ValueError('Tracking observation stale or clock reset')
    if now > stamp(state.valid_until):
        raise ValueError('Tracking validity expired')
    point = (state.position.x + offset[0], state.position.y + offset[1], altitude)
    if not all(math.isfinite(x) for x in point):
        raise ValueError('Non-finite destination')
    return point


def follow_offset(own, target, distance):
    """Fixed world-frame offset along the initial person-to-drone direction."""
    dx, dy = own[0]-target[0], own[1]-target[1]
    length = math.hypot(dx, dy)
    if not math.isfinite(length) or length < .25:
        raise ValueError('Initial horizontal separation too small')
    return (distance*dx/length, distance*dy/length)
