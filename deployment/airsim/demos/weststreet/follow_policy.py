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


def recovery_destination(state, *, now, frame, altitude, offset, previous=None, window=.5, freeze_after=.3):
    """Validate prediction lifetime; during extended recovery freeze the last waypoint.

    This stops advancing the requested position, not a claim of zero physical
    acceleration: the deployed planner/Behavior still owns braking and feasibility.
    """
    if not math.isfinite(window) or not .5<=window<=2.:
        raise ValueError('recovery window must be within .5..2 seconds')
    if not math.isfinite(freeze_after) or not 0<freeze_after<window:
        raise ValueError("freeze_after must be within (0, window)")
    point=destination(state,now=now,frame=frame,altitude=altitude,offset=offset,
                      max_age=max(.6,window))
    age=now-(state.observation_stamp.sec+state.observation_stamp.nanosec/1e9)
    recovering=window>.5 and age>freeze_after
    return (previous if recovering and previous is not None else point),recovering
