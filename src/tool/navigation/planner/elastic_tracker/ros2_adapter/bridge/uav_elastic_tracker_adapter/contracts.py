"""One payload acceptance result for transport and health (W1, no age policy)."""
from dataclasses import dataclass
import json
import math
import time
from collections import Counter


class InputRejected(ValueError):
    pass


# CAT-ORA state.py/README input window, not newly selected limits.
MAX_AGE_NS = 1_000_000_000
MAX_FUTURE_NS = 50_000_000


def check_time(stamp_ns, now_ns, previous_ns=None, max_age_ns=MAX_AGE_NS, max_future_ns=MAX_FUTURE_NS):
    if stamp_ns <= 0 or now_ns <= 0:
        raise InputRejected('zero_or_uninitialized_stamp')
    if previous_ns is not None and stamp_ns <= previous_ns:
        raise InputRejected('non_increasing_stamp')
    if stamp_ns-now_ns > max_future_ns:
        raise InputRejected('future_stamp')
    if now_ns-stamp_ns > max_age_ns:
        raise InputRejected('stale_stamp')


class InputDiagnostics:
    def __init__(self, logger):
        self.counts = Counter()
        self.warned = {}
        self.logger = logger

    def reject(self, reason, **fields):
        self.counts[reason] += 1
        now = time.monotonic()
        previous = self.warned.get(reason)
        # CAT-ORA input_node.py's existing two-second warning throttle; diagnostic only.
        if previous is None or now-previous > 2.0:
            self.logger.warning(reason+(' '+json.dumps(fields, sort_keys=True) if fields else ''))
            self.warned[reason] = now


@dataclass(frozen=True)
class AcceptedOdometry:
    stamp_ns: int
    payload: dict


def serialize_odometry(value):
    p, q = value.pose.pose.position, value.pose.pose.orientation
    v, w = value.twist.twist.linear, value.twist.twist.angular
    return {'header': {'stamp': [value.header.stamp.sec, value.header.stamp.nanosec],
                       'frame_id': value.header.frame_id}, 'child_frame_id': value.child_frame_id,
            'p': [p.x, p.y, p.z], 'q': [q.x, q.y, q.z, q.w],
            'v': [v.x, v.y, v.z], 'w': [w.x, w.y, w.z],
            'pose_covariance': list(value.pose.covariance),
            'twist_covariance': list(value.twist.covariance)}


def accept_odometry(message, frame, now_ns=None, previous_ns=None):
    payload = serialize_odometry(message)
    if message.header.frame_id != frame:
        raise InputRejected('unexpected_frame')
    sec, nsec = payload['header']['stamp']
    if not isinstance(sec, int) or not isinstance(nsec, int) or not 0 <= nsec < 1_000_000_000:
        raise InputRejected('invalid_stamp_encoding')
    sizes = {'p': 3, 'q': 4, 'v': 3, 'w': 3, 'pose_covariance': 36, 'twist_covariance': 36}
    for key, size in sizes.items():
        values = payload[key]
        if len(values) != size or not all(math.isfinite(v) for v in values):
            raise InputRejected('invalid_payload_'+key)
    if sum(v*v for v in payload['q']) == 0:
        raise InputRejected('undefined_orientation')
    stamp_ns = sec*1_000_000_000+nsec
    if now_ns is not None:
        check_time(stamp_ns, now_ns, previous_ns)
    return AcceptedOdometry(stamp_ns, payload)
