"""Timestamp-preserving odometry interpolation and camera pose composition."""

from __future__ import annotations

import math


def normalize(quaternion):
    norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError("invalid quaternion")
    result = tuple(float(value) / norm for value in quaternion)
    return tuple(-value for value in result) if result[3] < 0 else result


def multiply(left, right):
    x1, y1, z1, w1 = left
    x2, y2, z2, w2 = right
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def rotate(quaternion, vector):
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    return multiply(multiply(quaternion, (*vector, 0.0)), inverse)[:3]


def slerp(left, right, fraction):
    left, right = normalize(left), normalize(right)
    dot = sum(a*b for a, b in zip(left, right))
    if dot < 0:
        right, dot = tuple(-value for value in right), -dot
    if dot > 0.9995:
        return normalize(tuple(a + fraction*(b-a) for a, b in zip(left, right)))
    angle = math.acos(max(-1.0, min(1.0, dot)))
    scale = math.sin(angle)
    return normalize(tuple(
        (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / scale
        for a, b in zip(left, right)))


def interpolate(before, after, stamp):
    if after[0] <= before[0]:
        raise ValueError("odometry timestamps must increase")
    fraction = (stamp - before[0]) / (after[0] - before[0])
    if not -1e-9 <= fraction <= 1 + 1e-9:
        raise ValueError("stamp is outside odometry bracket")
    position = tuple(a + fraction*(b-a) for a, b in zip(before[1], after[1]))
    return position, slerp(before[2], after[2], fraction)


def compose(position, orientation, mount_position, mount_orientation):
    translated = rotate(orientation, mount_position)
    return (tuple(a+b for a, b in zip(position, translated)),
            normalize(multiply(orientation, mount_orientation)))
