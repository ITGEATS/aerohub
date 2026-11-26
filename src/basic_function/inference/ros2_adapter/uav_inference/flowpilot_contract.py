"""FlowPilot ROS payload validation without ROS runtime dependencies."""
import base64
import math
import zlib

import numpy as np


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid observation timestamp")
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def depth_array(image):
    """Depth in metres as a float32 array; invalid pixels are filled with 20.0 m."""
    formats = {"32FC1": ("f4", 1.0), "16UC1": ("u2", 0.001)}
    if image.encoding not in formats:
        raise ValueError("unsupported depth encoding")
    code, scale = formats[image.encoding]
    dtype = np.dtype((">" if image.is_bigendian else "<") + code)
    height, width = image.height, image.width
    if not (0 < height * width <= 1_048_576 and
            image.step >= width * dtype.itemsize and
            len(image.data) == height * image.step):
        raise ValueError("invalid depth dimensions or stride")
    raw = np.ndarray((height, width), dtype, buffer=bytes(image.data),
                     strides=(image.step, dtype.itemsize))
    meters = raw.astype(np.float32) * scale
    valid = np.isfinite(meters) & (meters > 0)
    if not valid.any():
        raise ValueError("depth frame has no valid measurements")
    return np.where(valid, meters, 20.0)


def depth_m(image):
    """Legacy JSON wire encoding (kept for tests and backward compatibility)."""
    return depth_array(image).tolist()


def encode_depth(image):
    """Compact depth wire encoding: base64(zlib(float32 row-major, LE)).

    Semantically identical to depth_m(): the same post-fill metres reach the
    worker. The encoding exists because the JSON float layout costs ~617 KiB
    per request, which over a remote uplink blew the 0.25 s inference budget
    (2026-11-23 server-deployment evidence, log/flowpilot-sitl-01); zlib on
    the smooth depth plane compresses to ~18%, ~57 KiB on the wire.
    """
    meters = np.ascontiguousarray(depth_array(image), dtype="<f4")
    blob = zlib.compress(meters.tobytes(), 6)
    return base64.b64encode(blob).decode("ascii"), [int(meters.shape[0]), int(meters.shape[1])]


def request_payload(request, now_ns, max_age_s=0.3, max_skew_s=0.05):
    image_time = stamp_ns(request.depth.header.stamp)
    state_time = stamp_ns(request.observation_header.stamp)
    if not request.depth.header.frame_id or not request.observation_header.frame_id:
        raise ValueError("depth and observation frames are required")
    if abs(image_time - state_time) > round(max_skew_s * 1e9):
        raise ValueError("depth and state are unsynchronized")
    if max(image_time, state_time) > now_ns or now_ns - min(image_time, state_time) > round(max_age_s * 1e9):
        raise ValueError("observation is stale or in the future")
    state = [float(value) for value in request.state_goal]
    if len(state) != 25 or not all(math.isfinite(value) for value in state):
        raise ValueError("state_goal must contain 25 finite values")
    if not math.isfinite(request.speed_mps) or not 0 < request.speed_mps <= 10:
        raise ValueError("speed_mps must be finite and in (0, 10]")
    if not 0 < len(request.request_id) <= 128:
        raise ValueError("request_id must contain 1..128 characters")
    encoded, shape = encode_depth(request.depth)
    return dict(request_id=request.request_id, depth_b64_gz=encoded, depth_shape=shape,
                state_goal=state, speed_mps=float(request.speed_mps))
