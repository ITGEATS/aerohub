"""ROS-independent conversion and validation for AeroVLA inference."""

import base64
import math

import cv2
import numpy as np

MAX_PIXELS = 1024 * 1024


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
        raise ValueError("invalid observation timestamp")
    return stamp.sec * 1000000000 + stamp.nanosec


def _bgr(image, name):
    if image.encoding not in ("rgb8", "bgr8"):
        raise ValueError("%s encoding must be rgb8 or bgr8" % name)
    if not image.width or not image.height or image.width * image.height > MAX_PIXELS:
        raise ValueError("%s must contain 1..%d pixels" % (name, MAX_PIXELS))
    if image.step < image.width * 3 or len(image.data) != image.step * image.height:
        raise ValueError("invalid %s row stride or data length" % name)
    raw = np.frombuffer(image.data, np.uint8).reshape(image.height, image.step)[:, :image.width * 3]
    bgr = raw.reshape(image.height, image.width, 3)
    if image.encoding == "rgb8":
        bgr = bgr[:, :, ::-1]
    ok, encoded = cv2.imencode(".png", np.ascontiguousarray(bgr))
    if not ok:
        raise ValueError("%s could not be PNG-encoded" % name)
    return base64.b64encode(encoded.tobytes()).decode("ascii")


def request_payload(request, now_ns, max_age_s, max_skew_s):
    if not request.front.header.frame_id or not request.down.header.frame_id:
        raise ValueError("front and down frame_id are required")
    front_time = stamp_ns(request.front.header.stamp)
    down_time = stamp_ns(request.down.header.stamp)
    if abs(front_time - down_time) > max_skew_s * 1e9:
        raise ValueError("front/down timestamps exceed synchronization tolerance")
    if any(stamp > now_ns or (max_age_s > 0 and now_ns - stamp > max_age_s * 1e9)
           for stamp in (front_time, down_time)):
        raise ValueError("observation is stale or in the future")
    if not 1 <= len(request.input_ids) <= 512:
        raise ValueError("input_ids must contain 1..512 entries")
    if any(v < 0 or v > 32064 for v in request.input_ids):
        raise ValueError("input_ids must be vocab indices in 0..32064")
    if len(request.request_id) > 128:
        raise ValueError("request_id exceeds 128 characters")
    return {
        "request_id": request.request_id,
        "front_b64": _bgr(request.front, "front"),
        "down_b64": _bgr(request.down, "down"),
        "input_ids": [int(v) for v in request.input_ids],
    }


def validate_result(result, request_id):
    if result["request_id"] != request_id:
        raise ValueError("worker returned a mismatched request_id")
    if not isinstance(result.get("text"), str):
        raise ValueError("worker text output is missing")
    gen = result.get("gen_ids")
    if not isinstance(gen, list) or any(type(v) is not int or v < 0 for v in gen):
        raise ValueError("gen_ids must be a list of nonnegative ints")
    if not result.get("model_version") or not result.get("device"):
        raise ValueError("worker identity is missing")
    timings = result.get("timings_ms", {})
    if any(not math.isfinite(float(v)) or float(v) < 0 for v in timings.values()):
        raise ValueError("worker timings must be nonnegative finite numbers")
