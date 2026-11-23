"""Host-side unit tests for the AeroVLA model layer and ROS contract helpers.

These modules are deliberately rclpy-free; the node itself is validated on the
board (see houmo/README.md). Run from the repository root:
    python -m pytest src/basic_function/inference/models/aerovla/houmo/tests/test_model_layer.py
"""

import base64
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
MODEL = HERE.parent.parent
sys.path.insert(0, str(MODEL))
sys.path.insert(0, str(MODEL.parent.parent / "ros2_adapter"))

import processing  # noqa: E402
from uav_inference import aerovla_contract  # noqa: E402


def _png(img):
    ok, buf = cv2.imencode(".png", img)
    assert ok
    return buf.tobytes()


def _image(img, encoding, stamp_ns):
    h, w = img.shape[:2]
    step = w * 3
    # img is stored in the byte order its encoding names: rgb8 -> RGB, bgr8 -> BGR.
    data = np.ascontiguousarray(img).tobytes()
    header = types.SimpleNamespace(frame_id="camera", stamp=types.SimpleNamespace(
        sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9))
    return types.SimpleNamespace(header=header, encoding=encoding, width=w, height=h,
                                 step=step, data=data, is_bigendian=False)


def _request(front, down, ids=None, skew_ns=0, age_ns=0):
    stamp = int(time.time() * 1e9)
    now = stamp + age_ns
    return types.SimpleNamespace(
        request_id="t-1",
        front=_image(front, "bgr8", stamp),
        down=_image(down, "bgr8", stamp + skew_ns),
        input_ids=ids if ids is not None else list(range(1, 10)),
    ), now


def make_request_payload():
    front = np.full((256, 256, 3), 120, np.uint8)
    down = np.full((256, 256, 3), 60, np.uint8)
    return front, down


# ---------- processing ----------

def test_prep_mosaic_contract():
    front, down = make_request_payload()
    pv = processing.prep_mosaic(front, down)
    assert pv.shape == (1, 6, 224, 224) and pv.dtype == np.float32
    assert np.isfinite(pv).all()


def test_prep_mosaic_matches_orchestrator_port():
    """Reference: verbatim orchestrator_board.prep_mosaic formula."""
    rng = np.random.default_rng(7)
    front = rng.integers(0, 256, (256, 256, 3), dtype=np.uint8)
    down = rng.integers(0, 256, (256, 256, 3), dtype=np.uint8)
    fr = cv2.resize(front, (224, 224), interpolation=cv2.INTER_CUBIC)
    dn = cv2.resize(down, (224, 224), interpolation=cv2.INTER_CUBIC)
    mos = cv2.resize(np.vstack([fr, dn]), (224, 224), interpolation=cv2.INTER_CUBIC)
    x = mos.astype(np.float32) / 255.0
    ref = np.concatenate([(x - processing.MEAN1) / processing.STD1,
                          (x - 0.5) / 0.5], axis=-1).transpose(2, 0, 1)[None]
    assert np.array_equal(ref, processing.prep_mosaic(front, down))


def test_decode_image_roundtrip_and_errors():
    img = np.arange(256 * 256 * 3, dtype=np.uint8).reshape(256, 256, 3)
    b64 = base64.b64encode(_png(img)).decode()
    back = processing.decode_image(b64)
    assert np.array_equal(img, back)
    with pytest.raises(ValueError):
        processing.decode_image("!!!not base64!!!")
    # A grayscale PNG is accepted and converted to 3 channels by IMREAD_COLOR.
    gray = cv2.imencode(".png", np.zeros((8, 8), np.uint8))[1].tobytes()
    assert processing.decode_image(base64.b64encode(gray).decode()).shape == (8, 8, 3)
    with pytest.raises(ValueError):
        processing.decode_image(base64.b64encode(b"not an image").decode())


def test_decode_text_unmapped_ids_vanish():
    assert processing.decode_text([1, 2, 3], {1: "a", 3: "c"}) == "ac"


# ---------- contract ----------

def test_payload_happy_path():
    front, down = make_request_payload()
    request, now = _request(front, down)
    payload = aerovla_contract.request_payload(request, now, 1.0, 0.1)
    assert payload["request_id"] == "t-1"
    assert payload["input_ids"] == list(range(1, 10))
    for key in ("front_b64", "down_b64"):
        raw = base64.b64decode(payload[key])
        assert cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR).shape == (256, 256, 3)


def test_payload_rejects_stale_skew_encoding_and_ids():
    front, down = make_request_payload()
    request, now = _request(front, down, age_ns=2 * 10**9)
    with pytest.raises(ValueError, match="stale"):
        aerovla_contract.request_payload(request, now, 1.0, 0.1)
    request, now = _request(front, down, skew_ns=200_000_000)
    with pytest.raises(ValueError, match="synchronization"):
        aerovla_contract.request_payload(request, now, 1.0, 0.1)
    request, now = _request(front, down)
    request.front.encoding = "mono8"
    with pytest.raises(ValueError, match="encoding"):
        aerovla_contract.request_payload(request, now, 1.0, 0.1)
    request, now = _request(front, down, ids=[])
    with pytest.raises(ValueError, match="1..512"):
        aerovla_contract.request_payload(request, now, 1.0, 0.1)
    request, now = _request(front, down, ids=[-1])
    with pytest.raises(ValueError, match="vocab"):
        aerovla_contract.request_payload(request, now, 1.0, 0.1)


def test_rgb8_converts_to_bgr_png():
    rgb = np.zeros((64, 64, 3), np.uint8)
    rgb[:, :, 0] = 255  # pure red in RGB
    stamp = int(time.time() * 1e9)
    request = types.SimpleNamespace(
        request_id="t-1",
        front=_image(rgb, "rgb8", stamp),
        down=_image(rgb, "bgr8", stamp),
        input_ids=[1, 2, 3])
    payload = aerovla_contract.request_payload(request, stamp, 1.0, 0.1)
    decoded = cv2.imdecode(np.frombuffer(base64.b64decode(payload["front_b64"]), np.uint8),
                           cv2.IMREAD_COLOR)
    assert decoded[0, 0, 2] == 255 and decoded[0, 0, 0] == 0  # red channel last in BGR


def test_validate_result():
    good = {"request_id": "t-1", "text": "94 44 49", "gen_ids": [1, 2],
            "model_version": "hmm=x", "device": "houmo:xh2:0",
            "timings_ms": {"total": 12.0}}
    aerovla_contract.validate_result(good, "t-1")
    with pytest.raises(ValueError, match="request_id"):
        aerovla_contract.validate_result(dict(good, request_id="other"), "t-1")
    with pytest.raises(ValueError, match="gen_ids"):
        aerovla_contract.validate_result(dict(good, gen_ids=["x"]), "t-1")
    with pytest.raises(ValueError, match="identity"):
        aerovla_contract.validate_result(dict(good, model_version=""), "t-1")
