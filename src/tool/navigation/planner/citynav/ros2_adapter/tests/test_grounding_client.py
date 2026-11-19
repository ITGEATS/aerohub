"""PNG codecs and planner behavior at the Inference ROS client interface."""
import base64
import struct
import sys
from concurrent.futures import Future
from types import SimpleNamespace
import zlib

import numpy as np
import pytest

import uav_citynav_planner.grounding_client as gc
import uav_citynav_planner.clip_client as cc
from uav_citynav_planner.grounding_client import (ClientError, decode_png_gray,
                                                  encode_png)


@pytest.fixture
def inference_node(monkeypatch):
    service = SimpleNamespace(Request=SimpleNamespace)
    monkeypatch.setitem(sys.modules, 'uav_inference_interfaces.srv',
                        SimpleNamespace(InferGroundedSam=service, InferClipScores=service))
    calls = []
    replies = []

    def create_client(_, name):
        def call_async(request):
            calls.append((name, request))
            future = Future()
            future.set_result(replies.pop(0))
            return future

        return SimpleNamespace(service_is_ready=lambda: True, call_async=call_async)

    return SimpleNamespace(create_client=create_client), calls, replies


def test_png_gray_roundtrip():
    arr = np.random.RandomState(7).randint(0, 256, (13, 9)).astype(np.uint8)
    out = decode_png_gray(encode_png(arr))
    assert np.array_equal(out, arr)


def test_png_rgb_roundtrip():
    arr = np.random.RandomState(3).randint(0, 256, (5, 6, 3)).astype(np.uint8)
    data = encode_png(arr)
    assert data[:8] == gc.PNG_SIG
    # 自写 color type 2 → 手工解析验证（decode 仅支持灰度，服务 mask 即灰度）
    pos = 8
    w, h, depth, ctype = struct.unpack(">IIBB", data[pos + 8:pos + 18])
    assert (w, h, depth, ctype) == (6, 5, 8, 2)
    idat = b""
    pos += 12 + 13  # 跳过 IHDR chunk（payload 13 字节）
    while data[pos + 4:pos + 8] != b"IEND":
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        if data[pos + 4:pos + 8] == b"IDAT":
            idat += data[pos + 8:pos + 8 + length]
        pos += 12 + length
    raw = zlib.decompress(idat)
    rows = np.frombuffer(raw, np.uint8).reshape(h, 1 + w * 3)
    assert np.array_equal(rows[:, 1:].reshape(h, w, 3), arr)


def test_decode_png_sub_filter():
    # 手工构造 filter-1（Sub）灰度 PNG，验证反滤波正确（服务用标准编码器，
    # 可能输出非零 filter）
    w, h = 4, 2
    original = np.array([[10, 13, 15, 16], [20, 24, 27, 29]], np.uint8)
    raw = bytearray()
    for y in range(h):
        raw.append(1)  # Sub
        for x in range(w):
            left = int(original[y, x - 1]) if x else 0
            raw.append((int(original[y, x]) - left) & 0xFF)
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)
    png = (gc.PNG_SIG + gc._chunk(b"IHDR", ihdr)
           + gc._chunk(b"IDAT", zlib.compress(bytes(raw), 9))
           + gc._chunk(b"IEND", b""))
    assert np.array_equal(decode_png_gray(png), original)


def test_grounding_client_success_and_fail(inference_node):
    node, calls, replies = inference_node
    mask = np.zeros((8, 8), np.uint8)
    mask[2:5, 3:6] = 255
    replies.extend([SimpleNamespace(success=True, seg_success=True,
                                    mask_b64=base64.b64encode(encode_png(mask)).decode()),
                    SimpleNamespace(success=True, seg_success=False, mask_b64='')])
    client = gc.GroundingClient(node)
    rgb = np.zeros((8, 8, 3), np.uint8)
    rgb[:, :, 2] = 200  # blue-ish in RGB
    m, ok = client.greedy_mask_predict(rgb, "building")
    assert ok and np.array_equal(m, mask)
    name, request = calls[0]
    assert name == 'inference/grounded_sam/infer'
    assert request.text_prompt == 'building'
    assert request.box_threshold == 0.3 and request.text_threshold == 0.25
    assert request.image_b64
    m, ok = client.greedy_mask_predict(rgb, "helicopter")
    assert not ok and m is None


def test_grounding_client_worker_error_raises(inference_node):
    node, _, replies = inference_node
    replies.append(SimpleNamespace(success=False, error_code='NOT_READY', message='offline'))
    client = gc.GroundingClient(node)
    with pytest.raises(ClientError, match='NOT_READY'):
        client.greedy_mask_predict(np.zeros((2, 2, 3), np.uint8), "t")


def test_clip_client_chunks_16(inference_node):
    node, calls, replies = inference_node
    replies.extend(SimpleNamespace(success=True, similarity=[0.1, 0.2] * n) for n in (16, 4))
    client = cc.ClipScoreClient(node)
    images = [f"img{i}" for i in range(20)]
    sim = client.score(images, ["a photo of x", "a photo of y"])
    assert [name for name, _ in calls] == ['inference/clip/score'] * 2
    assert [r.images_b64 for _, r in calls] == [images[:16], images[16:]]
    assert sim == [[0.1, 0.2]] * 20
