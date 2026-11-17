"""PNG 纯 Python 编解码与 grounding/clip HTTP 客户端单测（无网络：
monkeypatch urlopen）。"""
import base64
import json
import struct
import zlib

import numpy as np
import pytest

import uav_citynav_planner.grounding_client as gc
import uav_citynav_planner.clip_client as cc
from uav_citynav_planner.grounding_client import (ClientError, decode_png_gray,
                                                  encode_png)


def _fake_response(payload: dict):
    class Resp:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def read(self):
            return json.dumps(payload).encode("utf-8")
    return Resp()


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


def test_grounding_client_success_and_fail(monkeypatch):
    mask = np.zeros((8, 8), np.uint8)
    mask[2:5, 3:6] = 255
    calls = {}

    def fake_urlopen(req, timeout=None):
        calls["body"] = json.loads(req.data.decode("utf-8"))
        calls["url"] = req.full_url
        return _fake_response({"mask_b64": base64.b64encode(
            encode_png(mask)).decode("ascii"),
            "seg_success": True, "phrases": ["building"]})
    monkeypatch.setattr(gc.urllib.request, "urlopen", fake_urlopen)
    client = gc.GroundingClient("http://x/models/grounded_sam/greedy_mask_predict")
    rgb = np.zeros((8, 8, 3), np.uint8)
    rgb[:, :, 2] = 200  # blue-ish in RGB
    m, ok = client.greedy_mask_predict(rgb, "building")
    assert ok and np.array_equal(m, mask)
    assert calls["url"].endswith("/greedy_mask_predict")
    body = calls["body"]
    assert body["text_prompt"] == "building"
    assert body["box_threshold"] == 0.3 and body["text_threshold"] == 0.25
    assert body["image_b64"]  # RGB→BGR 交换后的 PNG base64

    def fake_fail(req, timeout=None):
        return _fake_response({"mask_b64": "", "seg_success": False})
    monkeypatch.setattr(gc.urllib.request, "urlopen", fake_fail)
    m, ok = client.greedy_mask_predict(rgb, "helicopter")
    assert not ok and m is None


def test_grounding_client_http_error_raises(monkeypatch):
    def boom(req, timeout=None):
        raise OSError("connection refused")
    monkeypatch.setattr(gc.urllib.request, "urlopen", boom)
    client = gc.GroundingClient("http://x")
    with pytest.raises(ClientError):
        client.greedy_mask_predict(np.zeros((2, 2, 3), np.uint8), "t")


def test_clip_client_chunks_16(monkeypatch):
    requests = []

    def fake_urlopen(req, timeout=None):
        body = json.loads(req.data.decode("utf-8"))
        requests.append(body)
        n = len(body["images_b64"])
        return _fake_response({"similarity": [[0.1] * len(body["texts"])
                                              for _ in range(n)],
                               "model_version": "clip-vit-large-patch14"})
    monkeypatch.setattr(cc.urllib.request, "urlopen", fake_urlopen)
    client = cc.ClipScoreClient("http://y/models/clip/score")
    images = [f"img{i}" for i in range(20)]
    sim = client.score(images, ["a photo of x", "a photo of y"])
    assert [len(r["images_b64"]) for r in requests] == [16, 4]
    assert len(sim) == 20 and len(sim[0]) == 2
