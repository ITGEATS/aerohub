"""http_client 纯函数单测（无网络、无 rclpy）：消息同构、PNG、缓存、重试、取帧。"""
import ast
import base64
import io
import json
import struct
import urllib.error
import zlib
from pathlib import Path

import pytest

from uav_citynav_llm_proxy import http_client as hc

UPSTREAM_Q = (Path(__file__).resolve().parents[2] / "upstream"
              / "src" / "llm" / "query_llm.py").read_text(encoding="utf-8")


# ---------- 上游同构消息 ----------

def test_viewpoint_content_matches_upstream_shape():
    """与 query_llm.py query_viewpoint_api 的 content_block 结构逐块比对
    （仅 data URL 的 jpeg→png 为已声明编码偏离）。"""
    content = hc.build_viewpoint_content(
        "PROMPT", [("left", "TEA="), ("front", "RlI=")])
    assert content == [
        {"type": "text", "text": "left image: "},
        {"type": "image_url", "image_url": {"url": "data:image/png;base64,TEA="}},
        {"type": "text", "text": "front image: "},
        {"type": "image_url", "image_url": {"url": "data:image/png;base64,RlI="}},
        {"type": "text", "text": "PROMPT"},
    ]
    # 上游 "{viewpoint} image: " 文本块含尾随空格，逐字保留
    tree = ast.parse(UPSTREAM_Q)
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        for fn in [n for n in cls.body if isinstance(n, ast.FunctionDef)
                   and n.name == "query_viewpoint_api"]:
            src = ast.get_source_segment(UPSTREAM_Q, fn)
            assert '"text": f"{viewpoint} image: "' in src


def test_caption_content_matches_upstream_shape():
    content = hc.build_caption_content("PROMPT", "TEA=")
    assert content[0] == {"type": "text", "text": "PROMPT"}
    assert content[1]["image_url"]["url"] == "data:image/png;base64,TEA="


# ---------- PNG 编码 ----------

def test_png_rgb_roundtrip():
    data = bytes(range(12))  # 2x2 rgb8
    png = hc.encode_png_rgb(2, 2, data, step=6)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    w, h, depth, ctype = struct.unpack(">IIBB", png[16:26])
    assert (w, h, depth, ctype) == (2, 2, 8, 2)
    # IHDR 数据 16..29，CRC 至 33；IDAT 长度 33..37，数据自 41（跳过 "IDAT" tag）
    idat_len = struct.unpack(">I", png[33:37])[0]
    raw = zlib.decompress(png[41:41 + idat_len])
    assert raw == b"\x00" + data[:6] + b"\x00" + data[6:]


def test_png_respects_row_step_padding():
    row0 = bytes(range(6))
    row1 = bytes(range(100, 106))
    data = row0 + b"\x00\x00" + row1 + b"\x00\x00"  # step=8
    png = hc.encode_png_rgb(2, 2, data, step=8)
    idat_len = struct.unpack(">I", png[33:37])[0]
    raw = zlib.decompress(png[41:41 + idat_len])
    assert raw == b"\x00" + row0 + b"\x00" + row1


def test_png_truncated_raises():
    with pytest.raises(ValueError):
        hc.encode_png_rgb(2, 2, b"\x00" * 10, step=6)


def test_b64_png_decodes():
    b64 = hc.b64_png(1, 1, bytes(range(3)))
    assert base64.b64decode(b64)[:8] == b"\x89PNG\r\n\x1a\n"


# ---------- cameras 解析 ----------

def test_parse_cameras_default_order():
    views = hc.parse_cameras([
        "left=CityLeft", "slightly_left=CitySlightlyLeft", "front=CityFront",
        "slightly_right=CitySlightlyRight", "right=CityRight"])
    assert list(views) == ["left", "slightly_left", "front",
                           "slightly_right", "right"]
    assert views["front"] == "CityFront"


@pytest.mark.parametrize("bad", ["noequals", "=Cam", "label=", "  =  "])
def test_parse_cameras_rejects_bad_entries(bad):
    with pytest.raises(ValueError):
        hc.parse_cameras([bad])


# ---------- 取帧/新鲜度 ----------

def test_select_frame_exact_and_latest():
    frames = {"front": (1000, "F"), "left": (900, "L")}
    images, stale = hc.select_frame(
        frames, {"front": 1000, "left": 850}, ["left", "front"],
        now_ns=1100, max_age_ns=500)
    assert stale is None
    assert images == [("left", "L"), ("front", "F")]  # 顺序=order，非 view_stamps


def test_select_frame_stale_and_missing():
    frames = {"front": (0, "F")}
    assert hc.select_frame(frames, {"front": 0}, ["front"],
                           now_ns=10**9, max_age_ns=10**8) == (None, "front")
    assert hc.select_frame({}, {"front": 0}, ["front"],
                           now_ns=0, max_age_ns=10**9) == (None, "front")


def test_select_frame_skips_unrequested_labels():
    frames = {"front": (1, "F"), "right": (1, "R")}
    images, stale = hc.select_frame(frames, {"front": 1}, ["left", "front"],
                                    now_ns=1, max_age_ns=10**9)
    assert stale is None and images == [("front", "F")]


# ---------- 缓存 ----------

def test_cache_hit_skips_request(tmp_path, monkeypatch):
    calls = []

    def fake_post(url, payload, api_key, timeout_s):
        calls.append(payload)
        return "RESP"

    monkeypatch.setattr(hc, "_post", fake_post)
    kw = dict(model="glm-4v-flash", base_url="https://x/api/v4", api_key="k",
              timeout_s=5.0, cache_prompt="PROMPT", cache_dir=str(tmp_path),
              sleep=lambda s: None)
    assert hc.chat([{"role": "user", "content": "c"}], **kw) == "RESP"
    assert hc.chat([{"role": "user", "content": "c"}], **kw) == "RESP"
    assert len(calls) == 1  # 命中不请求
    key = hc.cache_key("PROMPT", "glm-4v-flash")
    f = tmp_path / f"{key}.json"
    assert f.is_file()
    assert json.loads(f.read_text(encoding="utf-8")) == {
        "prompt": "PROMPT", "response": "RESP"}
    import hashlib
    assert key == hashlib.sha256("PROMPTglm-4v-flash".encode("utf-8")).hexdigest()


def test_cache_disabled_without_dir(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(hc, "_post",
                        lambda *a, **k: calls.append(1) or "R")
    for _ in range(2):
        hc.chat([], cache_prompt="P", cache_dir="", sleep=lambda s: None)
    assert len(calls) == 2


# ---------- 重试 ----------

def test_finite_retry_then_raise(monkeypatch):
    calls = []

    def always_fail(*a, **k):
        calls.append(1)
        raise hc.LlmHttpError("http", "500 boom")

    monkeypatch.setattr(hc, "_post", always_fail)
    slept = []
    with pytest.raises(hc.LlmHttpError) as e:
        hc.chat([], api_key="k", cache_dir="",
                sleep=lambda s: slept.append(s))
    assert len(calls) == hc.MAX_ATTEMPTS == 3
    assert slept == [2.0, 4.0]  # 线性退避，最后一次不 sleep
    assert e.value.code == "http"


# ---------- _post 错误映射与 NFKC ----------

class _Resp:
    def __init__(self, body):
        self._b = body

    def read(self):
        return self._b

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def _ok_payload(content):
    return json.dumps({"choices": [{"message": {"content": content}}]}).encode()


def test_post_http_error(monkeypatch):
    def raise_http(*a, **k):
        raise urllib.error.HTTPError("u", 429, "Too Many", {},
                                     io.BytesIO(b"rate limited"))
    monkeypatch.setattr(hc.urllib.request, "urlopen", raise_http)
    with pytest.raises(hc.LlmHttpError) as e:
        hc._post("https://x/chat/completions", {}, "", 1.0)
    assert e.value.code == "http" and "429" in e.value.detail


def test_post_timeout_vs_transport(monkeypatch):
    monkeypatch.setattr(hc.urllib.request, "urlopen",
                        lambda *a, **k: (_ for _ in ()).throw(
                            urllib.error.URLError(reason=TimeoutError())))
    with pytest.raises(hc.LlmHttpError) as e:
        hc._post("https://x/chat/completions", {}, "", 1.0)
    assert e.value.code == "timeout"

    monkeypatch.setattr(hc.urllib.request, "urlopen",
                        lambda *a, **k: (_ for _ in ()).throw(
                            urllib.error.URLError(reason="name not resolved")))
    with pytest.raises(hc.LlmHttpError) as e:
        hc._post("https://x/chat/completions", {}, "", 1.0)
    assert e.value.code == "http"


def test_post_bad_json_and_schema(monkeypatch):
    monkeypatch.setattr(hc.urllib.request, "urlopen",
                        lambda *a, **k: _Resp(b"<html>502</html>"))
    with pytest.raises(hc.LlmHttpError) as e:
        hc._post("https://x/chat/completions", {}, "", 1.0)
    assert e.value.code == "http"

    monkeypatch.setattr(hc.urllib.request, "urlopen",
                        lambda *a, **k: _Resp(b'{"error": "no choices"}'))
    with pytest.raises(hc.LlmHttpError):
        hc._post("https://x/chat/completions", {}, "", 1.0)


def test_post_nfkc_normalizes_like_upstream(monkeypatch):
    monkeypatch.setattr(hc.urllib.request, "urlopen",
                        lambda *a, **k: _Resp(_ok_payload("\uff21\uff22 json")))
    assert hc._post("https://x/chat/completions", {}, "", 1.0) == "AB json"
