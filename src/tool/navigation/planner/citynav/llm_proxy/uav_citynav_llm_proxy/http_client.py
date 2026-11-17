"""纯 stdlib OpenAI 兼容 HTTP 客户端 + node 复用纯辅助（迁移方案阶段 3）。

消息结构与 upstream src/llm/query_llm.py（子模块锁定提交
4b6e926c3325995483576523b18c9d0de5eb0710）同构：
  - query_viewpoint_api L256-305：逐视角 "{viewpoint} image: " 文本块 +
    base64 image_url 块，末尾追加 prompt 文本块，单条 user 消息；
  - query_api L307-354：prompt 文本块 + 可选 image_url 块；
  - 响应文本按上游做 unicodedata.normalize("NFKC")。
偏离（README 详述）：
  - 图像 data URL 为 image/png（本包禁 cv2/requests/pydantic，用 zlib 最小 PNG
    编码）；上游为 image/jpeg。属传输编码偏离，不改 prompt 语义。
  - 重试：上游异常 sleep 10 s 无限递归（query_llm.py L346）→ 有限 attempts 次
    线性退避（D10，与 legacy_stub 同策略）。
  - 缓存：上游 LLM.get_cache 按 prompt 前缀命中、键不含图像（query_llm.py
    L29-35）→ 本包同语义，键 sha256(prompt+model)，落盘 {prompt, response}
    JSON，命中不请求；cache_dir 为空则禁用。
node.py 的纯辅助（PNG 编码、cameras 解析、取帧/新鲜度、消息构造）集中在本模块，
tests 不依赖 rclpy 即可覆盖。
"""
import base64
import hashlib
import json
import os
import struct
import time
import unicodedata
import urllib.error
import urllib.request
import zlib

DEFAULT_MODEL = "glm-4v-flash"
DEFAULT_BASE_URL = "https://open.bigmodel.cn/api/paas/v4"
DEFAULT_API_KEY_ENV = "OPENAI_API_KEY"
MAX_ATTEMPTS = 3


class LlmHttpError(Exception):
    """code 直接对齐 result.error_code 枚举：http / timeout。"""

    def __init__(self, code: str, detail: str = ""):
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


# ---------- 图像编码（纯 stdlib 最小 PNG，替代上游 cv2 JPEG） ----------

def _png_chunk(tag: bytes, payload: bytes) -> bytes:
    return (struct.pack(">I", len(payload)) + tag + payload
            + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))


def encode_png_rgb(width: int, height: int, data, step: int = None) -> bytes:
    """rgb8 行缓冲 → 8bit truecolor PNG（filter 0，无优化）。

    data 为 bytes/bytearray（msg.data），step 为 msg.step 行字节跨度；
    step 缺省 width*3。行宽不足或数据截断抛 ValueError。
    """
    if step is None:
        step = width * 3
    if width <= 0 or height <= 0:
        raise ValueError(f"bad image size {width}x{height}")
    data = bytes(data)
    raw = bytearray()
    for y in range(height):
        row = data[y * step:y * step + width * 3]
        if len(row) < width * 3:
            raise ValueError("rgb8 buffer truncated")
        raw.append(0)
        raw.extend(row)
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", ihdr)
            + _png_chunk(b"IDAT", zlib.compress(bytes(raw), 9))
            + _png_chunk(b"IEND", b""))


def b64_png(width: int, height: int, data, step: int = None) -> str:
    return base64.b64encode(encode_png_rgb(width, height, data, step)).decode("ascii")


# ---------- 上游同构消息构造 ----------

def build_viewpoint_content(prompt: str, images) -> list:
    """images: [(view_label, png_b64), ...]，顺序=上游 obs_viewpoint 顺序。

    逐字对应 query_viewpoint_api 的 content_block 结构（jpeg→png 为编码偏离）。
    """
    content = []
    for label, b64 in images:
        content.append({"type": "text", "text": f"{label} image: "})
        content.append({"type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{b64}"}})
    content.append({"type": "text", "text": f"{prompt}"})
    return content


def build_caption_content(prompt: str, image_b64: str) -> list:
    """逐字对应 query_api（带 image_path）的 content 结构（jpeg→png 为编码偏离）。"""
    return [
        {"type": "text", "text": prompt},
        {"type": "image_url",
         "image_url": {"url": f"data:image/png;base64,{image_b64}"}},
    ]


# ---------- 磁盘缓存（上游 prompt 键语义，键不含图像） ----------

def cache_key(prompt: str, model: str) -> str:
    return hashlib.sha256((prompt + model).encode("utf-8")).hexdigest()


def cache_path(cache_dir: str, prompt: str, model: str) -> str:
    return os.path.join(cache_dir, cache_key(prompt, model) + ".json")


def cache_read(cache_dir: str, prompt: str, model: str):
    path = cache_path(cache_dir, prompt, model)
    if not os.path.isfile(path):
        return None
    try:
        with open(path, encoding="utf-8") as f:
            obj = json.load(f)
    except (OSError, ValueError):
        return None
    resp = obj.get("response")
    return resp if isinstance(resp, str) else None


def cache_write(cache_dir: str, prompt: str, model: str, response: str) -> None:
    os.makedirs(cache_dir, exist_ok=True)
    with open(cache_path(cache_dir, prompt, model), "w", encoding="utf-8") as f:
        json.dump({"prompt": prompt, "response": response}, f, ensure_ascii=False)


# ---------- HTTP ----------

def _post(url: str, payload: dict, api_key: str, timeout_s: float) -> str:
    req = urllib.request.Request(url, data=json.dumps(payload).encode("utf-8"),
                                 method="POST")
    req.add_header("Content-Type", "application/json")
    req.add_header("Authorization", f"Bearer {api_key}")
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            body = resp.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as exc:
        detail = ""
        try:
            detail = exc.read().decode("utf-8", errors="replace")[:400]
        except Exception:
            pass
        raise LlmHttpError("http", f"{exc.code} {detail}")
    except urllib.error.URLError as exc:
        reason = getattr(exc, "reason", None)
        if isinstance(reason, TimeoutError) or "timed out" in str(reason).lower():
            raise LlmHttpError("timeout", str(reason))
        raise LlmHttpError("http", str(reason))
    except TimeoutError as exc:
        raise LlmHttpError("timeout", str(exc))
    try:
        obj = json.loads(body)
        content = obj["choices"][0]["message"]["content"]
    except (ValueError, KeyError, IndexError, TypeError) as exc:
        raise LlmHttpError("http", f"bad_payload: {exc}")
    # 上游对响应做 NFKC 归一（query_llm.py L287/L335）
    return unicodedata.normalize("NFKC", content or "")


def chat(messages: list, *, model: str = DEFAULT_MODEL,
         base_url: str = DEFAULT_BASE_URL, api_key: str = "",
         timeout_s: float = 30.0, cache_prompt: str = None,
         cache_dir: str = "", attempts: int = MAX_ATTEMPTS,
         backoff_s: float = 2.0, sleep=time.sleep) -> str:
    """一次 OpenAI 兼容 /chat/completions 调用；失败抛 LlmHttpError。

    cache_dir 非空且 cache_prompt 给定：sha256(prompt+model) 命中直接返回、
    不请求；成功响应落盘 {prompt, response}。重试有限次线性退避（D10）。
    """
    use_cache = bool(cache_dir) and cache_prompt is not None
    if use_cache:
        hit = cache_read(cache_dir, cache_prompt, model)
        if hit is not None:
            return hit
    url = base_url.rstrip("/") + "/chat/completions"
    payload = {"model": model, "messages": messages}
    last = None
    attempts = max(1, int(attempts))
    for i in range(attempts):
        try:
            text = _post(url, payload, api_key, timeout_s)
            if use_cache:
                cache_write(cache_dir, cache_prompt, model, text)
            return text
        except LlmHttpError as exc:
            last = exc
            if i + 1 < attempts:
                sleep(backoff_s * (i + 1))
    raise last


# ---------- node 复用的纯辅助 ----------

def parse_cameras(entries):
    """["left=CityLeft", ...] → 有序 {view_label: camera_name}。

    条目顺序即上游 obs_viewpoint 的视角顺序（left, slightly_left, front,
    slightly_right, right）。格式错误抛 ValueError。
    """
    out = {}
    for e in entries:
        label, sep, cam = str(e).partition("=")
        if not sep or not label.strip() or not cam.strip():
            raise ValueError(f"bad cameras entry: {e!r} (want label=camera)")
        out[label.strip()] = cam.strip()
    return out


def select_frame(frames, view_stamps, order, now_ns, max_age_ns):
    """按 label 取缓存帧：仅缓存最新帧，stamp 精确匹配即该帧，否则退化为最新；
    帧龄超 max_age_ns 判 stale。返回 ([(label, png_b64)], None) 或
    (None, stale_label)。order 决定消息内视角顺序。
    """
    images = []
    for label in order:
        if label not in view_stamps:
            continue
        frame = frames.get(label)
        if frame is None:
            return None, label
        stamp_ns, b64 = frame
        if now_ns - stamp_ns > max_age_ns:
            return None, label
        images.append((label, b64))
    return images, None
