"""grounded_sam Inference 服务客户端（纯 stdlib：urllib + zlib PNG 编解码）。

服务契约（src/basic_function/inference/models/grounded_sam/README.md，字段名
锁定不可改）：POST /models/grounded_sam/greedy_mask_predict，请求
{"image_b64": BGR PNG/JPEG base64, "text_prompt": str, "box_threshold": 0.3,
"text_threshold": 0.25} → {"mask_b64": 单通道 PNG(0/255) base64,
"seg_success": bool, "phrases": [str]}。seg_success=false 时 mask_b64 为空串。

容器禁 cv2/PIL：PNG 编码用纯 Python zlib writer（非交织、filter-0 扫描线），
mask 解码支持 8-bit 灰度非交织 PNG（filter 0-4 反滤波）。ROS rgb8 输入按服务
BGR 语义交换通道（与 legacy 客户端 cv2 行为一致）。

greedy_mask_predict 返回形态对齐上游 vlm 对象：(mask uint8 HxW | None,
seg_success bool)；HTTP/解码层错误抛 ClientError（node 计连续失败→FAULT），
seg_success=false 不是错误（上游"小数组+False"语义）。
"""
import base64
import json
import struct
import urllib.error
import urllib.request
import zlib

import numpy as np

PNG_SIG = b"\x89PNG\r\n\x1a\n"


class ClientError(RuntimeError):
    pass


def _chunk(tag: bytes, data: bytes) -> bytes:
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def encode_png(arr: np.ndarray) -> bytes:
    """uint8 HxW（灰度，color type 0）或 HxWx3（color type 2）→ PNG bytes。"""
    arr = np.asarray(arr)
    if arr.dtype != np.uint8:
        raise ValueError("encode_png expects uint8")
    if arr.ndim == 2:
        h, w = arr.shape
        color_type, ch = 0, 1
        rows = arr
    elif arr.ndim == 3 and arr.shape[2] == 3:
        h, w = arr.shape[:2]
        color_type, ch = 2, 3
        rows = arr
    else:
        raise ValueError("encode_png expects HxW or HxWx3 uint8")
    raw = np.concatenate((np.zeros((h, 1), np.uint8),
                          rows.reshape(h, w * ch)), axis=1).tobytes()
    ihdr = struct.pack(">IIBBBBB", w, h, 8, color_type, 0, 0, 0)
    return (PNG_SIG + _chunk(b"IHDR", ihdr)
            + _chunk(b"IDAT", zlib.compress(raw, 9)) + _chunk(b"IEND", b""))


def decode_png_gray(data: bytes) -> np.ndarray:
    """8-bit 灰度非交织 PNG → uint8 HxW（支持 filter 0-4）。"""
    if data[:8] != PNG_SIG:
        raise ClientError("not a PNG")
    pos, idat, ihdr = 8, b"", None
    while pos + 8 <= len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        tag = data[pos + 4:pos + 8]
        payload = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", payload)
        elif tag == b"IDAT":
            idat += payload
        elif tag == b"IEND":
            break
    if ihdr is None:
        raise ClientError("PNG missing IHDR")
    w, h, depth, ctype, interlace = ihdr[0], ihdr[1], ihdr[2], ihdr[3], ihdr[6]
    if depth != 8 or ctype != 0 or interlace != 0:
        raise ClientError(f"unsupported PNG (depth={depth} color={ctype} "
                          f"interlace={interlace}); service contract is 8-bit gray")
    raw = zlib.decompress(idat)
    stride = w
    out = np.zeros((h, w), np.int32)
    prev = np.zeros(stride, np.int32)
    for y in range(h):
        f = raw[y * (stride + 1)]
        line = np.frombuffer(raw[y * (stride + 1) + 1:(y + 1) * (stride + 1)],
                             dtype=np.uint8).astype(np.int32)
        if f == 0:
            cur = line
        elif f == 1:  # Sub
            cur = line.copy()
            for x in range(1, stride):
                cur[x] = (cur[x] + cur[x - 1]) & 0xFF
        elif f == 2:  # Up
            cur = (line + prev) & 0xFF
        elif f == 3:  # Average
            cur = line.copy()
            for x in range(stride):
                a = cur[x - 1] if x > 0 else 0
                cur[x] = (cur[x] + ((a + prev[x]) >> 1)) & 0xFF
        elif f == 4:  # Paeth
            cur = line.copy()
            for x in range(stride):
                a = cur[x - 1] if x > 0 else 0
                b = prev[x]
                c = prev[x - 1] if x > 0 else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                cur[x] = (cur[x] + pr) & 0xFF
        else:
            raise ClientError(f"bad PNG filter {f}")
        out[y] = cur
        prev = cur
    return out.astype(np.uint8)


def rgb_to_bgr_png_b64(rgb: np.ndarray) -> str:
    """ROS rgb8 (HxWx3) → 服务 BGR PNG base64（legacy cv2 语义等价）。"""
    bgr = np.asarray(rgb)[:, :, ::-1]
    return base64.b64encode(encode_png(np.ascontiguousarray(bgr))).decode("ascii")


class GroundingClient:
    def __init__(self, url: str, timeout_s: float = 30.0,
                 box_threshold: float = 0.3, text_threshold: float = 0.25):
        self.url = url
        self.timeout_s = timeout_s
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold

    def greedy_mask_predict(self, rgb: np.ndarray, text_prompt: str):
        payload = json.dumps({
            "image_b64": rgb_to_bgr_png_b64(rgb),
            "text_prompt": text_prompt,
            "box_threshold": self.box_threshold,
            "text_threshold": self.text_threshold,
        }).encode("utf-8")
        req = urllib.request.Request(
            self.url, data=payload,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                out = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise ClientError(f"grounding service failure: {exc}") from exc
        seg_success = bool(out.get("seg_success", False))
        if seg_success and out.get("mask_b64"):
            mask = decode_png_gray(base64.b64decode(out["mask_b64"]))
            return mask, True
        return None, False
