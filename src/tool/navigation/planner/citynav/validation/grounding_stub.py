#!/usr/bin/env python3
"""stuck1 注入用 grounding 桩（宿主运行）。

对任意 POST 返回 {"seg_success": false, "mask_b64": "", "phrases": []}
（服务契约字段名锁定；seg_success=false 是合法响应而非错误，planner 侧
greedy_mask_predict → (None, False)，不进入连续失败计数）。效果：每个探索
循环的 5 视角分割全失败 → waypoint None → timed None → 零位移零旋转 →
位移进展位 0；配合 5-landmark 指令（不可能在 4 循环内全部推进）确定性
触发升级后的卡死判据 → FAULT(stuck)。

GET /health/ready → 200（与真实服务同形）。用法：
  python grounding_stub.py --port 18100
"""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status":"stub"}')

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", 0))
        self.rfile.read(length)
        body = json.dumps({"seg_success": False, "mask_b64": "",
                           "phrases": []}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=18100)
    args = ap.parse_args()
    print(f"grounding stub (always seg_success=false) on :{args.port}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()
