"""Worker HTTP contract + determinism smoke for the board AeroVLA service.

Run on the board (or any host that can reach the worker):
    python3 http_smoke.py [--base http://127.0.0.1:8090] [--prompt-ids f.json]

Checks: /health/ready exposes model_version/device; a fixed synthetic 256x256
front/down pair + real prompt ids returns 200 with text/gen_ids; the same
request twice returns identical gen_ids (greedy determinism); malformed input
is rejected with 422. Module-level numeric parity against the bf16 export
golden is covered by aerovla_infer/smoke.py (cosine >= 0.98); behavior parity
against the stage-1 legacy baseline is verified at the planner level.
"""

import argparse
import base64
import json
import urllib.error
import urllib.request

import cv2
import numpy as np


def post(base, path, body):
    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=90) as resp:
        return resp.status, json.loads(resp.read())


def get(base, path):
    with urllib.request.urlopen(base + path, timeout=30) as resp:
        return resp.status, json.loads(resp.read())


def png_b64(color):
    ok, buf = cv2.imencode(".png", np.full((256, 256, 3), color, np.uint8))
    assert ok
    return base64.b64encode(buf.tobytes()).decode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8090")
    ap.add_argument("--prompt-ids", default="")
    args = ap.parse_args()

    if args.prompt_ids:
        with open(args.prompt_ids, encoding="utf-8") as f:
            ids = json.load(f)["straight ahead "]
    else:
        ids = [1, 529, 3027, 29958, 13, 29943, 368, 7812, 3078, 322]

    status, meta = get(args.base, "/health/ready")
    assert status == 200 and meta.get("model_version"), meta
    print("ready:", meta.get("model_version", "")[:40], "device", meta.get("device"))

    body = {"request_id": "smoke-1", "front_b64": png_b64(120), "down_b64": png_b64(60),
            "input_ids": ids}
    status, r1 = post(args.base, "/infer", body)
    assert status == 200 and isinstance(r1["text"], str) and "request_id" in r1, r1
    print("infer1: gen_ids=%s text=%r" % (r1["gen_ids"], r1["text"][:60]))
    body["request_id"] = "smoke-2"
    _, r2 = post(args.base, "/infer", body)
    assert r2["gen_ids"] == r1["gen_ids"], "greedy determinism broken"
    print("infer2 identical (deterministic)")

    bad = dict(body, request_id="smoke-bad", input_ids=[-5])
    try:
        post(args.base, "/infer", bad)
        raise AssertionError("negative id must be rejected")
    except urllib.error.HTTPError as err:
        assert err.code == 422, err.code
        print("malformed input rejected 422")
    print("SMOKE_PASS")


if __name__ == "__main__":
    main()
