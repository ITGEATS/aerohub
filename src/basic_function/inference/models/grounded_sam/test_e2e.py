"""End-to-end HTTP evidence run for the grounded_sam worker contract.

POSTs a real frame as base64 PNG/JPEG to the greedy_mask_predict endpoint,
decodes the single-channel mask PNG, writes overlay visualizations and a JSON
report (phrases, box count, mask area, round-trip time, seg_success).

Run with the model venv python:
  python test_e2e.py --image <frame.jpg> --out-dir <log/citynav/perception> \
      [--url http://127.0.0.1:8100/models/grounded_sam/greedy_mask_predict] \
      [--prompts "orange ball" "building"]
"""
import argparse
import base64
import hashlib
import json
import time
import urllib.request
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

DEFAULT_URL = "http://127.0.0.1:8100/models/grounded_sam/greedy_mask_predict"


def post(url, image_bytes, prompt, box_threshold, text_threshold, timeout=180):
    payload = {
        "image_b64": base64.b64encode(image_bytes).decode("ascii"),
        "text_prompt": prompt,
        "box_threshold": box_threshold,
        "text_threshold": text_threshold,
    }
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    started = time.perf_counter()
    with urllib.request.urlopen(request, timeout=timeout) as response:
        status = response.status
        out = json.loads(response.read().decode("utf-8"))
    return status, out, (time.perf_counter() - started) * 1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--image", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--prompts", nargs="*", default=["orange ball", "building"])
    parser.add_argument("--failure-prompt", default="helicopter")
    parser.add_argument("--box-threshold", type=float, default=0.3)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    args = parser.parse_args()

    image_path = Path(args.image)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    image_bytes = image_path.read_bytes()
    image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"cannot decode {image_path}")
    stem = image_path.stem
    (out_dir / f"{stem}_input.jpg").write_bytes(image_bytes)

    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "url": args.url,
        "image": str(image_path),
        "image_sha256": hashlib.sha256(image_bytes).hexdigest(),
        "image_shape": list(image.shape[:2]),
        "box_threshold": args.box_threshold,
        "text_threshold": args.text_threshold,
        "results": [],
    }
    failures = 0
    for prompt in list(args.prompts) + [args.failure_prompt]:
        status, out, ms = post(args.url, image_bytes, prompt, args.box_threshold, args.text_threshold)
        seg_success = bool(out.get("seg_success", False))
        entry = {
            "prompt": prompt,
            "http_status": status,
            "seg_success": seg_success,
            "phrases": out.get("phrases", []),
            "n_phrases": len(out.get("phrases", [])),
            "round_trip_ms": round(ms, 1),
            "mask_area": 0,
        }
        if seg_success and out.get("mask_b64"):
            mask = cv2.imdecode(np.frombuffer(base64.b64decode(out["mask_b64"]), np.uint8), cv2.IMREAD_GRAYSCALE)
            if mask.shape[:2] != image.shape[:2]:
                raise SystemExit(f"mask geometry mismatch: {mask.shape} vs {image.shape}")
            entry["mask_area"] = int((mask > 127).sum())
            overlay = image.copy()
            color = np.array([255, 144, 30], np.float64)  # BGR accent
            blended = (image * 0.55 + color * 0.45).astype(np.uint8)
            overlay[mask > 127] = blended[mask > 127]
            for suffix, frame in (("overlay", overlay), ("mask", mask)):
                ok, png = cv2.imencode(".png", frame)
                if not ok:
                    raise SystemExit("png encoding failed")
                (out_dir / f"{stem}_{prompt.replace(' ', '_')}_{suffix}.png").write_bytes(png.tobytes())
        else:
            if prompt == args.failure_prompt:
                if seg_success:
                    failures += 1
                    entry["failure_path"] = "expected seg_success=False"
            else:
                failures += 1
        report["results"].append(entry)
        print(f"{prompt!r}: http={status} seg_success={seg_success} "
              f"phrases={entry['n_phrases']} area={entry['mask_area']} {entry['round_trip_ms']:.0f} ms")

    report["ok"] = failures == 0
    (out_dir / f"{stem}_greedy_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"report: {out_dir / (stem + '_greedy_report.json')}")
    if failures:
        raise SystemExit(f"{failures} unexpected result(s)")


if __name__ == "__main__":
    main()
