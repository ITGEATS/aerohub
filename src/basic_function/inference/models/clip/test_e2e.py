"""End-to-end HTTP evidence run for the clip worker contract.

POSTs real frames (base64 PNG/JPEG) plus texts to /models/clip/score, records
the full similarity matrix, per-image margin between the target text and the
distractor "helicopter", round-trip time, model_version, and repeats the exact
request to prove idempotency. Also exercises the 422 rejection paths
(empty lists, 17 images, 17 texts). No numeric thresholds: values are recorded
as evidence, per the migration plan (D19/D23).

Run with the model venv python:
  python test_e2e.py --image <frame1> --image <frame2> --out-dir <log/citynav/perception/clip> \
      [--url http://127.0.0.1:8101/models/clip/score] \
      [--texts "orange ball" "building" "helicopter"]
"""
import argparse
import base64
import hashlib
import json
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path

DEFAULT_URL = "http://127.0.0.1:8101/models/clip/score"


def post(url, payload, timeout=120):
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    started = time.perf_counter()
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            status = response.status
            out = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        status = error.code
        out = json.loads(error.read().decode("utf-8"))
    return status, out, (time.perf_counter() - started) * 1000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--image", action="append", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--texts", nargs="*",
                        default=["orange ball", "building", "helicopter"])
    parser.add_argument("--target-text", default="orange ball")
    parser.add_argument("--distractor-text", default="helicopter")
    parser.add_argument("--report-name", default="clip_score_report.json")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    images, image_meta = [], []
    for image_path in args.image:
        path = Path(image_path)
        raw = path.read_bytes()
        images.append(base64.b64encode(raw).decode("ascii"))
        image_meta.append({"image": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
                           "bytes": len(raw)})

    payload = {"images_b64": images, "texts": args.texts}
    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "url": args.url,
        "images": image_meta,
        "texts": args.texts,
        "results": [],
    }
    failures = 0

    # Repeated identical requests must return identical matrices (idempotency).
    runs = []
    for run_index in range(2):
        status, out, ms = post(args.url, payload)
        runs.append({"run": run_index, "http_status": status, "round_trip_ms": round(ms, 1),
                     "similarity": out.get("similarity"), "model_version": out.get("model_version")})
        if status != 200 or not out.get("similarity"):
            failures += 1
    idempotent = (len(runs) == 2 and runs[0]["similarity"] == runs[1]["similarity"]
                  and runs[0]["model_version"] == runs[1]["model_version"])
    if not idempotent:
        failures += 1
    report["runs"] = runs
    report["idempotent"] = idempotent
    report["model_version"] = runs[0]["model_version"]

    target = args.texts.index(args.target_text)
    distractor = args.texts.index(args.distractor_text)
    for image_index, meta in enumerate(image_meta):
        matrix = runs[0]["similarity"]
        row = matrix[image_index] if matrix else []
        entry = {
            "image": meta["image"],
            "image_sha256": meta["sha256"],
            "similarity": row,
            "margin_orange_ball_vs_helicopter": (row[target] - row[distractor]) if len(row) > max(target, distractor) else None,
        }
        report["results"].append(entry)
        margin = entry["margin_orange_ball_vs_helicopter"]
        margin_text = f"{margin:.4f}" if margin is not None else "n/a"
        print(f"{Path(meta['image']).name}: similarity={[round(v, 4) for v in row]} "
              f"margin({args.target_text}-{args.distractor_text})={margin_text}")

    # Contract rejection paths must answer 422.
    for name, bad_payload in [
        ("empty_images", {"images_b64": [], "texts": args.texts}),
        ("empty_texts", {"images_b64": images[:1], "texts": []}),
        ("17_images", {"images_b64": images * 9, "texts": args.texts}),
        ("17_texts", {"images_b64": images[:1], "texts": args.texts * 6}),
    ]:
        status, _, _ = post(args.url, bad_payload)
        entry = {"case": name, "http_status": status, "expected": 422}
        report.setdefault("rejection_checks", []).append(entry)
        if status != 422:
            failures += 1
        print(f"{name}: http={status} (expected 422)")

    report["ok"] = failures == 0
    out_file = out_dir / args.report_name
    out_file.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"report: {out_file}")
    if failures:
        raise SystemExit(f"{failures} unexpected result(s)")


if __name__ == "__main__":
    main()
