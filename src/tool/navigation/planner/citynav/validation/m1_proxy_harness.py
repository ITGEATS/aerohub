#!/usr/bin/env python3
"""M-1 代理侧驱动器（工作站容器内运行，与 uav_citynav_llm_proxy 同 DDS 域）。

读 capture_frames.py 的 manifest，把同一批 jpg 逐组发布为 sensor_msgs/Image
（rgb8，wall-clock stamp，sensor-data QoS），再按 planner 权威契约发
citynav/llm/query（op=viewpoint），收 citynav/llm/result，逐组落 JSON。
代理单飞（busy 即回），本驱动器严格串行。
"""
import argparse
import io
import json
from pathlib import Path
import time
import uuid

from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

LABEL_CAMERA = [
    ("left", "CityLeft"),
    ("slightly_left", "CitySlightlyLeft"),
    ("front", "CityFront"),
    ("slightly_right", "CitySlightlyRight"),
    ("right", "CityRight"),
]


class Driver(Node):
    def __init__(self):
        super().__init__("citynav_m1_proxy_driver")
        self.img_pubs = {}
        for label, cam in LABEL_CAMERA:
            self.img_pubs[label] = self.create_publisher(
                Image, f"/drone0/sensor_measurements/{cam}/rgb/image",
                qos_profile_sensor_data)
        self.query_pub = self.create_publisher(
            String, "/drone0/citynav/llm/query", 10)
        self.result = None
        self.result_id = None
        self.create_subscription(String, "/drone0/citynav/llm/result",
                                 self._on_result, 10)

    def _on_result(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if payload.get("request_id") == self.result_id:
            self.result = payload

    def publish_frames(self, paths):
        stamps = {}
        for label, path in paths.items():
            with PILImage.open(path) as im:
                rgb = im.convert("RGB")
            w, h = rgb.size
            now = self.get_clock().now().to_msg()
            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = f"drone0/{label}_image"
            msg.height, msg.width = h, w
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = w * 3
            msg.data = bytearray(rgb.tobytes())
            self.img_pubs[label].publish(msg)
            stamps[label] = now.sec * 10**9 + now.nanosec
        return stamps

    def ask(self, stamps, instruction, landmarks, traversed, next_lm,
            timeout_s=120.0):
        self.result_id = uuid.uuid4().hex
        self.result = None
        query = {"request_id": self.result_id, "op": "viewpoint",
                 "instruction": instruction, "landmarks": landmarks,
                 "traversed": traversed, "next": next_lm,
                 "view_stamps": stamps, "max_age_s": 90.0}
        self.query_pub.publish(String(data=json.dumps(query)))
        deadline = time.time() + timeout_s
        while self.result is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--context", type=Path, default=None,
                    help="JSON {instruction,landmarks,traversed,next}；给定则"
                         "覆盖以下散装参数（绕开 shell 引号问题）")
    ap.add_argument("--instruction", default=None)
    ap.add_argument("--landmarks", default=None, help="JSON 数组")
    ap.add_argument("--traversed", default="[]", help="JSON 数组")
    ap.add_argument("--next", dest="next_lm", default=None)
    ap.add_argument("--only", type=int, nargs="*", default=None,
                    help="限定 set id 子集（断点续跑）")
    args = ap.parse_args()
    if args.context:
        ctx = json.loads(args.context.read_text())
        args.instruction = ctx["instruction"]
        args.landmarks = json.dumps(ctx["landmarks"])
        args.traversed = json.dumps(ctx.get("traversed", []))
        args.next_lm = ctx["next"]
    if not args.instruction or not args.landmarks or not args.next_lm:
        raise SystemExit("need --context or --instruction/--landmarks/--next")

    manifest = json.loads(args.manifest.read_text())
    frames_dir = args.manifest.parent
    rclpy.init()
    driver = Driver()
    results = []
    try:
        for entry in manifest["sets"]:
            if args.only is not None and entry["id"] not in args.only:
                continue
            paths = {label: frames_dir / fname
                     for label, fname in entry["files"].items()}
            stamps = driver.publish_frames(paths)
            time.sleep(0.5)  # 让 Image 先于 query 到达代理
            res = driver.ask(stamps, args.instruction,
                             json.loads(args.landmarks),
                             json.loads(args.traversed), args.next_lm)
            rec = {"set": entry["id"], "pose": entry["pose"],
                   "result": res}
            results.append(rec)
            print(f"set {entry['id']:02d}: "
                  f"{'ok' if res and res.get('ok') else res}", flush=True)
            args.out.write_text(json.dumps(results, indent=2, ensure_ascii=False))
    finally:
        driver.destroy_node()
        rclpy.try_shutdown()
    ok = sum(1 for r in results if r["result"] and r["result"].get("ok"))
    print(f"proxy side done: {ok}/{len(results)} ok")


if __name__ == "__main__":
    main()
