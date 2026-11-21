#!/usr/bin/env python3
"""M-1 抓帧器（宿主 legacy venv 运行，Blocks RPC 45481）。

同一批五目帧同时喂给代理（DDS Image）与 legacy SDK（jpg 文件），是 M-1
"同一批抓帧"的输入源。相机名与视角标签映射 = 生产 planner.yaml 的
cameras 契约（left=CityLeft ...，顺序=上游 SimRun.py L239 obs_viewpoint）。
车辆地面 teleport（与 mission 起飞前同一机制），每姿态取 5 张 Scene jpeg。
"""
import argparse
import json
import math
from pathlib import Path
import time

import airsim

VIEW_CAMERA = [
    ("left", "CityLeft"),
    ("slightly_left", "CitySlightlyLeft"),
    ("front", "CityFront"),
    ("slightly_right", "CitySlightlyRight"),
    ("right", "CityRight"),
]

# NED (x, y, yaw_deg)，z=0 地面。覆盖：橙球可见/不可见、建筑近景、远角、
# 多朝向（OrangeBall 真值约 NED (34.25, 33.10)）。
POSES = [
    (10, 10, 45), (10, 10, 0), (15, 15, 45), (20, 20, 45), (20, 20, 90),
    (20, 20, 0), (25, 25, 45), (30, 30, 45), (30, 30, 135), (34, 25, 0),
    (25, 34, 90), (5, 20, 60), (20, 5, 30), (0, 0, 45), (-10, -10, 45),
    (-15, 10, 90), (10, -15, 0), (40, 10, 45), (10, 40, 135), (30, 15, 60),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--rpc-host", default="127.0.0.1")
    ap.add_argument("--rpc-port", type=int, default=45481)
    ap.add_argument("--vehicle", default="PX4")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    client = airsim.MultirotorClient(ip=args.rpc_host, port=args.rpc_port)
    client.confirmConnection()

    manifest = {"rpc": f"{args.rpc_host}:{args.rpc_port}", "vehicle": args.vehicle,
                "captured": time.strftime("%Y-%m-%d %H:%M:%S"), "sets": []}
    for i, (x, y, yaw) in enumerate(POSES):
        pose = airsim.Pose(airsim.Vector3r(x, y, 0.0),
                           airsim.to_quaternion(0.0, 0.0, math.radians(yaw)))
        client.simSetVehiclePose(pose, True, args.vehicle)
        time.sleep(1.0)  # 渲染稳定
        entry = {"id": i, "pose": {"x": x, "y": y, "yaw_deg": yaw}, "files": {}}
        for label, cam in VIEW_CAMERA:
            responses = client.simGetImages(
                [airsim.ImageRequest(cam, airsim.ImageType.Scene, False, True)],
                vehicle_name=args.vehicle)
            path = args.out / f"set{i:03d}_{label}.jpg"
            path.write_bytes(responses[0].image_data_uint8)
            entry["files"][label] = path.name
        manifest["sets"].append(entry)
        print(f"set {i:02d} @ ({x},{y},{yaw})", flush=True)

    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"captured {len(manifest['sets'])} sets -> {args.out}")


if __name__ == "__main__":
    main()
