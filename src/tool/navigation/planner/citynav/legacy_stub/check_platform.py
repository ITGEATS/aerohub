"""平台层验证：场景桩协议 -> airsim 客户端 -> 全景取图(teleport 旋转) -> 尺寸/深度断言。

前置：Blocks.exe 以 settings.citynav_legacy.json 运行；scene_server.py 已启动。
"""
import json
import os
import sys
import time

STUB_DIR = os.path.dirname(os.path.abspath(__file__))
UPSTREAM = os.path.abspath(os.path.join(STUB_DIR, os.pardir, "upstream"))
OUT = os.path.abspath(os.path.join(STUB_DIR, os.pardir, os.pardir, os.pardir, os.pardir, os.pardir, os.pardir,
                                   "log", "citynav", "legacy", "platform-check-" + time.strftime("%Y%m%d-%H%M%S")))
os.makedirs(OUT, exist_ok=True)

sys.argv = ["check_platform.py", "--run_type", "eval",
            "--Image_Height_RGB", "512", "--Image_Width_RGB", "512",
            "--Image_Height_DEPTH", "512", "--Image_Width_DEPTH", "512"]
os.chdir(UPSTREAM)
sys.path.insert(0, UPSTREAM)

import numpy as np  # noqa: E402
import cv2  # noqa: E402
import airsim  # noqa: E402

# cv2.imwrite 对非 ASCII 路径（Desktop/无人机/）静默失败：全局包装为 imencode+write
_orig_imwrite = cv2.imwrite


def _safe_imwrite(path, img, *a, **k):
    try:
        if _orig_imwrite(path, img, *a, **k):
            return True
    except Exception:
        pass
    ext = os.path.splitext(str(path))[1] or ".png"
    ok, buf = cv2.imencode(ext, img, *a, **k)
    if ok:
        with open(path, "wb") as f:
            f.write(buf.tobytes())
    return bool(ok)


cv2.imwrite = _safe_imwrite

from airsim_plugin.AirVLNSimulatorClientTool import AirVLNSimulatorClientTool  # noqa: E402
from utils.env_utils import get_pano_observations, getPoseAfterMakeActions  # noqa: E402

tool = AirVLNSimulatorClientTool(machines_info=[{
    "MACHINE_IP": "127.0.0.1", "SOCKET_PORT": 30000,
    "MAX_SCENE_NUM": 8, "open_scenes": [0],
}])
tool.run_call()
print("[check] scene stub + airsim client connected", flush=True)

pose = airsim.Pose(airsim.Vector3r(-10.0, -10.0, -8.0), airsim.to_quaternion(0.0, 0.0, 0.0))
tool.setPoses([[pose]])
print("[check] setPoses ok", flush=True)

pano_obs, pano_pose = get_pano_observations(pose, tool, scene_id=0)
report = {"views": len(pano_obs), "per_view": []}
for i, (rgb, dep) in enumerate(pano_obs):
    entry = {"i": i,
             "rgb_shape": list(rgb.shape), "rgb_dtype": str(rgb.dtype),
             "dep_shape": list(dep.shape), "dep_dtype": str(dep.dtype),
             "dep_min": float(np.min(dep)), "dep_max": float(np.max(dep)),
             "dep_mean": float(np.mean(dep))}
    assert rgb.shape[:2] == (512, 512), f"rgb size {rgb.shape}"
    assert dep.shape[:2] == (512, 512), f"depth size {dep.shape}"
    report["per_view"].append(entry)
    if i in (0, 2):
        cv2.imwrite(os.path.join(OUT, f"rgb_view{i}.png"), rgb)
        dv = (np.clip(dep.squeeze(), 0, 1) * 255).astype(np.uint8)
        cv2.imwrite(os.path.join(OUT, f"dep_view{i}.png"), np.stack([dv] * 3, axis=2))

# 深度为 /300 归一（v2）：反投影 ×100 后为米/3
report["depth_norm_check"] = "dep_max<=1.0 means normalized(/300)"
report["pano_poses"] = [list(map(float, p)) for p in pano_pose]
_p = tool.airsim_clients[0][0].simGetVehiclePose(vehicle_name="Drone_1").position
report["pose_after_pano"] = [_p.x_val, _p.y_val, _p.z_val]
with open(os.path.join(OUT, "report.json"), "w") as f:
    json.dump(report, f, indent=2)
print("[check] PANOK", json.dumps(report["per_view"][0]), flush=True)
