import os
import base64
import binascii
import zlib
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import torch
from weights import verify_weights

MODEL = Path(__file__).resolve().parent
UPSTREAM = MODEL.parents[4] / "src/tool/navigation/planner/flowpilot/upstream"
sys.path.insert(0, str(UPSTREAM))
from inference.depth_aug import clip_and_noise_depth
from inference.runtime import FlowPilotRuntime


def decode_depth(request):
    """Depth in metres as a float32 array from either wire encoding.

    Both encodings carry identical values: the ROS adapter's 20.0 m fill for
    invalid pixels happens client-side, so no semantic difference exists
    between the JSON and the base64+zlib representations.
    """
    encoded = request.get("depth_b64_gz")
    if encoded is None:
        rows = request.get("depth_m")
        if rows is None:
            raise ValueError("request carries no depth representation")
        return np.asarray(rows, dtype=np.float32)
    shape = request.get("depth_shape")
    if (not isinstance(shape, (list, tuple)) or len(shape) != 2
            or not all(isinstance(value, int) and value > 0 for value in shape)):
        raise ValueError("depth_shape must be two positive integers")
    try:
        raw = zlib.decompress(base64.b64decode(encoded, validate=True))
    except (binascii.Error, ValueError, zlib.error) as error:
        raise ValueError("depth_b64_gz must be base64-encoded zlib data") from error
    rows, cols = shape
    if len(raw) != rows * cols * 4:
        raise ValueError("depth payload length does not match depth_shape")
    return np.frombuffer(raw, dtype="<f4").reshape(rows, cols).copy()


def prepare_depth(meters):
    meters = np.asarray(meters, dtype=np.float32)
    if meters.ndim != 2 or not meters.size or meters.size > 1048576:
        raise ValueError("expected one rectangular depth image in metres")
    if not np.isfinite(meters).all() or (meters < 0).any():
        raise ValueError("expected finite nonnegative depth in metres")
    return clip_and_noise_depth(np.minimum(meters, 20.0) / 20.0, max_depth_m=20.0)

class Predictor:
    def __init__(self):
        if not torch.cuda.is_available():
            raise RuntimeError("FlowPilot requires a CUDA-enabled PyTorch environment")
        a = verify_weights(Path(os.environ.get("FLOWPILOT_WEIGHTS_DIR", MODEL / "artifacts")), MODEL / "weights.sha256")
        self.runtime = FlowPilotRuntime(SimpleNamespace(
            device="cuda", num_steps=3, num_layers=12, v_cmd=6.,
            phase2_ckpt=a/"ckpt_final.pt", vae_pth=a/"Wan2.2_VAE.pth",
            action_stats_path=a/"action_stats.pt", coeff_stats_path=a/"coeff_stats.pt",
            use_prev_conditioning=False))
        self.runtime.warmup()
        self.metadata = {"model": "flowpilot", "revision": "f29f1bfee1399c96e5fd384276d5d7a09876286b", "device": "cuda"}

    def infer(self, request):
        result = self.runtime.infer(prepare_depth(decode_depth(request)),
            np.asarray(request["state_goal"], dtype=np.float32), v_cmd=request["speed_mps"])
        if not (np.isfinite(result.wp_body).all() and np.isfinite(result.ctrl_pts_body).all()):
            raise ValueError("nonfinite FlowPilot trajectory")
        return dict(request_id=request["request_id"], wp_body=result.wp_body.tolist(),
                    ctrl_pts_body=result.ctrl_pts_body.tolist())

    def close(self):
        self.runtime.shutdown()
