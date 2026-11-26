"""Compare the port to the pinned upstream using a deterministic observation."""
import argparse
from pathlib import Path
import sys
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src/basic_function/inference/models/flowpilot"))
sys.path.insert(0, str(ROOT / "src/tool/navigation/planner/flowpilot/ros2_adapter"))
from adapter import Predictor, prepare_depth
from flowpilot.models.poly_action import BernsteinActionBasis
from flowpilot.coords import quat_to_rot
from inference.depth_aug import clip_and_noise_depth
from inference.state import WorldState, build_state_goal
from uav_flowpilot_planner.curve import Segment, state_derivatives
from schemas import Request, Response


def main(cuda):
    q = np.array([.5, .5, .5, .5])
    rot = quat_to_rot(q)
    origin = np.array([4., 8., 1.])
    velocity, acc, jerk = np.eye(3)
    state = WorldState(origin, velocity, acc, jerk)
    sg, _, _ = build_state_goal(state, origin+velocity, None, q, rot)
    req = Request(request_id="baseline", depth_m=np.linspace(2,25,90*160).reshape(90,160).tolist(),
                  state_goal=sg.tolist(), speed_mps=3.)
    meters = np.asarray(req.depth_m, dtype=np.float32)
    normalized = clip_and_noise_depth(np.minimum(meters,20.) / 20., max_depth_m=20.)
    np.testing.assert_array_equal(prepare_depth(req.depth_m), normalized)
    times = np.arange(-.14, .001, .02)
    a, j = state_derivatives(times, velocity + times[:,None]*acc + .5*times[:,None]**2*jerk, 0.)
    np.testing.assert_allclose([a,j], [acc,jerk], atol=1e-10)
    basis = BernsteinActionBasis(device="cpu")
    cp = torch.randn((1,8,3), generator=torch.Generator().manual_seed(123))
    wp = basis.expand_to_12d(cp[:,3:], cp[:,:3])[0].numpy().reshape(48,4,3)
    seg = Segment(cp.numpy(), origin, rot, 10., 10.215, .2)
    for k in range(7,48):
        pva = wp[k,:3] @ rot.T
        pva[0] += origin
        np.testing.assert_allclose(seg.sample(10.+k/30)[0], pva, atol=1e-5, rtol=1e-4)
    assert abs(seg.start+seg.times[-1] - (10.+47/30)) < 1e-9
    print("Depth preprocessing, velocity derivatives and trimmed trajectory match.")
    if cuda:
        model = Predictor()
        try:
            torch.manual_seed(725)
            native = model.runtime.infer(normalized, sg, v_cmd=3.)
            torch.manual_seed(725)
            response = Response(**model.infer(req.model_dump()))
            for expected, actual in [(native.wp_body, response.wp_body), (native.ctrl_pts_body, response.ctrl_pts_body)]:
                np.testing.assert_allclose(actual, expected, atol=1e-5, rtol=1e-4)
            print(model.metadata, torch.cuda.get_device_name(), "CUDA parity passed")
        finally:
            model.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cuda", action="store_true")
    main(parser.parse_args().cuda)
