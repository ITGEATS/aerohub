"""CPU-only contract checks for the FlowPilot worker adapter."""
import base64
import unittest
import zlib

import numpy as np
from pydantic import ValidationError

from adapter import decode_depth, prepare_depth
from schemas import Request, Response


def binary_payload(meters):
    blob = zlib.compress(np.ascontiguousarray(meters, dtype="<f4").tobytes())
    return dict(depth_b64_gz=base64.b64encode(blob).decode("ascii"),
                depth_shape=list(meters.shape))


class DepthTest(unittest.TestCase):
    def test_constant_depth_normalizes_to_network_size(self):
        out = prepare_depth(np.full((90, 160), 5., dtype=np.float32).tolist())
        self.assertEqual(out.shape, (96, 160))
        np.testing.assert_allclose(out, 0.25)

    def test_overrange_depth_clips_to_one(self):
        out = prepare_depth(np.full((96, 160), 25., dtype=np.float32).tolist())
        np.testing.assert_allclose(out, 1.)

    def test_invalid_inputs_rejected(self):
        for rows in ([1., 2., 3.], [[-1., 2.], [3., 4.]], [[float("nan")]], [], np.zeros((1025, 1025)).tolist()):
            with self.subTest(rows=str(rows)[:40]):
                with self.assertRaises(ValueError):
                    prepare_depth(rows)


class DepthDecodeTest(unittest.TestCase):
    """The compact base64+zlib encoding must decode to the same metres."""

    def test_binary_round_trip_matches_json(self):
        meters = np.array([[1., 2.], [20., 4.]], dtype=np.float32)
        request = dict(request_id="cpu", state_goal=[0.] * 25, speed_mps=2., **binary_payload(meters))
        np.testing.assert_array_equal(decode_depth(request), meters)
        json_request = dict(request, depth_m=meters.tolist(), depth_b64_gz=None, depth_shape=None)
        np.testing.assert_array_equal(decode_depth(json_request), meters)

    def test_binary_corruption_rejected(self):
        meters = np.array([[1., 2.], [20., 4.]], dtype=np.float32)
        truncated = zlib.compress(meters.tobytes())[:-4]
        changes = ({"depth_b64_gz": "not-base64!!"},
                   {"depth_shape": [2, 3]},
                   {"depth_shape": "2x2"},
                   {"depth_b64_gz": base64.b64encode(truncated).decode("ascii")},
                   {"depth_b64_gz": None})
        base = dict(request_id="cpu", state_goal=[0.] * 25,
                    speed_mps=2., **binary_payload(meters))
        for change in changes:
            with self.subTest(change=sorted(change)):
                broken = dict(base)
                broken.update(change)
                with self.assertRaises(ValueError):
                    decode_depth(broken)


class SchemaTest(unittest.TestCase):
    def test_request_and_response_shapes(self):
        request = Request(request_id="cpu", depth_m=[[1., 2.]], state_goal=[float(i) for i in range(25)], speed_mps=3.)
        self.assertEqual(len(request.state_goal), 25)
        response = Response(request_id="cpu", wp_body=[[float(j) for j in range(12)] for _ in range(48)],
                            ctrl_pts_body=[[float(j) for j in range(3)] for _ in range(8)])
        self.assertEqual(len(response.ctrl_pts_body), 8)

    def test_binary_request_accepted(self):
        request = Request(request_id="cpu", state_goal=[float(i) for i in range(25)], speed_mps=3.,
                          **binary_payload(np.ones((2, 2), dtype=np.float32)))
        self.assertEqual(request.depth_shape, [2, 2])
        self.assertIsNone(request.depth_m)

    def test_depth_representation_rules(self):
        base = dict(request_id="cpu", state_goal=[0.] * 25, speed_mps=3.)
        binary = binary_payload(np.ones((2, 2), dtype=np.float32))
        for change in (dict(base, depth_m=[[1.]], depth_b64_gz=binary["depth_b64_gz"]),
                       dict(base, depth_b64_gz=binary["depth_b64_gz"]),
                       dict(base, depth_m=[[1.]], depth_shape=[2, 2]),
                       dict(base)):
            with self.subTest(change=sorted(change)):
                with self.assertRaises(ValidationError):
                    Request(**change)

    def test_wrong_shapes_and_extra_fields_rejected(self):
        base = dict(request_id="cpu", depth_m=[[1.]], state_goal=[0.] * 25, speed_mps=3.)
        for change in ({"state_goal": [0.] * 24}, {"speed_mps": 0.}, {"unknown": 1}):
            with self.subTest(change=list(change)):
                with self.assertRaises(ValidationError):
                    Request(**dict(base, **change))
        with self.assertRaises(ValidationError):
            Response(request_id="cpu", wp_body=[[0.] * 12] * 47, ctrl_pts_body=[[0.] * 3] * 8)


if __name__ == "__main__":
    unittest.main()
