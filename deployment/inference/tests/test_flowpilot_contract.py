"""FlowPilot payload regressions for depth loss, transport layout and stamps."""
import base64
import copy
import json
import struct
import sys
import unittest
import zlib
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/basic_function/inference/ros2_adapter'))
from uav_inference.flowpilot_contract import depth_m, encode_depth, request_payload, stamp_ns


def decode(encoded, shape):
    raw = zlib.decompress(base64.b64decode(encoded))
    return np.frombuffer(raw, dtype='<f4').reshape(shape)


class ContractTest(unittest.TestCase):
    def setUp(self):
        header = SimpleNamespace(frame_id='camera', stamp=SimpleNamespace(sec=10, nanosec=0))
        image = SimpleNamespace(header=header, height=2, width=2, encoding='32FC1', is_bigendian=False,
                                step=12, data=struct.pack('<6f', 1, 2, 99, 3, 4, 99))
        self.request = SimpleNamespace(request_id='test', depth=image, observation_header=header,
                                       state_goal=[0.] * 25, speed_mps=2.)

    def test_float_padding_and_endian(self):
        np.testing.assert_allclose(depth_m(self.request.depth), [[1, 2], [3, 4]])
        image = self.request.depth
        image.is_bigendian = True
        image.data = struct.pack('>6f', 1, 2, 99, 3, 4, 99)
        np.testing.assert_allclose(depth_m(image), [[1, 2], [3, 4]])

    def test_millimeters_with_padding(self):
        image = self.request.depth
        image.encoding, image.step = '16UC1', 6
        image.data = struct.pack('<6H', 1000, 2000, 99, 3000, 4000, 99)
        np.testing.assert_allclose(depth_m(image), [[1, 2], [3, 4]])

    def test_invalid_frame_and_sparse_holes(self):
        image = self.request.depth
        for value in (0., -1., float('nan'), float('inf')):
            image.data = struct.pack('<6f', *([value] * 6))
            with self.assertRaisesRegex(ValueError, 'no valid'):
                depth_m(image)
        image.data = struct.pack('<6f', 1, 0, 99, float('nan'), 4, 99)
        self.assertEqual(depth_m(image), [[1, 20], [20, 4]])

    def test_encoded_depth_carries_the_same_metres(self):
        encoded, shape = encode_depth(self.request.depth)
        np.testing.assert_allclose(decode(encoded, shape), [[1, 2], [3, 4]])
        image = self.request.depth
        image.is_bigendian = True
        image.data = struct.pack('>6f', 1, 2, 99, 3, 4, 99)
        encoded, shape = encode_depth(image)
        np.testing.assert_allclose(decode(encoded, shape), [[1, 2], [3, 4]])

    def test_stamps_and_finite_request(self):
        payload = request_payload(self.request, 10_000_000_000)
        np.testing.assert_allclose(decode(payload['depth_b64_gz'], payload['depth_shape']),
                                   [[1, 2], [3, 4]])
        self.assertEqual(payload['depth_shape'], [2, 2])
        with self.assertRaises(ValueError):
            request_payload(self.request, 10_300_000_001)
        with self.assertRaises(ValueError):
            request_payload(self.request, 9_999_999_999)
        request = copy.deepcopy(self.request)
        request.observation_header = copy.deepcopy(request.observation_header)
        request.observation_header.stamp.nanosec = 50_000_001
        with self.assertRaises(ValueError):
            request_payload(request, 10_100_000_000)
        for values in ([0.] * 24, [float('nan')] * 25):
            request = copy.deepcopy(self.request)
            request.state_goal = values
            with self.assertRaises(ValueError):
                request_payload(request, 10_000_000_000)
        with self.assertRaises(ValueError):
            stamp_ns(SimpleNamespace(sec=0, nanosec=1_000_000_000))

    def test_ros_fixed_array_uses_json_numbers(self):
        self.request.state_goal = np.arange(25, dtype=np.float32)
        self.request.speed_mps = np.float32(2.)
        payload = request_payload(self.request, 10_000_000_000)
        self.assertEqual(json.loads(json.dumps(payload))['state_goal'], list(range(25)))


if __name__ == '__main__':
    unittest.main()
