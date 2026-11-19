"""Actual tensor execution tests; set TEST_TORCH_DEVICE=cuda for real GPU coverage."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch
from aerohub_inference.backends.torch import TorchBackend


class Network(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.scale = torch.nn.Parameter(torch.tensor(2.0))

    def forward(self, value):
        return value * self.scale


class TorchBackendTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        weights = Path(self.directory.name) / 'weights.pth'
        torch.save(Network().state_dict(), weights)
        self.backend = TorchBackend(Network, weights, 'tiny',
                                    {'value': dict(shape=[1, 2], dtype='float32')},
                                    {'result': dict(shape=[1, 2], dtype='float32')},
                                    device=os.environ.get('TEST_TORCH_DEVICE', 'cpu'))

    def tearDown(self):
        self.backend.close()
        self.directory.cleanup()

    def test_named_tensors_and_output_ownership(self):
        feed = {'value': np.array([[1, 3]], dtype=np.float64)}
        outputs, timings = self.backend.infer(feed, {'result': np.float32})
        self.backend.infer({'value': np.zeros((1, 2))})
        np.testing.assert_array_equal(outputs['result'], [[2, 6]])
        self.assertEqual(set(timings), {'input_transfer', 'forward_and_sync', 'output_transfer'})
        self.assertTrue(all(value >= 0 for value in timings.values()))
        self.assertEqual(self.backend.metadata['device'], os.environ.get('TEST_TORCH_DEVICE', 'cpu'))

    def test_invalid_tensor_contracts(self):
        for feed in ({'other': np.zeros((1, 2))}, {'value': np.zeros(2)},
                     {'value': np.array([['x', 'y']])}):
            with self.assertRaises(ValueError):
                self.backend.infer(feed)
        with self.assertRaises(ValueError):
            self.backend.infer({'value': np.zeros((1, 2))}, {'unknown': np.float32})

    def test_cuda_unavailable_never_falls_back(self):
        with patch('torch.cuda.is_available', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'no CPU fallback'):
                TorchBackend(Network, 'unused', 'tiny',
                    {'value': dict(shape=[1, 2], dtype='float32')},
                    {'result': dict(shape=[1, 2], dtype='float32')}, device='cuda')

    def test_variable_dimensions_and_custom_state_loader(self):
        with tempfile.TemporaryDirectory() as directory:
            weights = Path(directory) / 'opaque-weights'
            weights.write_bytes(b'loader-owned format')
            loaded = []
            def loader(path):
                loaded.append(path)
                return Network().state_dict()
            backend = TorchBackend(Network, weights, 'dynamic',
                {'value': dict(shape=[1, None], dtype='float32')},
                {'result': dict(shape=[1, None], dtype='float32')},
                device=os.environ.get('TEST_TORCH_DEVICE', 'cpu'), state_loader=loader)
            try:
                for width in (2, 5):
                    output, _ = backend.infer({'value': np.ones((1, width))})
                    np.testing.assert_array_equal(output['result'], np.full((1, width), 2.))
                for shape in ((1, 0), (2, 3), (1, 2, 3)):
                    with self.assertRaises(ValueError):
                        backend.infer({'value': np.ones(shape)})
                self.assertEqual(loaded, [weights])
            finally:
                backend.close()

    def test_close_is_idempotent_and_rejects_execution(self):
        self.backend.close()
        self.backend.close()
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            self.backend.infer({'value': np.zeros((1, 2))})


if __name__ == '__main__':
    unittest.main()
