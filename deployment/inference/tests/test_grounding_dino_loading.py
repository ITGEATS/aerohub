"""Real local snapshot strict-loading regression; no architecture mocks/downloads."""
from pathlib import Path
import sys
import unittest
import torch
from safetensors.torch import load_file
from transformers import GroundingDinoConfig

MODEL = Path(__file__).resolve().parents[3] / 'src/basic_function/inference/models/grounding_dino'
sys.path.insert(0, str(MODEL))
from adapter import Network


class LoadingTest(unittest.TestCase):
    def test_tied_aliases_load_but_missing_independent_weight_fails(self):
        torch.set_num_threads(2)
        config = GroundingDinoConfig.from_pretrained(MODEL / 'artifacts/snapshot', local_files_only=True)
        network = Network(config)
        state = load_file(MODEL / 'artifacts/snapshot/model.safetensors')
        result = network.load_state_dict(state)
        self.assertEqual(result.missing_keys, [])
        self.assertEqual(result.unexpected_keys, [])
        del state['model.query_position_embeddings.weight']
        with self.assertRaisesRegex(RuntimeError, 'Missing key'):
            network.load_state_dict(state)


if __name__ == '__main__':
    unittest.main()
