import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('spf_sensor', Path(__file__).resolve().parents[1] / 'spf_sensor.py')
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)


class SensorHealth(unittest.TestCase):
    def test_missing_or_corrupt_state_is_unhealthy(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'state.json'
            self.assertFalse(sensor.healthy(path, 100.))
            path.write_text('incomplete')
            self.assertFalse(sensor.healthy(path, 100.))

    def test_only_fresh_valid_sample_is_healthy(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'state.json'
            for stamp, updated, expected in [(123, 99., True), (123, 94., False),
                                             (123, 101., False), (0, 99., False)]:
                with self.subTest(stamp=stamp, updated=updated):
                    path.write_text(json.dumps(dict(stamp_ns=stamp, updated_monotonic=updated)))
                    self.assertEqual(sensor.healthy(path, 100.), expected)


if __name__ == '__main__':
    unittest.main()
