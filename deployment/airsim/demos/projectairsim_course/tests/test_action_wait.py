import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from action_wait import wait_action_stage


class ActionWaitTest(unittest.TestCase):
    def test_stopped_clock_reports_action_and_stage(self):
        def wait(*args):
            raise TimeoutError('condition timeout: {}')
        mission = SimpleNamespace(wait=wait, last_clock_ns=123,
                                  platform=SimpleNamespace(armed=True, offboard=True))
        with self.assertRaisesRegex(TimeoutError, 'LandBehavior: timeout waiting for result.*sim_clock_advanced=False.*armed=True'):
            wait_action_stage(mission, lambda: False, '/drone0/LandBehavior', 'result', 60)

    def test_result_ready_is_not_failed(self):
        mission = SimpleNamespace(wait=lambda predicate, timeout: self.assertTrue(predicate()), last_clock_ns=123)
        wait_action_stage(mission, lambda: True, 'Land', 'result', 60)


if __name__ == '__main__':
    unittest.main()
