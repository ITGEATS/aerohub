import sys
import unittest
from unittest.mock import MagicMock

sys.modules.setdefault("msgpack", MagicMock())
from mission import Mission


class EvidenceWindowTests(unittest.TestCase):
    def test_excludes_pre_exploration_samples_from_route_gate(self):
        route = [
            {"position": [0.0, 0.0, -0.01]},
            {"position": [0.0, 0.0, 2.0]},
            {"position": [1.0, 0.0, 2.0]},
        ]

        positions = Mission.positions_since(route, 1)

        self.assertEqual(positions, [[0.0, 0.0, 2.0], [1.0, 0.0, 2.0]])
        self.assertTrue(all(Mission.in_bounds(position) for position in positions))

    def test_accepts_both_planner_exit_timing_windows(self):
        self.assertTrue(Mission.is_planner_exit_failure(
            "FALCON replacement trajectory timeout"))
        self.assertTrue(Mission.is_planner_exit_failure(
            "FALCON trajectory coverage expired without replacement"))
        self.assertTrue(Mission.is_planner_exit_failure(
            "FollowTimedPath execution failed"))
        self.assertFalse(Mission.is_planner_exit_failure("depth stream expired"))


if __name__ == "__main__":
    unittest.main()
