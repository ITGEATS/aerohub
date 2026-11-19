import unittest

from uav_falcon_adapter.node import replan_deadline


class ReplanDeadlineTests(unittest.TestCase):
    def test_emergency_stop_deadline_is_not_shortened_by_planning_notice(self):
        deadline = replan_deadline(0.0, 1, 100.0, 15.0)
        self.assertEqual(deadline, 115.0)
        self.assertEqual(replan_deadline(deadline, 0, 101.0, 15.0), 115.0)

    def test_emergency_stop_upgrades_an_existing_rolling_deadline(self):
        rolling_deadline = replan_deadline(0.0, 0, 100.0, 15.0)
        self.assertEqual(rolling_deadline, 100.5)
        self.assertEqual(
            replan_deadline(rolling_deadline, 1, 100.04, 15.0), 115.04)

    def test_rolling_replan_uses_half_second_deadline(self):
        self.assertEqual(replan_deadline(0.0, 0, 100.0, 15.0), 100.5)


if __name__ == "__main__":
    unittest.main()
