import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diffphys_session import check_start


class SessionTest(unittest.TestCase):
    def test_failed_flight_retained_compute_explains_armed_recovery(self):
        with self.assertRaisesRegex(RuntimeError, 'PX4仍解锁.*--reset-sim'):
            check_start(['aerohub-diffphys-course-compute-old'], 'arming_state: 2\n')

    def test_live_flight_never_recommends_reset(self):
        with self.assertRaisesRegex(RuntimeError, '任务客户端仍运行'):
            check_start(['aerohub-diffphys-course-flight-now'], 'arming_state: 1\n')

    def test_unknown_nonempty_status_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, '状态未知'):
            check_start([], 'listener returned no sample')

    def test_disarmed_retained_compute_has_cleanup_instruction(self):
        with self.assertRaisesRegex(RuntimeError, '上轮算力容器.*--reset-sim'):
            check_start(['aerohub-diffphys-course-compute-old'], 'arming_state: 1\n')

    def test_empty_px4_status_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, '状态未知'):
            check_start(['aerohub-course-wmm-px4'], '')

    def test_clean_start(self):
        check_start([], None)
        check_start(['aerohub-course-wmm-px4'], 'arming_state: 1\n')



if __name__ == '__main__':
    unittest.main()
