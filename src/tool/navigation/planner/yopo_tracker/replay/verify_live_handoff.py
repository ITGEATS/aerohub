"""Reproduce the short handoff observed with revision 5 using a controlled clock."""

from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from ros2_adapter.bridge.live_node import LiveTracker


MS = 1_000_000
EPOCH = 10_000 * MS


def image_at(stamp_ns):
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))


class HandoffFixture:
    tick = LiveTracker.tick
    brake = LiveTracker.brake

    def __init__(self, active=4, revision=5, brake_ms=240):
        axis = SimpleNamespace(get_position=lambda t: t, get_velocity=lambda _: 1.,
                               get_acceleration=lambda _: 0.)
        curve = SimpleNamespace(brake_from_ns=brake_ms * MS, axes=(axis, axis, axis),
                                knots=[(0, .3), (brake_ms * MS, .4)])
        self.now = EPOCH - 180 * MS
        self.last_input_ns = self.now
        self.active, self.revision = active, revision
        self.accepted = None, EPOCH, curve
        self.task = 'handoff-fixture'
        self.pending = self.result = self.inference = self.observation = self.brake_reason = None
        self.receive = Mock()
        self.finish = Mock()
        self.record = Mock()
        self.submit = Mock()
        self.select = Mock()
        self.worker = Mock()
        self.worker.submit.side_effect = lambda *_: Future()
        self.snapshot = Mock(return_value=None)

    def now_ns(self):
        return self.now

    def at(self, offset_ms, capture_ms=None):
        self.now = EPOCH + offset_ms * MS
        message = image_at(EPOCH + capture_ms * MS) if capture_ms is not None else None
        self.snapshot.return_value = (message, message, message) if message else None
        self.tick()

    def ready(self):
        self.inference[0].set_result((object(), [1., 2., 3.], None))


class TestLiveHandoff(unittest.TestCase):
    def test_prepare_before_activation_and_submit_at_fixed_splice(self):
        fixture = HandoffFixture()
        fixture.at(-180, -200)
        fixture.worker.submit.assert_called_once()
        anchor = fixture.worker.submit.call_args.args[-1]
        np.testing.assert_array_equal(anchor[0], [.24] * 3)
        np.testing.assert_array_equal(anchor[1], [1.] * 3)
        np.testing.assert_array_equal(anchor[2], [0.] * 3)
        self.assertEqual(anchor[3], .4)
        fixture.ready()
        fixture.at(-40)
        fixture.submit.assert_not_called()

        # In AirSim no new pair reached planning between this feedback and the lead gate.
        fixture.active = 5
        fixture.at(16)
        fixture.submit.assert_called_once()
        now, start, _, source = fixture.submit.call_args.args
        self.assertEqual((now, start), (EPOCH + 16 * MS, EPOCH + 240 * MS))
        self.assertEqual(source['anchor_revision'], 5)
        self.assertEqual(source['rgb_ns'], EPOCH - 200 * MS)
        self.assertIsNone(fixture.brake_reason)

    def test_first_revision_waits_for_feedback_and_keeps_the_latest_pair(self):
        fixture = HandoffFixture(active=0, revision=1)
        fixture.at(-90, -100)
        fixture.at(-40, -55)
        fixture.worker.submit.assert_not_called()
        fixture.active = 1
        fixture.at(16)
        fixture.worker.submit.assert_called_once()
        self.assertEqual(fixture.inference[2]['depth_ns'], EPOCH - 55 * MS)

    def test_discards_expired_frame_before_feedback(self):
        fixture = HandoffFixture()
        fixture.at(-180, -200)
        fixture.ready()
        fixture.at(110, 105)
        self.assertIsNone(fixture.inference)
        fixture.active = 5
        fixture.at(119)
        fixture.submit.assert_not_called()
        fixture.at(241, 230)
        self.assertEqual(fixture.brake_reason, 'BRAKE_RESERVE')

    def test_rejects_frame_that_will_expire_before_activation(self):
        fixture = HandoffFixture()
        fixture.at(-190, -295)
        fixture.worker.submit.assert_not_called()

    def test_stop_prevents_a_precomputed_update(self):
        fixture = HandoffFixture()
        fixture.at(-180, -200)
        fixture.ready()
        fixture.brake('REQUESTED_STOP')
        fixture.active = 5
        fixture.at(16, 5)
        fixture.submit.assert_not_called()
        fixture.finish.assert_called_once()

    def test_preview_without_a_target_allows_a_fresh_precomputation(self):
        fixture = HandoffFixture()
        fixture.at(-180, -200)
        fixture.inference[0].set_result((None, None, 'TARGET_LOST'))
        fixture.at(-90, -100)
        self.assertIsNone(fixture.brake_reason)
        self.assertEqual(fixture.worker.submit.call_count, 2)
        fixture.ready()
        fixture.active = 5
        fixture.at(16)
        fixture.submit.assert_called_once()
        self.assertEqual(fixture.submit.call_args.args[-1]['rgb_ns'], EPOCH - 100 * MS)

    def test_no_precomputation_when_normal_post_activation_time_is_available(self):
        fixture = HandoffFixture(brake_ms=800)
        fixture.at(-100, -110)
        fixture.worker.submit.assert_not_called()
        fixture.active = 5
        fixture.at(16)
        fixture.worker.submit.assert_called_once()


if __name__ == '__main__':
    unittest.main()
