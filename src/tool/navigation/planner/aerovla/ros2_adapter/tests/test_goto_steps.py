"""Action branch fidelity and measured completion, without replacing a Behavior."""

import math

import pytest
from scipy.spatial.transform import Rotation

from uav_aerovla_planner.goto_steps import Convergence, Step, action_steps


def state(yaw=0):
    return {'position': [10., 20., -3.],
            'orientation': Rotation.from_euler('z', yaw).as_quat()}


def test_small_turn_keeps_full_displacement_and_fixed_yaw():
    steps = action_steps(state(), 2., -.5, .1)
    assert len(steps) == 1
    step = steps[0]
    assert step.position_enu == pytest.approx([20 + 2 * math.sin(.1), 10 + 2 * math.cos(.1), 3.5])
    assert step.yaw_enu == pytest.approx(math.pi / 2 - .1)
    assert step.max_speed == 1.


def test_large_turn_rotates_before_vertical_and_drops_horizontal():
    steps = action_steps(state(), 4., 1., .5)
    assert [step.kind for step in steps] == ['rotate', 'vertical']
    assert steps[0].position_enu == (20., 10., 3.)
    assert steps[1].position_enu == (20., 10., 2.)
    assert steps[1].max_speed == 2.
    assert steps[0].yaw_enu == steps[1].yaw_enu


def test_pure_rotation_is_a_real_step_not_zero_duration_path():
    steps = action_steps(state(), 0., 0., -.4)
    assert len(steps) == 1 and steps[0].kind == 'rotate'
    assert steps[0].yaw_enu == pytest.approx(math.pi / 2 + .4)


def test_micro_move_preserves_upstream_speed_request():
    assert action_steps(state(), .2, 0., 0.)[0].max_speed == pytest.approx(.2)


def test_noop_needs_no_goto():
    assert action_steps(state(), 0., 0., 0.) == []


@pytest.mark.parametrize('bad', [math.inf, math.nan])
def test_nonfinite_action_rejected(bad):
    with pytest.raises(ValueError):
        action_steps(state(), bad, 0., 0.)


def test_native_success_with_wrong_yaw_does_not_finish():
    gate = Convergence(.2, .1, .15, .3)
    step = Step((0., 0., 0.), 1., 1., 'rotate')
    for stamp in [1., 2., 3.]:
        assert not gate.update(step, [0., 0., 0.], 0., 0., stamp, True)
    assert not gate.update(step, [0., 0., 0.], 1., 0., 4., True)
    assert gate.update(step, [0., 0., 0.], 1., 0., 4.4, True)


def test_speed_or_stale_samples_reset_stable_period():
    gate = Convergence(.2, .1, .15, .3)
    step = Step((0., 0., 0.), 0., 1., 'move')
    assert not gate.update(step, [0., 0., 0.], 0., 0., 1., True)
    assert not gate.update(step, [0., 0., 0.], 0., .4, 1.4, True)
    assert not gate.update(step, [0., 0., 0.], 0., 0., 1.5, True)
    assert not gate.update(step, [0., 0., 0.], 0., 0., 1.9, False)
    assert not gate.update(step, [0., 0., 0.], 0., 0., 2., True)
    assert gate.update(step, [0., 0., 0.], 0., 0., 2.4, True)


def test_clock_reset_and_repeated_stamp_do_not_complete():
    gate = Convergence(.2, .1, .15, .3)
    step = Step((0., 0., 0.), -math.pi, 1., 'rotate')
    for stamp in [3., 3., 1., 1.]:
        assert not gate.update(step, [0., 0., 0.], math.pi, 0., stamp, True)
    assert gate.update(step, [0., 0., 0.], math.pi, 0., 1.4, True)
