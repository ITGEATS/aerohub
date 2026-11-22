"""Physical heading and body-axis regressions for the ROS/AirSim boundary."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from uav_aerovla_planner import policy
from uav_aerovla_planner.frames import (
    enu_ned_vector, enu_to_ned_quat, enu_to_ned_yaw,
)


@pytest.mark.parametrize('yaw_enu,yaw_ned,forward_ned', [
    (0., math.pi / 2, [0., 1., 0.]),
    (math.pi / 2, 0., [1., 0., 0.]),
    (math.pi, -math.pi / 2, [0., -1., 0.]),
    (-math.pi / 2, math.pi, [-1., 0., 0.]),
])
def test_heading_and_forward_step(yaw_enu, yaw_ned, forward_ned):
    q = enu_to_ned_quat(Rotation.from_euler('z', yaw_enu).as_quat())
    actual = policy.yaw_from_quat(q)
    assert math.remainder(actual - yaw_ned, 2 * math.pi) == pytest.approx(0., abs=1e-12)
    assert math.remainder(actual - enu_to_ned_yaw(yaw_enu), 2 * math.pi) == pytest.approx(0., abs=1e-12)
    points = policy.action_to_setpoints(
        {'position': [0., 0., 0.], 'orientation': q}, 1., 0., 0.)
    np.testing.assert_allclose(points[-1]['position'], forward_ned, atol=1e-12)


@pytest.mark.parametrize('angles', [[0., 0., 0.], [.3, -.4, .8], [-.5, .2, -1.2]])
def test_full_attitude_preserves_physical_body_axes(angles):
    ros = Rotation.from_euler('xyz', angles)
    q = ros.as_quat()
    converted = Rotation.from_quat(enu_to_ned_quat(dict(zip('xyzw', q))))
    # FRD forward/right/down correspond to FLU forward/-left/-up.
    for frd_axis, flu_axis in zip(np.eye(3), np.diag([1., -1., -1.])):
        expected = enu_ned_vector(ros.apply(flu_axis))
        np.testing.assert_allclose(converted.apply(frd_axis), expected, atol=1e-12)


@pytest.mark.parametrize('yaw', [0., .7, -1.4, math.pi])
@pytest.mark.parametrize('body_target,hint', [
    ([10., 0., 0.], 'straight ahead '),
    ([0., 10., 0.], 'to your right '),
    ([0., -10., 0.], 'to your left '),
])
def test_hint_uses_frd_right_and_left(yaw, body_target, hint):
    ros = Rotation.from_euler('z', yaw)
    # Target is specified in FRD; convert to the ROS FLU observation frame.
    flu_target = np.asarray(body_target) * [1., -1., -1.]
    target_ned = enu_ned_vector(ros.apply(flu_target))
    q = enu_to_ned_quat(ros.as_quat())
    assert policy.semantic_hint([0., 0., 0.], q, target_ned) == hint
