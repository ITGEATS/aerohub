"""Check the decision boundary independently of GPU availability or motion."""
import importlib.util
from pathlib import Path
import unittest

from uav_inference_interfaces.srv import InferFlowPilot
from uav_flowpilot_planner.lifecycle import set_time

path = Path(__file__).resolve().parents[2]/'airsim/demos/flowpilot_single/preflight.py'
spec = importlib.util.spec_from_file_location('flowpilot_preflight_check', path)
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class PreflightDecisionTest(unittest.TestCase):
    def setUp(self):
        self.request = InferFlowPilot.Request(request_id='gate')
        set_time(self.request.depth.header.stamp, 10.)
        set_time(self.request.observation_header.stamp, 10.)
        self.response = InferFlowPilot.Response(
            success=True, request_id='gate', observation_header=self.request.observation_header,
            waypoints=[0.]*576, control_points=[0.]*24)
        self.context = dict(request=self.request, observation=10., epoch=10.2)

    def assess(self, now=10.08, elapsed=.08, response=None):
        return preflight.assess(response or self.response, self.context, now, elapsed, .1, .3, .05)

    def test_valid_trajectory_must_also_fit_the_time_budget(self):
        healthy = self.assess()
        self.assertTrue(all(healthy[k] for k in ['response_valid', 'deadline_met', 'observation_age_met', 'submission_reserve_met']))
        late = self.assess(now=10.21, elapsed=.21)
        self.assertTrue(late['response_valid'])
        self.assertFalse(late['deadline_met'])
        self.assertFalse(late['submission_reserve_met'])

    def test_frozen_observation_must_remain_fresh_at_response(self):
        self.context['epoch'] = 10.5
        result = self.assess(now=10.31, elapsed=.08)
        self.assertTrue(result['deadline_met'])
        self.assertFalse(result['observation_age_met'])
        self.assertTrue(result['submission_reserve_met'])

    def test_response_identity_and_finite_values_are_required(self):
        self.response.request_id = 'other'
        self.assertFalse(self.assess()['response_valid'])
        self.response.request_id = 'gate'
        self.response.waypoints[575] = float('nan')
        self.assertFalse(self.assess()['response_valid'])

    def test_missing_response_is_a_failure(self):
        result = preflight.assess(None, self.context, 10.1, .1, .1, .3, .05)
        self.assertFalse(result['response_valid'])
        self.assertEqual(result['message'], 'ROS response timeout')


if __name__ == '__main__':
    unittest.main()
