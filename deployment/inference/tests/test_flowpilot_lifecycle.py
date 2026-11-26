"""Exercise FlowPilot lifecycle methods with Humble nodes and real ROS futures."""
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from as2_msgs.msg import ControlMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from uav_inference_interfaces.srv import InferFlowPilot
from uav_flowpilot_planner.node import FlowPilotPlanner
from uav_flowpilot_planner.lifecycle import set_time
from uav_flowpilot_planner.curve import Segment


class LifecycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Isolate original lifecycle semantics; the public facade has separate tests.
        self.node = FlowPilotPlanner(parameter_overrides=[Parameter('navigation_action', value=False)])
        self.node.now_s = lambda: 100.
        self.node.inputs_fresh = lambda _: True
        self.node.pose, self.node.twist = PoseStamped(), TwistStamped()
        self.fresh_state(100.)
        goal = PoseStamped()
        goal.pose.position.x = 15.
        self.node.begin(goal)

    def tearDown(self):
        self.node.discard_inference()
        self.node.destroy_node()

    def fresh_state(self, epoch):
        set_time(self.node.pose.header.stamp, epoch)
        set_time(self.node.twist.header.stamp, epoch)
        self.node.mode = ControlMode.HOVER
        self.node.mode_at = epoch

    def pending_request(self):
        future = self.node.infer_client.call_async(InferFlowPilot.Request())
        self.node.inference = dict(future=future, generation=self.node.generation,
                                   expired=False, sent=time.monotonic()-1.)
        return future

    def test_first_inference_timeout_can_reach_stopped(self):
        future = self.pending_request()
        self.node.tick()
        self.assertEqual(self.node.state, 'STOPPING')
        self.assertEqual(self.node.reason, 'INFERENCE_TIMEOUT')
        self.assertTrue(future.cancelled())
        self.assertIsNone(self.node.inference)
        self.assertNotIn(future, self.node.infer_client._pending_requests.values())
        self.node.stop_ack = True
        self.node.stop_tick(100.)
        self.fresh_state(101.01)
        self.node.stop_tick(101.01)
        self.assertEqual(self.node.state, 'STOPPED')

    def test_cancel_detaches_request_and_invalidates_generation(self):
        future = self.pending_request()
        generation = self.node.generation
        self.node.stop('CANCELED')
        self.assertTrue(future.cancelled())
        self.assertGreater(self.node.generation, generation)
        self.assertIsNone(self.node.inference)

    def test_initial_plan_deadline_without_any_request(self):
        self.node.first_plan_deadline = time.monotonic()-1
        self.node.tick()
        self.assertEqual(self.node.reason, 'INITIAL_PLAN_TIMEOUT')
        self.assertEqual(self.node.state, 'STOPPING')

    def test_arrival_trigger_leaves_brake_margin_inside_radius(self):
        # 2026-11-24 stop review: flights settled 5.03-5.10 m from the goal
        # because ARRIVED_5M triggered on the moving 5 m crossing and the
        # hover loop freezes wherever braking ends. The trigger must fire
        # inside the radius by the brake margin so the settled position
        # stays within 5 m.
        self.node.arrival_margin = .5
        self.node.pose.pose.position.x = 15. - 4.7
        self.node.tick()
        self.assertEqual(self.node.state, 'RUNNING')
        self.node.pose.pose.position.x = 15. - 4.4
        self.node.tick()
        self.assertEqual(self.node.state, 'STOPPING')
        self.assertEqual(self.node.reason, 'ARRIVED_5M')

    def test_goal_altitude_tracking_lifts_descending_plan(self):
        # The model commands descent even with the goal above; points 3..7 must
        # ride a reference easing from the start altitude to the goal z within
        # the vertical rate, and may only exceed it by the climb margin.
        self.node.goal = np.array([0., 15., 2.])
        world = np.tile(np.array([0., 0., .5]), (8, 1))
        world[:, 1] = np.arange(8) * .2
        info = self.node.track_altitude(world, 1.)
        self.assertTrue(info['raised'])
        self.assertTrue(np.all(np.diff(world[3:, 2]) >= -1e-9))
        self.assertTrue(np.all(world[3:, 2] >= .5 - 1e-9))
        self.assertLess(world[7, 2], 2.)  # rate-limited inside one horizon
        self.assertAlmostEqual(world[7, 2], info['reference_end'], places=2)  # diagnostic is rounded to mm
        high = np.tile(np.array([0., 0., 3.]), (8, 1))
        info_high = self.node.track_altitude(high, 1.)
        # Starting above the goal: the reference eases DOWN from the start
        # altitude (rate-limited); the plan may not stay above it, so a model
        # that refuses to descend is pulled onto the reference.
        self.assertTrue(np.all(high[3:, 2] <= 3. + 1e-9))
        self.assertTrue(np.all(high[3:, 2] >= 2. - 1e-9))
        self.assertTrue(2. <= high[7, 2] < 3.)
        self.assertAlmostEqual(high[7, 2], info_high['reference_end'], places=2)
        flat = np.tile(np.array([0., 0., .5]), (8, 1))
        self.node.track_altitude_enabled = False
        self.node.track_altitude(flat, 1.)
        np.testing.assert_allclose(flat[:, 2], .5)

    def test_duplicate_and_reversed_images_then_clock_reset(self):
        image = Image()
        image.header.frame_id = self.node.optical_frame
        set_time(image.header.stamp, 99.9)
        self.node.on_depth(image)
        for epoch in (99.9, 99.8):
            old = Image()
            old.header.frame_id = self.node.optical_frame
            set_time(old.header.stamp, epoch)
            self.node.on_depth(old)
            self.assertIs(self.node.depth, image)
        self.node.last_clock = 101.
        self.node.tick()
        self.assertEqual(self.node.reason, 'CLOCK_RESET')
        self.assertIsNone(self.node.depth)
        self.assertIsNone(self.node.last_image)

    def test_pose_and_twist_use_the_same_measurement_epoch(self):
        self.node.depth = Image()
        set_time(self.node.depth.header.stamp, 100.)
        self.node.pose_history.append(self.node.pose)
        mismatch = TwistStamped()
        set_time(mismatch.header.stamp, 100.02)
        self.node.twist_history.append(mismatch)
        self.assertIsNone(self.node.observation_pair())
        self.node.twist_history.append(self.node.twist)
        self.assertEqual(self.node.observation_pair(), (self.node.pose, self.node.twist))

    def test_discontinuous_candidate_keeps_accepted_revision(self):
        old = Segment(np.zeros((8, 3)), np.zeros(3), np.eye(3), 99.7, 99.9, 0.)
        self.node.timeline.propose(old)
        self.node.timeline.acknowledge(True)
        request = InferFlowPilot.Request(request_id='handoff')
        set_time(request.depth.header.stamp, 100.)
        set_time(request.observation_header.stamp, 100.)
        response = InferFlowPilot.Response(success=True, request_id=request.request_id,
                                           observation_header=request.observation_header,
                                           control_points=np.ones(24).tolist())
        context = dict(future=Mock(result=lambda: response), request=request,
                       observation=100., epoch=100.2, origin=np.zeros(3),
                       rotation=np.eye(3), yaw=0., sent=time.monotonic(),
                       velocity=np.zeros(3), acceleration=np.zeros(3))
        self.node.submit = Mock()
        with patch('uav_flowpilot_planner.node.time.monotonic', return_value=context['sent']):
            self.node.inferred(context, 100.)
        self.node.submit.assert_not_called()
        self.assertIs(self.node.timeline.latest, old)
        self.assertEqual(self.node.timeline.revision, 1)
        self.assertIsNone(self.node.timeline.pending)
        response.control_points = np.zeros(24).tolist()
        with patch('uav_flowpilot_planner.node.time.monotonic', return_value=context['sent']):
            self.node.inferred(context, 100.)
        self.node.submit.assert_called_once()

    def test_lagged_candidate_is_spliced_onto_the_reference(self):
        # 2026-11-24 4090 flight: a healthy model candidate replans from the
        # measured (lagging) state, so its raw start trails the plan
        # prediction at the takeover epoch by v*(lead+age) (0.21 m here).
        # The raw gate must judge the anchor against the measured origin,
        # not that displacement, and the splice must still land the
        # submitted curve exactly on the reference PVA.
        speed = np.array([1.1, 0., 0.])
        controls = np.outer(np.arange(8) / 7 * 1.6, speed)
        old = Segment(controls + np.array([0., 0., 1.]), np.zeros(3), np.eye(3), 99.9, 99.9, 0.)
        self.node.timeline.propose(old)
        self.node.timeline.acknowledge(True)
        request = InferFlowPilot.Request(request_id='handoff')
        set_time(request.depth.header.stamp, 100.)
        set_time(request.observation_header.stamp, 100.)
        response = InferFlowPilot.Response(success=True, request_id=request.request_id,
                                           observation_header=request.observation_header,
                                           control_points=controls.flatten().tolist())
        context = dict(future=Mock(result=lambda: response), request=request,
                       observation=100., epoch=100.2, origin=np.array([-.1, 0., 1.]),
                       rotation=np.eye(3), yaw=0., sent=time.monotonic(),
                       velocity=speed, acceleration=np.zeros(3))
        self.node.submit = Mock()
        with patch('uav_flowpilot_planner.node.time.monotonic', return_value=context['sent']):
            self.node.inferred(context, 100.)
        self.node.submit.assert_called_once()
        curve = self.node.submit.call_args[0][0]
        reference = old.execution_sample(100.2)
        pva, yaw = curve.sample(curve.start)
        np.testing.assert_allclose(pva, reference[0], atol=1e-9)
        self.assertAlmostEqual(yaw, reference[1])

    def test_candidate_ignoring_measured_velocity_is_rejected(self):
        # The warp stretches a candidate's time axis including its start, so
        # a warped start velocity must not be the gate: a candidate that
        # anchors its raw start at the measured velocity is spliceable even
        # when the executing plan predicts a different velocity. A candidate
        # whose raw start ignores the measured velocity (here 0.9 m/s off) is
        # a contract violation and must be rejected before splicing.
        speed = np.array([1.1, 0., 0.])
        old = Segment(np.outer(np.arange(8) / 7 * 1.6, speed), np.zeros(3), np.eye(3), 99.9, 99.9, 0.)
        self.node.timeline.propose(old)
        self.node.timeline.acknowledge(True)
        request = InferFlowPilot.Request(request_id='handoff')
        set_time(request.depth.header.stamp, 100.)
        set_time(request.observation_header.stamp, 100.)
        controls = np.outer(np.arange(8) / 7 * 1.6, np.array([2., 0., 0.]))
        response = InferFlowPilot.Response(success=True, request_id=request.request_id,
                                           observation_header=request.observation_header,
                                           control_points=controls.flatten().tolist())
        context = dict(future=Mock(result=lambda: response), request=request,
                       observation=100., epoch=100.2, origin=np.array([-.1, 0., 1.]),
                       rotation=np.eye(3), yaw=0., sent=time.monotonic(),
                       velocity=speed, acceleration=np.zeros(3))
        self.node.submit = Mock()
        with patch('uav_flowpilot_planner.node.time.monotonic', return_value=context['sent']):
            self.node.inferred(context, 100.)
        self.node.submit.assert_not_called()
        self.assertIs(self.node.timeline.latest, old)
        self.assertEqual(self.node.timeline.revision, 1)


if __name__ == '__main__':
    unittest.main()
