"""Exercise the actual facade state machine without ROS transport or flight.

The class is compiled from its real AST; generated ROS bindings are replaced
only at the boundary. These are unit checks, never DDS acceptance evidence.
"""
import ast
import copy
import math
from pathlib import Path
import threading
from concurrent.futures import Future
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'src/tool/navigation/planner/flowpilot/ros2_adapter/uav_flowpilot_planner/navigation.py'


def pose(x=0.):
    return NS(header=NS(frame_id='drone0/odom', stamp=NS(sec=100, nanosec=0)),
              pose=NS(position=NS(x=x, y=0., z=0.)))


tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
scope = dict(copy=copy, math=math, threading=threading, PoseStamped=pose,
             Future=Future, GoalResponse=NS(ACCEPT=1, REJECT=0),
             NavigateToPose=NS(Result=lambda **kw: NS(**kw), Feedback=lambda **kw: NS(**kw)))
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)], type_ignores=[]), str(SOURCE), 'exec'), scope)
Facade = scope['FlowPilotNavigation']


def request():
    return NS(target=pose(0.), UNCONSTRAINED=0, COMPLETE=0, HOLD=1,
              orientation_mode=0, arrival_behavior=0, use_target_timeout=False,
              use_timeout=False, use_max_speed=False, use_position_tolerance=False)


class NavigationTest(unittest.TestCase):
    def setUp(self):
        self.node = NS(frame='drone0/odom', age=.3, max_speed=2., state='IDLE', reason='',
                       queued_goal=None, goal=[0., 0., 0.], pose=pose(), timeline=NS(revision=2),
                       now_s=lambda: 100., observations_ready=lambda _: True, ready=lambda: True,
                       inputs_fresh=lambda _: True)
        def begin(target):
            self.node.state = 'RUNNING'
        def stop(reason):
            self.node.state, self.node.reason = 'STOPPING', reason
        self.node.begin, self.node.stop = begin, stop
        self.facade = Facade.__new__(Facade)
        self.facade.node, self.facade.lock, self.facade.reserved = self.node, threading.Lock(), False

    def active(self, goal=None):
        handle = NS(request=goal or request(), is_cancel_requested=False, goal_id='owner',
                    succeed=lambda: None, abort=lambda: None, canceled=lambda: None,
                    publish_feedback=lambda _: None)
        f = self.facade
        f.handle, f.finished, f.pending, f.started = handle, Future(), True, False
        f.deadline_reason = f.terminal_reason = None
        f.tick()
        return f

    def test_invalid_and_unsupported_do_not_start(self):
        for change, code in [({'orientation_mode': 1}, 'UNSUPPORTED'),
                             ({'arrival_behavior': 1}, 'UNSUPPORTED'),
                             ({'use_max_speed': True, 'max_speed': float('nan')}, 'INVALID_GOAL'),
                             ({'use_position_tolerance': True, 'position_tolerance': 1.}, 'UNSUPPORTED')]:
            goal = request()
            for k, v in change.items(): setattr(goal, k, v)
            self.assertEqual(self.facade.validate(goal)[0], code)
            self.assertEqual(self.node.state, 'IDLE')

    def test_busy_reservation(self):
        self.assertEqual(self.facade.accept(request()), 1)
        self.assertEqual(self.facade.accept(request()), 0)

    def test_final_radius_and_freshness(self):
        for x, sec, passed in [(4.9999, 100, True), (5.0001, 100, False), (1., 90, False)]:
            self.node.state = 'IDLE'
            f = self.active()
            self.node.state, self.node.reason = 'STOPPED', 'ARRIVED_5M'
            self.node.pose = pose(x)
            self.node.pose.header.stamp.sec = sec
            f.tick()
            self.assertEqual(f.finished.result().success, passed)

    def test_cancel_overrides_arrival_while_stopping(self):
        f = self.active()
        self.node.state, self.node.reason = 'STOPPING', 'ARRIVED_5M'
        f.handle.is_cancel_requested = True
        f.tick()
        self.node.state = 'STOPPED'
        f.tick()
        self.assertEqual(f.finished.result().code, 'CANCELED')

    def test_timeout_and_unconfirmed_stop(self):
        f = self.active()
        self.node.now_s = lambda: 500.
        f.tick()
        self.assertEqual(self.node.reason, 'NAVIGATION_TIMEOUT')
        self.node.state, self.node.reason = 'FAULT', 'STOP_TIMEOUT'
        f.tick()
        self.assertEqual(f.finished.result().code, 'STOP_UNCONFIRMED')

    def test_non_owner_update_cannot_change_goal(self):
        f = self.active()
        response = NS()
        f.update_target(NS(goal_id='other'), response)
        self.assertEqual(response.code, 'NOT_OWNER')
        self.assertEqual(self.node.goal, [0., 0., 0.])


class MissionBoundaryTest(unittest.TestCase):
    def load_class(self, relative, name):
        tree = ast.parse((ROOT/relative).read_text(encoding='utf-8'))
        namespace = dict(Node=object, time=NS(monotonic=lambda: 100.))
        exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef) and n.name==name], type_ignores=[]), relative, 'exec'), namespace)
        return namespace[name]

    def test_offboard_cache_must_be_fresh(self):
        cls=self.load_class('deployment/airsim/demos/flowpilot_single/mission.py','Mission')
        node=cls.__new__(cls);node.platform=NS(offboard=False);node.platform_at=99.9
        def wait(predicate, timeout):
            if not predicate(): raise TimeoutError()
        node.wait=wait
        self.assertTrue(node.offboard_state_released(1))
        node.platform_at=90.
        self.assertFalse(node.offboard_state_released(1))

    def test_pose_requires_receipt_and_measurement_freshness(self):
        cls=self.load_class('deployment/airsim/demos/flowpilot_single/mission.py','Mission')
        node=cls.__new__(cls);node.pose=pose();node.pose_at=99.9
        node.get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000))
        self.assertTrue(node.fresh_pose())
        node.pose_at=99.;self.assertFalse(node.fresh_pose())
        node.pose_at=99.9;node.pose.header.stamp.sec=101
        self.assertFalse(node.fresh_pose())

    def test_private_cancel_cannot_bypass_public_owner(self):
        cls=self.load_class('src/tool/navigation/planner/flowpilot/ros2_adapter/uav_flowpilot_planner/lifecycle.py','FlowPilotLifecycle')
        node=cls.__new__(cls);node.navigation=object();response=NS()
        node.cancel(None,response)
        self.assertFalse(response.success)


if __name__ == '__main__':
    unittest.main()
