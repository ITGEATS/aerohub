"""Tool lifecycle boundary tests in the Humble environment."""
import json
import time
import unittest
from concurrent.futures import Future
from unittest.mock import Mock

try:
    from uav_planner_framework.tool_execution import ToolExecution
    from uav_planner_framework.ros_node import PlannerFrameworkNode
    from uav_planner_framework.behavior import BehaviorStatus, TransitionError
except ImportError:
    ToolExecution = None


@unittest.skipIf(ToolExecution is None, 'requires ROS')
class ToolExecutionTest(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.tool = ToolExecution(self.node, '123-456')
        self.tool.cancel.call_async.return_value = Future()

    def status(self, **overrides):
        value = dict(task_id='123-456', state='STOPPED', phase='IDLE', stop_confirmed=True, generation=1)
        value.update(overrides)
        self.tool._status(Mock(data=json.dumps(value)))

    def test_ack_alone_and_foreign_feedback_cannot_confirm_stop(self):
        self.tool.stop()
        self.status()
        self.assertFalse(self.tool.quiescent)
        future = Future()
        future.set_result(Mock(success=True))
        self.tool.stop_future = future
        self.assertFalse(self.tool.quiescent)
        self.status(task_id='another-task')
        self.assertFalse(self.tool.quiescent)
        self.status()
        self.assertTrue(self.tool.quiescent)

    def test_rejected_stop_and_malformed_status_do_not_unlock(self):
        self.tool.stop()
        future = Future()
        future.set_result(Mock(success=False))
        self.tool.stop_future = future
        self.status()
        self.tool._status(Mock(data='[]'))
        self.tool._status(Mock(data='invalid json'))
        self.assertFalse(self.tool.quiescent)
        with self.assertRaises(ValueError):
            self.tool.resume()

    def test_resume_clears_old_stop_confirmation(self):
        self.tool.stop()
        future = Future()
        future.set_result(Mock(success=True))
        self.tool.stop_future = future
        self.status()
        self.tool.resume()
        self.assertFalse(self.tool.quiescent)
        self.assertIsNone(self.tool.status)
        self.status()  # late feedback cannot relatch without a new stop request
        self.assertFalse(self.tool.quiescent)

    def test_previous_stop_feedback_cannot_confirm_a_new_cancel(self):
        self.status(generation=1)
        self.tool.stop()
        future = Future()
        future.set_result(Mock(success=True))
        self.tool.stop_future = future
        self.status(generation=1)
        self.assertFalse(self.tool.quiescent)
        self.status(generation=2)
        self.assertTrue(self.tool.quiescent)

    def test_land_success_is_valid_tool_feedback(self):
        self.status(state='SUCCEEDED', phase='SUCCEEDED', land_success=True, stop_confirmed=False)
        self.assertTrue(self.tool.status['land_success'])
        self.assertFalse(self.tool.quiescent)

    def test_only_successful_land_result_completes_framework_task(self):
        node = Mock(tool_execution=self.tool)
        node.lifecycle.snapshot.status = BehaviorStatus.RUNNING
        node.lifecycle.snapshot.task_id = '123-456'
        node.lifecycle.snapshot.failure_reason = 'NONE'
        node.tool_resume_future = None
        node.planner_process.poll.return_value = None
        node._active_time.return_value = 1.0
        node.get_parameter.return_value.value = 0.0
        node.resolved = {'vehicles': ['drone0'], 'completion': {'task_timeout': 100.0}}
        self.tool.last_received = time.monotonic()
        for status in (dict(state='RUNNING', phase='LANDING'),
                       dict(state='SUCCEEDED', phase='SUCCEEDED', land_success=False)):
            self.tool.status = status
            PlannerFrameworkNode._tick_tool(node)
            node.lifecycle.succeed.assert_not_called()
        self.tool.status = dict(state='SUCCEEDED', phase='SUCCEEDED', land_success=True)
        PlannerFrameworkNode._tick_tool(node)
        node.lifecycle.succeed.assert_called_once()
        self.assertTrue(self.tool.stop_requested)

    def test_start_cannot_replace_unconfirmed_tool(self):
        node = Mock(tool_execution=Mock(quiescent=False))
        with self.assertRaisesRegex(TransitionError, 'Tool stop confirmation'):
            PlannerFrameworkNode._start(node, 'unused-task', 'unused-deployment')
        node._stop_runtime.assert_not_called()

    def test_failure_requests_stop_without_killing_tool(self):
        node = Mock(tool_execution=self.tool)
        PlannerFrameworkNode._fail(node, 'TASK_TIMEOUT')
        self.assertTrue(self.tool.stop_requested)
        node.lifecycle.fail.assert_called_once_with('TASK_TIMEOUT')
        node._stop_planner_adapter.assert_not_called()

    def test_terminal_task_waits_for_stop_before_process_exit(self):
        node = Mock(tool_execution=self.tool)
        node.lifecycle.snapshot.status = BehaviorStatus.CANCELED
        node.lifecycle.snapshot.task_id = '123-456'
        node.lifecycle.snapshot.failure_reason = 'NONE'
        node.resolved = {'vehicles': ['drone0']}
        self.tool.stop()
        PlannerFrameworkNode._tick_tool(node)
        node._stop_planner_adapter.assert_not_called()
        self.tool.stop_confirmed = True
        PlannerFrameworkNode._tick_tool(node)
        node._stop_planner_adapter.assert_called_once()
