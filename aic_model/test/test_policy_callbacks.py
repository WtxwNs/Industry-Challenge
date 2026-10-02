"""Dependency-free unit checks of policy callbacks; no ROS or robot is started."""

import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace
import unittest
import threading
from unittest.mock import Mock


# Compile only the callback methods under test. This keeps the tests runnable
# without generated ROS messages while exercising the actual method bodies.
source_path = Path(__file__).parents[1] / "aic_model" / "aic_model.py"
source = ast.parse(source_path.read_text())
model_class = next(node for node in source.body if isinstance(node, ast.ClassDef))
method_names = {
    "insert_cable_goal_callback",
    "policy_goal_is_active",
    "move_robot_for_goal",
    "send_feedback_for_goal",
    "action_thread_func",
    "release_goal",
    "insert_cable_execute_callback",
}
model_class.bases = []
model_class.body = [
    node for node in model_class.body if getattr(node, "name", None) in method_names
]


class InlineThread:
    def __init__(self, target, kwargs):
        self.target = target
        self.kwargs = kwargs

    def start(self):
        self.target(**self.kwargs)

    def is_alive(self):
        return False


namespace = {
    "threading": SimpleNamespace(Thread=InlineThread),
    "rclpy": SimpleNamespace(ok=lambda: True),
    "Future": asyncio.Future,
    "InsertCable": SimpleNamespace(Result=SimpleNamespace),
    "ServerGoalHandle": object,
    "GoalResponse": SimpleNamespace(REJECT=False, ACCEPT=True),
}
exec(
    compile(ast.Module(body=[model_class], type_ignores=[]), str(source_path), "exec"),
    namespace,
)
AicModel = namespace["AicModel"]


class PolicyCallbackTests(unittest.TestCase):
    def setUp(self):
        self.node = AicModel()
        self.goal = SimpleNamespace(
            is_active=True,
            is_cancel_requested=False,
            request=SimpleNamespace(task=object()),
        )
        self.node.goal_handle = self.goal
        self.node._goal_lock = threading.Lock()
        self.node._goal_pending = False
        self.node.is_active = True
        self.node._action_thread = None
        self.node.move_robot = Mock(return_value=True)
        self.node.send_feedback = Mock()
        self.node.get_logger = Mock(return_value=Mock())
        self.node.observation_callable = Mock()

    def test_active_goal_forwards_motion_and_feedback(self):
        command = object()
        self.assertTrue(self.node.move_robot_for_goal(self.goal, command))
        self.node.move_robot.assert_called_once_with(command, None)
        self.node.send_feedback_for_goal(self.goal, "progress")
        self.node.send_feedback.assert_called_once_with(self.goal, "progress")

    def test_inactive_canceled_and_replaced_goals_cannot_send_callbacks(self):
        for changed_attribute, value in (
            ("is_active", False),
            ("is_cancel_requested", True),
        ):
            setattr(self.goal, changed_attribute, value)
            self.assertFalse(self.node.move_robot_for_goal(self.goal, object()))
            self.node.send_feedback_for_goal(self.goal, "progress")
            setattr(self.goal, changed_attribute, not value)
        self.node.goal_handle = object()
        self.assertFalse(self.node.move_robot_for_goal(self.goal, object()))
        self.node.send_feedback_for_goal(self.goal, "progress")
        self.node.goal_handle = self.goal
        self.node.is_active = False
        self.assertFalse(self.node.move_robot_for_goal(self.goal, object()))
        self.node.send_feedback_for_goal(self.goal, "progress")
        self.node.move_robot.assert_not_called()
        self.node.send_feedback.assert_not_called()

    def test_running_previous_thread_prevents_new_goal(self):
        self.node.goal_handle = None
        self.node._action_thread = Mock()
        self.node._action_thread.is_alive.return_value = True
        self.assertFalse(self.node.insert_cable_goal_callback(object()))
        self.node._action_thread.is_alive.return_value = False
        self.assertTrue(self.node.insert_cable_goal_callback(object()))

    def test_accepted_goal_reserves_slot_until_callback(self):
        self.node.goal_handle = None
        self.assertTrue(self.node.insert_cable_goal_callback(object()))
        self.assertFalse(self.node.insert_cable_goal_callback(object()))

    def test_finishing_old_goal_does_not_clear_a_new_goal(self):
        other_goal = object()
        self.node.goal_handle = other_goal
        self.node.release_goal(self.goal)
        self.assertIs(self.node.goal_handle, other_goal)
        self.node.release_goal(other_goal)
        self.assertIsNone(self.node.goal_handle)

    def test_policy_exception_produces_false_result(self):
        self.node._policy = Mock()
        self.node._policy.insert_cable.side_effect = RuntimeError("policy failed")
        self.node.action_thread_func(self.goal)
        self.assertIs(self.node._action_thread_result, False)

    def test_policy_results_choose_matching_action_status(self):
        for outcome in (True, False, None, RuntimeError("policy failed")):
            with self.subTest(outcome=outcome):
                self.node.goal_handle = self.goal
                self.goal.succeed = Mock()
                self.goal.abort = Mock()
                self.node._policy = Mock()
                if isinstance(outcome, Exception):
                    self.node._policy.insert_cable.side_effect = outcome
                else:
                    self.node._policy.insert_cable.return_value = outcome
                self.node.get_clock = Mock()
                self.node.destroy_timer = Mock()

                def create_timer(period, callback, clock):
                    callback()
                    return Mock()

                self.node.create_timer = create_timer
                result = asyncio.run(self.node.insert_cable_execute_callback(self.goal))
                self.assertIs(result.success, outcome is True)
                if outcome is True:
                    self.goal.succeed.assert_called_once()
                    self.goal.abort.assert_not_called()
                else:
                    self.goal.abort.assert_called_once()
                    self.goal.succeed.assert_not_called()
                self.assertIsNone(self.node.goal_handle)

    def test_callbacks_kept_by_policy_remain_bound_to_original_goal(self):
        self.node._policy = Mock()
        self.node._policy.insert_cable.return_value = True
        self.node.action_thread_func(self.goal)
        callbacks = self.node._policy.insert_cable.call_args.kwargs
        self.node.goal_handle = object()
        self.assertFalse(callbacks["move_robot"](motion_update=object()))
        callbacks["send_feedback"]("old progress")
        self.node.move_robot.assert_not_called()
        self.node.send_feedback.assert_not_called()
        self.assertIs(self.node._action_thread_result, True)


if __name__ == "__main__":
    unittest.main()
