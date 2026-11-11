#!/usr/bin/env python3
"""Offline guards for ROS calibration bridge; no ROS node or robot is started."""
import contextlib
import io
import json
import math
from types import SimpleNamespace
from unittest import TestCase, main, mock
import ros_calibration_bridge as bridge


class Tests(TestCase):
    def setUp(self):
        self.arm = SimpleNamespace(pending=None, goal_handle=None, closing=False,
                                   positions=[0.] * 6, ready=lambda: None,
                                   action=mock.Mock())
        self.arm.action.server_is_ready.return_value = True
        self.arm.action.send_goal_async.return_value = 'pending'

    def command(self, command):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            bridge.Bridge.command(self.arm, command)
        return json.loads(out.getvalue()) if out.getvalue() else None

    def test_rejects_stale_start_and_large_step_before_action(self):
        for target, expected in [([0.]*6, [math.radians(1)]*6),
                                 ([math.radians(8)]*6, [0.]*6)]:
            result = self.command({'op': 'move', 'target': target, 'expected': expected, 'duration': 4})
            self.assertFalse(result['ok'])
            self.arm.action.send_goal_async.assert_not_called()

    def test_valid_goal_and_close(self):
        self.command({'op': 'move', 'target': [0.01]*6, 'expected': [0.]*6, 'duration': 1.})
        self.arm.action.send_goal_async.assert_called_once()
        goal = self.arm.action.send_goal_async.call_args.args[0]
        self.assertEqual(goal.trajectory.joint_names, bridge.NAMES)
        self.assertEqual(list(goal.trajectory.points[0].positions), [0.01]*6)
        self.command({'op': 'close'})
        self.assertTrue(self.arm.closing)

    def test_close_cancels_active_goal(self):
        result_future = mock.Mock()
        result_future.done.return_value = False
        handle = mock.Mock()
        arm = SimpleNamespace(pending=None, goal_handle=handle, result=result_future,
                              closing=True, cancel_sent=False, deadline=0.)
        bridge.Bridge.tick(arm)
        handle.cancel_goal_async.assert_called_once()
        self.assertTrue(arm.cancel_sent)

if __name__ == '__main__':
    main()
