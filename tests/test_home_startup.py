"""Check HOME move options and feedback guards without robot hardware."""
import unittest
from unittest import mock

import numpy as np

from robot_follow.runtime.home import HomeOptions, initialize_home


class HomeInitializationTests(unittest.TestCase):
    def make_robot(self, initial, actual):
        robot = mock.Mock()
        robot.q_min = np.full(6, -3.)
        robot.q_max = np.full(6, 3.)
        robot.get_current_pos.side_effect = [initial, actual]
        robot.get_command_pos.return_value = initial.copy()
        robot.forward_kinematics.return_value = {'rotation': np.eye(3), 'position': np.zeros(3)}
        return robot

    def test_normal_home_move_and_orientation(self):
        home = np.zeros(6)
        robot = self.make_robot(home, home)
        options = HomeOptions(False, 0., 3.5, 8., True)
        with mock.patch('robot_follow.runtime.home.time.sleep'):
            result = initialize_home(robot, home, options, lambda *a, **k: False, mock.Mock())
        self.assertFalse(result.near_limit)
        np.testing.assert_array_equal(result.desired_rotation, np.eye(3))
        kwargs = robot.move_to_joints.call_args.kwargs
        self.assertEqual(kwargs['speed_deg_s'], 8.)
        self.assertFalse(kwargs['allow_limit_recovery'])
        np.testing.assert_array_equal(kwargs['actual_tolerance_deg'], [2., 2., 3.5, 3.5, 2., 2.])

    def test_near_limit_compensation_keeps_actual_target_and_speed_caps(self):
        home = np.zeros(6)
        home[1] = .08
        robot = self.make_robot(np.zeros(6), home)
        options = HomeOptions(True, 1., 3.5, 15., True)
        validator = mock.Mock(return_value=True)
        kinematics = mock.Mock(return_value=(np.zeros(3), np.eye(3), np.eye(6)))
        with mock.patch('robot_follow.runtime.home.time.sleep'):
            result = initialize_home(robot, home, options, validator, kinematics)
        self.assertTrue(result.near_limit)
        args, kwargs = robot.move_to_joints.call_args
        self.assertAlmostEqual(args[0][1], home[1] + np.deg2rad(1.))
        np.testing.assert_array_equal(kwargs['actual_target_rad'], home)
        self.assertEqual(kwargs['speed_deg_s'], 4.)
        np.testing.assert_array_equal(kwargs['actual_tolerance_deg'], [2., .5, 3., 3., 2., 2.])
        robot.set_joint_speed_cap.assert_called_once_with(4.)
        self.assertAlmostEqual(robot.recovery_max_speed, np.deg2rad(2.))

    def test_mismatched_starting_feedback_prevents_motion(self):
        home = np.zeros(6)
        robot = self.make_robot(home, home)
        robot.get_command_pos.return_value = np.ones(6)
        with self.assertRaisesRegex(RuntimeError, '实际位置与命令不一致'):
            initialize_home(robot, home, HomeOptions(False, 0., 2., 15., True),
                            mock.Mock(), mock.Mock())
        robot.move_to_joints.assert_not_called()


if __name__ == '__main__':
    unittest.main()
