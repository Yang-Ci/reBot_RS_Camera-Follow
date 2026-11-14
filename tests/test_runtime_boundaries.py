"""Checks for runtime boundaries without camera or robot dependencies."""
import unittest
import threading
from dataclasses import FrozenInstanceError
from unittest import mock

import numpy as np

from robot_follow.control.feedback import near_limit_feedback_error, verify_home_joint_position
from robot_follow.control.state import ServoState
from robot_follow.perception.ir_filter import in_ir_gripper_region
from robot_follow.runtime.cli import PROJECT_ROOT, parse_args
from robot_follow.runtime.observation import TargetObservation
from robot_follow.runtime.control import RunControl
from robot_follow.runtime.vision import VisionOptions
from robot_follow.control.guards import pause_runtime_workspace


class RuntimeBoundaryTests(unittest.TestCase):
    def test_vision_options_are_immutable(self):
        options = VisionOptions(headless=True, web_control=False, no_arm=True,
                                follow_enabled=True, gripper_enabled=True, infrared=False,
                                desired_u=320, desired_v=240, move_speed_threshold=.05,
                                camera_hw_timestamp=False, depth_filter_cutoff=.5,
                                depth_filter_beta=.1)
        with self.assertRaises(FrozenInstanceError):
            options.headless = False

    def test_cli_defaults_and_overrides(self):
        defaults = parse_args([])
        self.assertEqual(defaults.grasp_cfg, str(PROJECT_ROOT / 'config' / 'grasp.yaml'))
        selected = parse_args(['--camera', 'd435i', '--no-arm', '--max-vel', '0.1'])
        self.assertEqual(selected.camera, 'd435i')
        self.assertTrue(selected.no_arm)
        self.assertEqual(selected.max_vel, 0.1)
        self.assertIsNone(defaults.stationary_joint_hold)
        self.assertTrue(parse_args(['--stationary-joint-hold']).stationary_joint_hold)
        self.assertFalse(parse_args(['--no-stationary-joint-hold']).stationary_joint_hold)

    def test_observations_do_not_share_mutable_state(self):
        first, second = TargetObservation(), TargetObservation()
        first.position_cam = [0.1, 0.2, 0.3]
        first.sequence += 1
        self.assertIsNone(second.position_cam)
        self.assertEqual(second.sequence, 0)

    def test_observation_publication_and_loss_behavior(self):
        observation = TargetObservation()
        track = {'x3d': .1, 'y3d': .2, 'z3d': .3, 'lost': 0, 'z3d_frozen': True}
        observation.publish(track, observed_at=1., arrived_at=1.01,
                            published_at=1.03, hardware_time=True)
        self.assertEqual(observation.position_cam, [.1, .2, .3])
        self.assertEqual(observation.sequence, 1)
        self.assertTrue(observation.frozen)
        self.assertTrue(observation.visible)
        self.assertAlmostEqual(observation.processing_ms, 20.)
        track['lost'] = 1
        observation.publish(track, observed_at=2., arrived_at=2.,
                            published_at=2., hardware_time=False)
        self.assertFalse(observation.visible)
        self.assertEqual(observation.position_cam, [.1, .2, .3])
        self.assertEqual(observation.sequence, 1)
        observation.publish(None, observed_at=3., arrived_at=3.,
                            published_at=3., hardware_time=False)
        self.assertIsNone(observation.position_cam)

    def test_servo_vector_state_is_independent_between_runs(self):
        first, second = ServoState(), ServoState()
        first.actual_velocity[0] = 1.0
        first.coast_offset[1] = 2.0
        np.testing.assert_array_equal(second.actual_velocity, np.zeros(6))
        np.testing.assert_array_equal(second.coast_offset, np.zeros(3))

    def test_home_feedback_checks_and_correction_boundary(self):
        np.testing.assert_allclose(verify_home_joint_position(np.zeros(6), np.zeros(6)), 0)
        with self.assertRaises(RuntimeError):
            verify_home_joint_position(np.ones(6), np.zeros(6))
        self.assertTrue(near_limit_feedback_error(None, 10.0, 0.08, 0.175))
        self.assertFalse(near_limit_feedback_error(
            {'time': 10.0, 'actual': [0, .079, 0, 0, 0, 0],
             'command': [0, .08, 0, 0, 0, 0]}, 10.0, .08, .175))

    def test_workspace_pause_clears_shared_follow_state_and_holds(self):
        control = RunControl(following=True)
        status = {'active': True, 'joint_speed': 1., 'error': ''}
        robot = mock.Mock()
        pause_runtime_workspace(robot, threading.Lock(), control, status, [0., 0., .66])
        self.assertFalse(control.following)
        self.assertFalse(status['active'])
        self.assertEqual(status['joint_speed'], 0.)
        self.assertIn('暂停', status['error'])
        robot.hold.assert_called_once_with()
        self.assertFalse(control.quit_requested.is_set())

    def test_quit_events_are_not_shared_between_sessions(self):
        first, second = RunControl(), RunControl()
        first.quit_requested.set()
        self.assertFalse(second.quit_requested.is_set())

    def test_ir_exclusion_boundaries(self):
        self.assertTrue(in_ir_gripper_region(200, 300, 400, 480))
        self.assertFalse(in_ir_gripper_region(100, 300, 400, 480))
        self.assertFalse(in_ir_gripper_region(None, 300, 400, 480))


if __name__ == '__main__':
    unittest.main()
