#!/usr/bin/env python3
"""Offline tests for camera identity and calibration provenance."""
import json
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.modules['pynput'] = types.SimpleNamespace(keyboard=types.SimpleNamespace())
from palm_follow_grasp import (verify_orbbec_camera, load_camera_follow_cfg,
                               validate_follow_cfg, apply_stationary_joint_hold)


class Gemini336Tests(unittest.TestCase):
    def setUp(self):
        self.camera = SimpleNamespace(device_name='Orbbec Gemini 336', camera_serial='serial-336',
                                      width=640, height=480, camera_matrix=np.eye(3),
                                      dist_coeffs=np.array([.01, -.04, .001, -.002, .03]))
        self.calibration = {'camera': 'Orbbec Gemini 336', 'camera_serial': 'serial-336',
                            'image_stream': 'color', 'image_size': [640, 480],
                            'camera_matrix': self.camera.camera_matrix.tolist(),
                            'dist_coeffs': self.camera.dist_coeffs.tolist(),
                            'joint_offset_rad': json.loads((ROOT/'config/joint_offset.json').read_text())['joint_offset_rad']}

    def test_reject_other_camera_and_old_calibration(self):
        with self.assertRaises(ValueError):
            verify_orbbec_camera(self.camera, 'gemini2', {}, require_calibration=True)
        self.camera.device_name = 'Orbbec Gemini 2'
        with self.assertRaises(ValueError):
            verify_orbbec_camera(self.camera, 'gemini336', self.calibration, require_calibration=True)

    def test_only_matching_336_calibration_is_accepted(self):
        verify_orbbec_camera(self.camera, 'gemini336', self.calibration, require_calibration=True)
        for key, bad in [('camera_serial', 'other'), ('image_stream', 'infrared'),
                         ('image_size', [1280, 720]), ('camera_matrix', np.zeros((3,3)).tolist()),
                         ('dist_coeffs', [0]*5), ('joint_offset_rad', [0]*6)]:
            data = dict(self.calibration); data[key] = bad
            with self.subTest(key=key), self.assertRaises(ValueError):
                verify_orbbec_camera(self.camera, 'gemini336', data, require_calibration=True)

    def test_preview_does_not_require_calibration(self):
        verify_orbbec_camera(self.camera, 'gemini336', None, require_calibration=False)

    def test_profile_preserves_speed_and_is_not_applied_to_other_cameras(self):
        config_dir = str(ROOT / 'config')
        base = load_camera_follow_cfg('gemini2', None, config_dir)
        tuned = load_camera_follow_cfg('gemini336', None, config_dir)
        explicit = load_camera_follow_cfg('gemini336', str(ROOT / 'config/follow.yaml'), config_dir)
        validate_follow_cfg(tuned)
        self.assertEqual(base, explicit)
        self.assertEqual(base['follow']['max_vel'], tuned['follow']['max_vel'])
        self.assertEqual(base['workspace'], tuned['workspace'])
        self.assertGreater(tuned['follow']['kf_ff_deadband_off'], base['follow']['kf_ff_deadband_off'])
        self.assertLess(tuned['follow']['kf_lead_max_ms'], base['follow']['kf_lead_max_ms'])
        self.assertTrue(tuned['follow']['stationary_joint_hold'])
        self.assertFalse(base['follow']['stationary_joint_hold'])

    def test_stationary_lock_holds_once_and_releases_on_motion(self):
        robot = mock.Mock()
        state = [False]
        velocity = np.ones(6)
        np.testing.assert_array_equal(apply_stationary_joint_hold(robot, True, True, state, velocity), np.zeros(6))
        np.testing.assert_array_equal(apply_stationary_joint_hold(robot, True, True, state, velocity), np.zeros(6))
        robot.hold.assert_called_once_with()
        np.testing.assert_array_equal(apply_stationary_joint_hold(robot, True, False, state, velocity), velocity)
        self.assertFalse(state[0])
        apply_stationary_joint_hold(robot, True, True, state, velocity)
        self.assertEqual(robot.hold.call_count, 2)
        np.testing.assert_array_equal(apply_stationary_joint_hold(robot, False, True, state, velocity), velocity)


if __name__ == '__main__':
    unittest.main()
