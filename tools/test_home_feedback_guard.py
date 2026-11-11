#!/usr/bin/env python3
"""Offline tests for mandatory near-limit J2 feedback protection."""
import sys
import types
import unittest
import threading
from unittest import mock
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
sys.modules['pynput'] = types.SimpleNamespace(keyboard=types.SimpleNamespace())
from palm_follow_grasp import near_limit_feedback_error, verify_home_joint_position, feedback_corrected_joints, near_limit_j2_floor, pause_outside_workspace

class GuardTests(unittest.TestCase):
    def test_workspace_breach_pauses_and_holds_without_return_motion(self):
        robot = mock.Mock()
        following = [True]
        status = {'active': True, 'joint_speed': 1.0, 'error': ''}
        pause_outside_workspace(robot, threading.Lock(), following, status, [0, 0, .66])
        self.assertFalse(following[0])
        self.assertFalse(status['active'])
        self.assertEqual(status['joint_speed'], 0.0)
        self.assertIn('暂停', status['error'])
        robot.hold.assert_called_once_with()
        robot.move_to_joints.assert_not_called()

    def test_missing_stale_and_nonfinite_feedback_stops(self):
        self.assertTrue(near_limit_feedback_error(None,10.,.08,.175))
        self.assertTrue(near_limit_feedback_error({'time':8.4},10.,.08,.175))
        self.assertTrue(near_limit_feedback_error({'time':10.,'actual':[0,np.nan],'command':[0,.08]},10.,.08,.175))
        self.assertTrue(near_limit_feedback_error({'time':10.,'read_ms':1600},10.,.08,.175))
    def test_joint_margin_and_tracking_error_stop(self):
        good={'time':10.,'actual':[0,.079,0,0,0,0],'command':[0,.08,0,0,0,0]}
        self.assertFalse(near_limit_feedback_error(good,10.,.08,.175))
        below={'time':10.,'actual':[0,.06,0,0,0,0],'command':[0,.08,0,0,0,0]}
        self.assertTrue(near_limit_feedback_error(below,10.,.08,.175))
        bad_tracking={'time':10.,'actual':[0,.082,0,0,0,0],'command':[0,.12,0,0,0,0]}
        self.assertTrue(near_limit_feedback_error(bad_tracking,10.,.08,.175))
        bad_other={'time':10.,'actual':[0,.08,0,0,0,0],'command':[0,.08,0,.07,0,0]}
        self.assertTrue(near_limit_feedback_error(bad_other,10.,.08,.175))
    def test_position_estimate_corrects_command_with_measured_error(self):
        snap={'actual':np.deg2rad([0,4.5,40,0,0,0]),
              'command':np.deg2rad([0,5.5,42,0,0,0])}
        now=np.deg2rad([0,6,43,0,0,0])
        np.testing.assert_allclose(np.rad2deg(feedback_corrected_joints(now,snap)),
                                   [0,5,41,0,0,0],atol=1e-10)
    def test_floor_never_below_four_degree_model_margin(self):
        self.assertAlmostEqual(np.rad2deg(near_limit_j2_floor(np.deg2rad(4),np.deg2rad(10))),4)
    def test_j2_near_limit_arrival_tolerance(self):
        target=np.zeros(6)
        actual=np.deg2rad([0,.49,1.99,0,0,0])
        verify_home_joint_position(actual,target,[2,.5,2,2,2,2])
        actual[1]=np.deg2rad(.51)
        with self.assertRaises(RuntimeError):
            verify_home_joint_position(actual,target,[2,.5,2,2,2,2])
if __name__=='__main__':unittest.main()
