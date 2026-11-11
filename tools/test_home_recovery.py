#!/usr/bin/env python3
"""No-hardware tests for HOME recovery near the J2 software limit."""
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebot_adapter import (RebotAdapter, clamp_recovery_target,
                           validate_home_recovery, validate_home_target,
                           load_soft_joint_limits)

class HomeRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.lo = np.deg2rad([-150,10,10,-80,-80,-170.])
        self.hi = np.deg2rad([150,170,170,80,80,170.])
        self.current = np.deg2rad([0,0,0,0,0,0.])
        self.home = np.deg2rad([-14.48,4.5,42.12,-41.66,12.68,-2.61])

    def test_only_inward_recovery_from_below_limit(self):
        a = np.deg2rad([0,4.5,20,0,0,0.])
        toward_hard_limit = a.copy(); toward_hard_limit[1] -= .04
        held = clamp_recovery_target(a,toward_hard_limit,self.lo,self.hi)
        self.assertAlmostEqual(held[1],a[1])
        inward = a.copy(); inward[1] += .02
        np.testing.assert_allclose(clamp_recovery_target(a,inward,self.lo,self.hi),inward)
        inside = a.copy(); inside[1] = np.deg2rad(11.)
        inward[1] = np.deg2rad(4.5)
        self.assertAlmostEqual(clamp_recovery_target(inside,inward,self.lo,self.hi)[1],self.lo[1])

    def test_user_candidate_only_explicit_and_from_below(self):
        with self.assertRaises(ValueError):
            validate_home_recovery(self.current,self.home,self.lo,self.hi)
        self.assertTrue(validate_home_recovery(self.current,self.home,self.lo,self.hi,enabled=True))
        above = self.current.copy(); above[1] = np.deg2rad(12.)
        with self.assertRaises(ValueError):
            validate_home_recovery(above,self.home,self.lo,self.hi,enabled=True)
        lower = self.current.copy(); lower[1] = np.deg2rad(5.)
        with self.assertRaises(ValueError):
            validate_home_recovery(lower,self.home,self.lo,self.hi,enabled=True)
        bad = self.home.copy(); bad[1] = np.deg2rad(3.)
        with self.assertRaises(ValueError):
            validate_home_recovery(self.current,bad,self.lo,self.hi,enabled=True)
        bad = self.home.copy(); bad[3] = np.deg2rad(-81.)
        with self.assertRaises(ValueError):
            validate_home_recovery(self.current,bad,self.lo,self.hi,enabled=True)

    def test_recovery_move_does_not_clip_candidate_to_ten_degrees(self):
        arm = RebotAdapter.__new__(RebotAdapter)
        arm.q_min,arm.q_max = self.lo,self.hi
        arm._q_target = self.current.copy()
        arm.io = SimpleNamespace(arm=SimpleNamespace(set_max_speeds=mock.Mock()),
                                 send_joint_positions_deg=mock.Mock(),gripper_deg=0.)
        arm.get_command_pos = mock.Mock(return_value=self.current.copy())
        arm.move_to_joints(self.home,speed_deg_s=4,wait=False,allow_limit_recovery=True)
        self.assertAlmostEqual(np.rad2deg(arm._q_target[1]),4.5)
        self.assertAlmostEqual(arm.io.send_joint_positions_deg.call_args.args[0][1],4.5)

    def test_servo_does_not_snap_or_move_outward_from_candidate(self):
        arm = RebotAdapter.__new__(RebotAdapter)
        arm.q_min, arm.q_max = self.lo, self.hi
        arm._q_target = self.home.copy()
        arm._last_cmd_time = 10.0
        arm.recovery_max_speed = np.deg2rad(2.)
        arm.io = SimpleNamespace(send_joint_positions_deg=mock.Mock(), gripper_deg=0.)
        with mock.patch('rebot_adapter.time.time', return_value=10.01):
            arm.joint_vel_cmd(np.array([0.,-1.,0.,0.,0.,0.]))
        self.assertAlmostEqual(arm._q_target[1], self.home[1])
        with mock.patch('rebot_adapter.time.time', return_value=10.02):
            arm.joint_vel_cmd(np.array([0.,1.,0.,0.,0.,0.]))
        step = arm._q_target[1] - self.home[1]
        self.assertGreater(step, 0.)
        self.assertLessEqual(step, np.deg2rad(2.) * .01 + 1e-9)

    def test_recovery_rejects_clipped_home_before_sending(self):
        arm = RebotAdapter.__new__(RebotAdapter)
        arm.q_min, arm.q_max = self.lo, self.hi
        command = self.current.copy(); command[1] = np.deg2rad(5.)
        arm._q_target = command.copy()
        arm.io = SimpleNamespace(arm=SimpleNamespace(set_max_speeds=mock.Mock()),
                                 send_joint_positions_deg=mock.Mock(), gripper_deg=0.)
        arm.get_command_pos = mock.Mock(return_value=command)
        with self.assertRaises(ValueError):
            arm.move_to_joints(self.home, wait=False, allow_limit_recovery=True,
                               require_exact_target=True)
        arm.io.send_joint_positions_deg.assert_not_called()
        arm.io.arm.set_max_speeds.assert_not_called()

    def test_home_timeout_keeps_low_speed_and_reports_both_errors(self):
        arm = RebotAdapter.__new__(RebotAdapter)
        arm.q_min, arm.q_max = self.lo, self.hi
        arm._q_target = self.current.copy()
        arm.io = SimpleNamespace(arm=SimpleNamespace(set_max_speeds=mock.Mock()),
                                 send_joint_positions_deg=mock.Mock(), gripper_deg=0.)
        arm.get_command_pos = mock.Mock(return_value=self.current.copy())
        arm.get_current_pos = mock.Mock(return_value=self.current.copy())
        with mock.patch('rebot_adapter.time.time', side_effect=[0., 0., 2.]), \
             mock.patch('rebot_adapter.time.sleep'):
            with self.assertRaisesRegex(TimeoutError, '指令误差.*实际误差'):
                arm.move_to_joints(self.home, speed_deg_s=4, timeout=1,
                                   allow_limit_recovery=True, raise_on_timeout=True)
        arm.io.arm.set_max_speeds.assert_called_once_with([4.] * 6 + [500.])

    def test_compensated_command_arrives_only_at_actual_home(self):
        arm = RebotAdapter.__new__(RebotAdapter)
        arm.q_min, arm.q_max = self.lo, self.hi
        command = self.home.copy(); command[1] += np.deg2rad(1.2)
        arm._q_target = self.current.copy()
        arm.io = SimpleNamespace(arm=SimpleNamespace(set_max_speeds=mock.Mock()),
                                 send_joint_positions_deg=mock.Mock(), gripper_deg=0.)
        arm.get_command_pos = mock.Mock(side_effect=[self.current.copy(), command.copy()])
        arm.get_current_pos = mock.Mock(return_value=self.home.copy())
        arm.move_to_joints(command, speed_deg_s=4, allow_limit_recovery=True,
                           require_exact_target=True, raise_on_timeout=True,
                           actual_tolerance_deg=[2,.5,3,3,2,2], actual_target_rad=self.home)
        self.assertAlmostEqual(arm.io.send_joint_positions_deg.call_args.args[0][1],5.7)

    def test_shutdown_rejects_home_and_aborts_arrival_wait(self):
        arm = RebotAdapter.__new__(RebotAdapter)
        arm.q_min, arm.q_max = self.lo, self.hi
        controller = SimpleNamespace(shutdown_started=True, set_max_speeds=mock.Mock())
        arm.io = SimpleNamespace(arm=controller, send_joint_positions_deg=mock.Mock(), gripper_deg=0.)
        with self.assertRaisesRegex(RuntimeError, '正在停机'):
            arm.move_to_joints(self.home)
        arm.io.send_joint_positions_deg.assert_not_called()
        controller.shutdown_started = False
        arm.get_command_pos = mock.Mock(side_effect=lambda: setattr(controller, 'shutdown_started', True) or self.current.copy())
        arm.get_current_pos = mock.Mock(return_value=self.current.copy())
        with self.assertRaisesRegex(RuntimeError, '正在停机'):
            arm.move_to_joints(self.home, timeout=1)

    def test_joint_speed_cap_applies_to_sdk_and_integration(self):
        arm = RebotAdapter.__new__(RebotAdapter)
        arm.q_min, arm.q_max = self.lo, self.hi
        arm._q_target = np.deg2rad([0,20,20,0,0,0.])
        arm._last_cmd_time = 10.0
        arm.io = SimpleNamespace(arm=SimpleNamespace(set_max_speeds=mock.Mock()),
                                 send_joint_positions_deg=mock.Mock(),gripper_deg=0.)
        arm.set_joint_speed_cap(4.0)
        self.assertEqual(arm.io.arm.set_max_speeds.call_args.args[0][:6], [4.] * 6)
        before = arm._q_target.copy()
        with mock.patch('rebot_adapter.time.time', return_value=10.01):
            arm.joint_vel_cmd(np.ones(6))
        self.assertTrue(np.all(arm._q_target-before <= np.deg2rad(4.) * .01 + 1e-9))
        with self.assertRaises(ValueError):
            arm.set_joint_speed_cap(100.)

    def test_selected_home_matches_recorded_candidate(self):
        path=Path(__file__).resolve().parent.parent/'config/home_pose_candidate_20261110_105700_272184.json'
        if path.exists():
            with path.open() as f: home=np.array(json.load(f)['home_joint_pos'])
            lo,hi=load_soft_joint_limits()
            with self.assertRaises(ValueError):
                validate_home_target(home,lo,hi)
            self.assertTrue(validate_home_target(home,lo,hi,enabled=True))
            self.assertTrue(validate_home_recovery(self.current,home,lo,hi,enabled=True))

if __name__ == '__main__':
    unittest.main()
