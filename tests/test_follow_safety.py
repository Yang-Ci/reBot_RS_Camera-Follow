"""Hardware-free checks for the tabletop guard and deliberate startup."""
import math
import queue
import threading
import time
import types
import unittest
from unittest import mock

import numpy as np

from robot_follow.configuration.follow import FollowConfiguration
from robot_follow.control.components import ServoComponents
from robot_follow.control.guards import apply_stationary_joint_hold, pause_runtime_workspace
from robot_follow.control.servo import ServoContext, ServoSettings, run_servo_loop
from robot_follow.control.table_safety import feedback_error, limit_downward_velocity
from robot_follow.perception.start_gesture import OpenPalmStartGate, is_open_palm
from robot_follow.runtime.control import RunControl
from robot_follow.runtime.observation import TargetObservation
from robot_follow.runtime.telemetry import PhaseTelemetry


class GestureTests(unittest.TestCase):
    @staticmethod
    def open_hand():
        points = [types.SimpleNamespace(x=.5, y=.7) for _ in range(21)]
        for x, mcp in zip((.32, .43, .54, .65), (5, 9, 13, 17)):
            for index, y in ((mcp, .7), (mcp+1, .55), (mcp+2, .43), (mcp+3, .30)):
                points[index] = types.SimpleNamespace(x=x, y=y)
        points[4] = types.SimpleNamespace(x=.1, y=.5)
        return points

    def test_open_hand_requires_real_extended_fingers(self):
        points = self.open_hand()
        self.assertTrue(is_open_palm(points, 640, 480))
        points[8] = types.SimpleNamespace(x=.32, y=.69)
        self.assertFalse(is_open_palm(points, 640, 480))
        self.assertFalse(is_open_palm(None, 640, 480))

    def test_three_seconds_of_continuous_fresh_valid_frames(self):
        gate = OpenPalmStartGate()
        for i in range(30):
            self.assertFalse(gate.update(i*.1, eligible=True, opened=True))
        self.assertTrue(gate.update(3., eligible=True, opened=True))
        self.assertFalse(gate.update(3.1, eligible=True, opened=True))
        gate.reset()
        for i in range(20):
            self.assertFalse(gate.update(i*.1, eligible=True, opened=True))
        self.assertFalse(gate.update(2., eligible=False, opened=True))
        self.assertFalse(gate.update(5., eligible=True, opened=True))
        self.assertFalse(gate.update(5.3, eligible=True, opened=True))


class TableGuardTests(unittest.TestCase):
    def test_feedback_requires_fresh_finite_and_matching_angles(self):
        sample = {'time': 10., 'read_ms': 30., 'actual': np.zeros(6), 'command': np.zeros(6)}
        self.assertEqual(feedback_error(sample, 10.2), '')
        self.assertIn('过期', feedback_error(sample, 10.3))
        self.assertIn('缺失', feedback_error(None, 10.2))
        self.assertEqual(feedback_error({**sample, 'actual': np.deg2rad([39., 0, 0, 0, 0, 0])}, 10.2), '')
        self.assertIn('偏差', feedback_error({**sample, 'actual': np.deg2rad([41., 0, 0, 0, 0, 0])}, 10.2))
        self.assertIn('偏差', feedback_error({**sample, 'actual': np.ones(6)}, 10.2))
        self.assertIn('无效', feedback_error({**sample, 'actual': [float('nan')]*6}, 10.2))
        self.assertIn('过期', feedback_error({**sample, 'time': 11.}, 10.2))

    def test_guard_slows_downward_and_stops_below_limit(self):
        speed, stop = limit_downward_velocity(-.1, .265, .265)
        self.assertFalse(stop)
        self.assertAlmostEqual(speed, -.05)
        self.assertEqual(limit_downward_velocity(.1, .265, .265), (.1, False))
        self.assertLessEqual(abs(limit_downward_velocity(-.7, .3, .3)[0]), .07 / .6 + 1e-9)
        self.assertEqual(limit_downward_velocity(-.1, .3, .23), (0., True))
        with self.assertRaises(ValueError):
            limit_downward_velocity(-.1, float('nan'), .3)
        with self.assertRaises(ValueError):
            limit_downward_velocity(-.1, .3, float('nan'))


class FakeRobot:
    def __init__(self):
        self.holds = 0
        self.commands = []
        self.q = np.zeros(6)
        self.q_min = np.full(6, -3.)
    def hold(self): self.holds += 1
    def get_command_pos(self): return self.q.copy()
    def is_stopping(self): return False
    def joint_vel_cmd(self, q): self.commands.append(np.array(q))


class FakeFeedback:
    def __init__(self, robot): self.robot, self.available = robot, True
    def snapshot(self):
        if not self.available: return None
        return {'time': time.monotonic(), 'read_ms': 1.,
                'actual': self.robot.q.copy(), 'command': self.robot.q.copy()}


class ServoSafetyIntegrationTests(unittest.TestCase):
    def setUp(self):
        cfg = FollowConfiguration()
        parts = ServoComponents.from_configuration(cfg)
        self.robot = FakeRobot()
        self.feedback = FakeFeedback(self.robot)
        self.height = .31
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.control = RunControl()
        self.observation = TargetObservation(position_cam=[.03, 0, .35], visible=True,
                                             sequence=1, observed_at=time.monotonic())
        self.commands = queue.SimpleQueue()
        self.status = {'joint_speed': 0., 'error': '', 'active': False}
        self.context = ServoContext(
            stop_ctrl=self.stop, web_control=True, web_commands=self.commands,
            lock=self.lock, control=self.control, follow_enabled=True, robot=self.robot,
            servo_state=parts.state, observation=self.observation, near_limit_home=False,
            feedback_sampler=self.feedback, home_joint_pos=np.zeros(6), control_status=self.status,
            axis_q_gate=parts.axis_noise, kf=parts.target_filter, fast_start=parts.fast_start,
            slow_follow=parts.slow_follow, desired_depth=.35, lead_gate=parts.lead,
            t_tcp_camera=np.eye(4), lambda_gain_xyz=np.ones(3), desired_rotation=np.eye(3),
            max_vel_cart=.1, static_vel_hold=parts.static_hold, stationary_joint_hold=True,
            stationary_joint_hold_active=[False], diag_recorder=None, live_publisher=None,
            feedback_hz=20, compute_tcp_fk_and_jacobian=lambda robot, q:(np.array([.1,0,self.height]), np.eye(3), np.eye(6)),
            pause_outside_workspace=pause_runtime_workspace, apply_stationary_joint_hold=apply_stationary_joint_hold,
            table_safety_enabled=True)
        self.thread = threading.Thread(target=run_servo_loop,
                                       args=(self.context, ServoSettings.from_configuration(cfg)))
        self.thread.start()

    def tearDown(self):
        self.stop.set()
        self.thread.join(2)
        self.assertFalse(self.thread.is_alive())

    def wait_for(self, predicate):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if predicate(): return
            time.sleep(.005)
        self.fail(f'timed out: following={self.control.following} status={self.status}')

    def refresh(self, visible=True, frozen=False):
        with self.lock:
            self.observation.visible = visible
            self.observation.frozen = frozen
            self.observation.observed_at = time.monotonic()
            self.observation.sequence += 1

    def test_hand_loss_keeps_follow_enabled_and_accepts_new_position(self):
        self.refresh(); self.commands.put(True)
        self.wait_for(lambda: self.status['active'])
        with self.lock:
            self.observation.position_cam = None
        self.refresh(visible=False)
        time.sleep(.35)
        self.assertTrue(self.control.following)
        self.assertGreater(self.robot.holds, 0)
        with self.lock:
            self.observation.position_cam = [.15, 0., .35]
        self.refresh()
        self.wait_for(lambda: self.status['active'])
        self.assertTrue(self.control.following)

    def test_frozen_depth_does_not_trigger_added_loss_gate(self):
        self.refresh(); self.commands.put(True)
        self.wait_for(lambda: self.status['active'])
        for _ in range(12):
            self.refresh(frozen=True); time.sleep(.025)
        self.assertTrue(self.control.following)
        self.assertTrue(self.status['active'])
        self.assertNotIn('深度估算', self.status['error'])

    def test_stale_observation_holds_without_locking_follow(self):
        self.refresh(); self.commands.put(True)
        self.wait_for(lambda: self.status['active'])
        with self.lock:
            self.observation.observed_at = time.monotonic() - 1.
        time.sleep(.35)
        self.assertTrue(self.control.following)
        self.assertFalse(self.status['active'])
        self.refresh()
        self.wait_for(lambda: self.status['active'])
        self.assertTrue(self.control.following)

    def test_feedback_loss_still_latches_even_when_hand_is_lost(self):
        self.refresh(); self.commands.put(True)
        self.wait_for(lambda: self.status['active'])
        self.refresh(visible=False)
        self.feedback.available = False
        self.wait_for(lambda: not self.control.following and '反馈' in self.status['error'])
        self.feedback.available = True
        self.refresh(); time.sleep(.03)
        self.assertFalse(self.control.following)

    def test_invalid_target_never_commands_motion(self):
        with self.lock:
            self.observation.position_cam = [float('nan'), 0., .35]
        self.refresh(); self.commands.put(True)
        self.wait_for(lambda: not self.control.following and '无效' in self.status['error'])
        self.assertFalse(self.robot.commands)

    def test_startup_gesture_may_start_only_before_a_pause(self):
        self.refresh(); self.commands.put('gesture')
        self.wait_for(lambda: self.control.following)
        self.commands.put(False)
        self.wait_for(lambda: not self.control.following)
        self.refresh(); self.commands.put('gesture')
        time.sleep(.04)
        self.assertFalse(self.control.following)

    def test_floor_boundary_holds_without_joint_command(self):
        self.height = .22
        self.refresh(); before = len(self.robot.commands); self.commands.put(True)
        self.wait_for(lambda: not self.control.following and '桌面' in self.status['error'])
        self.assertEqual(len(self.robot.commands), before)
        self.assertGreater(self.robot.holds, 0)

    def test_hold_failure_latches_visible_fault_without_restarting(self):
        rows = []
        self.context.live_publisher = types.SimpleNamespace(due=lambda _now: False, offer=rows.append)
        with mock.patch.object(self.robot, 'hold', side_effect=RuntimeError('CAN failure')):
            self.wait_for(lambda: '保持失败' in self.status['error'])
            self.wait_for(lambda: bool(rows))
            self.assertFalse(self.control.following)
            self.assertFalse(self.control.gesture_start_allowed)
            self.assertTrue(self.control.servo_failed)
            self.assertTrue(self.thread.is_alive())
            self.commands.put(True)
            time.sleep(.04)
            self.assertFalse(self.control.following)
            from web.telemetry import state_from_row
            row = {key: str(value) for key, value in rows[-1].items()}
            state = state_from_row('.', row, 0.)
            self.assertTrue(state['servo_failed'])
            self.assertIn('硬件急停', state['error'])


class PhaseTelemetryTests(unittest.TestCase):
    def test_home_and_return_use_labeled_command_angles(self):
        rows = []
        robot = FakeRobot()
        publisher = types.SimpleNamespace(offer=rows.append)
        phase = PhaseTelemetry(robot, publisher, period=.005).start()
        try:
            phase.set_phase('home'); time.sleep(.03)
            phase.set_phase(None); time.sleep(.02)
            boundary = len(rows); time.sleep(.02)
            self.assertEqual(len(rows), boundary)
            robot.q[:] = 1.
            phase.set_phase('return'); time.sleep(.03)
        finally:
            phase.close()
        self.assertTrue(any(r['phase'] == 'home' for r in rows))
        returns = [r for r in rows if r['phase'] == 'return']
        self.assertTrue(returns)
        self.assertEqual(returns[-1]['q_cmd_1'], 1.)
        self.assertEqual(returns[-1]['fb_valid'], 0)


if __name__ == '__main__':
    unittest.main()
