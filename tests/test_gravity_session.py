import threading
import unittest
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np

from robot_follow.runtime.gravity_session import GravitySession


class FakeRobot:
    def __init__(self):
        self.q = np.zeros(6)
        self.bad = False
        self.arm = SimpleNamespace(_send_mit_positions=Mock(), shutdown_started=False,
            target_lock=threading.RLock(), target_positions=[0.]*7, command_positions=[0.]*7,
            motor_count=7, motors=[SimpleNamespace(send_mit=Mock()) for _ in range(7)],
            config=SimpleNamespace(motors=[SimpleNamespace(kp=50., kd=3.) for _ in range(7)]),
            _io_lock_guard=lambda timeout: nullcontext())
        self.io = SimpleNamespace(arm=self.arm)
        self.joint_names = [f'joint{i}' for i in range(6)]
        self.model = SimpleNamespace(effortLimit=np.array([36.]*3 + [14.]*3),
            joints=[SimpleNamespace(idx_v=i) for i in range(6)],
            getJointId=lambda name: int(name[-1]))
        self.q_min, self.q_max = np.full(6, -2.), np.full(6, 2.)
        self.stop = Mock()
    def get_paced_feedback(self, event):
        if self.bad: raise TimeoutError('CAN lost')
        return self.q.copy(), self.q.copy()
    def is_stopping(self): return False
    def gravity_torque(self, q): return np.array([0., 1., 2., 1., 0., 0.])
    def tcp_pose_and_jacobian(self, q, offset): return np.array([.1, .1, .3]), None, np.eye(6)


class GravitySessionTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.time = 1.
        self.publisher = SimpleNamespace(offer=Mock())
        self.session = GravitySession(self.robot, self.publisher, clock=lambda: self.time)

    def test_commands_and_safe_handoff(self):
        original = self.robot.arm._send_mit_positions
        self.session.start()
        self.assertIsNot(self.robot.arm._send_mit_positions, original)
        self.time += .05
        self.robot.q[0] = .01
        self.session.step()
        self.robot.arm._send_mit_positions([0.]*7)
        self.assertEqual(self.robot.arm.motors[0].send_mit.call_count, 1)
        self.session.stop()
        self.assertIs(self.robot.arm._send_mit_positions, original)
        self.assertAlmostEqual(self.robot.arm.command_positions[0], .01)
        self.robot.stop.assert_called_once()

    def test_capture_ready_requires_a_window_of_stable_feedback(self):
        self.session.start()
        for _ in range(14):
            self.time += .05
            self.session.step()
        row = self.publisher.offer.call_args.args[0]
        self.assertEqual(row['gravity_state'], 'LOCKED')
        self.assertEqual(row['home_ready'], 1)
        self.robot.q[0] = .02
        self.time += .05
        self.session.step()
        self.assertEqual(self.publisher.offer.call_args.args[0]['home_ready'], 0)

    def test_feedback_failure_latches_fault_without_fabricating_stop_pose(self):
        self.session.start()
        self.robot.bad = True
        self.time += .05
        with self.assertRaises(TimeoutError): self.session.step()
        self.assertTrue(self.session.sender.fault.is_set())
        self.robot.arm._send_mit_positions([0.]*7)
        self.assertEqual(self.robot.arm.motors[0].send_mit.call_args.args[-1], 0.)
        with self.assertRaises(TimeoutError): self.session.stop()
        self.assertTrue(self.session.attached)
        self.robot.stop.assert_not_called()

    def test_torque_limit_failure_never_attaches(self):
        self.robot.gravity_torque = lambda q: np.full(6, 100.)
        with self.assertRaisesRegex(ValueError, '力矩'): self.session.start()
        self.assertFalse(self.session.attached)

    def test_teaching_ignores_soft_limits_and_height_in_both_directions(self):
        self.robot.q_min[1:3] = np.deg2rad(10.)
        self.robot.tcp_pose_and_jacobian = lambda q, offset: (
            np.array([.9, .9, .1]), None, np.eye(6))
        self.session.start()
        self.time += .05
        self.robot.q[1:3] = np.deg2rad(5.)
        self.session.step()
        self.time += .05
        self.robot.q[1:3] = np.deg2rad(-3.)
        self.session.step()
        for _ in range(18):
            self.time += .05
            self.session.step()
        self.assertEqual(self.publisher.offer.call_args.args[0]['home_ready'], 1)


if __name__ == '__main__': unittest.main()
