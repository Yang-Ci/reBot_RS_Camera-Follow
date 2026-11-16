import threading
import unittest
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock

from robot_follow.control.gravity_sender import GravitySender


class GravitySenderTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.original = Mock()
        self.arm = SimpleNamespace(_send_mit_positions=self.original, shutdown_started=False,
            target_lock=threading.RLock(), target_positions=[0.]*7, command_positions=[0.]*7,
            motor_count=7, motors=[SimpleNamespace(send_mit=Mock()) for _ in range(7)],
            config=SimpleNamespace(motors=[SimpleNamespace(kp=50., kd=3.) for _ in range(7)]),
            _io_lock_guard=lambda timeout: nullcontext())
        self.sender = GravitySender(self.arm, clock=lambda: self.now)
        self.command = {'position':[.1]*6,'torque':[2.]*6,'kp':[8.]*6,'kd':[1.]*6}

    def test_normal_then_stale_holds_and_flags(self):
        self.sender.publish(self.command, 1.)
        self.sender.attach()
        self.arm._send_mit_positions([0.]*7)
        self.arm.motors[0].send_mit.assert_called_with(.1, 0., 8., 1., 2.)
        self.assertFalse(self.sender.fault.is_set())
        self.now = 1.31
        self.arm._send_mit_positions([0.]*7)
        self.arm.motors[0].send_mit.assert_called_with(.1, 0., 50., 3., 0.)
        self.assertTrue(self.sender.fault.is_set())

    def test_shutdown_uses_original_and_detach_restores(self):
        self.sender.publish(self.command, 1.); self.sender.attach()
        self.arm.shutdown_started = True
        self.arm._send_mit_positions([0.]*7)
        self.original.assert_called_once()
        self.sender.detach([.2]*6)
        self.assertEqual(self.arm.target_positions[:6], [.2]*6)
        self.assertEqual(self.arm.command_positions[:6], [.2]*6)
        self.assertIs(self.arm._send_mit_positions, self.original)

    def test_bad_command_never_attaches(self):
        self.command['torque'] = [float('nan')]*6
        with self.assertRaises(ValueError): self.sender.publish(self.command, 1.)
        with self.assertRaises(RuntimeError): self.sender.attach()
        self.assertIs(self.arm._send_mit_positions, self.original)


if __name__ == '__main__': unittest.main()
