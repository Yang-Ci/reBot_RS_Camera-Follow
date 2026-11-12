"""Gripper mapping tests cover smoothing, hold, deadband and commit semantics."""
import math
import unittest

from robot_follow.control.gripper import GripperMapper


def mapper(**overrides):
    params = dict(closed_px=20., open_px=200., closed_deg=0., open_deg=100.,
                  initial_deg=0., filter_mode='ema', smoothing=.5, deadband=2.,
                  hold_while_moving=True, euro_filter=lambda value: value)
    params.update(overrides)
    return GripperMapper(**params)


class GripperMapperTests(unittest.TestCase):
    def test_clamp_and_ema(self):
        controller = mapper()
        controller.update_target(300.)
        self.assertEqual(controller.candidate(False), 50.)
        controller.committed(50.)
        controller.update_target(-1.)
        self.assertEqual(controller.candidate(False), 25.)

    def test_freeze_continues_filter_but_does_not_commit(self):
        controller = mapper()
        controller.update_target(200.)
        self.assertIsNone(controller.candidate(True))
        self.assertTrue(controller.frozen)
        self.assertEqual(controller.last_sent, 0.)
        controller.update_target(200.)
        self.assertEqual(controller.candidate(False), 75.)
        controller.committed(75.)
        self.assertEqual(controller.last_sent, 75.)

    def test_deadband_and_invalid_target_do_not_create_command(self):
        controller = mapper(smoothing=1.)
        controller.update_target(21.)
        self.assertIsNone(controller.candidate(False))
        controller.update_target(math.nan)
        self.assertIsNone(controller.candidate(False))

    def test_euro_filter_is_primed_with_current_angle(self):
        calls = []
        controller = mapper(initial_deg=42., filter_mode='one_euro',
                            euro_filter=lambda value: calls.append(value) or value)
        controller.update_target(200.)
        self.assertEqual(calls, [42., 100.])
        self.assertEqual(controller.candidate(False), 100.)


if __name__ == '__main__':
    unittest.main()
