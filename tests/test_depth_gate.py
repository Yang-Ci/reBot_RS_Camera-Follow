"""Depth gate regression checks, independent of camera SDKs."""
import unittest

from robot_follow.perception.depth import DepthGate


def project(u, v, depth):
    return u * depth, v * depth, depth


def track(lost=0):
    return {'u': 0.1, 'v': 0.2, 'lost': lost, 'x3d': 0., 'y3d': 0., 'z3d': 0.}


class DepthGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = DepthGate(desired_depth=.35, max_jump=.08, max_step=.05,
                              freeze_seconds=.3, depth_filter=lambda value: value)

    def acquire(self):
        for i in range(5):
            observation = track()
            self.gate.update(observation, .4, project, 1 + .01 * i)
        return observation

    def test_five_consistent_frames_required_and_background_rejected(self):
        for i in range(4):
            observation = track()
            self.gate.update(observation, .4, project, 1 + .01 * i)
            self.assertEqual(observation['z3d'], 0)
        observation = track()
        self.gate.update(observation, .4, project, 1.04)
        self.assertAlmostEqual(observation['z3d'], .4)
        self.gate.reset_reference()
        for i in range(6):
            observation = track()
            self.gate.update(observation, .8, project, 2 + .01 * i)
            self.assertEqual(observation['z3d'], 0)
        self.assertEqual(self.gate.reacquisition, [])

    def test_short_invalid_depth_freezes_and_then_expires(self):
        self.acquire()
        observation = track()
        self.gate.update(observation, 0., project, 1.1)
        self.assertTrue(observation['z3d_frozen'])
        self.assertEqual(observation['z3d'], .35)
        self.assertAlmostEqual(observation['x3d'], .04)
        self.gate.update(observation, 0., project, 1.5)
        self.assertFalse(observation['z3d_frozen'])
        self.assertEqual(observation['z3d'], 0.)

    def test_jump_requires_15_frames_and_still_limits_step(self):
        self.acquire()
        for i in range(14):
            observation = track()
            self.gate.update(observation, .7, project, 1.05 + .01 * i)
            self.assertTrue(observation['z3d_frozen'])
            self.assertAlmostEqual(self.gate.reference, .4)
        observation = track()
        self.gate.update(observation, .7, project, 1.20)
        self.assertFalse(observation['z3d_frozen'])
        self.assertAlmostEqual(observation['z3d'], .45)

    def test_short_loss_retains_reference_long_loss_resets(self):
        self.acquire()
        self.gate.update(track(lost=2), 0, project, 2.)
        self.assertAlmostEqual(self.gate.reference, .4)
        self.gate.update(track(lost=30), 0, project, 2.)
        self.assertEqual(self.gate.reference, 0)
        self.assertEqual(self.gate.rejected_frames, 0)

    def test_diagnostics_distinguish_invalid_depth_jump_and_tracking_loss(self):
        observation = track()
        self.gate.update(observation, .4, project, 1.)
        self.assertEqual(observation['depth_status'], '建立深度参考')
        self.acquire()
        observation = track()
        self.gate.update(observation, 0., project, 1.1)
        self.assertEqual(observation['depth_status'], '深度样本无效')
        observation = track()
        self.gate.update(observation, .7, project, 1.11)
        self.assertEqual(observation['depth_status'], '深度跳变拒绝')
        self.assertAlmostEqual(observation['depth_raw_m'], .7)
        observation = track(lost=1)
        self.gate.update(observation, 0., project, 1.12)
        self.assertEqual(observation['depth_status'], '追踪丢失')


if __name__ == '__main__':
    unittest.main()
