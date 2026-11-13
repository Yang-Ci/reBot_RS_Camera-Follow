"""Depth dropout estimates must account for wrist camera movement."""
import unittest
import numpy as np

from robot_follow.control.depth_fallback import CachedDepthPlane
from robot_follow.perception.depth import DepthGate


class CachedDepthPlaneTests(unittest.TestCase):
    def test_forward_camera_motion_changes_estimated_depth(self):
        plane = CachedDepthPlane()
        transform = np.eye(4)
        plane.remember([0, 0, .5], [0, 0, 0], np.eye(3), transform)
        np.testing.assert_allclose(plane.project([0, 0, .5], [0, 0, .1], np.eye(3), transform), [0, 0, .4])
        np.testing.assert_allclose(plane.project([-.1, 0, .5], [.1, 0, 0], np.eye(3), transform), [-.1, 0, .5])

    def test_invalid_estimates_are_rejected(self):
        plane = CachedDepthPlane()
        t = np.eye(4)
        with self.assertRaises(ValueError): plane.project([0, 0, .5], [0, 0, 0], np.eye(3), t)
        plane.remember([0, 0, .5], [0, 0, 0], np.eye(3), t)
        with self.assertRaises(ValueError): plane.project([.5, 0, .5], [0, 0, 0], np.eye(3), t)
        with self.assertRaises(ValueError): plane.project([0, 0, .5], [0, 0, .4], np.eye(3), t)
        with self.assertRaises(ValueError): plane.project([float('nan'), 0, .5], [0, 0, 0], np.eye(3), t)

    def test_depth_gate_retains_last_measured_depth_during_dropout(self):
        gate = DepthGate(desired_depth=.35, max_jump=.1, max_step=.02, freeze_seconds=.2,
                         depth_filter=lambda d: d, reuse_last_depth=True)
        track = {'lost': 0, 'u': .5, 'v': .25}
        project = lambda u, v, z: (u*z, v*z, z)
        for i in range(5): gate.update(track, .4, project, i*.01)
        gate.update(track, 0, project, 10.)
        self.assertTrue(track['z3d_frozen'])
        self.assertAlmostEqual(track['z3d'], .4)
        self.assertAlmostEqual(track['x3d'], .2)
        gate.update(None, 0, project, 11.)
        self.assertEqual(gate.reference, 0)


if __name__ == '__main__': unittest.main()
