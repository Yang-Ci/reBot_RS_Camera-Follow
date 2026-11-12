"""Offline checks for the extracted control and configuration modules."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from robot_follow.configuration.gripper import load_grasp_cfg
from robot_follow.control.kinematics import (
    camera_velocity_to_tcp, compute_damped_pseudoinverse,
    compute_orientation_error, skew,
)


class KinematicsTests(unittest.TestCase):
    def test_damped_inverse_handles_singular_jacobian(self):
        jacobian = np.diag([1.0, 0.0])
        result = compute_damped_pseudoinverse(jacobian, damping=0.1)
        np.testing.assert_allclose(result, np.diag([1 / 1.01, 0.0]))

    def test_orientation_error_for_quarter_turn(self):
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        np.testing.assert_allclose(compute_orientation_error(np.eye(3), rotation),
                                   [0., 0., np.pi / 2], atol=1e-12)

    def test_camera_transform_includes_mount_translation(self):
        transform = np.eye(4)
        transform[:3, 3] = [1., 0., 0.]
        velocity = np.array([0., 0., 0., 0., 0., 1.])
        np.testing.assert_allclose(camera_velocity_to_tcp(velocity, transform),
                                   [0., -1., 0., 0., 0., 1.])
        np.testing.assert_allclose(skew([1., 0., 0.]) @ [0., 0., 1.],
                                   [0., -1., 0.])


class GripperConfigurationTests(unittest.TestCase):
    def test_defaults_and_legacy_aliases(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'grasp.yaml'
            path.write_text('pinch:\n  filter_type: legacy\n  beta: 0.2\n', encoding='utf-8')
            cfg = load_grasp_cfg(path)
        self.assertEqual(cfg['pinch']['filter'], 'legacy')
        self.assertEqual(cfg['pinch']['oe_beta'], 0.2)
        self.assertEqual(cfg['gripper']['open_deg'], 293.6)


if __name__ == '__main__':
    unittest.main()
