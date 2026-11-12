"""Hardware kinematics can be tested without loading the robot SDK."""
import sys
import types
import unittest
from unittest import mock

import numpy as np

from robot_follow.hardware.kinematics import tcp_pose_and_jacobian


class HardwareKinematicsTests(unittest.TestCase):
    def test_tcp_offset_and_jacobian_use_joint_columns(self):
        pin = types.ModuleType('pinocchio')
        pin.forwardKinematics = mock.Mock()
        pin.updateFramePlacements = mock.Mock()
        pin.computeJointJacobians = mock.Mock()
        pin.ReferenceFrame = types.SimpleNamespace(LOCAL_WORLD_ALIGNED='world')
        pin.getJointJacobian = mock.Mock(return_value=np.eye(6))
        placement = types.SimpleNamespace(rotation=np.eye(3), translation=np.array([1., 2., 3.]))
        model = types.SimpleNamespace(getJointId=lambda name: int(name),
                                      joints=[types.SimpleNamespace(idx_v=i) for i in range(6)])
        data = types.SimpleNamespace(oMi=[placement] * 6)
        robot = types.SimpleNamespace(model=model, data=data, joint_names=[str(i) for i in range(6)],
                                      _build_q_full=lambda joints: joints)
        with mock.patch.dict(sys.modules, {'pinocchio': pin}):
            position, rotation, jacobian = tcp_pose_and_jacobian(
                robot, np.zeros(6), np.array([.1, 0., 0.]))
        np.testing.assert_allclose(position, [1.1, 2., 3.])
        np.testing.assert_allclose(rotation, np.eye(3))
        expected = np.eye(6)
        expected[1, 5] = .1
        expected[2, 4] = -.1
        np.testing.assert_allclose(jacobian, expected)
        pin.getJointJacobian.assert_called_once_with(model, data, 5, 'world')


if __name__ == '__main__':
    unittest.main()
