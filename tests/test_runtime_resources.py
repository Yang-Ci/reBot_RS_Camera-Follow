"""Verify resource cleanup on early exits and partial device failures."""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from robot_follow.runtime.resources import RuntimeResources

ROOT = Path(__file__).resolve().parents[1]


class ResourceTests(unittest.TestCase):
    def test_cleanup_continues_and_preserves_original_error(self):
        events = []
        robot = types.SimpleNamespace(
            is_stopping=lambda: False,
            hold=lambda: events.append('hold'),
            stop=lambda: events.append('stop'))
        def broken_detector():
            events.append('detector')
            raise RuntimeError('detector failed')
        detector = types.SimpleNamespace(close=broken_detector)
        camera = types.SimpleNamespace(stop=lambda: events.append('camera'))
        with mock.patch('robot_follow.runtime.resources.time.sleep'):
            with self.assertRaisesRegex(ValueError, 'original'):
                with RuntimeResources() as resources:
                    resources.robot, resources.camera, resources.detector = robot, camera, detector
                    raise ValueError('original')
        self.assertEqual(events, ['detector', 'hold', 'stop', 'camera'])
        resources.close()
        self.assertEqual(events.count('stop'), 1)

    def test_control_timeout_is_reported_and_robot_stop_is_attempted(self):
        thread = types.SimpleNamespace(ident=1, join=mock.Mock(), is_alive=lambda: True)
        robot = types.SimpleNamespace(is_stopping=lambda: True, stop=mock.Mock())
        with self.assertRaisesRegex(RuntimeError, '伺服线程仍在运行'):
            with RuntimeResources() as resources:
                resources.control_thread = thread
                resources.robot = robot
        robot.stop.assert_called_once_with()

    def test_robot_stop_failure_is_reported_and_camera_is_released(self):
        camera = types.SimpleNamespace(stop=mock.Mock())
        robot = types.SimpleNamespace(is_stopping=lambda: True,
                                      stop=mock.Mock(side_effect=RuntimeError('CAN stop failed')))
        with self.assertRaisesRegex(RuntimeError, 'CAN stop failed'):
            with RuntimeResources() as resources:
                resources.robot, resources.camera = robot, camera
        robot.stop.assert_called_once_with()
        camera.stop.assert_called_once_with()

    def test_hold_failure_still_attempts_robot_stop(self):
        robot = types.SimpleNamespace(is_stopping=lambda: False,
                                      hold=mock.Mock(side_effect=RuntimeError('hold failed')),
                                      stop=mock.Mock())
        with self.assertRaisesRegex(RuntimeError, '保持机械臂'):
            with RuntimeResources() as resources:
                resources.robot = robot
        robot.stop.assert_called_once_with()


class AdapterInitializationTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('adapter_for_cleanup_test', ROOT / 'rebot_adapter.py')
        self.adapter = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'pinocchio': types.ModuleType('pinocchio')}):
            spec.loader.exec_module(self.adapter)

    def test_sdk_start_failure_requests_return_to_zero(self):
        arm = mock.Mock()
        arm.start.side_effect = RuntimeError('connection failed')
        sdk = types.SimpleNamespace(ReBotRSMITController=lambda: arm)
        with mock.patch.dict(sys.modules, {'rebot': sdk}):
            with self.assertRaisesRegex(RuntimeError, 'connection failed'):
                self.adapter._ArmIO()
        arm.stop.assert_called_once_with(return_to_zero=True, wait=True)

    def test_speed_setup_failure_requests_return_to_zero(self):
        arm = mock.Mock()
        arm.set_max_speeds.side_effect = RuntimeError('speed setup failed')
        sdk = types.SimpleNamespace(ReBotRSMITController=lambda: arm)
        with mock.patch.dict(sys.modules, {'rebot': sdk}):
            with self.assertRaisesRegex(RuntimeError, 'speed setup failed'):
                self.adapter._ArmIO()
        arm.stop.assert_called_once_with(return_to_zero=True, wait=True)

    def test_stop_error_is_reported_to_caller(self):
        io = self.adapter._ArmIO.__new__(self.adapter._ArmIO)
        io.arm = mock.Mock()
        io.arm.stop.side_effect = RuntimeError('CAN stop failed')
        with self.assertRaisesRegex(RuntimeError, 'CAN stop failed'):
            io.stop()
        io.arm.stop.assert_called_once_with(return_to_zero=True, wait=True)

    def test_start_and_stop_failures_remain_visible(self):
        arm = mock.Mock()
        arm.start.side_effect = RuntimeError('start failed')
        arm.stop.side_effect = RuntimeError('stop failed')
        sdk = types.SimpleNamespace(ReBotRSMITController=lambda: arm)
        with mock.patch.dict(sys.modules, {'rebot': sdk}):
            with self.assertRaisesRegex(RuntimeError, '回零失能也失败') as raised:
                self.adapter._ArmIO()
        self.assertIn('stop failed', str(raised.exception))
        self.assertIn('start failed', str(raised.exception.__cause__.__context__))

    def test_first_feedback_failure_stops_initialized_io(self):
        model = mock.Mock()
        model.existJointName.return_value = True
        model.getJointId.return_value = 0
        model.joints = [types.SimpleNamespace(idx_q=0)]
        model.lowerPositionLimit = np.array([-3.])
        model.upperPositionLimit = np.array([3.])
        io = mock.Mock()
        io.read_joint_positions_deg.side_effect = RuntimeError('CAN feedback failed')
        self.adapter.pin.buildModelFromUrdf = mock.Mock(return_value=model)
        with mock.patch.object(self.adapter, '_ArmIO', return_value=io):
            with self.assertRaisesRegex(RuntimeError, 'CAN feedback failed'):
                self.adapter.RebotAdapter()
        io.stop.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
