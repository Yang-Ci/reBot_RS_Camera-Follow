"""Hardware-free checks for the Gemini 305 centered mount and identity."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from robot_follow.perception.calibration import verify_orbbec_camera
from robot_follow.configuration.motion import resolve_motion_options
from urdf_camera_pose import gemini305_nominal_tcp_camera


class Gemini305Tests(unittest.TestCase):
    def test_auto_exposure_is_restored_only_for_305_auto_mode(self):
        import importlib.util
        import sys
        import types
        from unittest import mock
        sdk = types.ModuleType('pyorbbecsdk')
        for name in ('Pipeline', 'Config', 'OBSensorType', 'OBAlignMode', 'OBFormat', 'OBError'):
            setattr(sdk, name, mock.Mock())
        sdk.OBPropertyID = types.SimpleNamespace(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL='auto_exposure')
        spec = importlib.util.spec_from_file_location('gemini305_exposure_test', Path(__file__).resolve().parents[1] / 'gemini2_camera.py')
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'pyorbbecsdk': sdk}):
            spec.loader.exec_module(module)
            device = mock.Mock()
            module.enable_gemini305_auto_exposure(device, 'Orbbec Gemini 305', 0)
            device.set_bool_property.assert_called_once_with('auto_exposure', True)
            device.reset_mock()
            module.enable_gemini305_auto_exposure(device, 'Orbbec Gemini 305', 6)
            module.enable_gemini305_auto_exposure(device, 'Orbbec Gemini 336', 0)
            device.set_bool_property.assert_not_called()

    def test_camera_identity_mismatch_is_rejected(self):
        camera = SimpleNamespace(device_name='Orbbec Gemini 305')
        verify_orbbec_camera(camera, 'gemini305', None, require_calibration=False)
        with self.assertRaises(ValueError):
            verify_orbbec_camera(camera, 'gemini336', None, require_calibration=False)
        with self.assertRaises(ValueError):
            verify_orbbec_camera(camera, 'gemini2', {}, require_calibration=True)
        camera.device_name = 'Orbbec Gemini 336'
        with self.assertRaises(ValueError):
            verify_orbbec_camera(camera, 'gemini305', None, require_calibration=False)

    def test_nominal_optical_axes_and_tcp_offset(self):
        urdf = '''<robot name="test"><link name="joint6_link"/><link name="gripper_end"/>
          <joint name="joint6" type="revolute"><parent link="base"/><child link="joint6_link"/></joint>
          <joint name="wrist" type="fixed"><parent link="joint6_link"/><child link="gripper_end"/>
          <origin xyz="0 0 0.05"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'arm.urdf'
            path.write_text(urdf)
            transform = gemini305_nominal_tcp_camera(path, [.01, 0, 0])
        self.assertTrue(np.allclose(transform[:3, 3], [-.09247829, -.00036825, .12237040]))
        np.testing.assert_allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-7)
        np.testing.assert_allclose(transform[:3, 2], [.99974245, -.01932233, -.01190336], atol=1e-6)
        self.assertAlmostEqual(np.linalg.det(transform[:3, :3]), 1.)

    def test_v1_mount_uses_same_mechanical_alignment_as_336(self):
        import xml.etree.ElementTree as ET
        root = Path(__file__).resolve().parents[1] / 'models/wrist_camera'
        model305 = ET.parse(root / 'gemini305_centered.urdf').getroot()
        model336 = ET.parse(root / 'gemini336.urdf').getroot()
        for expression in ("joint[@name='gripper_to_mount']/origin", "link[@name='camera_mount_link']/visual/origin"):
            first, second = model305.find(expression), model336.find(expression)
            self.assertIsNotNone(first)
            self.assertIsNotNone(second)
            np.testing.assert_allclose([float(x) for x in first.get('xyz').split()],
                                       [float(x) for x in second.get('xyz').split()])

    def test_gemini305_defaults_to_automatic_exposure_and_30_fps(self):
        args = SimpleNamespace(camera='gemini305', distance=None, gain=None, gain_x=None,
                               max_vel=None, fps=None, color_exposure=None)
        config = dict(distance=.35, gain=1.5, gain_x=1.5, max_vel=.1,
                      fps=60, color_exposure_ms=6.)
        options = resolve_motion_options(args, config)
        self.assertEqual((options.camera_fps, options.color_exposure_ms), (30, 0.))

if __name__ == '__main__':
    unittest.main()
