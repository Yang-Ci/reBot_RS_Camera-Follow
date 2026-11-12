"""Camera constructors must release a partially started stream on failure."""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


class CameraInitializationTests(unittest.TestCase):
    def test_orbbec_start_or_post_start_failure_stops_pipeline(self):
        for stage in ('start', 'finish'):
            with self.subTest(stage=stage):
                pipeline = mock.Mock()
                info = pipeline.get_device.return_value.get_device_info.return_value
                info.get_name.return_value = 'Orbbec Gemini 2'
                info.get_serial_number.return_value = 'test'
                profile = pipeline.get_stream_profile_list.return_value.get_video_stream_profile.return_value
                profile.get_width.return_value = 640
                profile.get_height.return_value = 480
                profile.get_fps.return_value = 30
                sdk = types.ModuleType('pyorbbecsdk')
                sdk.Pipeline = lambda: pipeline
                sdk.Config = mock.Mock
                sdk.OBSensorType = types.SimpleNamespace(COLOR_SENSOR=1, DEPTH_SENSOR=2)
                sdk.OBAlignMode = types.SimpleNamespace(HW_MODE=1)
                sdk.OBFormat = types.SimpleNamespace(MJPG=1, Y16=2)
                sdk.OBError = type('OBError', (Exception,), {})
                spec = importlib.util.spec_from_file_location('camera_cleanup_test', ROOT / 'gemini2_camera.py')
                camera_module = importlib.util.module_from_spec(spec)
                with mock.patch.dict(sys.modules, {'pyorbbecsdk': sdk}):
                    spec.loader.exec_module(camera_module)
                    if stage == 'start':
                        pipeline.start.side_effect = RuntimeError('start failed')
                    with mock.patch.object(camera_module.Gemini2Camera, '_finish_initialization',
                                           side_effect=RuntimeError('finish failed')):
                        with self.assertRaises(RuntimeError):
                            camera_module.Gemini2Camera(align_to_color=False)
                pipeline.stop.assert_called_once_with()

    def test_realsense_start_or_post_start_failure_stops_pipeline(self):
        spec = importlib.util.spec_from_file_location('rs_cleanup_test', ROOT / 'realsense_camera.py')
        camera_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(camera_module)
        for stage in ('start', 'finish'):
            with self.subTest(stage=stage):
                pipeline = mock.Mock()
                if stage == 'start':
                    pipeline.start.side_effect = RuntimeError('start failed')
                else:
                    pipeline.start.return_value.get_stream.side_effect = RuntimeError('intrinsics failed')
                device = mock.Mock()
                device.query_sensors.return_value = []
                sdk = types.ModuleType('pyrealsense2')
                sdk.pipeline = lambda: pipeline
                sdk.config = mock.Mock
                sdk.context = lambda: types.SimpleNamespace(query_devices=lambda: [device])
                sdk.stream = types.SimpleNamespace(color=1, depth=2, infrared=3)
                sdk.format = types.SimpleNamespace(z16=1, bgr8=2, y8=3)
                sdk.camera_info = types.SimpleNamespace(serial_number=1)
                with mock.patch.dict(sys.modules, {'pyrealsense2': sdk}):
                    with self.assertRaises(RuntimeError):
                        camera_module.RealSenseCamera(ir_emitter='on')
                pipeline.stop.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
