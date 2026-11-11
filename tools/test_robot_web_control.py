#!/usr/bin/env python3
"""Exercise web-managed process lifecycle without opening CAN or a camera."""
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import unittest
import time
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from robot_process import RobotProcess


class FakeProcess:
    pid = 12345
    def __init__(self, *args, **kwargs):
        self.args = args[0]
        self.kwargs = kwargs
        self.signals = []
        self.returncode = None
    def poll(self):
        return self.returncode
    def send_signal(self, sig):
        self.signals.append(sig)
    def terminate(self):
        self.signals.append(signal.SIGTERM)
        self.returncode = 0
    def wait(self, timeout=None):
        return self.returncode


class RobotProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.project = Path(self.tmp.name)
        (self.project / '.venv-arm/bin').mkdir(parents=True)
        (self.project / '.venv-arm/bin/python').touch()
        self.robot = RobotProcess(self.project)
        self.other = mock.patch.object(self.robot, '_other_controller', return_value=False)
        self.other.start()
        self.popen = mock.patch('robot_process.subprocess.Popen', side_effect=FakeProcess)
        self.popen.start()
    def tearDown(self):
        if self.robot.process is not None:
            self.robot.process.returncode = 0
            self.robot.status()
        self.popen.stop(); self.other.stop(); self.tmp.cleanup()

    def test_start_ready_follow_pause_shutdown(self):
        self.robot.start()
        process = self.robot.process
        self.assertIn('--web-control', process.args)
        self.assertIn('--web-auto-follow', process.args)
        self.assertEqual(process.args[process.args.index('--home-j34-tolerance') + 1], '3.5')
        self.assertEqual(process.kwargs['env']['REBOT_WEB_PREVIEW_PATH'], str(self.robot.preview_path))
        self.assertIsNone(self.robot.camera_frame())
        self.robot.preview_path.write_bytes(b'jpeg')
        self.assertEqual(self.robot.camera_frame(), b'jpeg')
        os.utime(self.robot.preview_path, (time.time() - 2,) * 2)
        self.assertIsNone(self.robot.camera_frame())
        self.assertIn('--keep-gripper-position', process.args)
        self.assertEqual(process.args[process.args.index('--max-vel') + 1], '0.10')
        self.assertFalse(self.robot.status()['ready'])
        with self.assertRaises(RuntimeError): self.robot.follow(True)
        self.robot.ready_path.write_text(str(process.pid))
        self.assertTrue(self.robot.status()['ready'])
        self.robot.follow(True); self.robot.follow(True); self.robot.follow(False)
        self.assertEqual(process.signals, [signal.SIGUSR1, signal.SIGUSR1, signal.SIGUSR2])
        self.robot.shutdown(); self.robot.shutdown()
        self.assertEqual(process.signals[-1], signal.SIGINT)
        self.assertEqual(process.signals.count(signal.SIGINT), 1)
        self.assertFalse(self.robot.status()['ready'])
        with self.assertRaises(RuntimeError): self.robot.follow(True)
        process.returncode = 0
        self.assertFalse(self.robot.status()['running'])
        self.assertFalse(self.robot.preview_path.exists())

    def test_guard_options_external_controller_and_duplicate_start(self):
        with self.assertRaises(ValueError): self.robot.start('invalid', False)
        with self.assertRaises(ValueError): self.robot.start('slow', 'false')
        with mock.patch.object(self.robot, '_other_controller', return_value=True):
            with self.assertRaises(RuntimeError): self.robot.start()
        self.robot.start('configured', True)
        self.assertNotIn('--no-gripper', self.robot.process.args)
        self.assertNotIn('--gain', self.robot.process.args)
        with self.assertRaises(RuntimeError): self.robot.start()
        self.robot.ready_path.write_text('999')
        self.assertFalse(self.robot.status()['ready'])

    def test_second_dashboard_cannot_start_same_robot(self):
        self.robot.start()
        second = RobotProcess(self.project)
        with mock.patch.object(second, '_other_controller', return_value=False):
            with self.assertRaises(RuntimeError): second.start()
        self.robot.process.returncode = 0
        self.robot.status()
        with mock.patch.object(second, '_other_controller', return_value=False):
            second.start()
        second.process.returncode = 0
        second.status()

    def test_shutdown_during_startup_never_force_kills(self):
        self.robot.start()
        self.robot.shutdown()
        self.assertEqual(self.robot.process.signals, [signal.SIGINT])
        self.assertTrue(self.robot.status()['stopping'])

    def test_camera_preview_handoff_keeps_selected_camera(self):
        with mock.patch.object(self.robot, 'cameras', return_value=[{'serial': 'test336'}]):
            self.robot.preview_start('test336')
        preview = self.robot.preview_process
        self.assertTrue(self.robot.status()['preview_running'])
        self.robot.start()
        self.assertEqual(preview.signals, [signal.SIGTERM])
        self.assertEqual(self.robot.process.kwargs['env']['REBOT_CAMERA_SERIAL'], 'test336')
        self.assertFalse(self.robot.status()['preview_running'])
        with self.assertRaises(RuntimeError):
            self.robot.preview_stop()


class HttpSecurityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('web_dashboard', ROOT / 'web/server.py')
        cls.web = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.web)

    def request(self, origin, host='127.0.0.1:8765', body=b'{}', content_type='application/json'):
        from email.message import Message
        handler = object.__new__(self.web.Handler)
        handler.headers = Message()
        handler.headers['Host'] = host
        if origin is not None: handler.headers['Origin'] = origin
        handler.headers['Content-Type'] = content_type
        handler.headers['Content-Length'] = str(len(body))
        handler.client_address = ('127.0.0.1', 1234)
        handler.server = type('Server', (), {'server_address': ('127.0.0.1', 8765)})()
        handler.path = '/api/robot/start'; handler.rfile = io.BytesIO(body)
        handler.send_json = mock.Mock()
        handler.do_POST()
        return handler.send_json.call_args.args

    def test_origin_and_content_type_guard(self):
        with mock.patch.object(self.web.controller, 'start', return_value={'ok': True}) as start:
            self.assertEqual(self.request('https://attacker.example')[0], 403)
            self.assertEqual(self.request(None)[0], 403)
            self.assertEqual(self.request('http://127.0.0.1:8765', content_type='text/plain')[0], 403)
            start.assert_not_called()
            self.assertEqual(self.request('http://127.0.0.1:8765')[0], 200)
            start.assert_called_once_with('slow', False)

    def test_camera_endpoint_returns_only_current_frame(self):
        handler = object.__new__(self.web.Handler)
        handler.path = '/api/camera.jpg'
        handler.wfile = io.BytesIO()
        handler.send_response = mock.Mock()
        handler.send_header = mock.Mock()
        handler.end_headers = mock.Mock()
        with mock.patch.object(self.web.controller, 'camera_frame', return_value=b'jpeg'), \
             mock.patch.object(self.web.controller, 'camera_frame_info', return_value=(123, 59.7)):
            handler.do_GET()
        handler.send_response.assert_called_with(200)
        handler.send_header.assert_any_call('X-Frame-Id', '123')
        handler.send_header.assert_any_call('X-Capture-Fps', '59.7')
        self.assertEqual(handler.wfile.getvalue(), b'jpeg')
        handler.send_response.reset_mock()
        with mock.patch.object(self.web.controller, 'camera_frame', return_value=None):
            handler.do_GET()
        handler.send_response.assert_called_with(204)

    def test_controller_status_exposes_startup_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'controller.log'
            path.write_text('其他日志\n错误: HOME 等待超时；实际误差(度): [0, 0, -3.08]\nTraceback\n')
            with mock.patch.object(self.web.controller, 'status', return_value={'running': False, 'exit_code': 1}), \
                 mock.patch.object(self.web.controller, 'log_path', path):
                status = self.web.controller_status()
            self.assertEqual(status['error_summary'], 'HOME 等待超时；实际误差(度): [0, 0, -3.08]')


if __name__ == '__main__': unittest.main()
