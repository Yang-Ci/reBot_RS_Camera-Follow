#!/usr/bin/env python3
"""Exercise web-managed process lifecycle without opening CAN or a camera."""
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import socket
import sys
import tempfile
import unittest
import time
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from robot_process import RobotProcess, speed_presets, validate_motion_settings


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
        self.assertNotIn('--web-auto-follow', process.args)
        self.assertEqual(process.args[process.args.index('--home-j34-tolerance') + 1], '3.5')
        self.assertEqual(process.kwargs['env']['REBOT_WEB_PREVIEW_PATH'], str(self.robot.preview_path))
        self.assertEqual(process.kwargs['env']['REBOT_TELEMETRY_PORT'], str(self.robot.telemetry.port))
        self.assertEqual(process.kwargs['env']['REBOT_TELEMETRY_TOKEN'], self.robot.telemetry.token)
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
        self.assertIsNone(self.robot.telemetry)
        self.assertFalse(self.robot.preview_path.exists())

    def test_stationary_joint_hold_is_validated_forwarded_and_reported(self):
        for value, flag in [(True, '--stationary-joint-hold'), (False, '--no-stationary-joint-hold')]:
            with self.subTest(value=value):
                self.robot.start(stationary_joint_hold=value)
                self.assertIn(flag, self.robot.process.args)
                self.assertIs(self.robot.status()['stationary_joint_hold'], value)
                self.robot.process.returncode = 0
                self.robot.status()
                self.robot.process = None
        for invalid in ('false', 1, [], {}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.robot.start(stationary_joint_hold=invalid)
        self.assertIsNone(self.robot.process)

    def test_medium_sets_gain_speed_and_safe_home_speed(self):
        self.robot.start('medium', False)
        args = self.robot.process.args
        for flag, value in (('--gain', '1.9'), ('--gain-x', '1.9'),
                            ('--max-vel', '0.25'), ('--home-speed', '4')):
            self.assertEqual(args[args.index(flag) + 1], value)
        self.assertIn('--keep-gripper-position', args)

    def test_custom_motion_is_applied_and_cannot_be_changed_while_running(self):
        motion = {'gain': 1.8, 'gain_x': 1.7, 'max_vel': .2}
        self.robot.start('medium', False, motion=motion)
        args = self.robot.process.args
        for flag, value in (('--gain', '1.8'), ('--gain-x', '1.7'),
                            ('--max-vel', '0.2'), ('--home-speed', '4')):
            self.assertEqual(args[args.index(flag) + 1], value)
        self.assertEqual(self.robot.status()['motion_settings'], motion)
        with self.assertRaises(RuntimeError):
            self.robot.start('medium', False, motion=motion)

    def test_invalid_custom_motion_never_starts_process(self):
        valid = {'gain': 1.9, 'gain_x': 1.9, 'max_vel': .25}
        invalid = [None, {}, {'gain': 1, 'gain_x': 1},
                   {**valid, 'gain': float('nan')}, {**valid, 'gain_x': float('inf')},
                   {**valid, 'gain': True}, {**valid, 'gain_x': 2.51},
                   {**valid, 'max_vel': 0}, {**valid, 'max_vel': .71},
                   {**valid, 'max_vel': '0.25'}, {**valid, 'extra': 1}]
        for motion in invalid:
            with self.subTest(motion=motion), self.assertRaises(ValueError):
                validate_motion_settings(motion)
        with self.assertRaises(ValueError):
            self.robot.start('medium', False, motion={**valid, 'max_vel': .71})
        self.assertIsNone(self.robot.process)

    def test_presets_follow_camera_profile_without_changing_them(self):
        config = self.project / 'config'
        config.mkdir()
        (config / 'follow.yaml').write_text('follow:\n  gain: 2.35\n  gain_x: 2.4\n  max_vel: 0.7\n')
        (config / 'follow_gemini336.yaml').write_text('follow:\n  gain: 2.1\n')
        presets = speed_presets(self.project)
        self.assertEqual(presets['configured'], {'gain': 2.1, 'gain_x': 2.4, 'max_vel': .7})
        self.assertEqual(presets['medium'], {'gain': 1.9, 'gain_x': 1.9, 'max_vel': .25})

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

    def test_live_telemetry_is_bound_to_running_session(self):
        self.robot.start()
        token, port = self.robot.telemetry.token, self.robot.telemetry.port
        packet = {'token': token, 'row': {'host_time': str(time.monotonic())}}
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(json.dumps(packet).encode(), ('127.0.0.1', port))
        deadline = time.monotonic() + 1.0
        sample = None
        while sample is None and time.monotonic() < deadline:
            sample = self.robot.live_telemetry()
            if sample is None:
                time.sleep(.01)
        self.assertIsNotNone(sample)
        self.robot.process.returncode = 0
        self.assertIsNone(self.robot.live_telemetry())
        self.assertIsNone(self.robot.telemetry)

    def test_shutdown_during_startup_never_force_kills(self):
        self.robot.start()
        self.robot.shutdown()
        self.assertEqual(self.robot.process.signals, [signal.SIGINT])
        self.assertTrue(self.robot.status()['stopping'])

    def test_selected_305_is_verified_and_passed_to_controller(self):
        with mock.patch.object(self.robot, 'cameras', return_value=[{'serial': '305', 'name': 'Orbbec Gemini 305'}]):
            with self.assertRaises(ValueError):
                self.robot.start(serial='other', camera='gemini305')
            self.assertIsNone(self.robot.process)
            with self.assertRaises(ValueError):
                self.robot.start(serial='305', camera='gemini305', fps=90)
            self.robot.start(serial='305', camera='gemini305', fps=60)
        args = self.robot.process.args
        self.assertEqual(args[args.index('--fps') + 1], '60')
        self.assertEqual(self.robot.status()['camera_fps'], 60)
        self.assertEqual(args[args.index('--camera') + 1], 'gemini305')
        self.assertEqual(self.robot.process.kwargs['env']['REBOT_CAMERA_SERIAL'], '305')

    def test_legacy_cameras_require_matching_device_and_pose_before_start(self):
        for model, filename in [('gemini2', 'hand_eye_calibration.json'), ('d435i', 'hand_eye_calibration_d435i.json')]:
            with self.subTest(model=model), mock.patch.object(self.robot, 'cameras', return_value=[{'serial': 'chosen', 'model': model}]):
                with self.assertRaisesRegex(ValueError, '位姿文件'):
                    self.robot.start(serial='chosen', camera=model)
                self.assertIsNone(self.robot.process)
                (self.project / 'config').mkdir(exist_ok=True)
                (self.project / 'config' / filename).write_text('{}')
                with self.assertRaises(ValueError):
                    self.robot.start(serial='other', camera=model)
                self.robot.start(serial='chosen', camera=model)
                self.assertEqual(self.robot.process.args[self.robot.process.args.index('--camera') + 1], model)
                self.robot.process.returncode = 0
                self.robot.status()
                self.robot.process = None

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

    def request(self, origin, host='127.0.0.1:8765', body=b'{"confirm_home":true}', content_type='application/json'):
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

    def test_motion_validation_before_start(self):
        with mock.patch.object(self.web.controller, 'start', return_value={'ok': True}) as start:
            for confirmation in (None, False, 1, 'true'):
                body = json.dumps({'confirm_home': confirmation}).encode()
                self.assertEqual(self.request('http://127.0.0.1:8765', body=body)[0], 400)
            for motion in (None, {}, {'gain': 1.9, 'gain_x': 1.9, 'max_vel': .71},
                           {'gain': True, 'gain_x': 1.9, 'max_vel': .25}):
                body = json.dumps({'confirm_home': True, 'motion': motion}).encode()
                self.assertEqual(self.request('http://127.0.0.1:8765', body=body)[0], 400)
            start.assert_not_called()
            body = json.dumps({'confirm_home': True, 'speed': 'medium', 'motion': {'gain': 1.9, 'gain_x': 1.9, 'max_vel': .25}}).encode()
            self.assertEqual(self.request('http://127.0.0.1:8765', body=body)[0], 200)
            self.assertEqual(start.call_args.kwargs['motion']['max_vel'], .25)

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
