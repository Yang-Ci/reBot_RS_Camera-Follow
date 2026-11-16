#!/usr/bin/env python3
"""Read-only local dashboard. Never opens CAN or writes robot commands."""
import io
import json
import os
import sys
import atexit
import signal
import math
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
sys.path.insert(0, str(ROOT))
from robot_process import RobotProcess, speed_presets, validate_motion_settings
from telemetry import read_latest_state, state_from_row

controller = RobotProcess(PROJECT)
ASSETS = PROJECT / 'assets'


def home_preview(degrees):
    """Convert saved encoder angles to model angles for a read-only preview."""
    if (not isinstance(degrees, list) or len(degrees) != 6
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in degrees)):
        return None
    row = {'q_cmd_valid': '1', **{f'q_cmd_{i+1}': str(math.radians(value))
                                for i, value in enumerate(degrees)}}
    return state_from_row(PROJECT, row, 0).get('model_joints')


def homes_response():
    profiles = controller.homes.list()
    for profile in profiles:
        profile['model_joints'] = home_preview(profile['degrees'])
    default = None
    try:
        data = json.loads((PROJECT / 'config/home_pose.json').read_text(encoding='utf-8'))
        radians = data['home_joint_pos']
        if isinstance(radians, list) and len(radians) == 6:
            degrees = [math.degrees(value) for value in radians]
            model = home_preview(degrees)
            if model is not None:
                default = {'name': '项目默认 HOME', 'degrees': degrees, 'model_joints': model}
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        pass
    return {'profiles': profiles, 'default': default}


def latest_state():
    sample = controller.live_telemetry()
    if sample is not None:
        row, age = sample
        return state_from_row(PROJECT, row, age)
    if controller.status().get('process_mode') == 'gravity':
        return {'online': False, 'reason': '重力示教实时反馈不可用'}
    return read_latest_state(PROJECT, controller)


def controller_status():
    state = controller.status()
    if not state['running'] and state['exit_code'] not in (None, 0) and controller.log_path:
        try:
            with controller.log_path.open('rb') as stream:
                stream.seek(max(0, controller.log_path.stat().st_size - 4000))
                state['error_log'] = stream.read().decode('utf-8', errors='replace')
                for line in reversed(state['error_log'].splitlines()):
                    if line.startswith('错误: '):
                        state['error_summary'] = line[len('错误: '):][:500]
                        break
        except OSError:
            pass
    return state


class Handler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        path = urlparse(path).path
        mappings = {'/assets/mediapipe/': ROOT / 'lib/mediapipe',
                    '/model/': PROJECT / 'models/wrist_camera',
                    '/assets/description/': ASSETS / 'robot/description',
                    '/assets/camera/': ASSETS / 'wrist-cameras'}
        for prefix, root in mappings.items():
            if path.startswith(prefix):
                target = (root / path[len(prefix):]).resolve()
                return str(target if target.is_relative_to(root) else ROOT / '__missing__')
        target = (ROOT / path.lstrip('/')).resolve()
        return str(target if target.is_relative_to(ROOT) else ROOT / '__missing__')

    def send_model(self):
        path = PROJECT / 'config/hand_landmarker.task'
        try:
            with path.open('rb') as source:
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(path.stat().st_size))
                self.end_headers()
                import shutil
                shutil.copyfileobj(source, self.wfile)
        except FileNotFoundError:
            self.send_error(404)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def send_json(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        # Localhost binding plus exact Origin/Host and JSON prevent a remote
        # webpage from triggering robot operations via cross-origin requests.
        host = self.headers.get('Host', '')
        port = self.server.server_address[1]
        if (self.client_address[0] != '127.0.0.1'
                or host not in (f'127.0.0.1:{port}', f'localhost:{port}')
                or self.headers.get('Origin') != f'http://{host}'
                or self.headers.get_content_type() != 'application/json'):
            return self.send_json(403, {'error': '请求来源无效'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 1024:
                raise ValueError('请求长度无效')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError('请求格式无效')
            path = urlparse(self.path).path
            if path == '/api/robot/start':
                if body.get('confirm_home') is not True:
                    raise ValueError('请先确认 HOME 路径、工作区和硬件急停')
                kwargs = {}
                if body.get('serial'):
                    kwargs['serial'] = body['serial']
                if 'camera' in body:
                    kwargs['camera'] = body['camera']
                if 'fps' in body:
                    kwargs['fps'] = body['fps']
                if 'stationary_joint_hold' in body:
                    kwargs['stationary_joint_hold'] = body['stationary_joint_hold']
                if 'home_profile' in body:
                    kwargs['home_profile'] = body['home_profile']
                if 'motion' in body:
                    kwargs['motion'] = validate_motion_settings(body['motion'])
                result = controller.start(body.get('speed', 'slow'), body.get('gripper', False), **kwargs)
            elif path == '/api/robot/gravity/start':
                if body.get('confirm_workspace') is not True:
                    raise ValueError('请先确认工作区、安全支撑与硬件急停')
                result = controller.start_gravity()
            elif path == '/api/home/save':
                result = controller.save_home(body.get('name'), body.get('degrees'))
            elif path == '/api/home/gravity-save':
                result = controller.save_gravity_home(body.get('name'), body.get('replace', False))
            elif path == '/api/camera/start':
                result = controller.preview_start(body.get('serial'))
            elif path == '/api/camera/stop':
                result = controller.preview_stop()
            elif path == '/api/robot/follow':
                result = controller.follow(body.get('enabled'))
            elif path == '/api/robot/shutdown':
                result = controller.shutdown()
            else:
                return self.send_json(404, {'error': '未知操作'})
            return self.send_json(200, result)
        except (ValueError, json.JSONDecodeError) as exc:
            return self.send_json(400, {'error': str(exc)})
        except (RuntimeError, OSError) as exc:
            return self.send_json(409, {'error': str(exc)})

    def do_GET(self):
        if urlparse(self.path).path == '/api/home':
            return self.send_json(200, homes_response())
        if urlparse(self.path).path == '/api/robot/presets':
            try:
                return self.send_json(200, {'presets': speed_presets(PROJECT)})
            except Exception as exc:
                return self.send_json(503, {'error': f'无法读取速度配置: {exc}'})
        if urlparse(self.path).path == '/api/cameras':
            try:
                return self.send_json(200, {'cameras': controller.cameras()})
            except RuntimeError as exc:
                return self.send_json(503, {'error': str(exc)})
        if urlparse(self.path).path == '/api/camera.jpg':
            data = controller.camera_frame()
            if data is None:
                self.send_response(204)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(data)))
            frame_id, capture_fps = controller.camera_frame_info()
            if frame_id is not None:
                self.send_header('X-Frame-Id', str(frame_id))
            if capture_fps is not None:
                self.send_header('X-Capture-Fps', str(capture_fps))
            self.end_headers()
            self.wfile.write(data)
            return
        if urlparse(self.path).path == '/api/controller':
            return self.send_json(200, controller_status())
        if urlparse(self.path).path == '/assets/hand_landmarker.task':
            return self.send_model()
        if urlparse(self.path).path == '/api/state':
            data = json.dumps(latest_state(), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            super().do_GET()


if __name__ == '__main__':
    port = int(os.environ.get('PORT', '8765'))
    print(f'控制台: http://127.0.0.1:{port}  |  模型来源: {ASSETS}', flush=True)
    def cleanup():
        if controller.status()['running']:
            try:
                controller.shutdown()
            except (OSError, RuntimeError):
                pass
        try:
            controller.preview_stop()
        except (OSError, RuntimeError):
            pass
    atexit.register(cleanup)
    signal.signal(signal.SIGTERM, lambda _sig, _frame: sys.exit(0))
    ThreadingHTTPServer(('127.0.0.1', port), Handler).serve_forever()
