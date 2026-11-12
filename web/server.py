#!/usr/bin/env python3
"""Read-only local dashboard. Never opens CAN or writes robot commands."""
import io
import json
import os
import sys
import atexit
import signal
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
sys.path.insert(0, str(ROOT))
from robot_process import RobotProcess
from telemetry import read_latest_state, state_from_row

controller = RobotProcess(PROJECT)
TWIN = Path(os.environ.get('REBOT_TWIN_PATH', '/home/robot/ReBot_Arm_DigitalTwin_RS/reBotArm_simulator-RS')).resolve()


def latest_state():
    sample = controller.live_telemetry()
    if sample is not None:
        row, age = sample
        return state_from_row(PROJECT, row, age)
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
        mappings = {'/assets/mediapipe/': TWIN / 'public/lib/mediapipe',
                    '/model/': PROJECT / 'models/wrist_camera',
                    '/assets/description/': TWIN / 'description',
                    '/assets/camera/': TWIN / 'public/models/wrist-cameras'}
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
                if body.get('serial'):
                    result = controller.start(body.get('speed', 'slow'), body.get('gripper', False), body['serial'])
                else:
                    result = controller.start(body.get('speed', 'slow'), body.get('gripper', False))
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
    print(f'控制台: http://127.0.0.1:{port}  |  模型来源: {TWIN}', flush=True)
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
