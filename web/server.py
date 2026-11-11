#!/usr/bin/env python3
"""Read-only local dashboard. Never opens CAN or writes robot commands."""
import csv
import io
import json
import math
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

controller = RobotProcess(PROJECT)
TWIN = Path(os.environ.get('REBOT_TWIN_PATH', '/home/robot/ReBot_Arm_DigitalTwin_RS/reBotArm_simulator-RS')).resolve()


def latest_state():
    files = list((PROJECT / 'logs').glob('servo_*.csv'))
    if not files:
        return {'online': False, 'reason': '等待诊断日志'}
    path = max(files, key=lambda p: p.stat().st_mtime)
    try:
        modified = path.stat().st_mtime
    except OSError:
        return {'online': False, 'reason': '诊断日志暂不可用'}
    managed = controller.status()
    if managed['running'] and managed['started_at'] and modified < managed['started_at']:
        return {'online': False, 'reason': '等待本次真机程序的诊断数据'}
    if not managed['running'] and managed['ended_at'] and modified <= managed['ended_at']:
        return {'online': False, 'reason': '真机程序已退出'}
    age = max(0, __import__('time').time() - modified)
    if age > 3:
        return {'online': False, 'reason': '诊断数据已停止更新', 'age_s': age}
    try:
        with path.open('rb') as f:
            header = f.readline().decode('utf-8')
            f.seek(max(0, path.stat().st_size - 32768))
            tail = f.read().decode('utf-8')
        lines = tail.splitlines()
        # Ignore any incomplete trailing record while the recorder is writing.
        if not tail.endswith('\n'):
            lines = lines[:-1]
        names = next(csv.reader([header]))
        for line in reversed(lines):
            values = next(csv.reader([line]))
            if len(values) == len(names):
                row = dict(zip(names, values))
                break
        else:
            return {'online': False, 'reason': '等待完整诊断数据'}
        actual = row.get('fb_valid') == '1'
        if not actual and row.get('q_cmd_valid') != '1' and row.get('following') != '1':
            return {'online': True, 'source': 'unavailable', 'joints': None, 'age_s': age,
                    'visible': row.get('visible') == '1', 'following': False,
                    'hold': row.get('hold_latch') == '1', 'error': row.get('error', '')}
        source = 'actual' if actual else 'command'
        prefix = 'q_act_' if actual else 'q_cmd_'
        joints = [float(row[prefix + str(i)]) for i in range(1, 7)]
        if not all(math.isfinite(q) for q in joints):
            raise ValueError('nonfinite joints')
        try:
            offsets = json.loads((PROJECT / 'config/joint_offset.json').read_text())['joint_offset_rad']
            if len(offsets) != 6 or not all(math.isfinite(float(x)) for x in offsets):
                raise ValueError('invalid offsets')
            model_joints = [q + float(offset) for q, offset in zip(joints, offsets)]
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            model_joints = None
            offsets = None
        # The diagnostics thread reads actual and commanded angles together;
        # carry subsequent command motion forward for display only. This is an
        # estimate, not a new measurement or an input to the robot controller.
        estimated_model_joints = None
        if actual and offsets is not None and row.get('q_cmd_valid') == '1':
            try:
                command = [float(row['q_cmd_' + str(i)]) for i in range(1, 7)]
                paired = [float(row['q_fb_cmd_' + str(i)]) for i in range(1, 7)]
                if all(math.isfinite(q) for q in command + paired):
                    estimated_model_joints = [q + c - p + float(offset)
                                              for q, c, p, offset in zip(joints, command, paired, offsets)]
            except (KeyError, TypeError, ValueError):
                pass
        return {'online': True, 'source': source, 'joints': joints,
                'model_joints': model_joints, 'age_s': age,
                'estimated_model_joints': estimated_model_joints,
                'visible': row.get('visible') == '1', 'following': row.get('following') == '1',
                'hold': row.get('hold_latch') == '1', 'error': row.get('error', ''),
                'feedback_age_ms': row.get('fb_age_ms', ''),
                'feedback_rate_hz': row.get('fb_rate_hz', '')}
    except (OSError, ValueError, KeyError, csv.Error, UnicodeError):
        return {'online': False, 'reason': '诊断数据暂不可用'}


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
