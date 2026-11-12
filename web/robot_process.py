"""Own one local controller process; never opens CAN or accepts shell commands."""
import os
import fcntl
import secrets
import signal
import subprocess
import threading
import time
import json
from pathlib import Path

from live_telemetry import LiveTelemetryReceiver


class RobotProcess:
    def __init__(self, project):
        self.project = Path(project)
        self.lock = threading.Lock()
        self.process = None
        self.ready_path = None
        self.preview_path = None
        self.stopping = False
        self.log_path = None
        self.started_at = None
        self.ended_at = None
        self.lock_file = None
        self.preview_process = None
        self.preview_serial = None
        self.camera_lock = None
        self.telemetry = None

    def cameras(self):
        python = self.project / '.venv-arm/bin/python'
        try:
            result = subprocess.run([str(python), str(self.project / 'web/camera_preview_process.py'), '--list'],
                                    capture_output=True, text=True, timeout=8, check=True)
            line = next(line for line in result.stdout.splitlines() if line.startswith('CAMERA_LIST='))
            return json.loads(line.split('=', 1)[1])
        except (OSError, subprocess.SubprocessError, StopIteration, ValueError) as exc:
            raise RuntimeError(f'无法列出 Gemini 相机: {exc}') from exc

    def _camera_lock(self):
        if self.camera_lock is None:
            (self.project / 'logs').mkdir(exist_ok=True)
            lock = (self.project / 'logs/.web_camera.lock').open('a')
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lock.close()
                raise RuntimeError('相机正在被其他网页会话使用')
            self.camera_lock = lock

    def _unlock_camera(self):
        if self.camera_lock:
            fcntl.flock(self.camera_lock.fileno(), fcntl.LOCK_UN)
            self.camera_lock.close()
            self.camera_lock = None

    def _stop_preview(self, release=True):
        if self.preview_process is not None:
            if self.preview_process.poll() is None:
                self.preview_process.terminate()
                try:
                    self.preview_process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    raise RuntimeError('相机预览尚未释放，请稍后重试')
            self.preview_process = None
        if self.preview_path:
            for path in (self.preview_path, Path(str(self.preview_path) + '.tmp'),
                         Path(str(self.preview_path) + '.fps'), Path(str(self.preview_path) + '.fps.tmp')):
                try: path.unlink()
                except OSError: pass
        if release:
            self._unlock_camera()

    def preview_start(self, serial):
        if not isinstance(serial, str) or not serial or len(serial) > 128:
            raise ValueError('请选择 Gemini 336 相机')
        if serial not in [item['serial'] for item in self.cameras()]:
            raise ValueError('所选相机不在线，请刷新列表')
        with self.lock:
            if self._running() or self._other_controller():
                raise RuntimeError('跟随程序正在使用相机')
            self._stop_preview()
            self._camera_lock()
            self.preview_serial = serial
            self.preview_path = self.project / 'logs' / f'.web_preview_{secrets.token_hex(12)}.jpg'
            try:
                self.preview_process = subprocess.Popen(
                    [str(self.project / '.venv-arm/bin/python'), '-u',
                     str(self.project / 'web/camera_preview_process.py'), '--preview', serial, str(self.preview_path)],
                    cwd=str(self.project), stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            except BaseException:
                self._unlock_camera()
                raise
            return {'ok': True, 'message': '正在打开相机，不会连接机械臂'}

    def preview_stop(self):
        with self.lock:
            if self._running():
                raise RuntimeError('跟随程序正在使用相机，不能单独关闭')
            self._stop_preview()
            return {'ok': True, 'message': '相机预览已关闭'}

    def _running(self):
        running = self.process is not None and self.process.poll() is None
        if not running and self.process is not None and self.ended_at is None:
            self.ended_at = time.time()
            if self.preview_path:
                for path in (self.preview_path, Path(str(self.preview_path) + '.tmp'),
                             Path(str(self.preview_path) + '.fps'), Path(str(self.preview_path) + '.fps.tmp')):
                    try:
                        path.unlink()
                    except OSError:
                        pass
            self._unlock_camera()
        if not running and self.telemetry is not None:
            self.telemetry.close()
            self.telemetry = None
        if not running and self.lock_file is not None:
            fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_UN)
            self.lock_file.close()
            self.lock_file = None
        return running

    def _ready(self):
        if not self._running() or self.stopping or not self.ready_path:
            return False
        try:
            return int(self.ready_path.read_text().strip()) == self.process.pid
        except (OSError, ValueError):
            return False

    @staticmethod
    def _other_controller():
        # Avoid starting another copy against the same CAN bus. Other control
        # software must also be closed by the operator before initialization.
        for item in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                argv = item.read_bytes().split(b'\0')
                if any(Path(os.fsdecode(v)).name == 'palm_follow_grasp.py' for v in argv if v):
                    return True
            except (OSError, ValueError):
                continue
        return False

    def status(self):
        with self.lock:
            running = self._running()
            preview_running = self.preview_process is not None and self.preview_process.poll() is None
            preview_failed = self.preview_process is not None and not preview_running
            return {'running': running, 'ready': self._ready(), 'stopping': self.stopping and running,
                    'preview_running': preview_running, 'preview_failed': preview_failed,
                    'exit_code': None if self.process is None else self.process.poll(),
                    'started_at': self.started_at, 'ended_at': self.ended_at, 'log': self.log_path.name if self.log_path else None}

    def start(self, speed='slow', gripper=False, serial=None):
        if speed not in ('slow', 'configured') or type(gripper) is not bool:
            raise ValueError('无效的速度或夹爪选项')
        with self.lock:
            if self._running():
                raise RuntimeError('网页已启动真机程序')
            python = self.project / '.venv-arm/bin/python'
            if not python.is_file():
                raise RuntimeError('未找到 .venv-arm/bin/python，请先配置机械臂环境')
            if serial is not None:
                if not isinstance(serial, str) or not serial or len(serial) > 128:
                    raise ValueError('相机选择无效')
                self.preview_serial = serial
            self._stop_preview(release=False)
            self._camera_lock()
            logs = self.project / 'logs'
            logs.mkdir(exist_ok=True)
            self.lock_file = (logs / '.web_controller.lock').open('a')
            try:
                fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                if self._other_controller():
                    raise RuntimeError('检测到其他手掌跟随程序，请先关闭以避免争用 CAN')
            except (BlockingIOError, RuntimeError):
                self.lock_file.close()
                self.lock_file = None
                self._unlock_camera()
                raise RuntimeError('已有控制程序或网页会话，不能重复占用 CAN')
            token = secrets.token_hex(12)
            try:
                self.telemetry = LiveTelemetryReceiver(token)
            except OSError:
                self.telemetry = None  # CSV telemetry remains available.
            self.ready_path = logs / f'.web_ready_{token}'
            self.preview_path = logs / f'.web_preview_{token}.jpg'
            self.log_path = logs / f'web_controller_{time.strftime("%Y%m%d_%H%M%S")}_{token[:6]}.log'
            args = [str(python), '-u', str(self.project / 'palm_follow_grasp.py'),
                    '--camera', 'gemini336', '--headless', '--web-control',
                    '--web-auto-follow', '--home-j34-tolerance', '3.5']
            if not gripper:
                args += ['--no-gripper', '--keep-gripper-position']
            if speed == 'slow':
                args += ['--gain', '1.5', '--gain-x', '1.5', '--max-vel', '0.10', '--home-speed', '4']
            env = os.environ.copy()
            env.pop('REBOT_TELEMETRY_PORT', None)
            env.pop('REBOT_TELEMETRY_TOKEN', None)
            env['REBOT_WEB_READY_PATH'] = str(self.ready_path)
            env['REBOT_WEB_PREVIEW_PATH'] = str(self.preview_path)
            if self.telemetry is not None:
                env['REBOT_TELEMETRY_PORT'] = str(self.telemetry.port)
                env['REBOT_TELEMETRY_TOKEN'] = token
            if self.preview_serial:
                env['REBOT_CAMERA_SERIAL'] = self.preview_serial
            try:
                with self.log_path.open('w', encoding='utf-8') as output:
                    self.started_at = time.time()
                    self.ended_at = None
                    self.process = subprocess.Popen(args, cwd=str(self.project), env=env,
                                                    stdin=subprocess.DEVNULL, stdout=output,
                                                    stderr=subprocess.STDOUT, start_new_session=True)
            except BaseException:
                if self.telemetry is not None:
                    self.telemetry.close()
                    self.telemetry = None
                fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_UN)
                self.lock_file.close()
                self.lock_file = None
                self._unlock_camera()
                raise
            self.stopping = False
            return {'ok': True, 'message': '正在移动到 HOME；通过到位检查后将自动跟随'}

    def live_telemetry(self):
        with self.lock:
            if not self._running() or self.telemetry is None:
                return None
            return self.telemetry.latest()

    def camera_frame(self):
        with self.lock:
            live = self._running() or (self.preview_process is not None and self.preview_process.poll() is None)
            if not live or not self.preview_path:
                return None
            try:
                if time.time() - self.preview_path.stat().st_mtime > 1.0:
                    return None
                return self.preview_path.read_bytes()
            except OSError:
                return None

    def camera_frame_info(self):
        with self.lock:
            if not self.preview_path:
                return None, None
            try:
                stat = self.preview_path.stat()
                rate_path = Path(str(self.preview_path) + '.fps')
                fps = float(rate_path.read_text()) if time.time() - rate_path.stat().st_mtime < 2 else None
                return stat.st_mtime_ns, fps if fps is not None and 0 <= fps <= 240 else None
            except (OSError, ValueError):
                try:
                    return self.preview_path.stat().st_mtime_ns, None
                except OSError:
                    return None, None

    def follow(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('无效跟随状态')
        with self.lock:
            if not self._ready():
                raise RuntimeError('真机程序尚未就绪或正在退出')
            self.process.send_signal(signal.SIGUSR1 if enabled else signal.SIGUSR2)
            return {'ok': True, 'message': '已请求开始跟随' if enabled else '已请求暂停跟随'}

    def shutdown(self):
        with self.lock:
            if not self._running():
                raise RuntimeError('网页没有运行中的真机程序')
            if not self.stopping:
                self.stopping = True
                self.process.send_signal(signal.SIGINT)
            return {'ok': True, 'message': '已请求退出；程序将保持、回零并失能'}
