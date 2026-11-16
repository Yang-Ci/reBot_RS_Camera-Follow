"""Own one local controller process; never opens CAN or accepts shell commands."""
import os
import fcntl
import secrets
import signal
import subprocess
import threading
import time
import json
import math
from pathlib import Path

from live_telemetry import LiveTelemetryReceiver
from home_manager import HomeManager


def validate_motion_settings(settings):
    """Reject missing, non-numeric and out-of-range values before any robot action."""
    if not isinstance(settings, dict) or set(settings) != {'gain', 'gain_x', 'max_vel'}:
        raise ValueError('请完整填写跟随增益、水平增益和最大速度')
    limits = {'gain': (3.5, '跟随增益'), 'gain_x': (2.5, '水平增益'),
              'max_vel': (0.70, '最大速度')}
    result = {}
    for key, (maximum, label) in limits.items():
        value = settings[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not 0 < value <= maximum or not math.isfinite(value)):
            raise ValueError(f'{label}必须大于 0 且不超过 {maximum}')
        result[key] = float(value)
    return result


def speed_presets(project):
    """Read the same base and camera profiles used by the controller."""
    import yaml
    follow = {'gain': 2.5, 'gain_x': 1.8, 'max_vel': 0.30}
    for name in ('follow.yaml', 'follow_gemini336.yaml'):
        path = Path(project) / 'config' / name
        if not path.exists() and name == 'follow.yaml':
            continue
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict) or not isinstance(data.get('follow', {}), dict):
            raise ValueError(f'{name} 跟随配置无效')
        follow.update(data.get('follow', {}))
    configured = validate_motion_settings({key: follow[key] for key in ('gain', 'gain_x', 'max_vel')})
    return {'slow': {'gain': 1.5, 'gain_x': 1.5, 'max_vel': 0.10},
            'medium': {'gain': 1.9, 'gain_x': 1.9, 'max_vel': 0.25},
            'configured': configured}


class RobotProcess:
    def __init__(self, project):
        self.project = Path(project)
        self.homes = HomeManager(self.project)
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
        self.motion_settings = None
        self.speed_mode = None
        self.camera_fps = None
        self.stationary_joint_hold = None
        self.process_mode = None
        self.home_save = None

    def cameras(self):
        python = self.project / '.venv-arm/bin/python'
        try:
            result = subprocess.run([str(python), str(self.project / 'web/camera_preview_process.py'), '--list'],
                                    capture_output=True, text=True, timeout=8, check=True)
            line = next(line for line in result.stdout.splitlines() if line.startswith('CAMERA_LIST='))
            return json.loads(line.split('=', 1)[1])
        except (OSError, subprocess.SubprocessError, StopIteration, ValueError) as exc:
            raise RuntimeError(f'无法列出相机: {exc}') from exc

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
            raise ValueError('请选择相机')
        selected = next((item for item in self.cameras() if item['serial'] == serial), None)
        if selected is None:
            raise ValueError('所选相机不在线，请刷新列表')
        model = selected.get('model', 'gemini336')
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
                     str(self.project / 'web/camera_preview_process.py'), '--preview', serial, str(self.preview_path), model],
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
                if any(Path(os.fsdecode(v)).name in ('palm_follow_grasp.py', 'gravity_home.py') for v in argv if v):
                    return True
            except (OSError, ValueError):
                continue
        return False

    def status(self):
        with self.lock:
            running = self._running()
            gravity_fault = False
            if running and self.process_mode == 'gravity' and self.ready_path:
                try:
                    marker = self.ready_path.read_text().strip()
                    gravity_fault = marker in (f'fault={self.process.pid}', f'stop_failed={self.process.pid}')
                    if self.stopping and marker == f'stop_failed={self.process.pid}':
                        self.stopping = False
                except OSError:
                    pass
            preview_running = self.preview_process is not None and self.preview_process.poll() is None
            preview_failed = self.preview_process is not None and not preview_running
            return {'running': running, 'ready': self._ready(), 'stopping': self.stopping and running,
                    'process_mode': self.process_mode if running else None, 'gravity_fault': gravity_fault,
                    'home_save': dict(self.home_save) if self.home_save else None,
                    'motion_settings': self.motion_settings if running else None,
                    'speed_mode': self.speed_mode if running else None,
                    'camera_fps': self.camera_fps if running else None,
                    'stationary_joint_hold': self.stationary_joint_hold if running else None,
                    'preview_running': preview_running, 'preview_failed': preview_failed,
                    'exit_code': None if self.process is None else self.process.poll(),
                    'started_at': self.started_at, 'ended_at': self.ended_at, 'log': self.log_path.name if self.log_path else None}

    def start(self, speed='slow', gripper=False, serial=None, motion=None, camera='gemini336', fps=None, stationary_joint_hold=None, home_profile=None):
        if home_profile is not None and not isinstance(home_profile, str):
            raise ValueError('HOME 选择无效')
        if speed not in ('slow', 'medium', 'configured') or type(gripper) is not bool:
            raise ValueError('无效的速度或夹爪选项')
        if camera not in ('gemini2', 'gemini305', 'gemini336', 'd435i'):
            raise ValueError('不支持的相机型号')
        if stationary_joint_hold is not None and type(stationary_joint_hold) is not bool:
            raise ValueError('静止关节保持选项无效')
        if fps is not None and (camera != 'gemini305' or type(fps) is not int or fps not in (30, 60)):
            raise ValueError('Gemini 305 帧率仅支持 30 或 60 FPS')
        if serial is not None or camera != 'gemini336':
            available = self.cameras()
            matched = next((item for item in available if item.get('serial') == serial), None)
            if not matched or (matched.get('model') or ('gemini305' if 'gemini 305' in matched.get('name', '').lower() else 'gemini336')) != camera:
                raise ValueError('所选设备与相机型号不一致或已离线，请刷新相机列表')
        if motion is not None:
            motion = validate_motion_settings(motion)
        with self.lock:
            if self.home_save and self.home_save['state'] == 'returning':
                raise RuntimeError('HOME 保存和回零尚未完成')
            if self._running():
                raise RuntimeError('网页已启动真机程序')
            home_file = self.homes.checked_path(home_profile) if home_profile else None
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
                    '--camera', camera, '--headless', '--web-control',
                    '--home-j34-tolerance', '3.5']
            if home_file is not None:
                args += ['--home-file', str(home_file)]
            if fps is not None:
                args += ['--fps', str(fps)]
            if stationary_joint_hold is not None:
                args.append('--stationary-joint-hold' if stationary_joint_hold else '--no-stationary-joint-hold')
            if not gripper:
                args += ['--no-gripper', '--keep-gripper-position']
            if motion is not None:
                args += ['--gain', str(motion['gain']), '--gain-x', str(motion['gain_x']),
                         '--max-vel', str(motion['max_vel'])]
                if speed in ('slow', 'medium'):
                    args += ['--home-speed', '4']
            elif speed == 'slow':
                args += ['--gain', '1.5', '--gain-x', '1.5', '--max-vel', '0.10', '--home-speed', '4']
            elif speed == 'medium':
                args += ['--gain', '1.9', '--gain-x', '1.9', '--max-vel', '0.25', '--home-speed', '4']
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
                    self.process_mode = 'follow'
                    self.motion_settings = motion
                    self.speed_mode = speed
                    self.camera_fps = fps
                    self.stationary_joint_hold = (stationary_joint_hold if stationary_joint_hold is not None
                                                  else camera in ('gemini305', 'gemini336'))
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
            return {'ok': True, 'message': '正在移动到 HOME；到位后等待按 R 或张开手掌 3 秒'}

    def start_gravity(self):
        with self.lock:
            if self.home_save and self.home_save['state'] == 'returning':
                raise RuntimeError('HOME 保存和回零尚未完成')
            if self._running():
                raise RuntimeError('请先退出当前机械臂程序')
            python = self.project / '.venv-arm/bin/python'
            if not python.is_file():
                raise RuntimeError('未找到 .venv-arm/bin/python，请先配置机械臂环境')
            self._stop_preview()
            logs = self.project / 'logs'
            logs.mkdir(exist_ok=True)
            self.lock_file = (logs / '.web_controller.lock').open('a')
            try:
                fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                if self._other_controller():
                    raise RuntimeError('检测到其他机械臂程序')
            except (BlockingIOError, RuntimeError):
                self.lock_file.close(); self.lock_file = None
                raise RuntimeError('已有控制程序或网页会话，不能重复占用 CAN')
            token = secrets.token_hex(12)
            try:
                self.telemetry = LiveTelemetryReceiver(token)
            except OSError:
                self.telemetry = None
            self.ready_path = logs / f'.web_ready_{token}'
            self.preview_path = None
            self.log_path = logs / f'web_gravity_{time.strftime("%Y%m%d_%H%M%S")}_{token[:6]}.log'
            env = os.environ.copy()
            env.pop('REBOT_TELEMETRY_PORT', None); env.pop('REBOT_TELEMETRY_TOKEN', None)
            env['REBOT_WEB_READY_PATH'] = str(self.ready_path)
            env['REBOT_CONTROL_LOCK_FD'] = str(self.lock_file.fileno())
            if self.telemetry is not None:
                env['REBOT_TELEMETRY_PORT'] = str(self.telemetry.port)
                env['REBOT_TELEMETRY_TOKEN'] = self.telemetry.token
            try:
                with self.log_path.open('w', encoding='utf-8') as output:
                    self.started_at = time.time(); self.ended_at = None
                    self.process = subprocess.Popen(
                        [str(python), '-u', str(self.project / 'tools/gravity_home.py')],
                        cwd=str(self.project), env=env, stdin=subprocess.DEVNULL,
                        stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
                        pass_fds=(self.lock_file.fileno(),))
                self.process_mode = 'gravity'
                self.motion_settings = self.speed_mode = self.camera_fps = self.stationary_joint_hold = None
                self.stopping = False
            except BaseException:
                if self.telemetry is not None:
                    self.telemetry.close(); self.telemetry = None
                fcntl.flock(self.lock_file.fileno(), fcntl.LOCK_UN)
                self.lock_file.close(); self.lock_file = None
                raise
            return {'ok': True, 'message': '正在启动重力示教；就绪后可轻推并填入实际角度'}

    def save_home(self, name, degrees):
        with self.lock:
            if self._running():
                raise RuntimeError('请先退出真机程序，再保存 HOME')
            return self.homes.validate(name, degrees, save=True)

    def _stable_gravity_pose(self):
        if self.process_mode != 'gravity' or not self._ready() or self.telemetry is None:
            raise RuntimeError('重力示教尚未就绪或正在退出')
        sample = self.telemetry.latest()
        if sample is None:
            raise RuntimeError('没有新鲜的实际关节反馈')
        row, age = sample
        try:
            feedback_age = float(row['fb_age_ms']) / 1000. + age
            q = [float(row[f'q_act_{i}']) for i in range(1, 7)]
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError('实际关节反馈无效') from exc
        if (row.get('phase') != 'gravity' or row.get('home_ready') != '1'
                or row.get('fb_valid') != '1' or not math.isfinite(feedback_age)
                or not 0 <= feedback_age <= .3 or not all(math.isfinite(x) for x in q)):
            raise RuntimeError('请保持机械臂静止，等待新鲜的实际反馈')
        return [math.degrees(x) for x in q]

    def save_gravity_home(self, name, replace=False):
        if (not isinstance(name, str) or not name.strip() or len(name.strip()) > 40
                or any(ord(char) < 32 for char in name) or type(replace) is not bool):
            raise ValueError('请输入有效的 HOME 名称')
        name = name.strip()
        with self.lock:
            if self.home_save and self.home_save['state'] == 'returning':
                raise RuntimeError('HOME 保存和回零正在进行')
            if not replace and any(item['name'] == name for item in self.homes.list()):
                raise ValueError('同名 HOME 已存在，请确认替换')
            degrees = self._stable_gravity_pose()
            self.homes.validate(name, degrees, save=False)
            current = self._stable_gravity_pose()
            if max(abs(a-b) for a, b in zip(current, degrees)) > .3:
                raise RuntimeError('机械臂姿态已变化，请保持静止后重试')
            process = self.process
            self.stopping = True
            if self.ready_path:
                try: self.ready_path.unlink()
                except FileNotFoundError: pass
            try:
                process.send_signal(signal.SIGINT)
            except OSError:
                self.stopping = False
                raise
            self.home_save = {'state': 'returning', 'name': name}
            threading.Thread(target=self._finish_gravity_home,
                             args=(process, name, degrees), daemon=True).start()
            return {'ok': True, 'message': '已记录姿态，正在回零；完成后自动保存 HOME'}

    def _finish_gravity_home(self, process, name, degrees):
        while process.poll() is None:
            try:
                process.wait(timeout=.2)
            except subprocess.TimeoutExpired:
                with self.lock:
                    try: marker = self.ready_path.read_text().strip() if self.ready_path else ''
                    except OSError: marker = ''
                    if marker == f'stop_failed={process.pid}':
                        self.home_save = {'state': 'failed', 'error': '回零交接失败，姿态未保存；请扶持机械臂并检查急停和反馈'}
                        return
        try:
            log = self.log_path.read_text(encoding='utf-8', errors='replace') if self.log_path else ''
        except OSError:
            log = ''
        return_sent = ('Zero sent and hold complete' in log
                       and '[Return-to-zero error / 回零错误]' not in log)
        if process.returncode != 0 or not return_sent:
            result = {'state': 'failed', 'error': '未确认回零指令完成，姿态未保存；请检查机械臂和程序信息'}
        else:
            try:
                saved = self.homes.validate(name, degrees, save=True)
                result = {'state': 'saved', **saved}
            except Exception as exc:
                result = {'state': 'failed', 'error': f'已回零，但 HOME 保存失败：{exc}'}
        with self.lock:
            self.home_save = result

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
            if self.process_mode == 'gravity':
                raise RuntimeError('重力示教期间不能启动手掌跟随')
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
                if self.process_mode == 'gravity' and self.ready_path:
                    try:
                        self.ready_path.unlink()
                    except FileNotFoundError:
                        pass
                self.process.send_signal(signal.SIGINT)
            return {'ok': True, 'message': '已请求退出；程序将保持、回零并失能'}
