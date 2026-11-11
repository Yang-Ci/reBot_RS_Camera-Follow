#!/usr/bin/env python3
"""Run the full signal-controlled program with fake camera and fake arm only."""
import csv
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def child():
    source_path = ROOT / 'tools/offline_smoke_test.py'
    # Reuse its hardware fakes and module stubs; do not execute its scenarios.
    source = source_path.read_text().split('# 默认交付配置必须')[0]
    namespace = {'__name__': 'web_fake_harness', '__file__': str(source_path)}
    exec(compile(source, str(source_path), 'exec'), namespace)
    namespace['FakeCamera'].max_frames = 1200
    def slow_home(self, *args, **kwargs):
        time.sleep(1.5)
    namespace['FakeRobot'].move_to_joints = slow_home
    module = namespace['main_module']
    np = namespace['np']
    module.compute_tcp_fk_and_jacobian = lambda robot, q: (np.array([.10, 0, .30]), np.eye(3), np.eye(6))
    sys.argv = ['palm_follow_grasp.py', '--headless', '--web-control', '--web-auto-follow', '--no-gripper', '--keep-gripper-position']
    module.main()
    assert namespace['FakeRobot'].instances[-1].stopped
    print('FAKE_SAFE_STOP_OK', flush=True)


def parent():
    with tempfile.TemporaryDirectory() as directory:
        ready = Path(directory) / 'ready'
        preview = Path(directory) / 'preview.jpg'
        log = Path(directory) / 'child.log'
        env = os.environ.copy()
        env['REBOT_WEB_READY_PATH'] = str(ready)
        env['REBOT_WEB_PREVIEW_PATH'] = str(preview)
        env.setdefault('MPLCONFIGDIR', '/tmp/rebot-mpl')
        existing = set((ROOT / 'logs').glob('servo_*.csv'))
        with log.open('w') as output:
            process = subprocess.Popen([sys.executable, '-u', __file__, '--child'], cwd=ROOT,
                                       env=env, stdout=output, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 15
                while not preview.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(.05)
                assert preview.exists(), 'no preview during HOME initialization'
                assert not ready.exists(), 'preview should begin before HOME completes'
                assert preview.read_bytes().startswith(b'\xff\xd8')
                while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(.05)
                assert ready.exists(), 'fake controller did not become ready'
                assert int(ready.read_text()) == process.pid
                deadline = time.monotonic() + 3
                while not preview.exists() and time.monotonic() < deadline:
                    time.sleep(.05)
                assert preview.read_bytes().startswith(b'\xff\xd8'), 'no JPEG preview from fake camera'
                time.sleep(.8)
                process.send_signal(signal.SIGUSR2)
                time.sleep(.8)
                process.send_signal(signal.SIGUSR1)
                time.sleep(.4)
                process.send_signal(signal.SIGUSR2)
                time.sleep(.4)
                process.send_signal(signal.SIGINT)
                assert process.wait(timeout=12) == 0
            finally:
                if process.poll() is None:
                    # This child uses only FakeRobot and FakeCamera.
                    process.kill()
                    process.wait()
        text = log.read_text()
        assert 'FAKE_SAFE_STOP_OK' in text, text[-2000:]
        assert '[跟随] HOME 已就绪，自动开始跟随' in text
        assert not ready.exists()
        assert not preview.exists()
        generated = list(set((ROOT / 'logs').glob('servo_*.csv')) - existing)
        assert len(generated) == 1, generated
        with generated[0].open() as stream:
            states = [row['following'] for row in csv.DictReader(stream)]
        assert states[0] == '1' and '0' in states and states[-1] == '0', states[-5:]
        print('FAKE_WEB_CONTROL_OK: ready, follow, pause, safe shutdown')


if __name__ == '__main__':
    child() if '--child' in sys.argv else parent()
