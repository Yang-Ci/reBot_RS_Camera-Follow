"""Exercise web pause/resume/quit commands against the fake controller."""
import os
import signal
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from live_telemetry import LiveTelemetryReceiver


class WebCommandPipelineTests(unittest.TestCase):
    def test_pause_resume_and_shutdown(self):
        source_path = ROOT / 'tools/offline_smoke_test.py'
        source = source_path.read_text(encoding='utf-8').split('# 默认交付配置必须')[0]
        namespace = {'__name__': 'web_command_harness', '__file__': str(source_path)}
        exec(compile(source, str(source_path), 'exec'), namespace)
        module = namespace['main_module']
        np = namespace['np']
        namespace['FakeCamera'].max_frames = 1200
        module.compute_tcp_fk_and_jacobian = lambda robot, q: (
            np.array([.1, 0, .3]), np.eye(3), np.eye(6))
        receiver = LiveTelemetryReceiver('web-command-session')
        self.addCleanup(receiver.close)
        handlers, errors = {}, []
        original_init = namespace['FakeRobot'].__init__
        def sdk_sigint(*_args):
            pass
        def sdk_overwrites_sigint(robot):
            original_init(robot)
            module.signal.signal(signal.SIGINT, sdk_sigint)
        namespace['FakeRobot'].__init__ = sdk_overwrites_sigint

        def install(sig, handler):
            if callable(handler):
                handlers[sig] = handler
            return signal.SIG_DFL

        def run():
            try:
                module.main()
            except BaseException as exc:
                errors.append(exc)

        def wait_following(expected, after=0.):
            deadline = time.monotonic() + 5
            while thread.is_alive() and time.monotonic() < deadline:
                sample = receiver.latest()
                if sample:
                    row, _ = sample
                    if (row.get('following') == expected and not row.get('phase')
                            and float(row['host_time']) >= after):
                        return
                time.sleep(.01)
            self.fail(f'no following={expected} state after command; errors={errors}')

        env = {'REBOT_TELEMETRY_PORT': str(receiver.port), 'REBOT_TELEMETRY_TOKEN': receiver.token}
        args = ['palm_follow_grasp.py', '--headless', '--web-control']
        with mock.patch.dict(os.environ, env), mock.patch.object(sys, 'argv', args), \
             mock.patch.object(module.signal, 'SIGUSR1', 1001, create=True), \
             mock.patch.object(module.signal, 'SIGUSR2', 1002, create=True), \
             mock.patch.object(module.signal, 'signal', side_effect=install):
            thread = threading.Thread(target=run, daemon=True)
            thread.start()
            try:
                wait_following('0')
                self.assertIsNot(handlers[signal.SIGINT], sdk_sigint)
                handlers[1001](None, None)
                wait_following('1')
                command_time = time.monotonic()
                handlers[1002](None, None)
                wait_following('0', command_time)
                command_time = time.monotonic()
                handlers[1001](None, None)
                wait_following('1', command_time)
            finally:
                if signal.SIGINT in handlers:
                    handlers[signal.SIGINT](None, None)
                thread.join(timeout=10)
            self.assertFalse(thread.is_alive(), 'fake controller did not stop')
        self.assertFalse(errors, errors)
        self.assertTrue(namespace['FakeRobot'].instances[-1].stopped)


if __name__ == '__main__':
    unittest.main()
