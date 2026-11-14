"""Run a fake controller through its live telemetry path with CSV disabled."""
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from live_telemetry import LiveTelemetryReceiver
from telemetry import state_from_row


class LivePipelineTests(unittest.TestCase):
    def test_fake_controller_publishes_without_diagnostic_log(self):
        source = (ROOT / 'tools/offline_smoke_test.py').read_text(encoding='utf-8')
        source = source.split('# 默认交付配置必须')[0]
        namespace = {'__name__': 'live_fake_harness',
                     '__file__': str(ROOT / 'tools/offline_smoke_test.py')}
        exec(compile(source, namespace['__file__'], 'exec'), namespace)
        module = namespace['main_module']
        np = namespace['np']
        namespace['FakeCamera'].max_frames = 100
        module.compute_tcp_fk_and_jacobian = lambda robot, q: (
            np.array([.1, 0, .3]), np.eye(3), np.eye(6))
        original_load = module.load_follow_cfg

        def no_log(path):
            cfg = original_load(path)
            cfg['follow']['diag_enable'] = False
            return cfg

        module.load_follow_cfg = no_log
        receiver = LiveTelemetryReceiver('fake-controller-session')
        self.addCleanup(receiver.close)
        env = {'REBOT_TELEMETRY_PORT': str(receiver.port),
               'REBOT_TELEMETRY_TOKEN': receiver.token}
        errors = []

        def run():
            try:
                module.main()
            except BaseException as exc:
                errors.append(exc)

        args = ['palm_follow_grasp.py', '--headless', '--web-control']
        with mock.patch.dict(os.environ, env), mock.patch.object(sys, 'argv', args), \
             mock.patch.object(module.signal, 'signal', return_value=None), \
             mock.patch.object(module.signal, 'SIGUSR1', 1001, create=True), \
             mock.patch.object(module.signal, 'SIGUSR2', 1002, create=True):
            thread = threading.Thread(target=run)
            thread.start()
            sample = None
            deadline = time.monotonic() + 10
            while thread.is_alive() and time.monotonic() < deadline:
                sample = receiver.latest()
                if sample and sample[0].get('following') == '0' and not sample[0].get('phase'):
                    break
                time.sleep(.02)
            thread.join(timeout=10)
            self.assertFalse(thread.is_alive(), 'fake controller failed to exit')
        self.assertFalse(errors, errors)
        self.assertIsNotNone(sample, 'controller did not publish live telemetry')
        row, age = sample
        self.assertEqual(row['following'], '0')
        state = state_from_row(ROOT, row, age)
        self.assertTrue(state['online'])
        self.assertIsNotNone(state['gripper_open_fraction'])
        self.assertTrue(namespace['FakeRobot'].instances[-1].stopped)


if __name__ == '__main__':
    unittest.main()
