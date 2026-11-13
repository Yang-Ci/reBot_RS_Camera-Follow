#!/usr/bin/env python3
"""End-to-end startup gesture with fake camera, arm and telemetry."""
import os
import signal
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from live_telemetry import LiveTelemetryReceiver


class GesturePipelineTests(unittest.TestCase):
    def test_open_palm_after_home_starts_after_three_seconds(self):
        source_path = ROOT / 'tools/offline_smoke_test.py'
        source = source_path.read_text().split('# 默认交付配置必须')[0]
        ns = {'__name__': 'gesture_fake', '__file__': str(source_path)}
        exec(compile(source, str(source_path), 'exec'), ns)
        module, np = ns['main_module'], ns['np']
        ns['FakeCamera'].max_frames = 1200
        module.compute_tcp_fk_and_jacobian = lambda robot, q: (np.array([.1, 0, .3]), np.eye(3), np.eye(6))

        class OpenDetector:
            def __init__(self, *args, **kwargs):
                self.points = [types.SimpleNamespace(x=.5, y=.7) for _ in range(21)]
                for x, mcp in zip((.32, .43, .54, .65), (5, 9, 13, 17)):
                    for index, y in ((mcp, .7), (mcp+1, .55), (mcp+2, .43), (mcp+3, .3)):
                        self.points[index] = types.SimpleNamespace(x=x, y=y)
                self.points[4] = types.SimpleNamespace(x=.1, y=.5)
            def detect(self, image): return 320, 240, self.points
            def close(self): pass
        module.PalmDetector = OpenDetector
        receiver = LiveTelemetryReceiver('gesture-test')
        self.addCleanup(receiver.close)
        handlers, errors = {}, []
        def install(sig, handler):
            if callable(handler): handlers[sig] = handler
            return signal.SIG_DFL
        def run():
            try: module.main()
            except BaseException as exc: errors.append(exc)
        env = {'REBOT_TELEMETRY_PORT': str(receiver.port), 'REBOT_TELEMETRY_TOKEN': receiver.token}
        with mock.patch.dict(os.environ, env), mock.patch.object(sys, 'argv', ['palm_follow_grasp.py', '--headless', '--web-control']), \
             mock.patch.object(module.signal, 'SIGUSR1', 1001, create=True), \
             mock.patch.object(module.signal, 'SIGUSR2', 1002, create=True), \
             mock.patch.object(module.signal, 'signal', side_effect=install):
            thread = threading.Thread(target=run, daemon=True); thread.start()
            visible_at = started_at = None
            try:
                deadline = time.monotonic() + 9
                while thread.is_alive() and time.monotonic() < deadline:
                    sample = receiver.latest()
                    if sample:
                        row, _ = sample
                        if row.get('visible') == '1' and visible_at is None:
                            visible_at = float(row['host_time'])
                            self.assertEqual(row['following'], '0')
                        if row.get('following') == '1':
                            started_at = float(row['host_time']); break
                    time.sleep(.02)
                self.assertIsNotNone(visible_at, errors)
                self.assertIsNotNone(started_at, errors)
                self.assertGreaterEqual(started_at - visible_at, 2.8)
            finally:
                if signal.SIGINT in handlers: handlers[signal.SIGINT](None, None)
                thread.join(timeout=10)
            self.assertFalse(thread.is_alive())
        self.assertFalse(errors, errors)
        self.assertTrue(ns['FakeRobot'].instances[-1].stopped)


if __name__ == '__main__':
    unittest.main()
