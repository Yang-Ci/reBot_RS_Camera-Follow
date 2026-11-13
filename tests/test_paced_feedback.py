"""Hardware-free checks for paced CAN reads and stale-feedback protection."""
import importlib.util
import sys
import threading
import types
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest import mock

import numpy as np

from robot_follow.control.table_safety import feedback_error
from robot_follow.runtime.diagnostics import ActualFeedbackSampler


class PacedFeedbackTests(unittest.TestCase):
    def test_slow_sweep_keeps_five_hz_and_ages_from_start(self):
        robot = types.SimpleNamespace(get_paced_feedback=lambda stop: (np.zeros(6), np.zeros(6)))
        sampler = ActualFeedbackSampler(robot, hz=20, paced=True)
        with mock.patch('robot_follow.runtime.diagnostics.time.monotonic', side_effect=[10., 10.06]):
            self.assertTrue(sampler.sample_once())
        sample = sampler.snapshot()
        self.assertEqual(sample['rate_hz'], 5)
        self.assertAlmostEqual(sample['read_ms'], 60.)
        self.assertEqual(feedback_error(sample, 10.20), '')
        self.assertIn('过期', feedback_error(sample, 10.31))

    def test_failed_sweep_invalidates_previous_snapshot(self):
        reader = mock.Mock(return_value=(np.zeros(6), np.zeros(6)))
        sampler = ActualFeedbackSampler(types.SimpleNamespace(get_paced_feedback=reader), hz=5, paced=True)
        self.assertTrue(sampler.sample_once())
        reader.side_effect = TimeoutError('CAN timeout')
        self.assertFalse(sampler.sample_once())
        self.assertIsNone(sampler.snapshot())

    def test_joint_reads_release_lock_and_pair_each_command(self):
        spec = importlib.util.spec_from_file_location('paced_adapter_test', Path(__file__).resolve().parents[1] / 'rebot_adapter.py')
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'pinocchio': types.ModuleType('pinocchio')}):
            spec.loader.exec_module(module)
        events = []

        class Motor:
            def __init__(self, index):
                self.index = index
            def robstride_get_param_f32(self, parameter, timeout_ms):
                events.append(('read', self.index, parameter, timeout_ms))
                return self.index / 10

        class Arm:
            motors = [Motor(i) for i in range(6)]
            @contextmanager
            def _io_lock_guard(self, timeout):
                events.append(('lock', timeout))
                yield
                events.append(('unlock',))
            def get_command_angles(self):
                events.append(('command',))
                return [180] * 6

        adapter = module.RebotAdapter.__new__(module.RebotAdapter)
        adapter.io = types.SimpleNamespace(arm=Arm())
        stop = threading.Event()
        with mock.patch.object(stop, 'wait', return_value=False) as wait:
            actual, command = adapter.get_paced_feedback(stop)
        np.testing.assert_allclose(actual, np.arange(6) / 10)
        np.testing.assert_allclose(command, np.full(6, np.pi))
        self.assertEqual([event[0] for event in events], ['lock', 'read', 'unlock', 'command'] * 6)
        self.assertEqual(wait.call_count, 5)
        self.assertTrue(all(event[2:] == (0x7019, 50) for event in events if event[0] == 'read'))

    def test_interruption_does_not_publish_partial_sample(self):
        reader = mock.Mock(side_effect=RuntimeError('interrupted'))
        sampler = ActualFeedbackSampler(types.SimpleNamespace(get_paced_feedback=reader), hz=5, paced=True)
        self.assertFalse(sampler.sample_once())
        self.assertIsNone(sampler.snapshot())


if __name__ == '__main__':
    unittest.main()
