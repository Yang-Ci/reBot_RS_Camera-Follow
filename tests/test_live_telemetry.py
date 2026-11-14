"""Exercise the local live telemetry transport without robot hardware."""
import json
import socket
import sys
import time
import unittest
from unittest import mock
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from live_telemetry import LiveTelemetryReceiver
from telemetry import read_latest_state, state_from_row
from robot_follow.runtime.telemetry import LiveTelemetryPublisher, gripper_command_fields


class LiveTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.receiver = LiveTelemetryReceiver('test-session-token')
        self.addCleanup(self.receiver.close)

    def wait_for_sample(self):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            sample = self.receiver.latest()
            if sample:
                return sample
            time.sleep(.01)
        self.fail('live telemetry was not received')

    def test_publisher_to_dashboard(self):
        publisher = LiveTelemetryPublisher(self.receiver.port, self.receiver.token).start()
        self.addCleanup(publisher.close)
        self.assertTrue(publisher.due(10.0))
        self.assertFalse(publisher.due(10.01))
        self.assertTrue(publisher.due(10.06))
        row = {'host_time': time.monotonic(), 'fb_valid': 0, 'q_cmd_valid': 1,
               'following': 1, 'visible': 1, 'hold_latch': 0}
        row.update({f'q_cmd_{i}': i / 10 for i in range(1, 7)})
        publisher.offer(row)
        received, age = self.wait_for_sample()
        state = state_from_row(ROOT, received, age)
        self.assertTrue(state['online'])
        self.assertEqual(state['source'], 'command')
        self.assertEqual(state['joints'], [.1, .2, .3, .4, .5, .6])
        self.assertTrue(state['following'])

    def test_expired_actual_feedback_falls_back_to_command(self):
        row = {'fb_valid': '1', 'q_cmd_valid': '1', 'following': '1',
               'fb_age_ms': '900', 'fb_rate_hz': '20'}
        row.update({f'q_cmd_{i}': str(i / 10) for i in range(1, 7)})
        row.update({f'q_act_{i}': str(i / 20) for i in range(1, 7)})
        fresh = state_from_row(ROOT, row, .05)
        stale = state_from_row(ROOT, row, .2)
        self.assertEqual(fresh['source'], 'actual')
        self.assertEqual(stale['source'], 'command')

    def test_explicitly_invalid_command_is_never_displayed(self):
        row = {'fb_valid': '0', 'q_cmd_valid': '0', 'following': '1', 'visible': '1'}
        row.update({f'q_cmd_{i}': '0' for i in range(1, 7)})
        state = state_from_row(ROOT, row, 0.)
        self.assertEqual(state['source'], 'unavailable')
        self.assertIsNone(state['joints'])
        self.assertTrue(state['following'])
        # Older active logs lack the explicit validity field.
        del row['q_cmd_valid']
        self.assertEqual(state_from_row(ROOT, row, 0.)['source'], 'command')

    def test_gripper_command_is_normalized_and_never_mistaken_for_feedback(self):
        robot = mock.Mock()
        robot.get_gripper_deg.return_value = 145.2
        fields = gripper_command_fields(robot, 5.2, 285.2)
        row = {'fb_valid': '0', 'q_cmd_valid': '0',
               **{k: str(v) for k, v in fields.items()}}
        state = state_from_row(ROOT, row, 0.)
        self.assertAlmostEqual(state['gripper_open_fraction'], .5)
        self.assertIsNone(state['joints'])
        robot.get_gripper_deg.assert_called_once_with()
        for command, expected in [(1000, 1.), (-100, 0.)]:
            row['gripper_cmd_deg'] = str(command)
            self.assertEqual(state_from_row(ROOT, row, 0.)['gripper_open_fraction'], expected)
        row.update(gripper_closed_deg='285.2', gripper_open_deg='5.2', gripper_cmd_deg='145.2')
        self.assertAlmostEqual(state_from_row(ROOT, row, 0.)['gripper_open_fraction'], .5)
        for invalid in ('nan', '', None):
            row['gripper_cmd_deg'] = invalid
            self.assertIsNone(state_from_row(ROOT, row, 0.)['gripper_open_fraction'])
        self.assertEqual(gripper_command_fields(robot, 5, 5), {})
        self.assertIsNone(state_from_row(ROOT, {'fb_valid': '0', 'q_cmd_valid': '0'}, 0.)['gripper_open_fraction'])

    def test_log_disappearing_during_selection_is_offline(self):
        project, directory, file = mock.MagicMock(), mock.MagicMock(), mock.MagicMock()
        project.__truediv__.return_value = directory
        directory.glob.return_value = [file]
        file.stat.side_effect = FileNotFoundError('rotated')
        state = read_latest_state(project, mock.Mock())
        self.assertFalse(state['online'])
        self.assertIn('暂不可用', state['reason'])

    def test_wrong_session_and_stale_samples_are_ignored(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            def send(token, sampled):
                payload = {'token': token, 'row': {'host_time': str(sampled)}}
                sender.sendto(json.dumps(payload).encode(), ('127.0.0.1', self.receiver.port))
            send('wrong-token', time.monotonic())
            send(self.receiver.token, time.monotonic() - 4.0)
            send(self.receiver.token, float('nan'))
        time.sleep(.02)
        self.assertIsNone(self.receiver.latest())

    def test_old_sample_cannot_replace_newer_sample(self):
        now = time.monotonic()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for sampled in (now, now - 1):
                payload = {'token': self.receiver.token, 'row': {'host_time': str(sampled)}}
                sender.sendto(json.dumps(payload).encode(), ('127.0.0.1', self.receiver.port))
        sample, _ = self.wait_for_sample()
        self.assertEqual(float(sample['host_time']), now)


if __name__ == '__main__':
    unittest.main()
