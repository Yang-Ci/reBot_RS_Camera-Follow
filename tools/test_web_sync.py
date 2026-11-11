#!/usr/bin/env python3
"""Offline checks for shared camera transform and telemetry validity."""
import csv
import importlib.util
import os
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('dashboard', ROOT / 'web/server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.old_project = server.PROJECT
        server.PROJECT = Path(self.tmp.name)
        (server.PROJECT / 'logs').mkdir()
        (server.PROJECT / 'config').mkdir()
        (server.PROJECT / 'config/joint_offset.json').write_text('{"joint_offset_rad": [0.1,0,0,0,0,0]}')
        self.path = server.PROJECT / 'logs/servo_test.csv'

    def tearDown(self):
        server.PROJECT = self.old_project
        self.tmp.cleanup()

    def write(self, **values):
        row = {'fb_valid': '0', 'following': '0', 'visible': '1', 'hold_latch': '0'}
        row.update({f'q_cmd_{i}': str(i / 10) for i in range(1, 7)})
        row.update({f'q_act_{i}': str(i / 20) for i in range(1, 7)})
        row.update(values)
        with self.path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())
            writer.writeheader()
            writer.writerow(row)

    def test_feedback_priority_and_command_fallback(self):
        self.write(fb_valid='1', q_cmd_valid='1')
        state = server.latest_state()
        self.assertEqual(state['source'], 'actual')
        self.assertEqual(state['joints'][0], .05)
        self.assertAlmostEqual(state['model_joints'][0], .15)
        (server.PROJECT / 'config/joint_offset.json').unlink()
        self.assertIsNone(server.latest_state()['model_joints'])
        self.write(q_cmd_valid='1')
        state = server.latest_state()
        self.assertEqual(state['source'], 'command')
        self.assertEqual(state['joints'][0], .1)

    def test_estimated_model_uses_feedback_and_paired_command(self):
        self.write(fb_valid='1', q_cmd_valid='1',
                   **{f'q_fb_cmd_{i}': str(i / 20) for i in range(1, 7)})
        state = server.latest_state()
        self.assertEqual(state['source'], 'actual')
        self.assertAlmostEqual(state['joints'][0], .05)
        self.assertAlmostEqual(state['estimated_model_joints'][0], .2)
        self.write(fb_valid='1', q_cmd_valid='1', q_fb_cmd_1='nan')
        self.assertIsNone(server.latest_state()['estimated_model_joints'])

    def test_old_waiting_log_does_not_claim_zero_command(self):
        self.write()
        self.assertIsNone(server.latest_state()['joints'])
        self.write(following='1')
        self.assertEqual(server.latest_state()['source'], 'command')

    def test_stale_and_incomplete_logs(self):
        self.assertFalse(server.latest_state()['online'])
        self.write(q_cmd_valid='1')
        with self.path.open('a') as f:
            f.write('partial,record')
        self.assertEqual(server.latest_state()['source'], 'command')
        os.utime(self.path, (time.time() - 5,) * 2)
        self.assertFalse(server.latest_state()['online'])


class CameraModelTests(unittest.TestCase):
    def test_v1_base_holes_and_camera_tilt(self):
        import sys
        import xml.etree.ElementTree as ET
        sys.path.insert(0, str(ROOT))
        from urdf_camera_pose import _fixed_chain, _origin_matrix
        model = ET.parse(ROOT / 'models/wrist_camera/gemini336.urdf').getroot()
        offset = _origin_matrix(model.find("./link[@name='camera_mount_link']/visual/origin"))
        # V1 screw axes (mm) align with those of the previous base.
        for new_x, old_x in ((10.792, 10.99), (-11.208, -11.01)):
            point = offset @ np.array([new_x / 1000, .029999, -.022, 1])
            np.testing.assert_allclose(point[:3], [old_x / 1000, -.014721, .01252], atol=4e-6)
        # A camera mount follows the new plane, which tilts 15 degrees
        # relative to the original. The optical pose must therefore change.
        reference = Path('/home/robot/ReBot_Arm_DigitalTwin_RS/reBotArm_simulator-RS/public/models/wrist-cameras/urdf/gemini2.urdf')
        if reference.exists():
            old = ET.parse(reference).getroot()
            before = _fixed_chain(old, 'gripper_end', 'camera_color_optical_frame')
            after = _fixed_chain(model, 'gripper_end', 'camera_color_optical_frame')
            delta = before[:3, :3].T @ after[:3, :3]
            angle = np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))
            self.assertAlmostEqual(np.rad2deg(angle), 15, places=3)
            self.assertGreater(np.linalg.norm(after[:3, 3] - before[:3, 3]), .03)



if __name__ == '__main__':
    unittest.main()
