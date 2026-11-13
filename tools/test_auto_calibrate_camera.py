#!/usr/bin/env python3
"""No hardware: verify trajectory, calibration and persistence guards."""
import tempfile
import json
import sys
import types
from unittest import mock
import unittest
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import auto_calibrate_camera as c


def transform(rot, trans):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', rot).as_matrix()
    T[:3, 3] = trans
    return T


class CalibrationTests(unittest.TestCase):
    def test_recover_synthetic_transform_and_reject_bad_data(self):
        rng = np.random.default_rng(42)
        X = transform([.15, -.2, .3], [.03, -.04, .08])
        M = transform([.12, .22, -.1], [.4, -.1, .3])
        poses = [transform(rng.uniform(-.4, .4, 3), rng.uniform(-.2, .2, 3)) for _ in range(25)]
        obs = [np.linalg.inv(X) @ np.linalg.inv(p) @ M for p in poses]
        solved, metrics = c.evaluate_calibration(poses, obs)
        np.testing.assert_allclose(solved, X, atol=2e-3)
        self.assertLess(metrics['residual_spread_cm'], 0.01)
        broken = list(obs)
        broken[-1] = broken[-1].copy()
        broken[-1][:3, 3] += [0.3, 0, 0]
        with self.assertRaises(ValueError):
            c.evaluate_calibration(poses, broken)
        degenerate = [transform([0, 0, .03*i], [.01*i, 0, 0]) for i in range(15)]
        with self.assertRaises(ValueError):
            c.evaluate_calibration(degenerate, [np.linalg.inv(X) @ np.linalg.inv(p) @ M for p in degenerate])

    def test_path_rejects_intermediate_collision_and_start_outside_limits(self):
        lo, hi = np.full(6, -1.), np.full(6, 1.)
        fk = lambda q: np.array([.28, .05, .30])
        plan = c.make_plan(np.zeros(6), lo, hi, fk)
        self.assertEqual(len(plan), c.POSE_COUNT)
        for a, b in zip([np.zeros(6)]+plan[:-1], plan):
            self.assertLessEqual(np.max(np.abs(a-b)), c.MAX_STEP + 1e-12)
        q1 = np.zeros(6); q1[0] = .1
        def bad_mid(q):
            return np.array([.40 if .04 < q[0] < .06 else .28, .05, .30])
        self.assertFalse(c.segment_safe(np.zeros(6), q1, lo, hi, bad_mid))
        lo[1] = .1
        with self.assertRaises(ValueError):
            c.make_plan(np.zeros(6), lo, hi, fk)

    def test_ros_synchronized_path_is_checked_too(self):
        lo, hi = np.full(6, -1.), np.full(6, 1.)
        end = np.zeros(6)
        end[0], end[1] = .10, .05
        # Independent ramps leave this region early; the synchronized ROS
        # path passes through it, so checking only the SDK path misses it.
        def fk(q):
            bad = .044 < q[0] < .056 and .020 < q[1] < .030
            return np.array([.40 if bad else .28, .05, .30])
        self.assertFalse(c.segment_safe(np.zeros(6), end, lo, hi, fk))

    def test_changed_x_bound_still_checks_other_bounds(self):
        lo, hi = np.full(6, -1.), np.full(6, 1.)
        q = np.zeros(6)
        fk = lambda _: np.array([.114, -.01, .284])
        self.assertFalse(c.segment_safe(q, q, lo, hi, fk))
        with mock.patch.object(c, 'BOX_MIN', np.array([.10, -.06, .24])):
            self.assertTrue(c.segment_safe(q, q, lo, hi, fk))
            bad_z = lambda _: np.array([.114, -.01, .23])
            self.assertFalse(c.segment_safe(q, q, lo, hi, bad_z))

    def test_marker_width_and_id(self):
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        K = np.array([[400.,0,320],[0,400.,240],[0,0,1.]])
        marker = cv2.aruco.generateImageMarker(d,0,160)
        image = np.full((480,640),255,np.uint8)
        image[160:320,240:400] = marker
        detector = c.MarkerDetector(K,np.zeros(5))
        obs = detector.detect(cv2.cvtColor(image,cv2.COLOR_GRAY2BGR))
        self.assertGreater(obs.width, 120)
        small = np.full((480,640),255,np.uint8)
        small[197:283,277:363] = cv2.aruco.generateImageMarker(d,0,86)
        with self.assertRaises(ValueError):
            detector.detect(cv2.cvtColor(small,cv2.COLOR_GRAY2BGR))

    def test_small_start_is_only_used_for_visibility(self):
        from types import SimpleNamespace
        camera = SimpleNamespace(camera_matrix=np.eye(3), dist_coeffs=np.zeros(5))
        info = c.Observation(np.eye(4), 88, 0.4, (320, 240))
        with mock.patch.object(c, 'observe', side_effect=[c.MarkerUnusable('标记过小'), (np.eye(4), info)]):
            _, _, usable = c.observe_start(camera, None, show=False, allow_small=True)
        self.assertFalse(usable)

    def test_ros_move_waits_for_stable_feedback(self):
        from ros_calibration_client import RosCalibrationArm
        arm = RosCalibrationArm.__new__(RosCalibrationArm)
        target = np.zeros(6)
        readings = [np.ones(6)*v for v in (.01, .007, .004, .003, .002, .002, .002, .002, .002)]
        arm.request = mock.Mock(return_value={'positions': target.tolist()})
        arm.get_current_pos = mock.Mock(side_effect=readings)
        with mock.patch('ros_calibration_client.time.sleep', return_value=None):
            actual = arm.move(target, target, 4., lambda: None)
        self.assertGreaterEqual(arm.get_current_pos.call_count, 5)
        np.testing.assert_allclose(actual, np.ones(6)*.002)

    def test_file_creation_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'result.json'
            c.write_result(path, {'test': 1})
            old = path.read_text()
            with self.assertRaises(FileExistsError):
                c.write_result(path, {'test': 2})
            self.assertEqual(path.read_text(), old)

    def test_gemini2_existing_calibration_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'config').mkdir()
            existing = root / 'config' / 'hand_eye_calibration.json'
            existing.write_text('existing calibration')
            with mock.patch.object(c, 'ROOT', root), \
                 mock.patch.object(sys, 'argv', ['auto', '--camera', 'gemini2', '--run', '--headless']):
                with self.assertRaises(FileExistsError):
                    c.main()
            self.assertEqual(existing.read_text(), 'existing calibration')

    def test_end_to_end_fake_hardware_and_failure_cleanup(self):
        from types import SimpleNamespace
        X = transform([.15, -.2, .3], [.03, -.04, .08])
        target = transform([.12, .22, -.1], [.4, -.1, .3])
        class FakeRobot:
            last = None
            def __init__(self):
                FakeRobot.last = self
                self.q = np.zeros(6)
                self.q_min, self.q_max = np.full(6, -1.), np.full(6, 1.)
                self.io = SimpleNamespace(arm=SimpleNamespace(shutdown_started=False,
                                          is_stopped=False, last_error=None))
                self.stopped = False
            def get_current_pos(self): return self.q.copy()
            def get_command_pos(self): return self.q.copy()
            def move_to_joints(self, q, **kwargs): self.q = np.array(q)
            def hold(self): pass
            def stop(self): self.stopped = True
        class FakeCamera:
            last = None
            next_name = 'Orbbec Gemini 336'
            def __init__(self, **kwargs):
                FakeCamera.last = self
                self.camera_matrix = np.eye(3)
                self.dist_coeffs = np.zeros(5)
                self.width, self.height = 640, 480
                self.device_name, self.camera_serial = FakeCamera.next_name, 'fake-serial-orbbec'
                self.has_color = False
                self.rs = SimpleNamespace(camera_info=SimpleNamespace(serial_number=1))
                device = SimpleNamespace(get_info=lambda x: 'fake-serial')
                self.pipeline = SimpleNamespace(get_active_profile=lambda: SimpleNamespace(get_device=lambda: device))
                self.stopped = False
            def stop(self): self.stopped = True
        def fk(robot, q):
            return transform([q[0]+q[3], q[1]+q[4], q[2]+q[5]], [.28,.05,.30])
        def observation(*args, **kwargs):
            q = np.zeros(6) if FakeRobot.last is None else FakeRobot.last.q
            p = fk(None, q)
            T = np.linalg.inv(X) @ np.linalg.inv(p) @ target
            return T, c.Observation(T, 140, .1, (320,240))
        camera_module = SimpleNamespace(RealSenseCamera=FakeCamera)
        orbbec_module = SimpleNamespace(Gemini2Camera=FakeCamera)
        robot_module = SimpleNamespace(RebotAdapter=FakeRobot, JOINT_OFFSET=np.zeros(6))
        with tempfile.TemporaryDirectory() as d, \
             mock.patch.dict(sys.modules, {'realsense_camera': camera_module, 'gemini2_camera': orbbec_module, 'rebot_adapter': robot_module}), \
             mock.patch.object(c, 'MarkerDetector', return_value=None), \
             mock.patch.object(c, 'observe', side_effect=observation), \
             mock.patch.object(c, 'robot_fk', side_effect=fk), \
             mock.patch.object(c.time, 'sleep', return_value=None):
            path = Path(d)/'result.json'
            with mock.patch.object(sys, 'argv', ['auto', '--run', '--headless', '--output', str(path)]):
                c.main()
            self.assertTrue(path.exists())
            self.assertTrue(FakeRobot.last.stopped and FakeCamera.last.stopped)
            self.assertIn('fake-serial', path.read_text())
            path336 = Path(d)/'result336.json'
            FakeRobot.last = None
            with mock.patch.object(sys, 'argv', ['auto', '--camera', 'gemini336', '--run', '--headless', '--output', str(path336)]):
                c.main()
            result336 = json.loads(path336.read_text())
            self.assertEqual(result336['camera'], 'Orbbec Gemini 336')
            self.assertEqual(result336['camera_serial'], 'fake-serial-orbbec')
            self.assertEqual(result336['image_size'], [640, 480])
            self.assertEqual(result336['image_stream'], 'color')
            FakeCamera.next_name = 'Orbbec Gemini 2'
            FakeRobot.last = None
            path2cam = Path(d)/'result_gemini2.json'
            with mock.patch.object(sys, 'argv', ['auto', '--camera', 'gemini2', '--run', '--headless', '--output', str(path2cam)]):
                c.main()
            result2 = json.loads(path2cam.read_text())
            self.assertEqual(result2['camera'], 'Orbbec Gemini 2')
            self.assertEqual(result2['camera_serial'], 'fake-serial-orbbec')
            self.assertEqual(result2['image_stream'], 'color')
            self.assertEqual(np.asarray(result2['T_tcp_camera']).shape, (4, 4))
            FakeCamera.next_name = 'Orbbec Gemini 336'
            with mock.patch.object(sys, 'argv', ['auto', '--camera', 'gemini2', '--check', '--headless']):
                with self.assertRaisesRegex(ValueError, '实际设备'):
                    c.main()
            self.assertTrue(FakeCamera.last.stopped)
            early_path = Path(d)/'early.json'
            FakeRobot.last = None
            count = [0]
            def lose_marker_after_enough(*args, **kwargs):
                count[0] += 1
                if count[0] == 15:
                    raise c.MarkerLost('marker outside frame')
                return observation(*args, **kwargs)
            with mock.patch.object(c, 'observe', side_effect=lose_marker_after_enough), \
                 mock.patch.object(sys, 'argv', ['auto', '--run', '--headless', '--output', str(early_path)]):
                c.main()
            self.assertGreaterEqual(json.loads(early_path.read_text())['num_samples'], c.MIN_SAMPLES)
            path2 = Path(d)/'failed.json'
            FakeRobot.last = None
            count = [0]
            def fail_during_sample(*args, **kwargs):
                count[0] += 1
                if count[0] == 3: raise RuntimeError('lost marker')
                return observation(*args, **kwargs)
            with mock.patch.object(c, 'observe', side_effect=fail_during_sample), \
                 mock.patch.object(sys, 'argv', ['auto', '--run', '--headless', '--output', str(path2)]):
                with self.assertRaisesRegex(RuntimeError, 'lost marker'):
                    c.main()
            self.assertFalse(path2.exists())
            self.assertTrue(FakeRobot.last.stopped and FakeCamera.last.stopped)
            FakeRobot.last = None
            count = [0]
            def occasional_small_marker(*args, **kwargs):
                count[0] += 1
                if 3 <= count[0] <= 5:
                    raise c.MarkerUnusable('标记过小')
                return observation(*args, **kwargs)
            path3 = Path(d)/'partial.json'
            with mock.patch.object(c, 'observe', side_effect=occasional_small_marker), \
                 mock.patch.object(sys, 'argv', ['auto', '--run', '--headless', '--output', str(path3)]):
                c.main()
            self.assertTrue(path3.exists())
            self.assertTrue(FakeRobot.last.stopped and FakeCamera.last.stopped)

if __name__ == '__main__':
    unittest.main(verbosity=2)
