"""Integration checks for startup failures using fake camera and arm."""
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
source_path = ROOT / 'tools/offline_smoke_test.py'
source = source_path.read_text(encoding='utf-8').split('# 默认交付配置必须')[0]
namespace = {'__name__': 'startup_cleanup_harness', '__file__': str(source_path)}
exec(compile(source, str(source_path), 'exec'), namespace)
module = namespace['main_module']
np = namespace['np']


class TrackedCamera(namespace['FakeCamera']):
    instances = []
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.instances.append(self)


class TrackedDetector(namespace['FakeDetector']):
    instances = []
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.closed = False
        self.instances.append(self)
    def close(self):
        self.closed = True


class StartupCleanupTests(unittest.TestCase):
    def setUp(self):
        TrackedCamera.instances.clear()
        TrackedDetector.instances.clear()
        namespace['FakeRobot'].instances.clear()
        self.camera = mock.patch.object(sys.modules['gemini2_camera'], 'Gemini2Camera', TrackedCamera)
        self.camera.start()
        self.detector = mock.patch.object(module, 'PalmDetector', TrackedDetector)
        self.detector.start()
        self.fk = mock.patch.object(module, 'compute_tcp_fk_and_jacobian',
                                    lambda robot, q: (np.array([.1, 0, .3]), np.eye(3), np.eye(6)))
        self.fk.start()
        self.addCleanup(self.camera.stop)
        self.addCleanup(self.detector.stop)
        self.addCleanup(self.fk.stop)

    def run_main(self, argv):
        with mock.patch.object(sys, 'argv', ['palm_follow_grasp.py', '--headless'] + argv):
            module.main()

    def test_invalid_gripper_config_is_rejected_before_camera_open(self):
        original = module.load_grasp_cfg
        def invalid(path):
            config = original(path)
            config['gripper']['open_deg'] = float('nan')
            return config
        with mock.patch.object(module, 'load_grasp_cfg', side_effect=invalid):
            with self.assertRaisesRegex(ValueError, '数值必须有限'):
                self.run_main(['--no-arm'])
        self.assertEqual(TrackedCamera.instances, [])

    def test_detector_failure_closes_camera(self):
        with mock.patch.object(module, 'PalmDetector', side_effect=RuntimeError('detector failed')):
            with self.assertRaisesRegex(RuntimeError, 'detector failed'):
                self.run_main(['--no-arm'])
        self.assertTrue(TrackedCamera.instances[-1].stopped)

    def test_reconnected_camera_is_released_on_exit(self):
        class ReconnectingCamera(TrackedCamera):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.first = len(TrackedCamera.instances) == 1
                self.max_frames = 8
            def get_frames(self):
                if self.first:
                    return None, None, None
                return super().get_frames()
        with mock.patch.object(sys.modules['gemini2_camera'], 'Gemini2Camera', ReconnectingCamera):
            self.run_main(['--no-arm'])
        self.assertEqual(len(TrackedCamera.instances), 2)
        self.assertTrue(TrackedCamera.instances[0].stopped)
        self.assertTrue(TrackedCamera.instances[1].stopped)

    def test_reconnect_rejects_wrong_camera_then_retries(self):
        class ReconnectingCamera(TrackedCamera):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.number = len(TrackedCamera.instances)
                self.camera_serial = 'wrong' if self.number == 2 else 'original'
                self.max_frames = 8
            def get_frames(self):
                if self.number == 1:
                    return None, None, None
                return super().get_frames()
        with mock.patch.object(sys.modules['gemini2_camera'], 'Gemini2Camera', ReconnectingCamera):
            self.run_main(['--no-arm'])
        self.assertEqual(len(TrackedCamera.instances), 3)
        self.assertTrue(all(camera.stopped for camera in TrackedCamera.instances))

    def test_home_failure_stops_robot_and_releases_devices(self):
        class FailingHome(namespace['FakeRobot']):
            def move_to_joints(self, *args, **kwargs):
                raise RuntimeError('home failed')
        with mock.patch.object(sys.modules['rebot_adapter'], 'RebotAdapter', FailingHome):
            with self.assertRaisesRegex(RuntimeError, 'home failed'):
                self.run_main([])
        robot = namespace['FakeRobot'].instances[-1]
        self.assertTrue(robot.stopped)
        self.assertEqual(robot.events.count('safe_stop'), 1)
        self.assertTrue(TrackedDetector.instances[-1].closed)
        self.assertTrue(TrackedCamera.instances[-1].stopped)

    def test_post_home_failure_stops_robot_and_releases_devices(self):
        class FailingGripper(namespace['FakeRobot']):
            def get_gripper_deg(self):
                raise RuntimeError('gripper failed')
        with mock.patch.object(sys.modules['rebot_adapter'], 'RebotAdapter', FailingGripper):
            with self.assertRaisesRegex(RuntimeError, 'gripper failed'):
                self.run_main([])
        robot = namespace['FakeRobot'].instances[-1]
        self.assertTrue(robot.stopped)
        self.assertEqual(robot.events.count('safe_stop'), 1)
        self.assertTrue(TrackedDetector.instances[-1].closed)
        self.assertTrue(TrackedCamera.instances[-1].stopped)


if __name__ == '__main__':
    unittest.main()
