"""Exercise fresh-hand gripper mapping with fake devices."""
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
source_path = ROOT / 'tools/offline_smoke_test.py'
source = source_path.read_text(encoding='utf-8').split('# 默认交付配置必须')[0]
namespace = {'__name__': 'gripper_pipeline_harness', '__file__': str(source_path)}
exec(compile(source, str(source_path), 'exec'), namespace)
module = namespace['main_module']
np = namespace['np']


class FreshThenLost(namespace['FakeDetector']):
    def detect(self, image):
        self.i += 1
        return 320, 240, (object() if self.i <= 12 else None)


class RecordingRobot(namespace['FakeRobot']):
    def __init__(self):
        super().__init__()
        self.gripper_commands = []
    def set_gripper_deg(self, deg):
        self.gripper_commands.append((FreshThenLost.last_frame, float(deg)))
        super().set_gripper_deg(deg)


class RecordingDetector(FreshThenLost):
    last_frame = 0
    def detect(self, image):
        result = super().detect(image)
        FreshThenLost.last_frame = self.i
        return result


class GripperPipelineTests(unittest.TestCase):
    def test_fresh_landmarks_drive_gripper_then_missing_landmarks_hold(self):
        namespace['FakeCamera'].max_frames = 25
        namespace['FakeRobot'].instances.clear()
        module.compute_tcp_fk_and_jacobian = lambda robot, q: (
            np.array([.1, 0, .3]), np.eye(3), np.eye(6))
        original_config = module.load_grasp_cfg
        def config(path):
            cfg = original_config(path)
            cfg['gripper']['hold_while_moving'] = False
            return cfg
        with mock.patch.object(sys, 'argv', ['palm_follow_grasp.py', '--headless']), \
             mock.patch.object(module, 'PalmDetector', RecordingDetector), \
             mock.patch.object(module, 'pinch_distance_px', return_value=(200., (0, 0), (1, 1))), \
             mock.patch.object(module, 'load_grasp_cfg', side_effect=config), \
             mock.patch.object(sys.modules['rebot_adapter'], 'RebotAdapter', RecordingRobot):
            module.main()
        robot = namespace['FakeRobot'].instances[-1]
        self.assertTrue(robot.stopped)
        self.assertTrue(robot.gripper_commands)
        self.assertTrue(all(frame <= 12 for frame, _ in robot.gripper_commands))
        self.assertGreater(robot.gripper_commands[-1][1], 200.)


if __name__ == '__main__':
    unittest.main()
