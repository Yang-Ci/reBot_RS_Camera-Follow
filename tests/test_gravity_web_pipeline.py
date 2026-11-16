"""Exercise the managed gravity worker and UDP using only an injected fake robot."""
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from robot_process import RobotProcess

LAUNCHER = '''
import importlib.util, runpy, sys, types
from pathlib import Path
root = Path(sys.argv[1]); sys.path.insert(0, str(root))
spec=importlib.util.spec_from_file_location('fake_gravity_tests', root/'tests/test_gravity_session.py')
fake=importlib.util.module_from_spec(spec); spec.loader.exec_module(fake)
module=types.ModuleType('rebot_adapter'); module.RebotAdapter=fake.FakeRobot
sys.modules['rebot_adapter']=module
runpy.run_path(str(root/'tools/gravity_home.py'),run_name='__main__')
'''


class GravityWebPipelineTests(unittest.TestCase):
    def test_single_action_captures_stable_pose_returns_and_saves(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = Path(tmp); python = project / '.venv-arm/bin/python'
            python.parent.mkdir(parents=True); python.touch()
            robot = RobotProcess(project); real_popen = subprocess.Popen
            completed = LAUNCHER.replace(
                "module=types.ModuleType('rebot_adapter'); module.RebotAdapter=fake.FakeRobot",
                "class CompletedRobot(fake.FakeRobot):\n"
                "    def __init__(self):\n"
                "        super().__init__(); self.stop=lambda: print('Zero sent and hold complete', flush=True)\n"
                "module=types.ModuleType('rebot_adapter'); module.RebotAdapter=CompletedRobot")
            def fake_popen(_args, **kwargs):
                return real_popen([sys.executable, '-c', completed, str(ROOT)], **kwargs)
            def validate(name, degrees, *, save):
                return {'id': 'a'*64, 'name': name, 'degrees': degrees} if save else {'valid': True}
            with mock.patch.object(robot, '_other_controller', return_value=False), \
                    mock.patch('robot_process.subprocess.Popen', side_effect=fake_popen), \
                    mock.patch.object(robot.homes, 'validate', side_effect=validate) as validator:
                robot.start_gravity(); process = robot.process
                try:
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        sample = robot.live_telemetry()
                        if sample and sample[0].get('home_ready') == '1': break
                        time.sleep(.02)
                    else: self.fail('示教反馈未稳定')
                    result = robot.save_gravity_home('测试')
                    self.assertTrue(result['ok'])
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        state = robot.status()['home_save']
                        if state and state['state'] != 'returning': break
                        time.sleep(.02)
                    self.assertEqual(state['state'], 'saved')
                    self.assertEqual(state['name'], '测试')
                    self.assertEqual(state['degrees'], [0.]*6)
                    self.assertEqual(process.returncode, 0)
                    self.assertEqual([call.kwargs['save'] for call in validator.call_args_list], [False, True])
                finally:
                    if process.poll() is None: process.kill(); process.wait()
                    robot.status()

    def test_managed_fake_teaching_telemetry_and_shutdown(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = Path(tmp)
            python = project / '.venv-arm/bin/python'
            python.parent.mkdir(parents=True)
            python.touch()
            robot = RobotProcess(project)
            real_popen = subprocess.Popen
            def fake_popen(_args, **kwargs):
                return real_popen([sys.executable, '-c', LAUNCHER, str(ROOT)], **kwargs)
            with mock.patch.object(robot, '_other_controller', return_value=False), \
                    mock.patch('robot_process.subprocess.Popen', side_effect=fake_popen):
                robot.start_gravity()
                process = robot.process
                try:
                    deadline = time.monotonic() + 6
                    received = None
                    while time.monotonic() < deadline:
                        sample = robot.live_telemetry()
                        if sample and sample[0].get('home_ready') == '1':
                            received = sample[0]
                            break
                        if process.poll() is not None:
                            break
                        time.sleep(.02)
                    log = robot.log_path.read_text(encoding='utf-8')
                    self.assertIsNotNone(received, log)
                    self.assertTrue(robot.status()['ready'])
                    self.assertEqual(received['phase'], 'gravity')
                    self.assertEqual(received['fb_valid'], '1')
                    robot.shutdown()
                    self.assertEqual(process.wait(timeout=5), 0, robot.log_path.read_text())
                    self.assertFalse(robot.status()['running'])
                    self.assertIsNone(robot.telemetry)
                finally:
                    if process.poll() is None:
                        process.kill(); process.wait()
                    robot.status()

    def test_feedback_failure_remains_visible_and_refuses_stale_handoff(self):
        failing = LAUNCHER.replace(
            "module=types.ModuleType('rebot_adapter'); module.RebotAdapter=fake.FakeRobot",
            "class FailingRobot(fake.FakeRobot):\n"
            "    def __init__(self):\n"
            "        super().__init__(); self.reads=0\n"
            "    def get_paced_feedback(self, event):\n"
            "        self.reads += 1\n"
            "        if self.reads > 8: raise TimeoutError('simulated CAN loss')\n"
            "        return super().get_paced_feedback(event)\n"
            "module=types.ModuleType('rebot_adapter'); module.RebotAdapter=FailingRobot")
        with tempfile.TemporaryDirectory() as tmp:
            project = Path(tmp); python = project / '.venv-arm/bin/python'
            python.parent.mkdir(parents=True); python.touch()
            robot = RobotProcess(project); real_popen = subprocess.Popen
            def fake_popen(_args, **kwargs):
                return real_popen([sys.executable, '-c', failing, str(ROOT)], **kwargs)
            with mock.patch.object(robot, '_other_controller', return_value=False), \
                    mock.patch('robot_process.subprocess.Popen', side_effect=fake_popen):
                robot.start_gravity(); process = robot.process
                try:
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline and not robot.status()['gravity_fault']:
                        if process.poll() is not None: break
                        time.sleep(.02)
                    self.assertTrue(robot.status()['gravity_fault'], robot.log_path.read_text())
                    self.assertIsNone(process.poll())
                    robot.shutdown()
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline and robot.status()['stopping']:
                        time.sleep(.02)
                    state = robot.status()
                    self.assertTrue(state['running'])
                    self.assertTrue(state['gravity_fault'])
                    self.assertFalse(state['stopping'])
                    self.assertIn('无法确认安全交接', robot.log_path.read_text())
                finally:
                    if process.poll() is None:
                        process.kill(); process.wait()
                    robot.status()


if __name__ == '__main__': unittest.main()
