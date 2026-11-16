"""Exercise teaching process lifecycle with a fake robot, never touching CAN."""
import os
import fcntl
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class GravityProcessTests(unittest.TestCase):
    def test_direct_start_requires_controller_lock(self):
        env = os.environ.copy()
        env.pop('REBOT_CONTROL_LOCK_FD', None)
        proc = subprocess.run([sys.executable, str(ROOT / 'tools/gravity_home.py')],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, env=env, timeout=10)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn('控制器互斥锁不可用', proc.stdout)

    def test_start_and_exit_with_fake_robot(self):
        launcher = '''
import importlib.util, runpy, sys, types
from pathlib import Path
root = Path(sys.argv[1])
sys.path.insert(0, str(root))
spec = importlib.util.spec_from_file_location('fake_gravity_tests', root / 'tests/test_gravity_session.py')
fake = importlib.util.module_from_spec(spec); spec.loader.exec_module(fake)
module = types.ModuleType('rebot_adapter'); module.RebotAdapter = fake.FakeRobot
sys.modules['rebot_adapter'] = module
runpy.run_path(str(root / 'tools/gravity_home.py'), run_name='__main__')
'''
        with tempfile.TemporaryDirectory() as tmp:
            ready = Path(tmp) / 'ready'
            env = os.environ.copy()
            env['REBOT_WEB_READY_PATH'] = str(ready)
            env.pop('REBOT_TELEMETRY_PORT', None)
            env.pop('REBOT_TELEMETRY_TOKEN', None)
            lock = (Path(tmp) / 'controller.lock').open('w')
            self.addCleanup(lock.close)
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            env['REBOT_CONTROL_LOCK_FD'] = str(lock.fileno())
            proc = subprocess.Popen([sys.executable, '-c', launcher, str(ROOT)],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, env=env, pass_fds=(lock.fileno(),))
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and proc.poll() is None and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(ready.exists(), 'fake teaching process never became ready')
                self.assertEqual(ready.read_text(), str(proc.pid))
                proc.send_signal(signal.SIGINT)
                output, _ = proc.communicate(timeout=5)
                self.assertEqual(proc.returncode, 0, output)
                self.assertFalse(ready.exists())
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.communicate()


if __name__ == '__main__': unittest.main()
