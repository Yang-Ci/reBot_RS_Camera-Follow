import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'web'))
from home_manager import HomeManager
from robot_process import RobotProcess
from robot_follow.runtime.home_profiles import save_profile


class HomeManagerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.manager = HomeManager(self.root)
        self.path = save_profile(self.manager.directory, '用户甲', [0] * 6, [.1, .2, .3])

    def test_list_and_check_revalidate_before_use(self):
        item = self.manager.list()[0]
        self.assertEqual(item['name'], '用户甲')
        with mock.patch.object(self.manager, 'validate', return_value={'valid': True}) as validator:
            self.assertEqual(self.manager.checked_path(item['id']), self.path)
            validator.assert_called_once_with('用户甲', [0.] * 6, save=False)

    def test_tampered_or_external_paths_rejected(self):
        for profile_id in ('../other', 'a'*63, '', None):
            with self.subTest(profile_id=profile_id), self.assertRaises(ValueError):
                self.manager.read(profile_id)
        data = json.loads(self.path.read_text())
        data['name'] = '其他用户'
        self.path.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            self.manager.read(self.path.stem)
        self.assertEqual(self.manager.list(), [])

    def test_failed_validation_is_reported(self):
        result = subprocess.CompletedProcess([], 1, '', 'HOME_ERROR=HOME 超出工作空间\n')
        with mock.patch('home_manager.subprocess.run', return_value=result):
            with self.assertRaisesRegex(ValueError, '超出工作空间'):
                self.manager.validate('用户甲', [0]*6, save=True)

    def test_save_is_blocked_while_controller_running(self):
        robot = RobotProcess(self.root)
        with mock.patch.object(robot, '_running', return_value=True), \
                mock.patch.object(robot.homes, 'validate') as validate:
            with self.assertRaisesRegex(RuntimeError, '退出'):
                robot.save_home('用户甲', [0]*6)
            validate.assert_not_called()

    def test_gravity_save_refuses_stale_or_unstable_feedback(self):
        robot = RobotProcess(self.root)
        robot.process_mode = 'gravity'
        row = {'phase': 'gravity', 'home_ready': '1', 'fb_valid': '1', 'fb_age_ms': '10',
               **{f'q_act_{i}': '0' for i in range(1, 7)}}
        robot.telemetry = mock.Mock()
        with mock.patch.object(robot, '_ready', return_value=True), \
                mock.patch.object(robot.homes, 'validate') as validate:
            for sample in ((row, .4), ({**row, 'home_ready': '0'}, .01)):
                robot.telemetry.latest.return_value = sample
                with self.assertRaisesRegex(RuntimeError, '新鲜的实际反馈'):
                    robot.save_gravity_home('测试')
            validate.assert_not_called()

    def test_gravity_save_does_not_claim_success_when_return_was_aborted(self):
        robot = RobotProcess(self.root)
        robot.log_path = self.root / 'return.log'
        robot.log_path.write_text('[Return-to-zero error / 回零错误] CAN timeout')
        process = mock.Mock(returncode=0)
        process.poll.return_value = 0
        with mock.patch.object(robot.homes, 'validate') as validate:
            robot._finish_gravity_home(process, '测试', [0.]*6)
            self.assertEqual(robot.home_save['state'], 'failed')
            self.assertIn('未确认回零', robot.home_save['error'])
            validate.assert_not_called()

    def test_gravity_save_marks_failed_if_handoff_fails(self):
        robot = RobotProcess(self.root)
        robot.ready_path = self.root / 'ready'
        robot.ready_path.write_text('stop_failed=12345')
        process = mock.Mock(pid=12345)
        process.poll.return_value = None
        process.wait.side_effect = subprocess.TimeoutExpired('fake', .2)
        with mock.patch.object(robot.homes, 'validate') as validate:
            robot._finish_gravity_home(process, '测试', [0.]*6)
            self.assertEqual(robot.home_save['state'], 'failed')
            self.assertIn('交接失败', robot.home_save['error'])
            validate.assert_not_called()

    def test_gravity_is_exclusive_and_shutdown_can_be_retried(self):
        import signal
        robot = RobotProcess(self.root)
        python = self.root / '.venv-arm/bin/python'
        python.parent.mkdir(parents=True)
        python.touch()
        class Process:
            pid = 12345
            returncode = None
            signals = []
            def poll(self): return self.returncode
            def send_signal(self, signum): self.signals.append(signum)
        with mock.patch.object(robot, '_other_controller', return_value=False), \
                mock.patch('robot_process.subprocess.Popen', return_value=Process()) as popen:
            robot.start_gravity()
            self.assertEqual(robot.status()['process_mode'], 'gravity')
            self.assertIn('pass_fds', popen.call_args.kwargs)
            with self.assertRaises(RuntimeError): robot.start()
            with self.assertRaises(RuntimeError): robot.follow(True)
            with self.assertRaises(RuntimeError): robot.save_home('甲', [0]*6)
            robot.shutdown()
            self.assertEqual(robot.process.signals, [signal.SIGINT])
            robot.ready_path.write_text('stop_failed=12345')
            self.assertFalse(robot.status()['stopping'])
            robot.shutdown()
            self.assertEqual(robot.process.signals, [signal.SIGINT, signal.SIGINT])
            robot.process.returncode = 0
            robot.status()

    def test_start_passes_only_validated_profile_path(self):
        robot = RobotProcess(self.root)
        python = self.root / '.venv-arm/bin/python'
        python.parent.mkdir(parents=True)
        python.touch()
        class Process:
            pid = 12345
            returncode = None
            def poll(self): return self.returncode
        with mock.patch.object(robot, '_other_controller', return_value=False), \
                mock.patch.object(robot.homes, 'validate', return_value={'valid': True}) as validator, \
                mock.patch('robot_process.subprocess.Popen', return_value=Process()) as popen:
            robot.start(home_profile=self.path.stem)
            args = popen.call_args.args[0]
            self.assertEqual(args[args.index('--home-file')+1], str(self.path))
            validator.assert_called_once()
            robot.process.returncode = 0
            robot.status()


if __name__ == '__main__':
    unittest.main()
