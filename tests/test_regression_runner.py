"""The regression runner reports failures and does not launch hardware."""
import io
import subprocess
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import run_regression


class RegressionRunnerTests(unittest.TestCase):
    def test_list_does_not_launch_checks(self):
        with mock.patch.object(run_regression, 'run_check') as run, redirect_stdout(io.StringIO()) as out:
            self.assertEqual(run_regression.main(['--list']), 0)
        run.assert_not_called()
        self.assertIn('fake-device', out.getvalue())
        self.assertIn('Linux only', out.getvalue())

    def test_failure_exit_and_diagnostic_are_preserved(self):
        check = run_regression.Check('broken', ('fake.py',))
        result = mock.Mock(returncode=2, stdout='useful failure detail')
        with mock.patch.object(run_regression.subprocess, 'run', return_value=result):
            ok, detail = run_regression.run_check(check)
        self.assertFalse(ok)
        self.assertIn('exit 2', detail)
        self.assertIn('useful failure detail', detail)

    def test_extended_list_includes_calibration_and_browser(self):
        with redirect_stdout(io.StringIO()) as out:
            self.assertEqual(run_regression.main(['--list', '--extended']), 0)
        self.assertIn('automatic calibration', out.getvalue())
        self.assertIn('browser gesture policy', out.getvalue())

    def test_missing_optional_executable_is_a_failure(self):
        check = run_regression.Check('browser', ('--test', 'test.cjs'), executable='node')
        with mock.patch.object(run_regression.shutil, 'which', return_value=None):
            ok, detail = run_regression.run_check(check)
        self.assertFalse(ok)
        self.assertIn('node', detail)

    def test_timeout_is_reported(self):
        check = run_regression.Check('slow', ('fake.py',))
        with mock.patch.object(run_regression.subprocess, 'run',
                               side_effect=subprocess.TimeoutExpired('fake', 1)):
            ok, detail = run_regression.run_check(check, timeout=1)
        self.assertFalse(ok)
        self.assertIn('timed out', detail)


if __name__ == '__main__':
    unittest.main()
