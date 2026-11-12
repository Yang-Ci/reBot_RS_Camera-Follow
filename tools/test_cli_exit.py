"""The CLI must report startup failures to process supervisors."""
import contextlib
import io
import runpy
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
source_path = ROOT / 'tools/offline_smoke_test.py'
source = source_path.read_text(encoding='utf-8').split('# 默认交付配置必须')[0]
namespace = {'__name__': 'cli_failure_harness', '__file__': str(source_path)}
exec(compile(source, str(source_path), 'exec'), namespace)


class CliExitTests(unittest.TestCase):
    def test_invalid_startup_exits_nonzero_with_diagnostic(self):
        stderr = io.StringIO()
        argv = ['palm_follow_grasp.py', '--headless', '--no-arm', '--home-speed', '0']
        with mock.patch.object(sys, 'argv', argv), contextlib.redirect_stderr(stderr):
            with self.assertRaises(SystemExit) as raised:
                runpy.run_path(str(ROOT / 'palm_follow_grasp.py'), run_name='__main__')
        self.assertEqual(raised.exception.code, 1)
        self.assertIn('错误:', stderr.getvalue())
        self.assertIn('home-speed', stderr.getvalue())


if __name__ == '__main__':
    unittest.main()
