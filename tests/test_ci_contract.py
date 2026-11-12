"""Keep the Linux CI workflow aligned with the local hardware-free runner."""
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class CiContractTests(unittest.TestCase):
    def test_workflow_runs_extended_regression_with_test_dependencies(self):
        workflow = yaml.load((ROOT / '.github/workflows/offline-regression.yml').read_text(encoding='utf-8'),
                             Loader=yaml.BaseLoader)
        self.assertIn('push', workflow['on'])
        self.assertIn('pull_request', workflow['on'])
        self.assertEqual(workflow['permissions']['contents'], 'read')
        job = workflow['jobs']['linux-fake-hardware']
        self.assertEqual(job['runs-on'], 'ubuntu-latest')
        commands = [step.get('run', '') for step in job['steps']]
        self.assertIn('python -m pip install -r requirements-test.txt', commands)
        self.assertIn('python tools/run_regression.py --extended', commands)
        self.assertTrue((ROOT / 'requirements-test.txt').is_file())


if __name__ == '__main__':
    unittest.main()
