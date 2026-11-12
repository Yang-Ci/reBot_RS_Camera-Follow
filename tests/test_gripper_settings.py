"""Gripper settings must be valid before hardware is initialized."""
import copy
import unittest
from pathlib import Path

from robot_follow.configuration.gripper import GripperSettings, load_grasp_cfg

ROOT = Path(__file__).resolve().parents[1]


class GripperSettingsTests(unittest.TestCase):
    def setUp(self):
        self.config = load_grasp_cfg(ROOT / 'config/grasp.yaml')

    def test_current_profile_and_legacy_aliases(self):
        settings = GripperSettings.from_config(self.config)
        self.assertEqual((settings.closed_px, settings.open_px), (20., 200.))
        self.assertEqual(settings.min_cutoff, 2.)
        self.assertEqual(settings.beta, .01)
        self.assertEqual(settings.filter_mode, 'one_euro')

    def test_invalid_values_rejected(self):
        cases = [('pinch', 'open_px', 10.), ('pinch', 'smoothing', 1.1),
                 ('pinch', 'deadband_deg', -1.), ('pinch', 'oe_min_cutoff', 0.),
                 ('pinch', 'oe_beta', -1.), ('gripper', 'move_speed_threshold', -1.),
                 ('gripper', 'open_deg', float('nan')),
                 ('gripper', 'open_deg', self.config['gripper']['closed_deg'])]
        for section, key, value in cases:
            data = copy.deepcopy(self.config)
            data[section][key] = value
            with self.subTest(section=section, key=key), self.assertRaises(ValueError):
                GripperSettings.from_config(data)


if __name__ == '__main__':
    unittest.main()
