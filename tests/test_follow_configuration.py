"""Configuration profiles remain independent across runtime instances."""
import unittest
from pathlib import Path

from robot_follow.configuration.follow import FollowConfiguration
from robot_follow.configuration.validation import validate_follow_cfg
from robot_follow.control.servo import ServoSettings

ROOT = Path(__file__).resolve().parents[1]


class FollowConfigurationTests(unittest.TestCase):
    def test_profiles_are_isolated(self):
        current = FollowConfiguration()
        fallback = FollowConfiguration()
        current_cfg = current.load_follow_cfg(ROOT / 'config/follow.yaml')
        fallback_cfg = fallback.load_follow_cfg(ROOT / 'config/follow_v7.1_fallback.yaml')
        validate_follow_cfg(current_cfg)
        validate_follow_cfg(fallback_cfg)
        current.apply_follow_cfg(current_cfg)
        fallback.apply_follow_cfg(fallback_cfg)
        self.assertTrue(current.kf_axis_adapt_enable)
        self.assertFalse(fallback.kf_axis_adapt_enable)
        self.assertTrue(current.static_vel_hold_enable)
        self.assertFalse(fallback.static_vel_hold_enable)
        current.workspace_min[0] = -0.01
        self.assertNotEqual(current.workspace_min[0], fallback.workspace_min[0])

    def test_servo_settings_capture_independent_workspace(self):
        config = FollowConfiguration()
        values = config.load_follow_cfg(ROOT / 'config/follow.yaml')
        config.apply_follow_cfg(values)
        snapshot = ServoSettings.from_configuration(config)
        self.assertEqual(snapshot.max_vel_joint, config.max_vel_joint)
        original = snapshot.workspace_min[0]
        config.workspace_min[0] = original + .01
        self.assertEqual(snapshot.workspace_min[0], original)

    def test_missing_file_uses_new_instance_defaults(self):
        config = FollowConfiguration()
        defaults = config.load_follow_cfg(ROOT / 'config/this-file-does-not-exist.yaml')
        self.assertEqual(defaults['follow']['distance'], config.desired_depth)
        self.assertEqual(defaults['workspace']['max_radius'], config.workspace_radius)


if __name__ == '__main__':
    unittest.main()
