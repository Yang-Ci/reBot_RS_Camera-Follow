"""Verify configuration precedence and motion limits without hardware."""
import unittest
from types import SimpleNamespace

from robot_follow.configuration.motion import resolve_motion_options


def args(**overrides):
    values = dict(camera='gemini2', distance=None, gain=None, gain_x=None,
                  max_vel=None, fps=None, color_exposure=None)
    values.update(overrides)
    return SimpleNamespace(**values)


class MotionOptionsTests(unittest.TestCase):
    def setUp(self):
        self.profile = dict(distance=.35, gain=2., gain_x=1.5, max_vel=.5,
                            fps=60, color_exposure_ms=6.)

    def test_profile_defaults_and_camera_specific_values(self):
        options = resolve_motion_options(args(), self.profile)
        self.assertEqual(options.gains, (1.5, 2., 2.))
        self.assertEqual(options.camera_fps, 60)
        self.assertEqual(options.color_exposure_ms, 6.)
        d435i = resolve_motion_options(args(camera='d435i'), self.profile)
        self.assertEqual(d435i.camera_fps, 30)
        self.assertEqual(d435i.color_exposure_ms, 0.)

    def test_command_line_overrides_profile(self):
        options = resolve_motion_options(args(distance=.4, gain=1., gain_x=.8,
                                               max_vel=.1, fps=30, color_exposure=4.), self.profile)
        self.assertEqual(options.desired_depth, .4)
        self.assertEqual(options.gains, (.8, 1., 1.))
        self.assertEqual((options.max_velocity, options.camera_fps, options.color_exposure_ms), (.1, 30, 4.))
        self.assertEqual(self.profile['distance'], .35)

    def test_unsafe_overrides_are_rejected(self):
        for changes in (dict(distance=.1), dict(distance=float('nan')),
                        dict(gain=4.), dict(gain_x=0.), dict(max_vel=.8),
                        dict(max_vel=float('nan')), dict(fps=15)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                resolve_motion_options(args(**changes), self.profile)


if __name__ == '__main__':
    unittest.main()
