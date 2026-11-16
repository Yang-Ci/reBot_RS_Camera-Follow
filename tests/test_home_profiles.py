import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from robot_follow.runtime.home_profiles import profile_path, save_profile, validate_home_samples


class HomeProfileTests(unittest.TestCase):
    def setUp(self):
        self.samples = np.zeros((5, 6))
        self.stamps = [9.4, 9.5, 9.6, 9.7, 9.8]
        self.workspace = {'min': [-1, -1, 0], 'max': [1, 1, 1], 'max_radius': .8}

    def validate(self, samples=None, stamps=None, now=10., position=(.1, .2, .3)):
        return validate_home_samples(self.samples if samples is None else samples,
            self.stamps if stamps is None else stamps, now, [-1]*6, [1]*6,
            lambda _: position, self.workspace)

    def test_fresh_stationary_pose(self):
        q, p = self.validate()
        np.testing.assert_array_equal(q, np.zeros(6))
        np.testing.assert_allclose(p, [.1, .2, .3])

    def test_invalid_stale_moving_and_out_of_bounds_are_rejected(self):
        moving = self.samples.copy(); moving[-1, 0] = .01
        outside = self.samples.copy(); outside[:, 0] = 1.1
        for kwargs in ({'now': 11.}, {'samples': moving}, {'samples': outside},
                       {'stamps': [9.4]*5}, {'position': (0, 0, -1)}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.validate(**kwargs)

    def test_profiles_are_named_and_isolated(self):
        with tempfile.TemporaryDirectory() as tmp:
            a=save_profile(tmp, '小李', np.zeros(6), [.1, .2, .3])
            b=save_profile(tmp, '小王', np.ones(6), [.1, .2, .3])
            self.assertNotEqual(a, b)
            self.assertEqual(json.loads(a.read_text())['name'], '小李')
            self.assertEqual(json.loads(b.read_text())['home_joint_pos'], [1]*6)
            self.assertEqual(profile_path(tmp, '../../file').parent, Path(tmp))
            self.assertFalse((Path(tmp)/'home_pose.json').exists())


if __name__ == '__main__':
    unittest.main()
