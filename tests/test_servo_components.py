"""Servo component wiring follows configuration and is isolated per run."""
import unittest
from pathlib import Path

import numpy as np

from robot_follow.configuration.follow import FollowConfiguration
from robot_follow.control.components import ServoComponents

ROOT = Path(__file__).resolve().parents[1]


class ServoComponentsTests(unittest.TestCase):
    def test_components_capture_config_and_keep_independent_state(self):
        config = FollowConfiguration()
        config.apply_follow_cfg(config.load_follow_cfg(ROOT / 'config/follow.yaml'))
        first = ServoComponents.from_configuration(config)
        second = ServoComponents.from_configuration(config)
        self.assertEqual(first.target_filter.q, config.kf_q)
        np.testing.assert_allclose(first.target_filter.r_vector,
                                   [config.kf_r_xy, config.kf_r_xy, config.kf_r_z])
        np.testing.assert_allclose(first.axis_noise.refs,
                                   [config.kf_axis_adapt_ref_xy, config.kf_axis_adapt_ref_xy,
                                    config.kf_axis_adapt_ref_z])
        self.assertEqual(first.state.ff_taper_now, config.ff_taper_dist)
        first.state.actual_velocity[0] = 1.
        first.target_filter.x[0] = 2.
        first.axis_noise.scales[0] = 1.5
        self.assertEqual(second.state.actual_velocity[0], 0.)
        self.assertEqual(second.target_filter.x[0], 0.)
        self.assertEqual(second.axis_noise.scales[0], 1.)


if __name__ == '__main__':
    unittest.main()
