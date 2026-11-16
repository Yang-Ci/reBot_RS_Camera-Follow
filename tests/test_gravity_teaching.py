import unittest
import numpy as np
from robot_follow.control.gravity_teaching import GravityTeachingPolicy


class GravityTeachingTests(unittest.TestCase):
    def policy(self, **kwargs):
        return GravityTeachingPolicy(lambda q: np.zeros(6), lambda q: np.eye(6),
            [10]*6, hold_kp=[50]*6, hold_kd=[3]*6,
            transition_s=.1, lock_s=.1, **kwargs)

    def test_smooth_start_push_and_lock(self):
        policy = self.policy()
        q = np.zeros(6)
        self.assertEqual(policy.update(q, 0)['state'], 'STARTUP')
        self.assertEqual(policy.update(q, .05)['state'], 'STARTUP')
        self.assertEqual(policy.update(q, .10)['state'], 'LOCKED')
        q[0] = .02
        self.assertEqual(policy.update(q, .15)['state'], 'FOLLOW')
        result = None
        for step in range(4, 13):
            result = policy.update(q, step * .05)
        self.assertEqual(result['state'], 'LOCKED')
        np.testing.assert_allclose(result['position'], q)
        np.testing.assert_allclose(result['kp'], [8]*6)

    def test_bad_feedback_or_torque_rejected(self):
        policy = self.policy()
        policy.update(np.zeros(6), 0)
        for q, t in [(np.zeros(6), .2), ([float('nan')]*6, .05)]:
            with self.subTest(q=q, t=t), self.assertRaises(ValueError):
                policy.update(q, t)
        policy = self.policy()
        policy.gravity = lambda q: np.full(6, 11.)
        with self.assertRaisesRegex(ValueError, '力矩'):
            policy.update(np.zeros(6), 0)

    def test_joint_angles_do_not_restrict_dragging(self):
        policy = self.policy()
        self.assertEqual(policy.update([0, -2.1, -2.2, 0, 0, 0], 0)['state'], 'STARTUP')
        policy.update([0, -2.2, -2.3, 0, 0, 0], .05)

    def test_invalid_timestamp_is_distinct_from_joint_limits(self):
        with self.assertRaisesRegex(ValueError, '反馈时间无效'):
            self.policy().update(np.zeros(6), float('nan'))


if __name__ == '__main__':
    unittest.main()
