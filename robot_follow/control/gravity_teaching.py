"""Pure drag/lock policy for gravity-assisted HOME teaching.

This module computes candidate commands only. It does not connect, enable, or
send commands to motors; a hardware integration must provide fresh feedback,
interlock ownership, safe transitions, and an independently verified stop path.
"""
from __future__ import annotations

import math
import numpy as np


class GravityTeachingPolicy:
    """Generate hold/follow candidates with velocity hysteresis and limits."""

    def __init__(self, gravity, jacobian, torque_limit, *,
                 hold_kp, hold_kd, compliant_kp=8., compliant_kd=1.,
                 transition_s=.5, integral_gain=1., integral_limit=.3,
                 release_linear=.04, release_angular=.08,
                 lock_linear=.02, lock_angular=.04, lock_s=.15,
                 filter_s=.03, max_sample_s=.1):
        self.gravity, self.jacobian = gravity, jacobian
        self.torque_limit = self._vector(torque_limit)
        self.hold_kp, self.hold_kd = self._vector(hold_kp), self._vector(hold_kd)
        self.compliant_kp, self.compliant_kd = float(compliant_kp), float(compliant_kd)
        self.transition_s, self.integral_gain = float(transition_s), float(integral_gain)
        self.integral_limit = float(integral_limit)
        self.release_linear, self.release_angular = float(release_linear), float(release_angular)
        self.lock_linear, self.lock_angular = float(lock_linear), float(lock_angular)
        self.lock_s, self.filter_s, self.max_sample_s = float(lock_s), float(filter_s), float(max_sample_s)
        parameters = (self.compliant_kp, self.compliant_kd, self.transition_s,
                      self.integral_gain, self.integral_limit, self.release_linear,
                      self.release_angular, self.lock_linear, self.lock_angular,
                      self.lock_s, self.filter_s, self.max_sample_s)
        if (not all(math.isfinite(x) and x > 0 for x in parameters)
                or np.any(self.torque_limit <= 0)
                or np.any(self.hold_kp <= 0) or np.any(self.hold_kd <= 0)
                or self.lock_linear >= self.release_linear or self.lock_angular >= self.release_angular):
            raise ValueError('重力示教参数无效')
        self.previous = self.target = self.start_pose = self.started = self.last = None
        self.velocity = np.zeros(6)
        self.integral = np.zeros(6)
        self.quiet_since = None
        self.following = False

    @staticmethod
    def _vector(value):
        array = np.asarray(value, dtype=float)
        if array.shape != (6,) or not np.isfinite(array).all():
            raise ValueError('关节数据无效')
        return array.copy()

    def update(self, q, now):
        q = self._vector(q)
        if not math.isfinite(now):
            raise ValueError('反馈时间无效')
        if self.started is None:
            self.previous = q.copy()
            self.target = q.copy()
            self.start_pose = q.copy()
            self.last = self.started = now
            dt = 0.
        else:
            dt = now - self.last
            if dt <= 0 or dt > self.max_sample_s:
                raise ValueError('关节反馈超时或时间无效')
            raw = (q - self.previous) / dt
            self.velocity += (1. - math.exp(-dt / self.filter_s)) * (raw - self.velocity)
            self.previous = q.copy()
            self.last = now
        matrix = np.asarray(self.jacobian(q), dtype=float)
        if matrix.shape != (6, 6) or not np.isfinite(matrix).all():
            raise ValueError('雅可比矩阵无效')
        spatial = matrix @ self.velocity
        linear, angular = float(np.linalg.norm(spatial[:3])), float(np.linalg.norm(spatial[3:]))
        factor = min(max((now - self.started) / self.transition_s, 0.), 1.)
        blend = factor * factor * (3. - 2. * factor)
        if blend < 1.:
            self.target = self.start_pose + blend * (q - self.start_pose)
            self.integral.fill(0.)
            state = 'STARTUP'
        else:
            if not self.following and (linear > self.release_linear or angular > self.release_angular):
                self.following = True
                self.quiet_since = None
                self.integral.fill(0.)
            if self.following:
                self.target = q.copy()
                self.integral.fill(0.)
                if linear < self.lock_linear and angular < self.lock_angular:
                    if self.quiet_since is None:
                        self.quiet_since = now
                    elif now - self.quiet_since >= self.lock_s:
                        self.following = False
                        self.quiet_since = None
                else:
                    self.quiet_since = None
            if not self.following:
                self.integral += self.integral_gain * (self.target - q) * min(dt, .02)
                np.clip(self.integral, -self.integral_limit, self.integral_limit, out=self.integral)
            state = 'FOLLOW' if self.following else 'LOCKED'
        full_gravity = self._vector(self.gravity(q))
        if np.any(np.abs(full_gravity) + np.abs(self.integral) > self.torque_limit):
            raise ValueError('重力补偿力矩超出限制')
        torque = blend * full_gravity + self.integral
        return {'state': state, 'position': self.target.copy(), 'torque': torque,
                'kp': self.hold_kp + blend * (self.compliant_kp - self.hold_kp),
                'kd': self.hold_kd + blend * (self.compliant_kd - self.hold_kd),
                'linear_speed': linear, 'angular_speed': angular}
