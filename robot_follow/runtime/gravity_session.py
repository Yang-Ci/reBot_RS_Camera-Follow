"""Gravity teaching lifecycle using the existing single-owner CAN controller."""
from __future__ import annotations

import threading
import time
from collections import deque
import numpy as np

from robot_follow.control.gravity_sender import GravitySender
from robot_follow.control.gravity_teaching import GravityTeachingPolicy


class GravitySession:
    """Sample actual joints and submit bounded gravity-drag commands.

    Construction does not connect hardware. The supplied robot must already be
    initialized and holding its current pose with the existing SDK controller.
    """
    def __init__(self, robot, publisher=None, *, clock=time.monotonic):
        self.robot, self.publisher, self.clock = robot, publisher, clock
        self.stop_event = threading.Event()
        self.sender = GravitySender(robot.io.arm, clock=clock)
        self.policy = None
        self.last_actual = None
        self.last_sample = None
        self.error = ''
        self.last_rate = 0.
        self.recent = deque(maxlen=100)
        self.attached = False

    def _read(self):
        started = self.clock()
        q, _ = self.robot.get_paced_feedback(self.stop_event)
        finished = self.clock()
        q = np.asarray(q, dtype=float)
        if (q.shape != (6,) or not np.isfinite(q).all()
                or finished < started or finished - started > .10):
            raise RuntimeError('实际关节反馈无效或读取超时')
        return q, started, finished

    def start(self):
        arm = self.robot.io.arm
        if self.robot.is_stopping() or arm.motor_count != 7:
            raise RuntimeError('控制器正在停止或电机数量不匹配')
        q, sampled, finished = self._read()
        # The model and configured vendor scale are nominal. Limit commands to
        # half the URDF effort limits until hardware commissioning.
        limits = np.array([self.robot.model.effortLimit[
            self.robot.model.joints[self.robot.model.getJointId(name)].idx_v]
            for name in self.robot.joint_names], dtype=float) * .5
        scale = np.array([1., .98, .98, 1., 1., 1.])
        kp = np.array([motor.kp for motor in arm.config.motors[:6]], dtype=float)
        kd = np.array([motor.kd for motor in arm.config.motors[:6]], dtype=float)
        def jacobian(value):
            _, _, matrix = self.robot.tcp_pose_and_jacobian(value, np.zeros(3))
            return matrix
        self.policy = GravityTeachingPolicy(
            lambda value: self.robot.gravity_torque(value) * scale, jacobian,
            limits, hold_kp=kp, hold_kd=kd)
        command = self.policy.update(q, finished)
        self.sender.publish(command, sampled)
        self.sender.attach()
        self.attached = True
        self.last_actual, self.last_sample = q.copy(), sampled
        self._publish(q, sampled, command, 10.)

    def _publish(self, q, sampled, command, rate):
        self.recent.append((sampled, q.copy()))
        while self.recent and sampled - self.recent[0][0] > .8:
            self.recent.popleft()
        stable = (command['state'] == 'LOCKED' and len(self.recent) >= 5
                  and sampled - self.recent[0][0] >= .4
                  and np.max(np.ptp(np.array([item[1] for item in self.recent]), axis=0)) <= np.deg2rad(.3))
        if self.publisher is None:
            return
        age = max(0., self.clock() - sampled)
        row = {'host_time': self.clock(), 'phase': 'gravity', 'q_cmd_valid': 1,
               'fb_valid': 1, 'fb_age_ms': age * 1000., 'fb_rate_hz': rate,
               'following': 0, 'visible': 0, 'hold_latch': int(command['state'] == 'LOCKED'),
               'servo_failed': 0, 'gravity_state': command['state'], 'home_ready': int(stable)}
        for index in range(6):
            row[f'q_act_{index+1}'] = float(q[index])
            row[f'q_cmd_{index+1}'] = float(command['position'][index])
        self.publisher.offer(row)

    def step(self):
        if not self.attached or self.error:
            raise RuntimeError('重力示教未运行或已故障')
        try:
            if self.robot.is_stopping():
                raise RuntimeError('机械臂控制器正在停止')
            if self.sender.fault.is_set():
                raise RuntimeError('关节反馈过期，已请求位置保持')
            q, sampled, finished = self._read()
            command = self.policy.update(q, finished)
            self.sender.publish(command, sampled)
            self.last_rate = 1. / max(sampled - self.last_sample, .001)
            self.last_actual, self.last_sample = q.copy(), sampled
            self._publish(q, sampled, command, self.last_rate)
            return command['state']
        except Exception as exc:
            self.error = str(exc)
            self.sender.fault.set()
            self.publish_fault()
            raise

    def publish_fault(self):
        if self.publisher is not None and self.error:
            self.publisher.offer({'host_time': self.clock(), 'phase': 'gravity',
                'fb_valid': 0, 'q_cmd_valid': 0, 'following': 0, 'visible': 0,
                'hold_latch': 0, 'servo_failed': 1,
                'error': '示教反馈或控制异常，请支撑机械臂并检查急停：' + self.error})

    def stop(self):
        """Handoff from current measured pose and use the SDK return sequence."""
        if not self.attached:
            return
        q, _, _ = self._read()  # Never hand off using stale or fabricated values.
        self.sender.detach(q)
        self.attached = False
        self.robot.stop()
