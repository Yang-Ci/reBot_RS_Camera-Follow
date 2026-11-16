"""Bounded, freshness-checked MIT command bridge for the existing SDK sender."""
from __future__ import annotations

import threading
import time
import numpy as np


class GravitySender:
    """Replace only the SDK's command sender while teaching is active.

    The SDK retains its single CAN sender and temperature/shutdown workers.
    A stale sample holds the last target using normal position gains and marks
    a fault; it does not falsely report that the physical arm is safe.
    """
    def __init__(self, arm, max_age=.30, clock=time.monotonic):
        self.arm, self.max_age, self.clock = arm, max_age, clock
        self.original = arm._send_mit_positions
        self.lock = threading.Lock()
        self.snapshot = None
        self.fault = threading.Event()
        self.attached = False

    def publish(self, command, sampled_at):
        vectors = []
        for key in ('position', 'torque', 'kp', 'kd'):
            value = np.asarray(command[key], dtype=float)
            if value.shape != (6,) or not np.isfinite(value).all():
                raise ValueError('重力示教指令无效')
            vectors.append(value.copy())
        if not np.isfinite(sampled_at) or sampled_at > self.clock():
            raise ValueError('关节反馈时间无效')
        with self.lock:
            if self.arm.shutdown_started:
                raise RuntimeError('控制器正在停止')
            self.snapshot = (float(sampled_at), *vectors)
            # Keep the SDK fallback aligned with the most recent teaching
            # target if its shutdown worker cannot read the encoders.
            with self.arm.target_lock:
                self.arm.target_positions[:6] = vectors[0].tolist()
                self.arm.command_positions[:6] = vectors[0].tolist()

    def attach(self):
        with self.lock:
            if self.attached or self.snapshot is None:
                raise RuntimeError('重力示教发送器状态无效')
            self.attached = True
            self.arm._send_mit_positions = self.send

    def detach(self, hold_position):
        # Reset SDK's stale position-ramp state before handing control back.
        hold = np.asarray(hold_position, dtype=float)
        if hold.shape != (6,) or not np.isfinite(hold).all():
            raise ValueError('无法安全交接无效的关节位置')
        with self.lock:
            with self.arm.target_lock:
                self.arm.target_positions[:6] = hold.tolist()
                self.arm.command_positions[:6] = hold.tolist()
            self.attached = False
            self.arm._send_mit_positions = self.original

    def send(self, positions_rad, *, lock_timeout=None):
        with self.lock:
            if self.arm.shutdown_started:
                return self.original(positions_rad, lock_timeout=lock_timeout)
            if not self.attached:
                # An SDK iteration might have obtained this bound method before
                # detach; do not emit the pre-handoff ramp target afterward.
                with self.arm.target_lock:
                    positions_rad = self.arm.command_positions.copy()
                return self.original(positions_rad, lock_timeout=lock_timeout)
            sampled_at, position, torque, kp, kd = self.snapshot
            if self.fault.is_set() or self.clock() - sampled_at > self.max_age:
                self.fault.set()
                torque = np.zeros(6)
                kp = np.asarray([m.kp for m in self.arm.config.motors[:6]], dtype=float)
                kd = np.asarray([m.kd for m in self.arm.config.motors[:6]], dtype=float)
            if len(positions_rad) != self.arm.motor_count or self.arm.motor_count != 7:
                raise RuntimeError('机械臂电机数量不匹配')
            with self.arm._io_lock_guard(lock_timeout):
                for index, motor in enumerate(self.arm.motors):
                    config = self.arm.config.motors[index]
                    if index < 6:
                        motor.send_mit(float(position[index]), 0.0, float(kp[index]),
                                       float(kd[index]), float(torque[index]))
                    else:
                        motor.send_mit(float(positions_rad[index]), 0.0,
                                       float(config.kp), float(config.kd), 0.0)
