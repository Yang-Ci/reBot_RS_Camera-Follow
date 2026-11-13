"""Conservative table protection for the B601/Gemini 336 tabletop setup.

The envelope is a nominal URDF estimate, not a certified collision system.
The table is assumed to coincide with the horizontal base-link z=0 plane.
"""
from __future__ import annotations

import math
import numpy as np

# Furthest gripper vertex from joint6 across both modeled finger extremes is
# approximately 0.174 m. The remaining margin is provisional; verify in situ.
MIN_JOINT6_HEIGHT_M = 0.23
SLOWDOWN_HEIGHT_M = 0.30
MAX_FEEDBACK_AGE_S = 0.30
MAX_TRACKING_ERROR_RAD = math.radians(40.0)
# Conservative software horizon for stale feedback and commanded deceleration;
# not a measured braking guarantee for the physical arm.
DOWNWARD_HORIZON_S = 0.60


class FrozenDepthGate:
    """Hold for lost observations and verify distinct recovery frames."""

    def __init__(self, timeout_s=0.15, max_jump_m=0.05, recovery_frames=2):
        self.timeout_s = timeout_s
        self.max_jump_m = max_jump_m
        self.recovery_frames = recovery_frames
        self.reset()

    def reset(self):
        self.last_good = None
        self.started = None
        self.last_sequence = None
        self.good_frames = 0
        self.hold_kind = None
        self.last_recovery = None
        self.recovery_origin = None

    def update(self, target, frozen, sequence, now, *, kind='depth'):
        target = np.asarray(target, dtype=float)
        if frozen:
            if self.last_good is None:
                return 'pause'
            if self.started is None:
                self.started = now
                self.hold_kind = kind
            self.good_frames = 0
            self.last_recovery = None
            self.recovery_origin = None
            self.last_sequence = sequence
            if self.hold_kind == 'hand':
                return 'hold'
            return 'hold' if now - self.started < self.timeout_s else 'pause'
        if self.started is None:
            self.last_good = target.copy()
            return 'ready'
        if self.hold_kind == 'hand':
            # A hand can legitimately reappear elsewhere. Confirm three fresh,
            # mutually nearby samples before accepting it as a new target.
            if sequence != self.last_sequence:
                self.last_sequence = sequence
                if (self.recovery_origin is None or
                        np.linalg.norm(target - self.recovery_origin) > 0.02 or
                        (self.last_recovery is not None and
                         np.linalg.norm(target - self.last_recovery) > 0.02)):
                    self.recovery_origin = target.copy()
                    self.good_frames = 1
                else:
                    self.good_frames += 1
                self.last_recovery = target.copy()
            if self.good_frames < 3:
                return 'hold'
        else:
            jump = (np.linalg.norm(target - self.last_good) > self.max_jump_m or
                    (self.last_recovery is not None and
                     np.linalg.norm(target - self.last_recovery) > self.max_jump_m))
            if now - self.started >= self.timeout_s or jump:
                return 'pause'
            if sequence != self.last_sequence:
                self.last_sequence = sequence
                self.last_recovery = target.copy()
                self.good_frames += 1
            if self.good_frames < self.recovery_frames:
                return 'hold'
        self.started = None
        self.last_good = target.copy()
        self.hold_kind = None
        self.last_recovery = None
        self.recovery_origin = None
        return 'ready'


def feedback_error(snapshot, now):
    """Reject stale, incomplete or implausibly divergent actual feedback."""
    if snapshot is None:
        return '实际关节反馈缺失，跟随已暂停'
    try:
        stamp = float(snapshot['time'])
        read_s = float(snapshot['read_ms']) / 1000.0
        age = now - stamp + read_s
        actual = np.asarray(snapshot['actual'], dtype=float)
        command = np.asarray(snapshot['command'], dtype=float)
        if (not math.isfinite(age) or not math.isfinite(read_s) or read_s < 0
                or now < stamp or age > MAX_FEEDBACK_AGE_S):
            return '实际关节反馈过期，跟随已暂停'
        if (actual.shape != (6,) or command.shape != (6,)
                or not np.all(np.isfinite(actual)) or not np.all(np.isfinite(command))):
            return '实际关节反馈无效，跟随已暂停'
        if np.max(np.abs(actual - command)) > MAX_TRACKING_ERROR_RAD:
            return '实际关节与指令偏差过大，跟随已暂停'
    except (KeyError, TypeError, ValueError, OverflowError):
        return '实际关节反馈无效，跟随已暂停'
    return ''


def limit_downward_velocity(velocity, commanded_height, measured_height):
    """Scale only downward translation; signal hold below the conservative floor."""
    velocity, commanded_height, measured_height = map(
        float, (velocity, commanded_height, measured_height))
    if not all(map(math.isfinite, (velocity, commanded_height, measured_height))):
        raise ValueError('桌面保护位置无效')
    z = min(commanded_height, measured_height)
    if z <= MIN_JOINT6_HEIGHT_M:
        return 0.0, True
    factor = min(1.0, (z - MIN_JOINT6_HEIGHT_M) /
                 (SLOWDOWN_HEIGHT_M - MIN_JOINT6_HEIGHT_M))
    if velocity < 0:
        max_down = (z - MIN_JOINT6_HEIGHT_M) / DOWNWARD_HORIZON_S
        velocity = max(velocity * factor, -max_down)
    return velocity, False
