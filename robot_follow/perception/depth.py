"""Stateful depth validation for fresh palm observations."""
from __future__ import annotations

from collections.abc import Callable

import numpy as np


class DepthGate:
    """Reject jumps and validate reacquisition before projecting a 3D target."""

    def __init__(self, *, desired_depth: float, max_jump: float, max_step: float,
                 freeze_seconds: float, depth_filter: Callable[[float], float]):
        self.desired_depth = desired_depth
        self.max_jump = max_jump
        self.max_step = max_step
        self.freeze_seconds = freeze_seconds
        self.depth_filter = depth_filter
        self.reference = 0.0
        self.reference_time = 0.0
        self.rejected_frames = 0
        self.reacquisition = []

    def reset_reference(self):
        """Discard old-stream history after a camera reconnect or long loss."""
        self.reference = 0.0
        self.reference_time = 0.0
        self.rejected_frames = 0
        self.reacquisition.clear()

    def update(self, track, depth_m, project, now):
        """Update track in place; depth is used only for a fresh observation."""
        if track is None or track['lost'] >= 30:
            self.reset_reference()
            return
        if track['lost'] != 0:
            return

        if depth_m > 0 and self.reference > 0 and abs(depth_m - self.reference) > self.max_jump:
            self.rejected_frames += 1
            if self.rejected_frames < 15:
                depth_m = 0.0
        else:
            self.rejected_frames = 0

        if depth_m > 0 and self.reference == 0:
            if depth_m > self.desired_depth + 0.25:
                depth_m = 0.0
                self.reacquisition.clear()
            else:
                self.reacquisition.append(depth_m)
                if len(self.reacquisition) > 5:
                    self.reacquisition.pop(0)
                if (len(self.reacquisition) == 5 and
                        max(self.reacquisition) - min(self.reacquisition) <= self.max_jump):
                    depth_m = float(np.median(self.reacquisition))
                else:
                    depth_m = 0.0

        if depth_m > 0:
            if self.reference > 0:
                depth_m = float(np.clip(depth_m, self.reference - self.max_step,
                                        self.reference + self.max_step))
            depth_m = self.depth_filter(depth_m)
            self.reference = depth_m
            self.reference_time = now
            track['x3d'], track['y3d'], track['z3d'] = project(track['u'], track['v'], depth_m)
            track['z3d_frozen'] = False
        elif self.reference > 0 and now - self.reference_time <= self.freeze_seconds:
            track['x3d'], track['y3d'], _ = project(track['u'], track['v'], self.reference)
            track['z3d'] = self.desired_depth
            track['z3d_frozen'] = True
        else:
            track['z3d'] = 0.0
            track['z3d_frozen'] = False
