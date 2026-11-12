"""Map measured pinch distances to a filtered, gated gripper target."""
from __future__ import annotations

from collections.abc import Callable

import numpy as np


class GripperMapper:
    """Keep smoothing and deadband state separate from hardware I/O."""

    def __init__(self, *, closed_px: float, open_px: float, closed_deg: float,
                 open_deg: float, initial_deg: float, filter_mode: str,
                 smoothing: float, deadband: float, hold_while_moving: bool,
                 euro_filter: Callable[[float], float]):
        self.closed_px = closed_px
        self.open_px = open_px
        self.closed_deg = closed_deg
        self.open_deg = open_deg
        self.smoothed = initial_deg
        self.last_sent = initial_deg
        self.filter_mode = filter_mode
        self.smoothing = smoothing
        self.deadband = deadband
        self.hold_while_moving = hold_while_moving
        self.euro_filter = euro_filter
        self.pinch_px = None
        self.frozen = False
        if filter_mode == 'one_euro':
            self.euro_filter(initial_deg)

    def update_target(self, pinch_px: float) -> None:
        self.pinch_px = pinch_px
        fraction = float(np.interp(pinch_px, [self.closed_px, self.open_px], [0., 1.]))
        target = self.closed_deg + fraction * (self.open_deg - self.closed_deg)
        if self.filter_mode == 'one_euro':
            self.smoothed = self.euro_filter(target)
        else:
            self.smoothed = (self.smoothing * target + (1.0 - self.smoothing) * self.smoothed)

    def candidate(self, arm_moving: bool) -> float | None:
        self.frozen = self.hold_while_moving and arm_moving
        if not self.frozen and abs(self.smoothed - self.last_sent) >= self.deadband:
            return self.smoothed
        return None

    def committed(self, target: float) -> None:
        """Record a target only after the caller successfully sends it."""
        self.last_sent = target
