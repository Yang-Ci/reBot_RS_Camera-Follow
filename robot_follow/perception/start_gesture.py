"""A conservative, continuous open-palm gesture for the initial start only."""
from __future__ import annotations

import math


def is_open_palm(landmarks, width, height):
    """Require four extended fingers and a separated thumb in a fresh image."""
    try:
        if landmarks is None or len(landmarks) != 21 or width <= 0 or height <= 0:
            return False
        points = [(float(p.x) * width, float(p.y) * height) for p in landmarks]
        if any(not (math.isfinite(x) and math.isfinite(y)
                    and 0 <= x <= width and 0 <= y <= height) for x, y in points):
            return False
        distance = lambda a, b: math.dist(points[a], points[b])
        palm_width = distance(5, 17)
        if palm_width < 25 or distance(4, 8) < .4 * palm_width:
            return False
        for mcp in (5, 9, 13, 17):
            pip, tip = mcp + 1, mcp + 3
            ax, ay = points[mcp][0] - points[pip][0], points[mcp][1] - points[pip][1]
            bx, by = points[tip][0] - points[pip][0], points[tip][1] - points[pip][1]
            length = math.hypot(ax, ay) * math.hypot(bx, by)
            if length < 1e-6 or (ax * bx + ay * by) / length > -0.707:
                return False
            if distance(mcp, tip) < 1.25 * distance(mcp, pip):
                return False
        return True
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False


class OpenPalmStartGate:
    """Require an unbroken series of fresh detections for three seconds."""
    def __init__(self, hold_s=3.0, max_gap_s=0.15):
        self.hold_s, self.max_gap_s = hold_s, max_gap_s
        self.reset()

    def reset(self):
        self.started = self.last = None
        self.fired = False

    def update(self, now, *, eligible, opened):
        if self.fired:
            return False
        if not (eligible and opened and math.isfinite(now)):
            self.started = self.last = None
            return False
        if self.last is not None and (now <= self.last or now - self.last > self.max_gap_s):
            self.started = None
        if self.started is None:
            self.started = now
        self.last = now
        if now - self.started >= self.hold_s:
            self.fired = True
            return True
        return False
