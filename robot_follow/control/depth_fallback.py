"""Estimate a palm target on the last measured world plane during depth loss."""
import numpy as np


class CachedDepthPlane:
    """Compensate camera motion; unobserved hand motion along depth is unknown."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.anchor = None
        self.normal = None

    @staticmethod
    def _pose(position, rotation, transform):
        p = np.asarray(position, dtype=float)
        r = np.asarray(rotation, dtype=float)
        t = np.asarray(transform, dtype=float)
        if (p.shape != (3,) or r.shape != (3, 3) or t.shape != (4, 4)
                or not all(np.all(np.isfinite(v)) for v in (p, r, t))):
            raise ValueError('相机位姿无效')
        return p + r @ t[:3, 3], r @ t[:3, :3]

    def remember(self, target, position, rotation, transform):
        point = np.asarray(target, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)) or not .15 <= point[2] <= 1.5:
            raise ValueError('深度参考无效')
        origin, camera_rotation = self._pose(position, rotation, transform)
        normal = camera_rotation[:, 2]
        if abs(float(np.linalg.norm(normal)) - 1.) > .01:
            raise ValueError('相机方向无效')
        self.anchor = origin + camera_rotation @ point
        self.normal = normal.copy()

    def project(self, ray_point, position, rotation, transform):
        if self.anchor is None or self.normal is None:
            raise ValueError('缺少有效深度参考')
        point = np.asarray(ray_point, dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)) or point[2] <= 0:
            raise ValueError('手掌方向无效')
        origin, camera_rotation = self._pose(position, rotation, transform)
        ray = point / point[2]
        direction = camera_rotation @ ray
        denominator = float(self.normal @ direction)
        if denominator < .2:
            raise ValueError('估算深度方向无效')
        depth = float(self.normal @ (self.anchor - origin)) / denominator
        estimated = ray * depth
        world = origin + camera_rotation @ estimated
        if not .15 <= depth <= 1.5 or np.linalg.norm(world - self.anchor) > .30:
            raise ValueError('估算目标超出可信范围')
        return estimated
