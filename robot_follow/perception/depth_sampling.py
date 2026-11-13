"""Sample locally valid palm depths, excluding known gripper pixels."""
import numpy as np

MIN_DEPTH_M = 0.15
MAX_DEPTH_M = 1.50
DEPTH_PATCH = 3


def get_median_depth(depth_raw, depth_scale, u, v, patch=DEPTH_PATCH, allowed_mask=None):
    """Return a local valid-depth median in metres, or zero if unavailable."""
    h, w = depth_raw.shape
    if allowed_mask is not None:
        if allowed_mask.shape != depth_raw.shape:
            raise ValueError('夹爪屏蔽区与深度图尺寸不一致')
        if not (0 <= u < w and 0 <= v < h) or not allowed_mask[v, u]:
            return 0.0
    for p in (patch, patch * 2, patch * 3):
        x0, x1 = max(0, u - p), min(w, u + p + 1)
        y0, y1 = max(0, v - p), min(h, v + p + 1)
        window = depth_raw[y0:y1, x0:x1].astype(np.float32) * depth_scale
        valid_pixels = (window >= MIN_DEPTH_M) & (window <= MAX_DEPTH_M)
        if allowed_mask is not None:
            valid_pixels &= allowed_mask[y0:y1, x0:x1] != 0
        valid = window[valid_pixels]
        if valid.size >= 5:
            return float(np.median(valid))
    return 0.0
