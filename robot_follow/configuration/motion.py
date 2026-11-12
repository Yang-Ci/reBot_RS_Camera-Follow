"""Resolve command-line overrides into validated runtime motion settings."""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MotionOptions:
    desired_depth: float
    gains: tuple[float, float, float]
    max_velocity: float
    camera_fps: int
    color_exposure_ms: float


def resolve_motion_options(args, follow: dict) -> MotionOptions:
    """Apply CLI > profile > built-in precedence without changing the profile."""
    depth = float(follow['distance'])
    gains = np.array([float(follow['gain_x']), float(follow['gain']), float(follow['gain'])])
    max_velocity = float(follow['max_vel'])
    fps = 30 if args.camera == 'd435i' else int(follow['fps'])
    if args.distance is not None:
        if not 0.18 <= args.distance <= 0.60:
            raise ValueError('--distance 建议在 0.18~0.60 米之间（Gemini2 近距极限 ~0.15m，太远跟踪不稳）')
        depth = args.distance
    if args.gain is not None:
        gains[1:] = args.gain
    if args.gain_x is not None:
        gains[0] = args.gain_x
    if args.max_vel is not None:
        max_velocity = args.max_vel
    if args.fps is not None:
        fps = args.fps
    if (not np.all(np.isfinite(gains)) or not 0.0 < gains[0] <= 2.5
            or not np.all((gains[1:] > 0.0) & (gains[1:] <= 3.5))):
        raise ValueError('命令行增益超出安全范围')
    if not np.isfinite(max_velocity) or not 0.0 < max_velocity <= 0.70:
        raise ValueError('max_vel 须在 (0, 0.70]m/s')
    if fps not in (30, 60):
        raise ValueError('跟随模式仅允许 --fps 30 或 60')
    exposure = (float(args.color_exposure) if args.color_exposure is not None
                else 0.0 if args.camera in ('d435i', 'gemini336')
                else float(follow.get('color_exposure_ms', 0.0)))
    return MotionOptions(depth, tuple(float(value) for value in gains), max_velocity, fps, exposure)
