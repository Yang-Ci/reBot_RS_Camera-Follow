"""Mutable state owned by the servo loop.

A fresh instance is created for every runtime; vector defaults must never be
shared between runs. The servo thread is the sole writer of this state.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ServoState:
    actual_velocity: np.ndarray = field(default_factory=lambda: np.zeros(6))
    prev_cmd_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    prev_camera_twist: np.ndarray = field(default_factory=lambda: np.zeros(6))
    prev_command_joints: np.ndarray | None = None
    lost_counter: int = 0
    prev_target_cam: np.ndarray | None = None
    target_vel_f: np.ndarray = field(default_factory=lambda: np.zeros(3))
    in_deadband: bool = False
    coast_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    coast_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    coast_err: float = 1e9
    near_wall: bool = True
    reach_r_now: float = 0.0
    reach_att: float = 1.0
    static_hold_now: bool = False
    kf_time: float = 0.0
    kf_last_seq: int = -1
    kf_reject_count: int = 0
    kf_rejected_now: bool = False
    ff_moving: bool = False
    ff_exit_count: int = 0
    last_joint_speed: float = 0.0
    settled_frames: int = 0
    hold_latch: object | None = None
    hold_world: np.ndarray | None = None
    lead_weight_now: float = 0.0
    lead_target_weight: float = 0.0
    ff_taper_now: float = 0.0
    ff_approach_speed: float = 0.0
    ff_scale_now: float = 0.0
    slow_active: bool = False
