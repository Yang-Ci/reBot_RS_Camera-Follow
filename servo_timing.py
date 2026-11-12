"""Compatibility imports; new code uses robot_follow.control.timing."""
from robot_follow.control.timing import (
    AdaptiveLeadGate, AxisProcessNoiseGate, CameraTimestampCalibrator,
    FastStartGate, SlowFollowGate, StaticVelocityHold, _smoothstep,
    adaptive_feedforward_scale, camera_point_to_world, camera_twist_from_tcp,
    compensate_palm_velocity, dynamic_lead_seconds, horizontal_radial_soft_wall,
    measurement_age, scaled_alpha, scaled_decay, world_point_to_camera,
)

__all__ = [
    'AdaptiveLeadGate', 'AxisProcessNoiseGate', 'CameraTimestampCalibrator',
    'FastStartGate', 'SlowFollowGate', 'StaticVelocityHold', '_smoothstep',
    'adaptive_feedforward_scale', 'camera_point_to_world', 'camera_twist_from_tcp',
    'compensate_palm_velocity', 'dynamic_lead_seconds', 'horizontal_radial_soft_wall',
    'measurement_age', 'scaled_alpha', 'scaled_decay', 'world_point_to_camera',
]
