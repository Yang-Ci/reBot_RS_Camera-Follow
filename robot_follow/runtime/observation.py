"""Observation shared between perception and servo threads.

The caller owns the lock: compound updates and snapshots must hold the same
lock as the servo state. Times use the monotonic clock, in seconds.
"""

from dataclasses import dataclass


@dataclass
class TargetObservation:
    position_cam: list[float] | None = None
    frozen: bool = False
    sequence: int = 0
    observed_at: float = 0.0
    arrived_at: float = 0.0
    published_at: float = 0.0
    processing_ms: float = 0.0
    hardware_time: bool = False
    visible: bool = False

    def publish(self, track, *, observed_at, arrived_at, published_at, hardware_time):
        """Publish a valid target or mark it missing; the caller holds the lock."""
        if track is not None and track['lost'] == 0 and track['z3d'] > 0:
            self.position_cam = [track['x3d'], track['y3d'], track['z3d']]
            self.frozen = bool(track.get('z3d_frozen', False))
            self.sequence += 1
            self.observed_at = observed_at
            self.arrived_at = arrived_at
            self.published_at = published_at
            self.hardware_time = hardware_time
            self.processing_ms = (published_at - arrived_at) * 1000.0
            self.visible = True
        else:
            self.visible = False
            if track is None:
                self.position_cam = None
