"""Shared run controls; callers protect follow state with the runtime lock."""
from dataclasses import dataclass, field
from threading import Event


@dataclass
class RunControl:
    following: bool = False
    gesture_start_allowed: bool = True
    servo_failed: bool = False
    quit_requested: Event = field(default_factory=Event)
