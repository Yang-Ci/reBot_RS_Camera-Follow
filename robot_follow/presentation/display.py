"""Render the optional OpenCV window without controlling the robot."""
from dataclasses import dataclass
from typing import Any, Callable

import cv2
import numpy as np


@dataclass
class DisplayFrame:
    image: Any
    track: dict | None
    landmarks: Any
    pinch_points: tuple | None
    infrared: bool
    desired_u: int
    desired_v: int
    gripper_enabled: bool
    gripper_closed_deg: float
    gripper_open_deg: float
    gripper_deg: float
    gripper_frozen: bool
    no_arm: bool
    follow_enabled: bool
    following: bool
    status: dict
    fps: float


class VisionDisplay:
    """Draw a snapshot; return a key for the runtime to interpret."""
    window_name = 'Palm Follow + Grasp'

    def __init__(self, draw_palm: Callable, draw_landmarks: Callable):
        self.draw_palm = draw_palm
        self.draw_landmarks = draw_landmarks

    def show(self, frame: DisplayFrame) -> int:
        display = frame.image.copy()
        if frame.infrared:
            h, w = display.shape[:2]
            cv2.rectangle(display, (int(.37*w), int(.50*h)),
                          (int(.90*w), h-1), (0, 165, 255), 1)
            cv2.putText(display, 'GRIPPER EXCLUSION', (int(.37*w), int(.50*h)-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
        if frame.landmarks is not None:
            self.draw_landmarks(display, frame.landmarks)
        self.draw_palm(display, frame.track, frame.desired_u, frame.desired_v)
        if frame.pinch_points is not None:
            p1, p2 = frame.pinch_points
            cv2.line(display, p1, p2, (255, 0, 255), 3)
            cv2.circle(display, p1, 8, (255, 0, 255), cv2.FILLED)
            cv2.circle(display, p2, 8, (255, 0, 255), cv2.FILLED)
        if frame.gripper_enabled:
            span = frame.gripper_open_deg - frame.gripper_closed_deg
            opening = (float(np.clip((frame.gripper_deg - frame.gripper_closed_deg) / span,
                                     0.0, 1.0)) * 100.0 if abs(span) > 1e-6 else 0.0)
            frozen = ' [FROZEN]' if frame.gripper_frozen else ''
            cv2.putText(display, f'Gripper: {opening:.0f}% ({frame.gripper_deg:.1f}deg){frozen}',
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        if frame.no_arm:
            state = 'VISION ONLY (--no-arm)'
        elif not frame.follow_enabled:
            state = 'GRASP ONLY (arm holding)'
        else:
            state = ('FOLLOWING' if frame.following and frame.status['active'] else
                     'HOLD (hand lost)' if frame.following else 'IDLE (press g/space)')
        colors = {'FOLLOWING': (0, 255, 0), 'IDLE (press g/space)': (0, 200, 255)}
        cv2.putText(display, state, (10, 460), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, colors.get(state, (0, 165, 255)), 2)
        cv2.putText(display, f"FPS: {frame.fps:.0f}  JointSpeed: {frame.status['joint_speed']:.3f} rad/s",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        if frame.status['error']:
            cv2.putText(display, f"CTRL: {frame.status['error'][:60]}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow(self.window_name, display)
        return cv2.waitKey(1) & 0xFF
