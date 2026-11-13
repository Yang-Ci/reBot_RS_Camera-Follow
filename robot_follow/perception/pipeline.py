"""Frame-level hand detection, tracking and depth processing."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable

import cv2

from robot_follow.perception.depth import DepthGate
from robot_follow.perception.ir_filter import in_ir_gripper_region, ir_gripper_detection
from robot_follow.perception.gripper_mask import GripperMask


@dataclass
class HandFrame:
    track: dict | None
    landmarks: Any
    detection_finished: float


class HandPipeline:
    """Combine one hand detector with short-gap tracking and a depth gate."""

    def __init__(self, *, detector, bridge, tracker_factory: Callable,
                 depth_gate: DepthGate, median_depth: Callable, project: Callable,
                 infrared: bool = False, mask_rgb_gripper: bool = False):
        self.detector = detector
        self.bridge = bridge
        self.tracker_factory = tracker_factory
        self.tracker = tracker_factory()
        self.depth_gate = depth_gate
        self.median_depth = median_depth
        self.project = project
        self.infrared = infrared
        self.mask_rgb_gripper = mask_rgb_gripper
        self.gripper_mask = None

    def reset(self, depth_filter: Callable[[float], float]) -> None:
        """Discard camera-specific tracking and filtering history after reconnect."""
        self.depth_gate.reset_reference()
        self.depth_gate.depth_filter = depth_filter
        self.bridge.reset()
        self.tracker = self.tracker_factory()

    def process(self, color_image, depth_raw, depth_scale, now: float | None = None) -> HandFrame:
        detection_image = cv2.medianBlur(color_image, 9) if self.infrared else color_image
        if self.mask_rgb_gripper:
            h, w = color_image.shape[:2]
            if self.gripper_mask is None or (self.gripper_mask.width, self.gripper_mask.height) != (w, h):
                self.gripper_mask = GripperMask(w, h)
            detection_image = self.gripper_mask.apply_rgb(detection_image)
        u, v, landmarks = self.detector.detect(detection_image)
        if self.gripper_mask is not None and self.gripper_mask.contains(u, v):
            u, v, landmarks = None, None, None
            self.bridge.reset()
        if self.infrared and ir_gripper_detection(u, v, landmarks,
                                                  color_image.shape[1], color_image.shape[0]):
            u, v, landmarks = None, None, None
            self.bridge.reset()
        detection_finished = time.perf_counter()
        bridge_image = detection_image if self.mask_rgb_gripper else color_image
        bu, bv, source = self.bridge.update(bridge_image, (u, v) if u is not None else None)
        if source == 'lk':
            u, v = bu, bv
            landmarks = None
        if self.infrared and in_ir_gripper_region(u, v, color_image.shape[1], color_image.shape[0]):
            u, v, landmarks = None, None, None
            self.bridge.reset()
        if self.gripper_mask is not None and self.gripper_mask.contains(u, v):
            u, v, landmarks = None, None, None
            self.bridge.reset()
        track = self.tracker.update((u, v) if u is not None else None)
        if track is not None:
            track['bridged'] = source == 'lk'
        depth_m = 0.0
        if track is not None and track['lost'] == 0:
            ui, vi = int(round(track['u'])), int(round(track['v']))
            if self.gripper_mask is not None:
                depth_m = self.median_depth(depth_raw, depth_scale, ui, vi,
                                            allowed_mask=self.gripper_mask.allowed)
            else:
                depth_m = self.median_depth(depth_raw, depth_scale, ui, vi)
        self.depth_gate.update(track, depth_m, self.project, time.time() if now is None else now)
        return HandFrame(track, landmarks, detection_finished)
