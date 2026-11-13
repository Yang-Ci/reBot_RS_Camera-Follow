"""Fixed image-space exclusion for the user's Gemini 336 wrist mount.

The polygon was estimated from one empty-hand frame and must be checked if
the camera, mount, gripper, or jaw opening changes. Pixels hidden by the
gripper cannot provide a hand measurement.
"""
import cv2
import numpy as np


GRIPPER_POLYGON = (
    (.12, 1.00), (.13, .82), (.30, .78), (.37, .71), (.44, .67),
    (.53, .635), (.60, .64), (.71, .68), (.82, .78), (.92, .86),
    (1.00, .88), (1.00, 1.00),
)


class GripperMask:
    def __init__(self, width, height):
        self.width, self.height = int(width), int(height)
        if self.width <= 0 or self.height <= 0:
            raise ValueError('图像尺寸无效')
        self.polygon = np.array([
            (min(self.width - 1, round(x * self.width)),
             min(self.height - 1, round(y * self.height)))
            for x, y in GRIPPER_POLYGON
        ], dtype=np.int32)
        self.allowed = np.full((self.height, self.width), 255, dtype=np.uint8)
        cv2.fillPoly(self.allowed, [self.polygon], 0)

    def contains(self, u, v):
        if u is None or v is None:
            return False
        x, y = int(round(u)), int(round(v))
        return 0 <= x < self.width and 0 <= y < self.height and self.allowed[y, x] == 0

    def apply_rgb(self, image):
        return cv2.bitwise_and(image, image, mask=self.allowed)
