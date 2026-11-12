"""Interfaces and lifecycle for depth cameras used by the runtime."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Protocol

import numpy as np


class DepthCamera(Protocol):
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def get_frames(self) -> tuple[Any, Any, float | None]: ...
    def get_last_frame_timing(self) -> tuple[float | None, str]: ...
    def stop(self) -> None: ...


@dataclass(frozen=True)
class CameraSignature:
    """Parameters that must remain stable while the controller is running."""
    size: tuple[int, int]
    intrinsics: tuple[float, float, float, float]
    serial: str | None
    color: bool | None
    stream: str | None
    matrix: tuple[float, ...] | None
    distortion: tuple[float, ...] | None

    @classmethod
    def from_camera(cls, camera: DepthCamera) -> CameraSignature:
        intrinsics = tuple(float(getattr(camera, key)) for key in ('fx', 'fy', 'cx', 'cy'))
        if not np.all(np.isfinite(intrinsics)) or intrinsics[0] <= 0 or intrinsics[1] <= 0:
            raise ValueError('相机内参无效')
        size = int(camera.width), int(camera.height)
        if min(size) <= 0:
            raise ValueError('相机分辨率无效')

        def values(name):
            raw = getattr(camera, name, None)
            if raw is None:
                return None
            array = np.asarray(raw, dtype=float)
            if not np.all(np.isfinite(array)):
                raise ValueError(f'相机 {name} 无效')
            return tuple(array.ravel())

        serial = getattr(camera, 'camera_serial', None)
        stream = getattr(camera, 'image_stream', None)
        color = getattr(camera, 'has_color', None)
        return cls(size, intrinsics, str(serial) if serial else None,
                   bool(color) if color is not None else None,
                   str(stream) if stream is not None else None,
                   values('camera_matrix'), values('dist_coeffs'))

    def matches(self, other: CameraSignature) -> bool:
        if (self.size != other.size or self.color != other.color or self.stream != other.stream
                or (self.serial is not None and self.serial != other.serial)):
            return False
        for left, right in ((self.intrinsics, other.intrinsics),
                            (self.matrix, other.matrix), (self.distortion, other.distortion)):
            if left is None or right is None:
                if left != right:
                    return False
            elif len(left) != len(right) or not np.allclose(left, right, rtol=1e-5, atol=1e-6):
                return False
        return True


class CameraSession:
    """Own the active camera and reject incompatible replacements after reconnect."""

    def __init__(self, factory: Callable[[], DepthCamera], validate: Callable[[DepthCamera], None]):
        self.factory = factory
        self.validate = validate
        self.camera: DepthCamera | None = None
        self.signature: CameraSignature | None = None
        self.last_error: str | None = None

    def open(self) -> DepthCamera:
        if self.camera is not None:
            raise RuntimeError('相机已打开')
        camera = self.factory()
        try:
            self.validate(camera)
            signature = CameraSignature.from_camera(camera)
            if self.signature is not None and not self.signature.matches(signature):
                raise ValueError('重连相机的设备、图像流或内参与原相机不一致')
        except BaseException:
            try:
                camera.stop()
            except Exception:
                pass
            raise
        self.signature = signature
        self.camera = camera
        self.last_error = None
        return camera

    def get_frames(self):
        if self.camera is None:
            return None, None, None
        try:
            return self.camera.get_frames()
        except Exception as exc:
            self.last_error = str(exc)
            return None, None, None

    def stop(self):
        camera, self.camera = self.camera, None
        if camera is not None:
            camera.stop()

    def reconnect(self) -> DepthCamera:
        try:
            self.stop()
        except Exception:
            # A disconnected SDK may reject stop; a fresh open can still recover.
            pass
        return self.open()
