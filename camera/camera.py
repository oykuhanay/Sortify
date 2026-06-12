"""
WP4 -> WP3 interface: camera frame source.

Vision team usage:

    from camera import Camera

    with Camera() as cam:
        frame = cam.get_frame()  # numpy array, BGR, shape (H, W, 3)
        # ... run detection ...

A background thread keeps only the most recent frame, so get_frame() always
returns fresh data even if the caller is slower than the camera.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

DEFAULT_CAMERA_INDEX = 1
OPEN_TIMEOUT_SEC = 3.0


class CameraError(RuntimeError):
    pass


class Camera:
    def __init__(self, index: int = DEFAULT_CAMERA_INDEX) -> None:
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise CameraError(f"could not open camera at index {index}")

        self._latest: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

        deadline = time.time() + OPEN_TIMEOUT_SEC
        while time.time() < deadline:
            with self._lock:
                if self._latest is not None:
                    return
            time.sleep(0.02)
        self.close()
        raise CameraError("camera opened but produced no frames within timeout")

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self._lock:
                self._latest = frame

    def get_frame(self) -> np.ndarray:
        with self._lock:
            if self._latest is None:
                raise CameraError("no frame available yet")
            return self._latest.copy()

    @property
    def resolution(self) -> tuple[int, int]:
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._cap.release()

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
