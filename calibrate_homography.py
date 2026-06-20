"""
One-shot homography calibration for the Sortify overhead camera.

Why this exists:
    The camera sees the playing field with perspective distortion. A
    marker that looks square in the centre of the frame becomes a
    trapezoid near the edges, which throws off our gripper-position
    estimate and makes the bridge twitchy when the robot is far from
    centre. A 4-point homography flattens the playing field back to a
    rectangle in "world" coordinates (centimetres) so geometry stays
    consistent no matter where the robot is.

How to use:
    Run once:
        python3 calibrate_homography.py
    Click the FOUR CORNERS of the cardboard playing field in this
    order:
        1) top-left   2) top-right   3) bottom-right   4) bottom-left
    Press 's' to save, 'r' to redo, 'q' to quit without saving.

    Output: homography.npy in the project root.
    main.py picks it up automatically on next launch.

The field is taken to be FIELD_WIDTH_CM x FIELD_HEIGHT_CM. After
calibration, world coordinates inside the field run from
(0, 0) at top-left to (FIELD_WIDTH_CM, FIELD_HEIGHT_CM) at bottom-
right, all in centimetres.
"""

from __future__ import annotations

import sys
import numpy as np
import cv2

sys.path.insert(0, "camera")
from camera import Camera

# Physical size of the cardboard playing field, in centimetres.
# Measured on the table.
FIELD_WIDTH_CM = 100.0   # long edge
FIELD_HEIGHT_CM = 70.0   # short edge

OUT_PATH = "homography.npy"
WINDOW = "Calibrate homography - click 4 corners"


def _grab_frame() -> np.ndarray:
    """Pull a single frame from camera 0 at our usual resolution."""
    with Camera(index=0, width=3840, height=2160) as cam:
        # Warm-up: throw away a couple of frames so autoexposure settles.
        for _ in range(5):
            cam.get_frame()
        return cam.get_frame()


def main() -> int:
    print("Capturing one frame from the camera...")
    frame = _grab_frame()
    print(f"Got frame: {frame.shape[1]}x{frame.shape[0]}")

    # Resize for display so it fits on a laptop screen; we keep the
    # scale so we can map clicks back to the original pixel coords.
    h, w = frame.shape[:2]
    max_disp = 1400
    scale = min(1.0, max_disp / max(h, w))
    disp_size = (int(w * scale), int(h * scale))

    clicks: list[tuple[int, int]] = []

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if len(clicks) >= 4:
            return
        # Map display pixel back to full-resolution pixel.
        orig_x = int(round(x / scale))
        orig_y = int(round(y / scale))
        clicks.append((orig_x, orig_y))
        print(f"  corner {len(clicks)}: ({orig_x}, {orig_y})")

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse)

    instructions = [
        "Click TOP-LEFT corner of the cardboard",
        "Click TOP-RIGHT corner of the cardboard",
        "Click BOTTOM-RIGHT corner of the cardboard",
        "Click BOTTOM-LEFT corner of the cardboard",
        "Press 's' to save, 'r' to redo, 'q' to quit.",
    ]

    while True:
        disp = cv2.resize(frame, disp_size).copy()

        # Draw click history on the display copy.
        for i, (ox, oy) in enumerate(clicks):
            dx = int(round(ox * scale))
            dy = int(round(oy * scale))
            cv2.circle(disp, (dx, dy), 8, (0, 255, 0), -1)
            cv2.putText(disp, str(i + 1), (dx + 10, dy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        # Draw a closing polygon once all 4 corners are placed.
        if len(clicks) == 4:
            pts = np.array([[int(round(x * scale)), int(round(y * scale))] for x, y in clicks])
            cv2.polylines(disp, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        step = min(len(clicks), len(instructions) - 1)
        cv2.putText(disp, instructions[step], (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        cv2.imshow(WINDOW, disp)
        k = cv2.waitKey(20) & 0xFF
        if k in (ord("q"), 27):
            print("Quit without saving.")
            cv2.destroyAllWindows()
            return 1
        if k in (ord("r"), ord("R")):
            print("Reset — click corners again.")
            clicks.clear()
            continue
        if k in (ord("s"), ord("S")):
            if len(clicks) != 4:
                print("Need 4 corners first.")
                continue
            break

    # Image points (what the user clicked) in original full-res pixels.
    src = np.array(clicks, dtype=np.float32)
    # World points: corresponding corners in cm, same order.
    dst = np.array([
        [0.0,              0.0],
        [FIELD_WIDTH_CM,   0.0],
        [FIELD_WIDTH_CM,   FIELD_HEIGHT_CM],
        [0.0,              FIELD_HEIGHT_CM],
    ], dtype=np.float32)

    H, _ = cv2.findHomography(src, dst)
    if H is None:
        print("findHomography failed. Try again with cleaner clicks.")
        cv2.destroyAllWindows()
        return 1

    np.save(OUT_PATH, H)
    print(f"\nSaved homography to {OUT_PATH}")
    print("Pixel -> world conversion sanity check:")
    for label, pt in [("TL", clicks[0]), ("TR", clicks[1]),
                      ("BR", clicks[2]), ("BL", clicks[3])]:
        wp = cv2.perspectiveTransform(
            np.array([[pt]], dtype=np.float32), H
        )[0, 0]
        print(f"  {label} pixel {pt} -> world ({wp[0]:.2f}, {wp[1]:.2f}) cm")

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
