"""
WP4 Link 1 smoke test: phone (Iriun) -> Mac -> OpenCV.

Opens the Iriun virtual webcam, displays the live feed in a window,
overlays measured FPS, and prints actual capture resolution + FPS.

Press 'q' in the video window to quit.
"""

import time
import sys
import cv2

CAMERA_INDEX = 1
WINDOW_NAME = "Sortify WP4 - Camera Smoke Test"


def open_camera(index: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"ERROR: could not open camera at index {index}.", file=sys.stderr)
        print("Try a different index (0, 1, 2) or confirm Iriun is showing video.", file=sys.stderr)
        sys.exit(1)
    return cap


def main() -> None:
    cap = open_camera(CAMERA_INDEX)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Capture opened. Resolution: {width}x{height}, reported FPS: {reported_fps:.1f}")

    frame_count = 0
    measured_fps = 0.0
    window_start = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("WARN: frame read failed, retrying...", file=sys.stderr)
            time.sleep(0.05)
            continue

        frame_count += 1
        elapsed = time.time() - window_start
        if elapsed >= 1.0:
            measured_fps = frame_count / elapsed
            frame_count = 0
            window_start = time.time()

        overlay = f"{width}x{height}  |  {measured_fps:5.1f} FPS  |  press 'q' to quit"
        cv2.putText(frame, overlay, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow(WINDOW_NAME, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Closed cleanly.")


if __name__ == "__main__":
    main()
