"""
Example of how the vision team will consume the camera module.

This is *not* part of the bridge - it's a reference for WP3.
Run with: python3 demo_vision_consumer.py
Press 'q' to quit.
"""

import time
import cv2

from camera import Camera


def main() -> None:
    with Camera() as cam:
        print(f"Camera ready at {cam.resolution[0]}x{cam.resolution[1]}")

        last_log = time.time()
        n = 0
        while True:
            frame = cam.get_frame()

            # placeholder for what WP3 will do here:
            # detections = detector.run(frame)

            n += 1
            if time.time() - last_log >= 1.0:
                print(f"consumer FPS: {n}")
                n = 0
                last_log = time.time()

            cv2.imshow("vision consumer (demo)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
