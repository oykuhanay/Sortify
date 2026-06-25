import asyncio
import collections
import math
import sys
import threading
import time
import cv2
import numpy as np
from ultralytics import YOLO
sys.path.insert(0, "camera")
from camera import Camera
from sortify_path_finding import Detection, build_occupancy_grid, astar
from bridge import Bridge
from robot import Robot, RobotError
import web_dashboard

ARUCO_DICT = cv2.aruco.DICT_4X4_50
ROBOT_MARKER_ID = 0
ROBOT_TRAIL_LEN = 80  # how many past positions to show

# Clearance = marker_side × this factor.
# The marker ≈ robot width, so 0.6 ≈ "keep center at least 60% of robot width from obstacle edges".
ROBOT_CLEARANCE_FACTOR = 0.6

# Gripper tip in world (cm) = robot_center + forward * theta + right * theta+90.
# These are real centimetres on the table, not marker-side multiples.
# The chassis is asymmetric so right ≠ 0.
#   forward >0 = ahead of the marker, in the direction of theta
#   right   >0 = to the marker's right (clockwise from theta)
GRIPPER_FORWARD_CM = 20.0   # marker centre → between the jaws (live-tunable)
GRIPPER_RIGHT_CM   = 0.0    # chassis is symmetric; any side-drift is parallax (live-tunable)

# Persistent live-tune config file. Loaded at startup if present, written
# whenever the operator presses K in the OpenCV window. Lets us tweak
# offsets and parallax without restarting main.py — the previous
# stop/edit/start loop took 30-40 seconds per nudge.
TUNABLES_PATH = "tunables.json"

# Parallax correction.
#
# The homography flattens the table plane perfectly — but the marker
# sits ~3–4 cm above the table, so when the robot is away from directly
# under the camera, the marker's image appears shifted *away* from the
# camera nadir compared to its true ground-plane position. The jaws sit
# almost on the ground so they're not shifted, and the magenta dot
# drifts.
#
# Correction is a fraction of the vector from the camera's nadir on the
# table to the robot. PARALLAX_FACTOR = marker_height / camera_height.
# For a marker ~4 cm up and a camera ~150 cm up that's ~0.027. Tune by
# eye until the dot stays on the jaws at all four corners.
#
# CAMERA_NADIR_CM is where a plumb line from the camera meets the table,
# expressed in world cm. For an overhead camera roughly centred over the
# play field, (FIELD_WIDTH/2, FIELD_HEIGHT/2) is a fine starting point.
CAMERA_NADIR_CM   = (50.0, 35.0)
PARALLAX_FACTOR   = 0.18

# Exponential moving average factor for the ArUco marker readings. Without
# this the marker centre + theta + side jitter by a few cm / a few degrees
# each frame, which made the cyan gripper ring visibly twitch and made the
# bridge keep emitting little corrective TURNs. A=0.25 means each new
# sample contributes 25% and the EMA forgets the past with a half-life of
# ~2.4 frames — enough to kill jitter, fast enough that the robot
# actually moving is reflected immediately.
MARKER_EMA_ALPHA = 0.80

# Homography file produced by calibrate_homography.py. Maps full-resolution
# image pixels to world (cm) coordinates of the playing field. Without it
# we'd be falling back to the old marker-side geometry which drifted near
# the edges of the frame — refusing to start without it makes that bug
# impossible.
HOMOGRAPHY_PATH = "homography.npy"

SOURCE = 0                              # camera index or path to video/image
DETECTION_MODEL_PATH = "best_finetuned.pt"   # YOLO bounding-box model (blocks + fields + robot)

# Throttle how often we actually send commands to the robot. Vision runs at
# 30 FPS so without this we'd blast 30 commands/sec and the wheels would
# never finish a single pulse before the next override arrives. 2 s gives
# each small step (2 cm or 5 deg) plenty of time to execute and the camera
# time to see the new pose before we plan again.
COMMAND_INTERVAL_SEC = 0.9

# Motor trims pushed at startup. The chassis is asymmetric (left motor
# pulls harder) so the right side runs higher to keep it tracking straight.
# Override live from the OpenCV window with the trim keys if needed.
STARTUP_TRIM_RIGHT = 75.00
STARTUP_TRIM_LEFT  = 22.00


# ---- WP4 BLE robot driver wiring ------------------------------------------
# OpenCV runs on the main (sync) thread; bleak needs an asyncio loop. We run
# the loop in a daemon thread and dispatch send() calls to it from the
# detection loop. _bridge is the brains; _robot is the radio.
_bridge = Bridge()
_robot: Robot | None = None
_robot_loop: asyncio.AbstractEventLoop | None = None
_last_command_at = 0.0
_last_command_str: str | None = None


def _start_robot_thread():
    """Spin up an asyncio loop in a background thread and connect to BT05.
    Vision still works even if the robot fails to connect — the bridge just
    no-ops at send time, which is useful for debugging without hardware."""
    global _robot, _robot_loop

    ready = threading.Event()

    def runner():
        global _robot, _robot_loop
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        _robot_loop = loop

        async def connect():
            global _robot
            try:
                r = Robot()
                await r.connect()
                _robot = r
                # Don't override the firmware's trim values — whatever the
                # ESP booted with (currently 60/60) is what the chassis
                # was tested against. Send STOP only so we know motors
                # are idle in case the firmware was mid-pulse.
                await r.send("STOP")
                # Always open the gripper at startup so we don't have to
                # manually GRIP O after a crash/restart with jaws still closed.
                await asyncio.sleep(0.3)
                await r.send("GRIP O")
                print(f"[WP4] Robot connected (BT05). (Trims left at firmware defaults.)")
            except RobotError as e:
                print(f"[WP4] Robot connect failed: {e}.  Driving will be no-op.")
            finally:
                ready.set()

        loop.create_task(connect())
        loop.run_forever()

    t = threading.Thread(target=runner, daemon=True, name="robot-bleak-loop")
    t.start()
    ready.wait(timeout=12.0)


def _send_to_robot(command: str):
    """Schedule a non-blocking send onto the robot loop from any thread."""
    if _robot is None or _robot_loop is None:
        return
    asyncio.run_coroutine_threadsafe(_robot.send(command), _robot_loop)


def _reconnect_robot() -> str:
    """Force-tear-down and re-pair the BLE link. Called from the dashboard
    Reconnect button when the link drops without auto-recovery. Runs the
    coroutine on the bleak loop and waits up to 15 s for it to finish so
    the HTTP handler can return a real status string."""
    global _robot
    if _robot_loop is None:
        return "no robot loop"

    async def _do() -> str:
        global _robot
        try:
            if _robot is not None:
                try:
                    await _robot.disconnect()
                except Exception:
                    pass
            r = Robot()
            await r.connect()
            await r.send("STOP")
            _robot = r
            return "connected"
        except RobotError as e:
            _robot = None
            return f"failed: {e}"

    fut = asyncio.run_coroutine_threadsafe(_do(), _robot_loop)
    try:
        return fut.result(timeout=15.0)
    except Exception as e:
        return f"reconnect timed out: {e}"


def _start_ble_watchdog():
    """Spawn a daemon thread that pokes _reconnect_robot whenever the
    BLE link looks dead. robot.py has an internal heartbeat that tries
    to reconnect every ~10 s, but in practice the macOS BLE stack
    sometimes gets stuck and never recovers without a full pair-up
    cycle. This watchdog does that pair-up cycle on its own when the
    link has been down for `WATCHDOG_GRACE_SEC`."""
    WATCHDOG_GRACE_SEC = 8.0
    WATCHDOG_RETRY_SEC = 20.0   # don't hammer macOS BLE — failed retries cost ~15 s each

    def loop():
        last_attempt = 0.0
        while True:
            time.sleep(2.0)
            try:
                is_live = (
                    _robot is not None
                    and getattr(_robot, "_client", None) is not None
                    and _robot._client.is_connected
                )
            except Exception:
                is_live = False
            if is_live:
                continue
            # Link looks dead. Wait for the grace period (robot.py's own
            # heartbeat reconnect might still work) before we step in,
            # then rate-limit our attempts.
            now = time.monotonic()
            if now - last_attempt < WATCHDOG_RETRY_SEC:
                continue
            last_attempt = now
            print("[WP4] watchdog: BLE link down, attempting reconnect...")
            try:
                status = _reconnect_robot()
                print(f"[WP4] watchdog: {status}")
            except Exception as e:
                print(f"[WP4] watchdog: reconnect raised {e}")

    t = threading.Thread(target=loop, daemon=True, name="ble-watchdog")
    t.start()


def box_center(box):
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    return ((x1 + x2) // 2, (y1 + y2) // 2)

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def draw_arrow(frame, pt1, pt2, color, thickness=2):
    cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.04)

def _load_tunables() -> dict:
    """Read tunables.json if it exists; otherwise return {} so the
    module-level defaults stand. Silent on read errors — bad JSON should
    not crash the demo."""
    import json, os
    if not os.path.exists(TUNABLES_PATH):
        return {}
    try:
        with open(TUNABLES_PATH, "r") as f:
            data = json.load(f)
        print(f"[WP4] Loaded tunables from {TUNABLES_PATH}: {data}")
        return data
    except Exception as e:
        print(f"[WP4] Could not read {TUNABLES_PATH}: {e}. Using defaults.")
        return {}


def _reset_tunables(state: dict) -> None:
    """Reset in-memory tunables to the module-level defaults. Mirrors the
    '0' key in the OpenCV window so the dashboard's 'Reset to defaults'
    button doesn't need to know what the defaults are."""
    state["tun_gripper_forward_cm"] = GRIPPER_FORWARD_CM
    state["tun_gripper_right_cm"]   = GRIPPER_RIGHT_CM
    state["tun_parallax_factor"]    = PARALLAX_FACTOR
    state["tun_camera_nadir_cm"]    = CAMERA_NADIR_CM
    print("[WP4] Tunables reset to module defaults (not saved).")


def _save_tunables(state: dict) -> None:
    """Dump the live-tunable knobs to disk so a restart picks them up."""
    import json
    data = {
        "gripper_forward_cm": float(state["tun_gripper_forward_cm"]),
        "gripper_right_cm":   float(state["tun_gripper_right_cm"]),
        "parallax_factor":    float(state["tun_parallax_factor"]),
        "camera_nadir_cm":    [float(state["tun_camera_nadir_cm"][0]),
                               float(state["tun_camera_nadir_cm"][1])],
    }
    try:
        with open(TUNABLES_PATH, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[WP4] Saved tunables to {TUNABLES_PATH}: {data}")
    except Exception as e:
        print(f"[WP4] Could not write {TUNABLES_PATH}: {e}")


def _load_homography():
    """Load the pixel->world homography produced by
    calibrate_homography.py, or bail out with a clear message. Without it
    every downstream measurement (gripper offset, grab radius, MOVE cm)
    would silently be off near the edges of the frame."""
    import os
    if not os.path.exists(HOMOGRAPHY_PATH):
        print(f"[WP4] ERROR: '{HOMOGRAPHY_PATH}' not found.")
        print(f"[WP4]   Run:  python3 calibrate_homography.py")
        print(f"[WP4]   Then click the 4 corners of the cardboard.")
        sys.exit(2)
    H = np.load(HOMOGRAPHY_PATH)
    H_inv = np.linalg.inv(H)
    print(f"[WP4] Loaded homography from {HOMOGRAPHY_PATH}.")
    return H.astype(np.float32), H_inv.astype(np.float32)


def run_detection(source, detection_model_path):
    H_pixel_to_world, H_world_to_pixel = _load_homography()
    _start_robot_thread()
    _start_ble_watchdog()
    model = YOLO(detection_model_path)
    trail = collections.deque(maxlen=ROBOT_TRAIL_LEN)
    tunables = _load_tunables()
    state = {
        "H_pixel_to_world": H_pixel_to_world,
        "H_world_to_pixel": H_world_to_pixel,
        "robot_radius_cm": None,
        "last_robot_center_cm": None,
        "last_marker_side_cm": None,
        "last_theta_deg": None,
        "frames_since_marker": 0,
        # EMA-smoothed marker pose in WORLD (cm) coordinates, updated
        # each frame the marker is seen. Theta as unit-vector (cos, sin)
        # so averaging stays wrap-safe across ±180°.
        "ema_robot_center_cm": None,    # np.array([x_cm, y_cm])
        "ema_marker_side_cm": None,     # float, cm
        "ema_theta_cs": None,           # np.array([cos(theta), sin(theta)])
        # Pixel-space leftovers — A* still works on the pixel grid and
        # draw_path_overlay wants pixels, so we keep these around as the
        # last good pixel snapshot.
        "last_pixel_center": None,
        "last_robot_radius_px": None,
        # Live-tunable knobs. Loaded from tunables.json if present,
        # mutated by keypresses in the OpenCV window, saved back with K.
        # Module-level constants are just the initial defaults.
        "tun_gripper_forward_cm": tunables.get("gripper_forward_cm", GRIPPER_FORWARD_CM),
        "tun_gripper_right_cm":   tunables.get("gripper_right_cm",   GRIPPER_RIGHT_CM),
        "tun_parallax_factor":    tunables.get("parallax_factor",    PARALLAX_FACTOR),
        "tun_camera_nadir_cm":    tuple(tunables.get("camera_nadir_cm", CAMERA_NADIR_CM)),
        # Last-clicked pixel (for setting nadir by click).
        "pending_nadir_click_world": None,
    }

    # Launch the localhost dashboard. Opt-in: if it can't bind (port
    # taken, sandbox, whatever) the demo still runs from the OpenCV
    # window. A daemon thread serves /, /status.json, /stream.mjpg and
    # the POST control endpoints; everything mutates the same `state`
    # the WASD keys do.
    try:
        web_dashboard.start_dashboard(
            state, _bridge, _send_to_robot,
            save_tunables=lambda: _save_tunables(state),
            reset_tunables=lambda: _reset_tunables(state),
            reconnect_robot=_reconnect_robot,
        )
    except Exception as e:
        print(f"[WP4] Dashboard failed to start: {e}. Continuing without it.")

    # FPS smoothing for the dashboard HUD. Cheap EMA over frame intervals;
    # main.py is the only producer so we don't need a lock around the float.
    state["_fps_last_t"] = None
    state["fps"] = None

    try:
        cam_index = int(source)
        use_camera = True
    except ValueError:
        use_camera = False

    # Quit on q / Q / ESC. Use a longer waitKey so the OpenCV window has a
    # fair chance to process keypresses; with waitKey(1) it often misses.
    #
    # Operator keys (the OpenCV window has to have focus):
    #
    #   Flow control
    #     SPACE     START the flow (leave AWAITING_START)
    #     R         RESET back to AWAITING_START so we can re-align
    #     Q / ESC   quit
    #
    #   Live tune for the cyan gripper ring (no restart!)
    #     W / S     gripper FORWARD offset ±1 cm   (towards / away from jaws)
    #     A / D     gripper RIGHT   offset ±1 cm   (left / right of axis)
    #     - / +     PARALLAX_FACTOR ±0.01          (corner-edge correction)
    #     N         click on the image to set CAMERA_NADIR_CM there
    #     K         save current tunables to tunables.json
    #     0         reset all tunables to module defaults (in-memory only)
    #
    # Why two separate things to tune:
    #   GRIPPER_FORWARD/RIGHT_CM is the *chassis geometry* — marker centre
    #     to jaws. Fixed by the robot's build. If the ring sits in the same
    #     wrong place no matter where the robot is on the field, change
    #     these.
    #   PARALLAX_FACTOR is the *camera-vs-marker-height* correction. The
    #     ring drifts only near the corners of the field (and the direction
    #     of drift depends on where the robot is). Change this when the
    #     centre is fine but the corners aren't.
    def _handle_keys() -> bool:
        """Process one keypress. Returns True iff we should quit."""
        k = cv2.waitKey(20) & 0xFF
        if k == 255:                            # no key
            return False
        if k in (ord("q"), ord("Q"), 27):
            return True
        if k == ord(" "):
            _bridge.start()
            print(f"[WP4] START -> {_bridge.state}")
        elif k in (ord("r"), ord("R")):
            _bridge.reset()
            print("[WP4] RESET -> AWAITING_START. Re-align the robot, then press SPACE.")
        # Gripper offset tune — values are in cm of real-world distance.
        elif k in (ord("w"), ord("W")):
            state["tun_gripper_forward_cm"] += 1.0
            print(f"[WP4] gripper FORWARD = {state['tun_gripper_forward_cm']:+.2f} cm")
        elif k in (ord("s"), ord("S")):
            state["tun_gripper_forward_cm"] -= 1.0
            print(f"[WP4] gripper FORWARD = {state['tun_gripper_forward_cm']:+.2f} cm")
        elif k in (ord("d"), ord("D")):
            state["tun_gripper_right_cm"] += 1.0
            print(f"[WP4] gripper RIGHT   = {state['tun_gripper_right_cm']:+.2f} cm")
        elif k in (ord("a"), ord("A")):
            state["tun_gripper_right_cm"] -= 1.0
            print(f"[WP4] gripper RIGHT   = {state['tun_gripper_right_cm']:+.2f} cm")
        # Parallax — note these are tiny steps; 0.01 is a meaningful change.
        elif k in (ord("+"), ord("="), ord("]")):
            state["tun_parallax_factor"] += 0.01
            print(f"[WP4] PARALLAX_FACTOR = {state['tun_parallax_factor']:.3f}")
        elif k in (ord("-"), ord("_"), ord("[")):
            state["tun_parallax_factor"] -= 0.01
            print(f"[WP4] PARALLAX_FACTOR = {state['tun_parallax_factor']:.3f}")
        elif k in (ord("n"), ord("N")):
            print("[WP4] Click anywhere on the frame to set CAMERA_NADIR_CM "
                  "there (the spot directly under the camera lens).")
            state["pending_nadir_click_world"] = "armed"
        elif k in (ord("k"), ord("K")):
            _save_tunables(state)
        elif k == ord("0"):
            state["tun_gripper_forward_cm"] = GRIPPER_FORWARD_CM
            state["tun_gripper_right_cm"]   = GRIPPER_RIGHT_CM
            state["tun_parallax_factor"]    = PARALLAX_FACTOR
            state["tun_camera_nadir_cm"]    = CAMERA_NADIR_CM
            print("[WP4] Tunables reset to module defaults (not saved).")
        return False

    # Create the window up front so we can attach a mouse callback for
    # the "click to set nadir" feature. cv2.imshow would create it
    # lazily; doing it here means setMouseCallback always finds it.
    cv2.namedWindow("Sortify - Real-time Detection", cv2.WINDOW_AUTOSIZE)

    def _on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if state.get("pending_nadir_click_world") != "armed":
            return
        # Map the clicked pixel back to world cm via the homography.
        wp = _to_world(state["H_pixel_to_world"], np.array([[x, y]]))[0]
        state["tun_camera_nadir_cm"] = (float(wp[0]), float(wp[1]))
        state["pending_nadir_click_world"] = None
        print(f"[WP4] CAMERA_NADIR_CM = ({wp[0]:.1f}, {wp[1]:.1f}) cm")

    cv2.setMouseCallback("Sortify - Real-time Detection", _on_mouse)

    try:
        if use_camera:
            with Camera(index=cam_index, width=3840, height=2160) as cam:
                print(f"Real-time detection started (camera {cam_index}). Press Q or ESC to quit.")
                while True:
                    frame = cam.get_frame()
                    _process_frame(frame, model, trail, state)
                    if _handle_keys():
                        break
        else:
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                sys.exit(f"Error: could not open source '{source}'")
            print(f"Processing '{source}'. Press Q or ESC to quit.")
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                _process_frame(frame, model, trail, state)
                if _handle_keys():
                    break
            cap.release()
    finally:
        # Close the session recording (if we opened one) before tearing
        # down the OpenCV window. VideoWriter's footer is only written
        # on .release(); skipping this leaves an unplayable file.
        writer = state.get("video_writer")
        if writer is not None:
            writer.release()
            out_path = state.get("video_path")
            print(f"[WP4] Saved session recording to {out_path}")
        cv2.destroyAllWindows()



def _try_aruco(gray_img, aruco_dict, scale):
    """Returns the marker's 4 corners as a (4, 2) float array in
    full-resolution image pixels (we undo the resize via `scale`).
    The caller does the rest — center, theta, side — in world
    coordinates after a homography pass, so we don't compute them here
    anymore (the perspective-distorted pixel versions were misleading).
    """
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.adaptiveThreshConstant = 7
        params.errorCorrectionRate = 0.6
        corners, ids, _ = cv2.aruco.ArucoDetector(aruco_dict, params).detectMarkers(gray_img)
    else:
        params = cv2.aruco.DetectorParameters_create()
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 4
        params.errorCorrectionRate = 0.6
        corners, ids, _ = cv2.aruco.detectMarkers(gray_img, aruco_dict, parameters=params)

    if ids is None:
        return None
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) != ROBOT_MARKER_ID:
            continue
        pts = marker_corners.reshape(4, 2).astype(float) / scale
        return pts   # shape (4, 2), full-resolution pixels
    return None


def detect_marker_corners(frame):
    """Locate the robot marker and return its 4 corners in image pixels,
    or None if not visible. We keep the original multi-scale + CLAHE
    fallbacks because they were the difference between ArUco missing
    every other frame and the robust detection we have now."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # CLAHE helps with shiny/dark surfaces and uneven lighting
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_clahe = clahe.apply(gray)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    h, w = gray.shape

    for src in [gray, gray_clahe]:
        for scale in [0.5, 0.25, 1.0]:
            small = cv2.resize(src, (int(w * scale), int(h * scale))) if scale != 1.0 else src
            result = _try_aruco(small, aruco_dict, scale)
            if result is not None:
                return result

    return None


def _pose_from_world_corners(world_corners: np.ndarray):
    """Given the marker's 4 corners already mapped to world (cm) space,
    return (center_cm, theta_deg, side_cm). All geometric — no more
    perspective surprises because we're already in the flat plane."""
    center = world_corners.mean(axis=0)
    top_mid = (world_corners[0] + world_corners[1]) / 2.0
    front_vec = top_mid - center
    theta = math.degrees(math.atan2(float(front_vec[1]), float(front_vec[0])))
    side_len = float(np.mean([
        np.linalg.norm(world_corners[1] - world_corners[0]),
        np.linalg.norm(world_corners[2] - world_corners[1]),
        np.linalg.norm(world_corners[3] - world_corners[2]),
        np.linalg.norm(world_corners[0] - world_corners[3]),
    ]))
    return center, theta, side_len


def _to_world(H: np.ndarray, pixel_pts: np.ndarray) -> np.ndarray:
    """Map a (N, 2) array of image pixels into world (cm). Wraps the
    OpenCV plumbing that wants a (1, N, 2) float32 input."""
    pts = pixel_pts.astype(np.float32).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, H)
    return out.reshape(-1, 2)


def _to_pixel(H_inv: np.ndarray, world_pts: np.ndarray) -> np.ndarray:
    """Inverse of `_to_world`: world (cm) -> image pixels."""
    pts = world_pts.astype(np.float32).reshape(-1, 1, 2)
    out = cv2.perspectiveTransform(pts, H_inv)
    return out.reshape(-1, 2)








def draw_planned_path(frame, waypoints, color, thickness=2):
    if not waypoints:
        return
    pts = [(int(x), int(y)) for x, y in waypoints]
    for a, b in zip(pts[:-1], pts[1:]):
        cv2.line(frame, a, b, color, thickness)
    if len(pts) >= 2:
        cv2.arrowedLine(frame, pts[-2], pts[-1], color, thickness, tipLength=0.06)


def draw_path_overlay(frame, robot_center, path_to_block, path_to_field, trail, robot_radius=None):
    overlay = frame.copy()

    # Trail: fading line of past robot positions
    trail_list = list(trail)
    for i in range(1, len(trail_list)):
        alpha = i / len(trail_list)
        thickness = max(1, int(4 * alpha))
        color_val = int(100 + 155 * alpha)
        pt1 = (int(trail_list[i - 1][0]), int(trail_list[i - 1][1]))
        pt2 = (int(trail_list[i][0]), int(trail_list[i][1]))
        cv2.line(overlay, pt1, pt2, (0, color_val, 255), thickness)

    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

    # A*-planned paths
    draw_planned_path(frame, path_to_block, (0, 255, 255), 3)   # cyan: robot → block
    draw_planned_path(frame, path_to_field, (0, 140, 255), 3)   # orange: block → field

    # Robot: center dot + actual footprint circle
    if robot_center is not None:
        cx, cy = int(robot_center[0]), int(robot_center[1])
        if robot_radius is not None:
            cv2.circle(frame, (cx, cy), int(robot_radius), (0, 200, 255), 2)  # footprint ring
        cv2.circle(frame, (cx, cy), 8, (255, 0, 200), -1)
        cv2.circle(frame, (cx, cy), 8, (255, 255, 255), 1)



def _process_frame(frame, model, trail, state):
    # Hand the full-resolution frame to YOLO. We tried downscaling to
    # 1280 + imgsz=640 here for FPS — the model stopped detecting the
    # big coloured fields (they fell outside the size distribution it
    # was trained on) and the demo broke. Keep the full frame; we'll
    # claw back FPS elsewhere (Iriun resolution, dashboard subsampling).
    results = model(frame, verbose=False)[0]
    scaled_boxes = list(results.boxes)

    # --- detect robot marker, then convert everything to world (cm) ---
    # The whole point of the homography is that we stop reasoning in
    # pixels (which are perspective-distorted near the edges of the frame)
    # and start reasoning in real centimetres on the cardboard. So:
    #   1) get the marker's 4 corners as pixels
    #   2) map them straight into world coordinates
    #   3) compute centre / theta / side IN WORLD — they're truthful now
    #   4) EMA-smooth those world values
    #   5) gripper offset is a real-world translation (cm), no scaling
    # The bridge downstream gets world coordinates so its grab/release
    # radii are in cm and its MOVE commands match physical distance.
    theta = None
    marker_side_cm = None
    robot_center_cm = None
    H = state["H_pixel_to_world"]
    pixel_corners = detect_marker_corners(frame)

    if pixel_corners is not None:
        world_corners = _to_world(H, pixel_corners)
        raw_center, raw_theta, raw_side = _pose_from_world_corners(world_corners)
        # Parallax correction: marker sits a few cm above the table, so its
        # apparent ground-plane position drifts *away* from the camera
        # nadir as the robot moves toward the corners of the play field.
        # Pull the apparent position back toward the nadir by a constant
        # fraction; with the camera near the table centre this is a fine
        # first-order fix.
        nadir = np.asarray(state["tun_camera_nadir_cm"], dtype=float)
        raw_center = nadir + (1.0 - state["tun_parallax_factor"]) * (
            np.asarray(raw_center, dtype=float) - nadir
        )
        raw_center_arr = np.asarray(raw_center, dtype=float)
        raw_theta_cs = np.array(
            [math.cos(math.radians(raw_theta)), math.sin(math.radians(raw_theta))]
        )
        # First sighting: seed the EMA directly. Otherwise blend.
        a = MARKER_EMA_ALPHA
        if state["ema_robot_center_cm"] is None:
            state["ema_robot_center_cm"] = raw_center_arr
            state["ema_marker_side_cm"] = raw_side
            state["ema_theta_cs"] = raw_theta_cs
        else:
            state["ema_robot_center_cm"] = (
                a * raw_center_arr + (1 - a) * state["ema_robot_center_cm"]
            )
            state["ema_marker_side_cm"] = (
                a * raw_side + (1 - a) * state["ema_marker_side_cm"]
            )
            blended = a * raw_theta_cs + (1 - a) * state["ema_theta_cs"]
            norm = float(np.linalg.norm(blended))
            if norm > 1e-6:
                state["ema_theta_cs"] = blended / norm

        robot_center_cm = state["ema_robot_center_cm"]
        marker_side_cm = float(state["ema_marker_side_cm"])
        theta = math.degrees(
            math.atan2(float(state["ema_theta_cs"][1]), float(state["ema_theta_cs"][0]))
        )
        state["robot_radius_cm"] = marker_side_cm * ROBOT_CLEARANCE_FACTOR
        state["last_robot_center_cm"] = robot_center_cm.copy()
        state["last_marker_side_cm"] = marker_side_cm
        state["last_theta_deg"] = theta
        state["frames_since_marker"] = 0
        trail.append(robot_center_cm.copy())
    elif state["frames_since_marker"] < 10:
        # Use last known world pose for up to 10 missed frames.
        robot_center_cm = state["last_robot_center_cm"]
        marker_side_cm = state.get("last_marker_side_cm")
        theta = state.get("last_theta_deg")
        state["frames_since_marker"] += 1

    # --- gripper tip in world (cm) ---
    # Offsets are real centimetres — independent of camera distance or
    # where on the field the robot happens to be. This is the win from
    # the homography pass: the chassis geometry is the chassis geometry.
    gripper_xy_cm = None
    if robot_center_cm is not None and theta is not None:
        heading = math.radians(theta)
        right   = heading + math.pi / 2.0
        fwd_cm = state["tun_gripper_forward_cm"]
        rt_cm  = state["tun_gripper_right_cm"]
        gripper_xy_cm = (
            float(robot_center_cm[0]) + fwd_cm * math.cos(heading)
                                      + rt_cm  * math.cos(right),
            float(robot_center_cm[1]) + fwd_cm * math.sin(heading)
                                      + rt_cm  * math.sin(right),
        )

    # BGR colors per class color name
    COLOR_MAP = {
        "red":   (0,   0,   220),
        "green": (0,   200, 0  ),
        "blue":  (220, 80,  0  ),
    }
    COLOR_PRIORITY = {"red": 0, "blue": 1, "green": 2}

    all_detections = []
    blocks_by_color = {}   # color -> list of Detection
    fields_by_color = {}   # color -> Detection

    # Confidence threshold: below this the detection is too noisy to drive
    # decisions from. The robot's own body keeps getting tagged as a faint
    # "green field" at ~0.4, which would derail planning. We still draw the
    # box dimmed so we can see what the model is doing.
    BRIDGE_CONF_MIN = 0.35

    for box in scaled_boxes:
        cls = int(box.cls[0])
        name = results.names[cls]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        conf = float(box.conf[0])

        # Class names come from the YOLO model and have varied between
        # training runs: some used "red field" with a space, the latest
        # uses "red_field" with an underscore. Normalise both so we
        # don't silently ignore everything when the model changes.
        parts = name.lower().replace("_", " ").split()
        accepted = conf >= BRIDGE_CONF_MIN
        if len(parts) == 2 and accepted:
            color, kind = parts
            det = Detection(
                cls_name=name, color=color, kind=kind, conf=conf,
                xyxy=(x1, y1, x2, y2), center=(cx, cy),
            )
            all_detections.append(det)
            if kind == "block":
                blocks_by_color.setdefault(color, []).append(det)
            elif kind == "field":
                fields_by_color[color] = det

        # draw bounding box (dimmed if filtered out)
        label = f"{name} {conf:.2f}"
        detected_color = name.lower().split()[0] if name else ""
        box_color = COLOR_MAP.get(detected_color, (180, 180, 180))
        if not accepted:
            box_color = tuple(c // 3 for c in box_color)   # dim it
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), box_color, -1)
        cv2.putText(frame, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # --- pick block by priority (red > blue > green), then nearest ---
    # Nearest-by-world-cm now that we have the homography. Distances in
    # pixels would have unfairly preferred whichever block sat closer to
    # the centre of the frame.
    target_det = None
    field_det = None
    best_key = (float("inf"), float("inf"))
    robot_center_for_dist_cm = robot_center_cm

    for color, dets in blocks_by_color.items():
        if color not in fields_by_color:
            continue
        priority = COLOR_PRIORITY.get(color, 99)
        f = fields_by_color[color]
        fx1, fy1, fx2, fy2 = f.xyxy
        for det in dets:
            cx, cy = det.center
            if fx1 <= cx <= fx2 and fy1 <= cy <= fy2:
                continue  # already in its field
            if robot_center_for_dist_cm is not None:
                det_cm = _to_world(H, np.array([det.center]))[0]
                d = float(np.linalg.norm(det_cm - robot_center_for_dist_cm))
            else:
                d = 0.0
            key = (priority, d)
            if key < best_key:
                best_key = key
                target_det = det
                field_det = f

    # --- A* paths: robot → block, then block → field ---
    # A* still plans on the pixel grid (it's an image-occupancy grid; that
    # part is easier to keep in pixel space). The waypoints are converted
    # to world cm just before being handed to the bridge so the bridge
    # never sees pixels.
    path_to_block = []     # pixel waypoints
    path_to_field = []     # pixel waypoints

    if target_det is not None and robot_center_cm is not None:
        if state.get("_path_debug_frames", 0) % 60 == 0:
            print(f"[DBG] A* try: target_color={target_det.color} "
                  f"target_center_px={target_det.center} "
                  f"frame_shape={frame.shape[:2]}")
        state["_path_debug_frames"] = state.get("_path_debug_frames", 0) + 1
        # A* needs the robot's pixel centre; we keep it in `state` for
        # exactly this kind of round-trip use.
        last_pixel_center = None
        if pixel_corners is not None:
            last_pixel_center = pixel_corners.mean(axis=0)
            state["last_pixel_center"] = last_pixel_center
        else:
            last_pixel_center = state.get("last_pixel_center")

        if last_pixel_center is not None:
            rc_px = tuple(map(float, last_pixel_center))
            # Robot radius needs to be in pixels for the pixel-space grid.
            # The marker side in pixels varies across the frame because of
            # perspective, but for an A* clearance buffer it's a fine
            # approximation — use the current measurement.
            if pixel_corners is not None:
                marker_side_px = float(np.mean([
                    np.linalg.norm(pixel_corners[1] - pixel_corners[0]),
                    np.linalg.norm(pixel_corners[2] - pixel_corners[1]),
                    np.linalg.norm(pixel_corners[3] - pixel_corners[2]),
                    np.linalg.norm(pixel_corners[0] - pixel_corners[3]),
                ]))
                robot_radius_px = marker_side_px * ROBOT_CLEARANCE_FACTOR
                state["last_robot_radius_px"] = robot_radius_px
            else:
                robot_radius_px = state.get("last_robot_radius_px")
            scales = [1.0, 0.5, 0.25] if robot_radius_px is not None else [None]

            for scale in scales:
                r = robot_radius_px * scale if (robot_radius_px is not None and scale is not None) else None
                kwargs = {} if r is None else {"robot_radius": r}
                grid1 = build_occupancy_grid(frame.shape, all_detections, target_det, **kwargs)
                path_to_block = astar(grid1, rc_px, target_det.center)
                if path_to_block:
                    if state.get("_path_debug_frames", 0) % 60 == 0:
                        print(f"[DBG] A* OK at scale={scale}, "
                              f"len={len(path_to_block)}, "
                              f"start={rc_px}, end={target_det.center}")
                    break
            else:
                if state.get("_path_debug_frames", 0) % 60 == 0:
                    print(f"[DBG] A* FAILED at all scales. "
                          f"rc_px={rc_px}, target={target_det.center}, "
                          f"robot_radius_px={robot_radius_px}")

            if field_det is not None:
                for scale in scales:
                    r = robot_radius_px * scale if (robot_radius_px is not None and scale is not None) else None
                    kwargs = {} if r is None else {"robot_radius": r}
                    grid2 = build_occupancy_grid(frame.shape, all_detections, target_block=target_det, **kwargs)
                    path_to_field = astar(grid2, target_det.center, field_det.center)
                    if path_to_field:
                        break

    # --- draw path overlay (uses pixels because cv2.draw* are pixel-native) ---
    robot_pixel_center = None
    if pixel_corners is not None:
        robot_pixel_center = pixel_corners.mean(axis=0)
    elif state.get("last_pixel_center") is not None:
        robot_pixel_center = state["last_pixel_center"]
    robot_radius_px_for_draw = state.get("last_robot_radius_px")
    draw_path_overlay(frame, robot_pixel_center, path_to_block, path_to_field, trail,
                      robot_radius=robot_radius_px_for_draw)

    # Render the gripper-tip estimate. Colour reflects whether the bridge
    # currently believes the jaws are open (cyan) or closed (magenta) so we
    # can tell from a glance whether the cube is being held. The gripper is
    # computed in world cm; we map it back to pixels for drawing.
    H_inv = state["H_world_to_pixel"]
    if gripper_xy_cm is not None:
        gripper_px = _to_pixel(H_inv, np.array([gripper_xy_cm]))[0]
        gx, gy = int(gripper_px[0]), int(gripper_px[1])
        gripper_color = (255, 255, 0) if _bridge.gripper_open else (255, 0, 255)
        gripper_label = "OPEN" if _bridge.gripper_open else "CLOSED"
        cv2.circle(frame, (gx, gy), 12, gripper_color, 3)
        cv2.circle(frame, (gx, gy), 4, (255, 255, 255), -1)
        cv2.putText(frame, f"gripper {gripper_label}", (gx + 14, gy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, gripper_color, 1)
        # Grab radius is now in cm; project a circle of that radius from
        # gripper-world into pixels. The pixel circle is no longer truly
        # circular under perspective, but for a sanity overlay an ellipse-
        # less circle of approximate radius is plenty. We approximate by
        # mapping one point at (radius, 0) cm offset and using its pixel
        # distance.
        from bridge import BLOCK_GRAB_RADIUS_CM as _GRAB_R_CM
        ref_world = np.array([
            [gripper_xy_cm[0] + _GRAB_R_CM, gripper_xy_cm[1]]
        ])
        ref_px = _to_pixel(H_inv, ref_world)[0]
        grab_radius_px = int(math.hypot(ref_px[0] - gx, ref_px[1] - gy))
        cv2.circle(frame, (gx, gy), grab_radius_px, gripper_color, 1, lineType=cv2.LINE_AA)
        # Distance line: gripper-to-targeted-block. Label in cm because
        # that's what actually matters now.
        if target_det is not None:
            tx, ty = int(target_det.center[0]), int(target_det.center[1])
            cv2.line(frame, (gx, gy), (tx, ty), gripper_color, 1)
            mid = ((gx + tx) // 2, (gy + ty) // 2)
            block_cm = _to_world(H, np.array([target_det.center]))[0]
            d_cm = float(np.linalg.norm(block_cm - np.array(gripper_xy_cm)))
            inside = d_cm <= _GRAB_R_CM
            label = f"{d_cm:.1f}cm"
            if inside:
                label += " IN RANGE"
            cv2.putText(frame, label, mid,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, gripper_color, 2)

    # --- debug: heading arrow (drawn from pixel centre for the operator) ---
    if robot_pixel_center is not None and theta is not None:
        cx, cy = int(robot_pixel_center[0]), int(robot_pixel_center[1])
        # Arrow in world (cm) — 6 cm forward — then projected back to
        # pixels. This way the arrow length is consistent regardless of
        # where on the field the robot is.
        if robot_center_cm is not None:
            tip_cm = (
                robot_center_cm[0] + 6.0 * math.cos(math.radians(theta)),
                robot_center_cm[1] + 6.0 * math.sin(math.radians(theta)),
            )
            tip_px = _to_pixel(H_inv, np.array([tip_cm]))[0]
            tip = (int(tip_px[0]), int(tip_px[1]))
            cv2.arrowedLine(frame, (cx, cy), tip, (0, 0, 255), 4, tipLength=0.25)
        cv2.putText(frame, f"theta={theta:+.0f}", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # ---- WP4 bridge: vision -> single robot command per frame ----
    # Everything we hand the bridge is in world cm now: distances, paths,
    # gripper, blocks, fields. Bridge thresholds (grab radius, MOVE max,
    # waypoint reached) all live in cm.
    if robot_center_cm is not None and theta is not None:
        def _bxy(px_xy):
            wp = _to_world(H, np.array([px_xy]))[0]
            return (float(wp[0]), float(wp[1]))

        vision = {
            "robot_center": (float(robot_center_cm[0]), float(robot_center_cm[1])),
            "theta_deg":    float(theta),
            "gripper_pos":  tuple(map(float, gripper_xy_cm)) if gripper_xy_cm is not None else None,
            "blocks_by_color": {
                color: [_bxy(d.center) for d in dets]
                for color, dets in blocks_by_color.items()
            },
            "fields_by_color": {
                color: _bxy(d.center) for color, d in fields_by_color.items()
            },
            # Field rectangles in world cm. The bridge uses these to filter
            # out cubes already physically inside their target field (see
            # `_nearest_block_of_color` in bridge.py). The xyxy is in pixel
            # space, so both corners get the homography pass. After warping,
            # (x1, y1) and (x2, y2) aren't guaranteed to be the geometric
            # min/max corners anymore — `_box_contains` normalises that.
            "field_boxes_by_color": {
                color: (
                    *_bxy((d.xyxy[0], d.xyxy[1])),
                    *_bxy((d.xyxy[2], d.xyxy[3])),
                )
                for color, d in fields_by_color.items()
            },
            "path_to_block": [_bxy(p) for p in path_to_block],
            "path_to_field": [_bxy(p) for p in path_to_field],
        }
        # BLE link state-transition guard. If the link just came back up
        # after being down, the bridge was still ticking and may want to
        # fire a queued MOVE/TURN the instant the radio reattaches —
        # which is how we ended up with the robot suddenly lurching after
        # every reconnect. Force the bridge into AWAITING_START on the
        # *fall* (down) so it stops emitting commands; the operator has
        # to physically press SPACE to resume. This is the safest
        # behaviour: pause means pause.
        #
        # Computed here (before bridge.next_command) instead of in the
        # HUD block lower down, because the cmd-gate below needs it too
        # and the HUD path only runs when robot_center is visible.
        ble_live = bool(
            _robot is not None
            and getattr(_robot, "_client", None) is not None
            and _robot._client.is_connected
        )
        state["robot_connected"] = ble_live

        # BLE link state-transition guard. Bridge state is PRESERVED
        # across drops — the operator asked for the flow to resume
        # automatically when the link comes back, no SPACE re-press.
        # We still suppress commands while the radio is down so they
        # don't queue up; on reconnect we send a single STOP to make
        # sure the ESP wasn't mid-pulse, then let bridge.next_command
        # take over from whatever state it's in.
        _last_command_at_reset = False
        was_live = state.get("_prev_ble_live")
        if was_live is None:
            state["_prev_ble_live"] = ble_live
        elif was_live and not ble_live:
            print("[WP4] BLE link dropped — commands suppressed until reconnect.")
            state["_prev_ble_live"] = False
            # Reset throttle so the first command after reconnect doesn't
            # have a stale `_last_command_at` that allows an instant
            # double-fire when the radio reattaches.
            _last_command_at_reset = True
        elif (not was_live) and ble_live:
            _send_to_robot("STOP")
            print(f"[WP4] BLE link restored — resuming bridge state {_bridge.state}.")
            state["_prev_ble_live"] = True
            _last_command_at_reset = True

        cmd = _bridge.next_command(vision)
        global _last_command_at, _last_command_str
        now = time.monotonic()
        if _last_command_at_reset:
            # Pretend we just sent a command, so the throttle (next 2 s
            # window) lets the chassis settle / lets us read marker
            # fresh before anything new goes out on the wire.
            _last_command_at = now

        # Don't send anything if the radio is down. Bridge ticks freely
        # (state machine still advances on its own) but the wire is
        # gated; otherwise we'd be queueing commands the ESP never sees
        # and then dumping them all at once on reconnect.
        if not ble_live:
            cmd = None

        # Throttle: send a new MOVE/TURN/GRIP at most every COMMAND_INTERVAL_SEC.
        # State-transition pulses slip through immediately:
        #  - STOP (safety / target-radius latch — must arrive without delay)
        #  - GRIP C / GRIP O (single-shot servo command)
        #  - The backoff MOVE (one-shot reverse pulse). Bridge marks
        #    _backoff_sent=True after emitting it once and won't retry; if
        #    the throttle drops the command, the whole BACKING_OFF state
        #    times out and the robot just sits there. Treating any signed
        #    MOVE -XX.XX as urgent fixes this without complicating bridge.py.
        is_urgent = (
            cmd in ("STOP",)
            or (cmd is not None and cmd.startswith("GRIP "))
            or (cmd is not None and cmd.startswith("MOVE -"))
        )
        if cmd is not None and (is_urgent or now - _last_command_at >= COMMAND_INTERVAL_SEC):
            _send_to_robot(cmd)
            # Big TURN (>=12°): chassis is still rotating + ArUco needs
            # a clean frame, +1 s of quiet. Everything else uses the
            # base COMMAND_INTERVAL_SEC.
            extra_wait = 0.0
            if cmd.startswith("TURN "):
                try:
                    deg = abs(int(cmd[5:]))
                    if deg >= 12:
                        extra_wait = 1.0
                except ValueError:
                    pass
            _last_command_at = now + extra_wait
            _last_command_str = cmd
            # Mirror to state so the dashboard's status panel can show
            # the most recent command. The OpenCV HUD reads the global;
            # the dashboard reads the state key. Same value either way.
            state["last_command_str"] = cmd

        # HUD: show the most recent command we actually sent, plus current state.
        if _last_command_str is not None:
            cv2.putText(frame, f"> {_last_command_str}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"state: {_bridge.state}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        if _bridge.target_color:
            cv2.putText(frame, f"target: {_bridge.target_color}", (10, 78),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        # BLE link indicator — top-right. ble_live was already computed
        # above (we need it for the cmd-gate before bridge.next_command)
        # so just render it here.
        h_img_top, w_img_top = frame.shape[:2]
        dot_color = (0, 200, 0) if ble_live else (0, 0, 220)
        ble_text  = "BLE LIVE" if ble_live else "BLE DOWN"
        cv2.circle(frame, (w_img_top - 130, 30), 10, dot_color, -1)
        cv2.circle(frame, (w_img_top - 130, 30), 10, (255, 255, 255), 2)
        cv2.putText(frame, ble_text, (w_img_top - 110, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, dot_color, 2)
        # Operator hints — only when relevant so they don't clutter the
        # display once the flow is running.
        if _bridge.state == "AWAITING_START":
            cv2.putText(frame, "press SPACE to START", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "press R to RESET (re-align)", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    # Live-tune readout — current values + key hints. Bottom-left so it
    # doesn't fight with state/target labels at the top-left.
    tune_lines = [
        f"FWD={state['tun_gripper_forward_cm']:+.1f}cm  RT={state['tun_gripper_right_cm']:+.1f}cm",
        f"PARALLAX={state['tun_parallax_factor']:.3f}  NADIR=({state['tun_camera_nadir_cm'][0]:.0f},{state['tun_camera_nadir_cm'][1]:.0f})",
        "W/S=fwd  A/D=right  -/+=parallax  N=set nadir  K=save",
    ]
    h_img = frame.shape[0]
    for i, line in enumerate(tune_lines):
        cv2.putText(frame, line, (10, h_img - 20 - (len(tune_lines) - 1 - i) * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
    if state.get("pending_nadir_click_world") == "armed":
        cv2.putText(frame, "CLICK to set NADIR", (10, h_img - 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow("Sortify - Real-time Detection", frame)

    # --- session recording (smooth playback) ---
    # Last time we tried this we hard-coded 20 fps and the vision loop
    # only managed 3-5 fps, so every frame got duplicated 4-7× and the
    # video played back as a slide show. Fix: tag the writer with the
    # ACTUAL running FPS (from the EMA we already maintain for the
    # HUD) so 1 vision tick = 1 video frame. Playback speed will match
    # real time. We downscale to 1280 wide to keep the file size sane
    # — full 4K is overkill for a demo recording.
    REC_TARGET_W = 1280
    writer = state.get("video_writer")
    if writer is None and not state.get("video_skip"):
        # Wait until we have a reasonable FPS reading before opening
        # the writer — otherwise we'd seed it with the bootstrap 0.0.
        fps_now = float(state.get("fps") or 0.0)
        if fps_now >= 2.0:
            import os, time as _t
            h_full, w_full = frame.shape[:2]
            scale = min(1.0, REC_TARGET_W / w_full)
            w_v = int(w_full * scale)
            h_v = int(h_full * scale)
            os.makedirs("recordings", exist_ok=True)
            out_path = f"recordings/session_{int(_t.time())}.mp4"
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(out_path, fourcc, fps_now, (w_v, h_v))
            if writer.isOpened():
                state["video_writer"] = writer
                state["video_path"] = out_path
                state["video_scale"] = scale
                state["video_size"] = (w_v, h_v)
                print(f"[WP4] Recording to {out_path} at {fps_now:.1f} fps, "
                      f"{w_v}x{h_v}")
            else:
                print("[WP4] WARN: VideoWriter wouldn't open; recording disabled.")
                state["video_skip"] = True
                writer = None
    if writer is not None:
        scale = state.get("video_scale", 1.0)
        if scale != 1.0:
            w_v, h_v = state["video_size"]
            rec_frame = cv2.resize(frame, (w_v, h_v), interpolation=cv2.INTER_AREA)
        else:
            rec_frame = frame
        writer.write(rec_frame)

    # --- dashboard JPEG hand-off ---
    # Encode at q=60 only if a browser is actually subscribed; the encode
    # is the expensive bit (~5-15 ms on a 4K frame) so we don't pay for
    # it when nobody's watching. We downscale to 1280-wide first — the
    # dashboard is showing a thumbnail in a browser, full 4K is overkill
    # and bumps frame-rate noticeably.
    if state.get("dashboard_subscribers", 0) > 0:
        dash_h, dash_w = frame.shape[:2]
        DASH_TARGET_W = 1280
        if dash_w > DASH_TARGET_W:
            dash_scale = DASH_TARGET_W / dash_w
            dash_frame = cv2.resize(
                frame, (DASH_TARGET_W, int(dash_h * dash_scale)),
                interpolation=cv2.INTER_AREA,
            )
        else:
            dash_frame = frame
        ok, jpg = cv2.imencode(".jpg", dash_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if ok:
            lock = state.get("frame_lock")
            payload = jpg.tobytes()
            now = time.monotonic()
            if lock is not None:
                with lock:
                    state["latest_jpeg"] = payload
                    state["latest_jpeg_ts"] = now
            else:
                state["latest_jpeg"] = payload
                state["latest_jpeg_ts"] = now

    # FPS EMA for the dashboard. Cheap; one float, no allocations.
    now = time.monotonic()
    last_t = state.get("_fps_last_t")
    if last_t is not None:
        dt = now - last_t
        if dt > 1e-3:
            inst = 1.0 / dt
            prev = state.get("fps")
            state["fps"] = inst if prev is None else 0.85 * prev + 0.15 * inst
    state["_fps_last_t"] = now


if __name__ == "__main__":
    run_detection(SOURCE, DETECTION_MODEL_PATH)
