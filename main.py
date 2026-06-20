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
GRIPPER_FORWARD_CM = 20.0   # marker centre → between the jaws
GRIPPER_RIGHT_CM   = 0.0    # chassis is symmetric; any side-drift is parallax

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
MARKER_EMA_ALPHA = 0.25

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
COMMAND_INTERVAL_SEC = 2.0

# Motor trims pushed at startup. The chassis is asymmetric (left motor
# pulls harder) so the right side runs higher to keep it tracking straight.
# Override live from the OpenCV window with the trim keys if needed.
STARTUP_TRIM_RIGHT = 75.00
STARTUP_TRIM_LEFT  = 25.00


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
                # Push our calibrated trims as soon as the link is up so we
                # don't drive with whatever the firmware happened to boot
                # with. Order: STOP first to make sure the motors are idle
                # in case the firmware was mid-pulse from a previous run.
                await r.send("STOP")
                await r.send(f"TRIM R {STARTUP_TRIM_RIGHT:.2f}")
                await r.send(f"TRIM L {STARTUP_TRIM_LEFT:.2f}")
                print(f"[WP4] Robot connected (BT05). Trims set R={STARTUP_TRIM_RIGHT:.2f} L={STARTUP_TRIM_LEFT:.2f}.")
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


def box_center(box):
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    return ((x1 + x2) // 2, (y1 + y2) // 2)

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def draw_arrow(frame, pt1, pt2, color, thickness=2):
    cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.04)

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
    model = YOLO(detection_model_path)
    trail = collections.deque(maxlen=ROBOT_TRAIL_LEN)
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
    }

    try:
        cam_index = int(source)
        use_camera = True
    except ValueError:
        use_camera = False

    # Quit on q / Q / ESC. Use a longer waitKey so the OpenCV window has a
    # fair chance to process keypresses; with waitKey(1) it often misses.
    #
    # Operator keys (the OpenCV window has to have focus):
    #   SPACE or s/S  -> START the flow (leave AWAITING_START)
    #   r / R         -> RESET back to AWAITING_START so we can re-align
    #                    the robot by hand without restarting the script
    #   q / Q / ESC   -> quit
    def _handle_keys() -> bool:
        """Process one keypress. Returns True iff we should quit."""
        k = cv2.waitKey(20) & 0xFF
        if k == 255:                            # no key
            return False
        if k in (ord("q"), ord("Q"), 27):
            return True
        if k in (ord(" "), ord("s"), ord("S")):
            _bridge.start()
            print(f"[WP4] START -> {_bridge.state}")
        elif k in (ord("r"), ord("R")):
            _bridge.reset()
            print("[WP4] RESET -> AWAITING_START. Re-align the robot, then press SPACE.")
        return False

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
    results = model(frame, verbose=False)[0]

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
        nadir = np.asarray(CAMERA_NADIR_CM, dtype=float)
        raw_center = nadir + (1.0 - PARALLAX_FACTOR) * (np.asarray(raw_center, dtype=float) - nadir)
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
        gripper_xy_cm = (
            float(robot_center_cm[0]) + GRIPPER_FORWARD_CM * math.cos(heading)
                                      + GRIPPER_RIGHT_CM   * math.cos(right),
            float(robot_center_cm[1]) + GRIPPER_FORWARD_CM * math.sin(heading)
                                      + GRIPPER_RIGHT_CM   * math.sin(right),
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
    BRIDGE_CONF_MIN = 0.55

    for box in results.boxes:
        cls = int(box.cls[0])
        name = results.names[cls]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        conf = float(box.conf[0])

        parts = name.lower().split()
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
                    break

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
            "path_to_block": [_bxy(p) for p in path_to_block],
            "path_to_field": [_bxy(p) for p in path_to_field],
        }
        cmd = _bridge.next_command(vision)
        global _last_command_at, _last_command_str
        now = time.monotonic()

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
            _last_command_at = now
            _last_command_str = cmd

        # HUD: show the most recent command we actually sent, plus current state.
        if _last_command_str is not None:
            cv2.putText(frame, f"> {_last_command_str}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"state: {_bridge.state}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        if _bridge.target_color:
            cv2.putText(frame, f"target: {_bridge.target_color}", (10, 78),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        # Operator hints — only when relevant so they don't clutter the
        # display once the flow is running.
        if _bridge.state == "AWAITING_START":
            cv2.putText(frame, "press SPACE to START", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "press R to RESET (re-align)", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    cv2.imshow("Sortify - Real-time Detection", frame)


if __name__ == "__main__":
    run_detection(SOURCE, DETECTION_MODEL_PATH)
