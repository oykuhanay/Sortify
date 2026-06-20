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

# Gripper tip = robot_center + (forward * theta) + (right * theta+90).
# The gripper isn't dead-centre on the marker on this chassis — it sits a
# bit to one side. Both offsets are in marker-side units so they scale with
# camera distance. Tune by watching the cyan ring against the real jaws in
# the live overlay.
#   forward >0 = ahead of the marker, in the direction of theta
#   right   >0 = to the marker's right (clockwise from theta)
GRIPPER_FORWARD_MARKER_SIDES = 1.25
GRIPPER_RIGHT_MARKER_SIDES   = 0.20

# Exponential moving average factor for the ArUco marker readings. Without
# this the marker centre + theta + side jitter by a few px / a few degrees
# each frame, which made the cyan gripper ring visibly twitch and made the
# bridge keep emitting little corrective TURNs. A=0.25 means each new
# sample contributes 25% and the EMA forgets the past with a half-life of
# ~2.4 frames — enough to kill jitter, fast enough that the robot
# actually moving is reflected immediately.
MARKER_EMA_ALPHA = 0.25

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
STARTUP_TRIM_RIGHT = 70.00
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

def run_detection(source, detection_model_path):
    _start_robot_thread()
    model = YOLO(detection_model_path)
    trail = collections.deque(maxlen=ROBOT_TRAIL_LEN)
    state = {
        "robot_radius": None,
        "last_robot_center": None,
        "last_marker_side_px": None,
        "last_theta_deg": None,
        "frames_since_marker": 0,
        # EMA-smoothed marker pose, updated each frame the marker is seen.
        # Theta is stored as a unit-vector (cos, sin) so we can average
        # across the ±180° wrap without going through 0 incorrectly.
        "ema_robot_center": None,         # np.array([x, y])
        "ema_marker_side_px": None,       # float
        "ema_theta_cs": None,             # np.array([cos(theta), sin(theta)])
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
        center = pts.mean(axis=0)
        top_mid = (pts[0] + pts[1]) / 2.0
        front_vec = top_mid - center
        theta = math.degrees(math.atan2(float(front_vec[1]), float(front_vec[0])))
        side_len = float(np.mean([
            np.linalg.norm(pts[1] - pts[0]),
            np.linalg.norm(pts[2] - pts[1]),
            np.linalg.norm(pts[3] - pts[2]),
            np.linalg.norm(pts[0] - pts[3]),
        ]))
        return center, theta, side_len
    return None


def detect_marker(frame):
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

    # --- detect robot marker (gives center + heading) ---
    # Raw ArUco readings jitter by a few px / a few degrees frame-to-frame
    # even when the robot is sitting still. We pipe everything through an
    # EMA so the cyan gripper ring stops twitching and the bridge stops
    # second-guessing its own commands. EMA values are what we expose to
    # downstream code (gripper geometry, A*, bridge); the raw reading is
    # used only to update the filter.
    theta = None
    marker_side_px = None
    robot_center = None
    marker = detect_marker(frame)

    if marker is not None:
        raw_center, raw_theta, raw_side = marker
        raw_center_arr = np.asarray(raw_center, dtype=float)
        raw_theta_cs = np.array(
            [math.cos(math.radians(raw_theta)), math.sin(math.radians(raw_theta))]
        )
        # First sighting: seed the EMA directly. Otherwise blend.
        a = MARKER_EMA_ALPHA
        if state["ema_robot_center"] is None:
            state["ema_robot_center"] = raw_center_arr
            state["ema_marker_side_px"] = raw_side
            state["ema_theta_cs"] = raw_theta_cs
        else:
            state["ema_robot_center"] = (
                a * raw_center_arr + (1 - a) * state["ema_robot_center"]
            )
            state["ema_marker_side_px"] = (
                a * raw_side + (1 - a) * state["ema_marker_side_px"]
            )
            blended = a * raw_theta_cs + (1 - a) * state["ema_theta_cs"]
            # Renormalise so it stays a unit vector — otherwise the
            # implied angle drifts with magnitude.
            norm = float(np.linalg.norm(blended))
            if norm > 1e-6:
                state["ema_theta_cs"] = blended / norm

        robot_center = state["ema_robot_center"]
        marker_side_px = float(state["ema_marker_side_px"])
        theta = math.degrees(
            math.atan2(float(state["ema_theta_cs"][1]), float(state["ema_theta_cs"][0]))
        )
        state["robot_radius"] = marker_side_px * ROBOT_CLEARANCE_FACTOR
        state["last_robot_center"] = robot_center.copy()
        state["last_marker_side_px"] = marker_side_px
        state["last_theta_deg"] = theta
        state["frames_since_marker"] = 0
        trail.append(robot_center.copy())
    elif state["frames_since_marker"] < 10:
        # Use last known position for up to 10 missed frames, then drop it
        robot_center = state["last_robot_center"]
        marker_side_px = state.get("last_marker_side_px")
        theta = state.get("last_theta_deg")
        state["frames_since_marker"] += 1

    # --- compute the gripper tip position geometrically ---
    # The keypoint model wasn't reliable in early tests (it kept landing
    # inside the marker), so we derive the gripper from marker geometry:
    # start at the robot center, walk forward by GRIPPER_FORWARD_MARKER_SIDES
    # of marker_side_px along theta, then nudge sideways by
    # GRIPPER_RIGHT_MARKER_SIDES along (theta + 90°). Both offsets scale
    # with the marker's apparent size so this is robust to camera distance.
    gripper_xy = None
    if robot_center is not None and theta is not None and marker_side_px is not None:
        heading = math.radians(theta)
        right   = heading + math.pi / 2.0
        fwd_px = marker_side_px * GRIPPER_FORWARD_MARKER_SIDES
        rt_px  = marker_side_px * GRIPPER_RIGHT_MARKER_SIDES
        gripper_xy = (
            float(robot_center[0]) + fwd_px * math.cos(heading) + rt_px * math.cos(right),
            float(robot_center[1]) + fwd_px * math.sin(heading) + rt_px * math.sin(right),
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
    target_det = None
    field_det = None
    best_key = (float("inf"), float("inf"))

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
            d = dist(det.center, robot_center) if robot_center is not None else 0
            key = (priority, d)
            if key < best_key:
                best_key = key
                target_det = det
                field_det = f

    # --- A* paths: robot → block, then block → field ---
    path_to_block = []
    path_to_field = []

    if target_det is not None and robot_center is not None:
        rc = tuple(map(float, robot_center))
        robot_radius = state["robot_radius"]
        scales = [1.0, 0.5, 0.25] if robot_radius is not None else [None]

        for scale in scales:
            r = robot_radius * scale if (robot_radius is not None and scale is not None) else None
            kwargs = {} if r is None else {"robot_radius": r}
            grid1 = build_occupancy_grid(frame.shape, all_detections, target_det, **kwargs)
            path_to_block = astar(grid1, rc, target_det.center)
            if path_to_block:
                break

        if field_det is not None:
            for scale in scales:
                r = robot_radius * scale if (robot_radius is not None and scale is not None) else None
                kwargs = {} if r is None else {"robot_radius": r}
                grid2 = build_occupancy_grid(frame.shape, all_detections, target_block=target_det, **kwargs)
                path_to_field = astar(grid2, target_det.center, field_det.center)
                if path_to_field:
                    break

    # --- draw path overlay ---
    draw_path_overlay(frame, robot_center, path_to_block, path_to_field, trail,
                      robot_radius=state["robot_radius"])

    # Render the gripper-tip estimate. Colour reflects whether the bridge
    # currently believes the jaws are open (cyan) or closed (magenta) so we
    # can tell from a glance whether the cube is being held. Bump
    # GRIPPER_FORWARD/RIGHT_MARKER_SIDES if the ring doesn't sit where the real
    # gripper does.
    if gripper_xy is not None:
        gx, gy = int(gripper_xy[0]), int(gripper_xy[1])
        gripper_color = (255, 255, 0) if _bridge.gripper_open else (255, 0, 255)  # cyan vs magenta (BGR)
        gripper_label = "OPEN" if _bridge.gripper_open else "CLOSED"
        cv2.circle(frame, (gx, gy), 12, gripper_color, 3)
        cv2.circle(frame, (gx, gy), 4, (255, 255, 255), -1)
        cv2.putText(frame, f"gripper {gripper_label}", (gx + 14, gy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, gripper_color, 1)
        # Show the grab radius as a faint ring around the gripper. Any cube
        # whose centre falls inside this ring will trigger STOP + GRIP C.
        # If the cube is plainly inside but the bridge isn't latching, the
        # radius is too small — bump BLOCK_GRAB_RADIUS_PX in bridge.py.
        from bridge import BLOCK_GRAB_RADIUS_PX as _GRAB_R
        cv2.circle(frame, (gx, gy), int(_GRAB_R), gripper_color, 1, lineType=cv2.LINE_AA)
        # Draw a thin line from gripper to the currently-targeted block, so
        # we can eyeball whether the GRAB radius is being met.
        if target_det is not None:
            tx, ty = int(target_det.center[0]), int(target_det.center[1])
            cv2.line(frame, (gx, gy), (tx, ty), gripper_color, 1)
            mid = ((gx + tx) // 2, (gy + ty) // 2)
            d_px = int(math.hypot(tx - gx, ty - gy))
            inside = d_px <= _GRAB_R
            label = f"{d_px}px"
            if inside:
                label += " IN RANGE"
            cv2.putText(frame, label, mid,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, gripper_color, 2)

    # --- debug: heading arrow ---------------------------------------------
    # Draw the heading the bridge *thinks* the robot is facing (red), so we
    # can visually check whether the marker's theta matches reality. If this
    # arrow doesn't point in the direction the gripper points, the marker is
    # rotated wrong on the robot and we have to add an offset.
    if robot_center is not None and theta is not None:
        cx, cy = int(robot_center[0]), int(robot_center[1])
        arrow_len = 120
        rad = math.radians(theta)
        tip = (int(cx + arrow_len * math.cos(rad)),
               int(cy + arrow_len * math.sin(rad)))
        cv2.arrowedLine(frame, (cx, cy), tip, (0, 0, 255), 4, tipLength=0.25)
        cv2.putText(frame, f"theta={theta:+.0f}", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # ---- WP4 bridge: vision -> single robot command per frame ----
    if robot_center is not None and theta is not None:
        vision = {
            "robot_center": (float(robot_center[0]), float(robot_center[1])),
            "theta_deg":    float(theta),
            "gripper_pos":  tuple(map(float, gripper_xy)) if gripper_xy is not None else None,
            "blocks_by_color": {
                color: [d.center for d in dets]
                for color, dets in blocks_by_color.items()
            },
            "fields_by_color": {
                color: d.center for color, d in fields_by_color.items()
            },
            "path_to_block": path_to_block,
            "path_to_field": path_to_field,
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
