import collections
import math
import sys
import cv2
import numpy as np
from ultralytics import YOLO
sys.path.insert(0, "camera")
from camera import Camera
from sortify_path_finding import Detection, build_occupancy_grid, astar

ARUCO_DICT = cv2.aruco.DICT_4X4_50
ROBOT_MARKER_ID = 0
ROBOT_TRAIL_LEN = 80  # how many past positions to show

# Clearance = marker_side × this factor.
# The marker ≈ robot width, so 0.6 ≈ "keep center at least 60% of robot width from obstacle edges".
ROBOT_CLEARANCE_FACTOR = 0.6

SOURCE = 0        # camera index or path to video/image
MODEL_PATH = "best.pt"


def box_center(box):
    x1, y1, x2, y2 = map(int, box.xyxy[0])
    return ((x1 + x2) // 2, (y1 + y2) // 2)

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def draw_arrow(frame, pt1, pt2, color, thickness=2):
    cv2.arrowedLine(frame, pt1, pt2, color, thickness, tipLength=0.04)

def run_detection(source, model_path):
    model = YOLO(model_path)
    trail = collections.deque(maxlen=ROBOT_TRAIL_LEN)
    state = {
        "robot_radius": None,
        "last_robot_center": None,
        "frames_since_marker": 0,
    }

    try:
        cam_index = int(source)
        use_camera = True
    except ValueError:
        use_camera = False

    if use_camera:
        with Camera(index=cam_index, width=3840, height=2160) as cam:
            print(f"Real-time detection started (camera {cam_index}). Press Q to quit.")
            while True:
                frame = cam.get_frame()
                _process_frame(frame, model, trail, state)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    else:
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            sys.exit(f"Error: could not open source '{source}'")
        print(f"Processing '{source}'. Press Q to quit.")
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            _process_frame(frame, model, trail, state)
            if cv2.waitKey(1) & 0xFF == ord("q"):
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

    # --- detect robot marker ---
    marker = detect_marker(frame)
    robot_center = None
    if marker is not None:
        robot_center, theta, marker_side_px = marker
        state["robot_radius"] = marker_side_px * ROBOT_CLEARANCE_FACTOR
        state["last_robot_center"] = robot_center.copy()
        state["frames_since_marker"] = 0
        trail.append(robot_center.copy())
    elif state["frames_since_marker"] < 10:
        # Use last known position for up to 10 missed frames, then drop it
        robot_center = state["last_robot_center"]
        state["frames_since_marker"] += 1

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

    for box in results.boxes:
        cls = int(box.cls[0])
        name = results.names[cls]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        conf = float(box.conf[0])

        parts = name.lower().split()
        if len(parts) == 2:
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

        # draw bounding box
        label = f"{name} {conf:.2f}"
        detected_color = name.lower().split()[0] if name else ""
        box_color = COLOR_MAP.get(detected_color, (180, 180, 180))
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

    cv2.imshow("Sortify - Real-time Detection", frame)


if __name__ == "__main__":
    run_detection(SOURCE, MODEL_PATH)
