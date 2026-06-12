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

SOURCE = 0        # camera index or path to video/image
MODEL_PATH = "best.pt"

def pad_to_square(frame, size=1280):
    h, w = frame.shape[:2]
    diff = w - h
    top, bottom = diff // 2, diff - diff // 2
    padded = cv2.copyMakeBorder(frame, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    return cv2.resize(padded, (size, size))

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
                _process_frame(frame, model, trail)
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
            _process_frame(frame, model, trail)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cap.release()

    cv2.destroyAllWindows()



def detect_marker(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=cv2.aruco.DetectorParameters_create()
        )
    if ids is None:
        return None
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) == ROBOT_MARKER_ID:
            pts = marker_corners.reshape(4, 2).astype(float)
            center = pts.mean(axis=0)
            top_mid = (pts[0] + pts[1]) / 2.0
            front_vec = top_mid - center
            theta = math.degrees(math.atan2(float(front_vec[1]), float(front_vec[0])))
            return center, theta
    return None








def draw_planned_path(frame, waypoints, color, thickness=2):
    if not waypoints:
        return
    pts = [(int(x), int(y)) for x, y in waypoints]
    for a, b in zip(pts[:-1], pts[1:]):
        cv2.line(frame, a, b, color, thickness)
    if len(pts) >= 2:
        cv2.arrowedLine(frame, pts[-2], pts[-1], color, thickness, tipLength=0.06)


def draw_path_overlay(frame, robot_center, path_to_block, path_to_field, trail):
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

    # Robot dot
    if robot_center is not None:
        cv2.circle(frame, (int(robot_center[0]), int(robot_center[1])), 8, (255, 0, 200), -1)
        cv2.circle(frame, (int(robot_center[0]), int(robot_center[1])), 8, (255, 255, 255), 1)



def _process_frame(frame, model, trail):
    frame = pad_to_square(frame)
    results = model(frame, verbose=False)[0]

    # --- detect robot marker ---
    marker = detect_marker(frame)
    robot_center = None
    if marker is not None:
        robot_center, theta = marker
        trail.append(robot_center.copy())

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

    if target_det is not None:
        rc = tuple(map(float, robot_center)) if robot_center is not None else target_det.center
        grid1 = build_occupancy_grid(frame.shape, all_detections, target_det)
        path_to_block = astar(grid1, rc, target_det.center)

        if field_det is not None:
            grid2 = build_occupancy_grid(frame.shape, all_detections, target_block=None)
            path_to_field = astar(grid2, target_det.center, field_det.center)

    # --- draw path overlay ---
    draw_path_overlay(frame, robot_center, path_to_block, path_to_field, trail)

    cv2.imshow("Sortify - Real-time Detection", frame)


if __name__ == "__main__":
    run_detection(SOURCE, MODEL_PATH)
