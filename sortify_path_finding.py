"""
Sortify path finding + robot command template
--------------------------------------------
Pipeline:
1) Read overhead camera frame
2) Detect colored blocks/fields with YOLO
3) Detect robot center + orientation with ArUco ID 0
4) Select nearest block whose same-colored field exists
5) Plan A* path in image pixel coordinates
6) Send simple movement/gripper commands to robot over serial/Bluetooth

Install:
    pip install ultralytics opencv-contrib-python pyserial numpy

Run:
    python sortify_path_finding.py

Notes:
- This first version works in pixel coordinates. For a centered, stable overhead phone camera,
  this is usually enough for a demo.
- If perspective distortion is high, add homography calibration later.
- Firmware should understand the command characters defined in CMD_* below.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    import serial
except ImportError:
    serial = None


# =========================
# USER CONFIGURATION
# =========================

# Camera source:
# - 0 / 1 / 2 for webcam devices
# - DroidCam/Iriun may appear as a camera index
# - Or use a stream URL if your phone app provides one
CAMERA_SOURCE = 0

# YOLO model trained with these exact/similar class names:
#   blue block, blue field, green block, green field, red block, red field
MODEL_PATH = "best.pt"
YOLO_CONF = 0.45

# Bluetooth/serial port.
# Windows example: "COM5"
# macOS example: "/dev/tty.HC-05-DevB" or "/dev/tty.usbserial-xxxx"
# Linux example: "/dev/rfcomm0" or "/dev/ttyUSB0"
# Set to None for debug mode without sending real commands.
SERIAL_PORT = None
BAUD_RATE = 9600

# ArUco config from your setup
ARUCO_DICT = cv2.aruco.DICT_4X4_50
ROBOT_MARKER_ID = 0

# Movement/path parameters in pixels.
# Tune these based on your camera height and robot size.
GRID_SIZE_PX = 20          # smaller = more precise but slower
ROBOT_RADIUS_PX = 45       # used to inflate obstacles
GRIPPER_OFFSET_PX = 55     # distance from ArUco center to gripper/front
TARGET_REACHED_PX = 35     # when gripper/front is this close, target is reached
WAYPOINT_REACHED_PX = 25
LOOKAHEAD_PX = 70
ANGLE_TOL_DEG = 14
COMMAND_PERIOD_SEC = 0.12  # avoid spamming Bluetooth too fast

# If your robot drives opposite direction relative to marker top side, change this.
# The code assumes the marker's printed TOP side points toward the robot's FRONT/gripper.
MARKER_FRONT_IS_TOP_SIDE = True

# Optional manual workspace polygon in pixel coordinates: [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
# Use this if you want to restrict planning inside a table/field area.
# Leave None to use the whole camera frame.
WORKSPACE_POLYGON: Optional[List[Tuple[int, int]]] = None


# Commands sent to microcontroller firmware.
# Keep these very simple at first.
CMD_FORWARD = "F"
CMD_LEFT = "L"
CMD_RIGHT = "R"
CMD_STOP = "S"
CMD_GRAB = "G"
CMD_DROP = "D"


# =========================
# DATA STRUCTURES
# =========================

@dataclass
class Detection:
    cls_name: str
    color: str
    kind: str       # "block" or "field"
    conf: float
    xyxy: Tuple[float, float, float, float]
    center: Tuple[float, float]


@dataclass
class RobotPose:
    x: float
    y: float
    theta: float    # radians, image coordinate system: atan2(dy, dx)

    @property
    def center(self) -> Tuple[float, float]:
        return self.x, self.y

    @property
    def front_point(self) -> Tuple[float, float]:
        return (
            self.x + GRIPPER_OFFSET_PX * math.cos(self.theta),
            self.y + GRIPPER_OFFSET_PX * math.sin(self.theta),
        )


class SortState(Enum):
    SELECT_TARGET = auto()
    GO_TO_BLOCK = auto()
    GRAB = auto()
    GO_TO_FIELD = auto()
    DROP = auto()
    DONE = auto()


# =========================
# UTILITY FUNCTIONS
# =========================

def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def parse_class_name(name: str) -> Optional[Tuple[str, str]]:
    """
    Converts YOLO class names like 'blue block' into ('blue', 'block').
    Also accepts names like 'blue_block'.
    """
    clean = name.lower().replace("_", " ").strip()
    parts = clean.split()
    if len(parts) < 2:
        return None

    color = parts[0]
    kind = parts[-1]
    if color not in {"blue", "green", "red"}:
        return None
    if kind not in {"block", "field"}:
        return None
    return color, kind


# =========================
# YOLO DETECTION
# =========================

class YoloDetector:
    def __init__(self, model_path: str):
        if YOLO is None:
            raise ImportError("ultralytics is not installed. Run: pip install ultralytics")
        self.model = YOLO(model_path)

    def detect(self, frame: np.ndarray) -> List[Detection]:
        result = self.model(frame, conf=YOLO_CONF, verbose=False)[0]
        detections: List[Detection] = []

        if result.boxes is None:
            return detections

        names = result.names
        for box in result.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            cls_name = str(names[cls_id])
            parsed = parse_class_name(cls_name)
            if parsed is None:
                continue

            color, kind = parsed
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(float).tolist()
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            detections.append(
                Detection(
                    cls_name=cls_name,
                    color=color,
                    kind=kind,
                    conf=conf,
                    xyxy=(x1, y1, x2, y2),
                    center=(cx, cy),
                )
            )
        return detections


# =========================
# ARUCO ROBOT LOCALIZATION
# =========================

def detect_robot_pose(frame: np.ndarray) -> Optional[RobotPose]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

    # OpenCV version compatibility
    if hasattr(cv2.aruco, "ArucoDetector"):
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is None:
        return None

    ids = ids.flatten()
    for marker_corners, marker_id in zip(corners, ids):
        if int(marker_id) != ROBOT_MARKER_ID:
            continue

        pts = marker_corners.reshape(4, 2).astype(float)
        center = pts.mean(axis=0)

        # ArUco corner order: top-left, top-right, bottom-right, bottom-left
        top_mid = (pts[0] + pts[1]) / 2.0
        bottom_mid = (pts[2] + pts[3]) / 2.0

        if MARKER_FRONT_IS_TOP_SIDE:
            front_vec = top_mid - center
        else:
            front_vec = center - top_mid
            # Alternative if you mounted marker opposite: use bottom_mid - center
            # front_vec = bottom_mid - center

        theta = math.atan2(float(front_vec[1]), float(front_vec[0]))
        return RobotPose(float(center[0]), float(center[1]), theta)

    return None


# =========================
# A* PATH PLANNER
# =========================

GridCell = Tuple[int, int]
Point = Tuple[float, float]


def point_to_cell(p: Point) -> GridCell:
    return int(p[0] // GRID_SIZE_PX), int(p[1] // GRID_SIZE_PX)


def cell_to_point(c: GridCell) -> Point:
    return (c[0] + 0.5) * GRID_SIZE_PX, (c[1] + 0.5) * GRID_SIZE_PX


def build_occupancy_grid(
    frame_shape: Tuple[int, int, int],
    detections: List[Detection],
    target_block: Optional[Detection],
) -> np.ndarray:
    """
    Returns grid[y, x]. 0 = free, 1 = occupied.

    For first demo:
    - Boundaries are obstacles.
    - Other blocks are obstacles because the robot should not hit them.
    - Target block is NOT an obstacle because the robot must approach it.
    - Fields are not obstacles; they are zones on the floor.
    """
    h, w = frame_shape[:2]
    gw = int(math.ceil(w / GRID_SIZE_PX))
    gh = int(math.ceil(h / GRID_SIZE_PX))
    grid = np.zeros((gh, gw), dtype=np.uint8)

    # Optional workspace mask: cells outside polygon become obstacles.
    if WORKSPACE_POLYGON is not None:
        poly = np.array(WORKSPACE_POLYGON, dtype=np.int32)
        for gy in range(gh):
            for gx in range(gw):
                px, py = cell_to_point((gx, gy))
                inside = cv2.pointPolygonTest(poly, (float(px), float(py)), False) >= 0
                if not inside:
                    grid[gy, gx] = 1

    # Keep a small border as obstacle.
    border = max(1, int(ROBOT_RADIUS_PX / GRID_SIZE_PX))
    grid[:border, :] = 1
    grid[-border:, :] = 1
    grid[:, :border] = 1
    grid[:, -border:] = 1

    # Inflate non-target blocks.
    for det in detections:
        if det.kind != "block":
            continue
        if target_block is not None and det is target_block:
            continue

        x1, y1, x2, y2 = det.xyxy
        x1 -= ROBOT_RADIUS_PX
        y1 -= ROBOT_RADIUS_PX
        x2 += ROBOT_RADIUS_PX
        y2 += ROBOT_RADIUS_PX

        gx1 = max(0, int(x1 // GRID_SIZE_PX))
        gy1 = max(0, int(y1 // GRID_SIZE_PX))
        gx2 = min(gw - 1, int(x2 // GRID_SIZE_PX))
        gy2 = min(gh - 1, int(y2 // GRID_SIZE_PX))
        grid[gy1 : gy2 + 1, gx1 : gx2 + 1] = 1

    return grid


def neighbors(cell: GridCell, grid: np.ndarray) -> Iterable[Tuple[GridCell, float]]:
    x, y = cell
    h, w = grid.shape
    for dx, dy, cost in [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (1, 1, 1.414),
    ]:
        nx, ny = x + dx, y + dy
        if nx < 0 or ny < 0 or nx >= w or ny >= h:
            continue
        if grid[ny, nx] == 1:
            continue
        yield (nx, ny), cost


def heuristic(a: GridCell, b: GridCell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest_free_cell(cell: GridCell, grid: np.ndarray, max_radius: int = 8) -> Optional[GridCell]:
    x0, y0 = cell
    h, w = grid.shape
    if 0 <= x0 < w and 0 <= y0 < h and grid[y0, x0] == 0:
        return cell

    for r in range(1, max_radius + 1):
        candidates = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if abs(dx) != r and abs(dy) != r:
                    continue
                x, y = x0 + dx, y0 + dy
                if 0 <= x < w and 0 <= y < h and grid[y, x] == 0:
                    candidates.append((x, y))
        if candidates:
            return min(candidates, key=lambda c: heuristic(c, cell))
    return None


def astar(grid: np.ndarray, start_pt: Point, goal_pt: Point) -> List[Point]:
    start = nearest_free_cell(point_to_cell(start_pt), grid)
    goal = nearest_free_cell(point_to_cell(goal_pt), grid)
    if start is None or goal is None:
        return []

    open_heap: List[Tuple[float, GridCell]] = []
    heapq.heappush(open_heap, (0.0, start))
    came_from: Dict[GridCell, Optional[GridCell]] = {start: None}
    g_score: Dict[GridCell, float] = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            cells: List[GridCell] = []
            while current is not None:
                cells.append(current)
                current = came_from[current]
            cells.reverse()
            return simplify_path([cell_to_point(c) for c in cells], grid)

        for nxt, move_cost in neighbors(current, grid):
            new_cost = g_score[current] + move_cost
            if nxt not in g_score or new_cost < g_score[nxt]:
                g_score[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal)
                heapq.heappush(open_heap, (priority, nxt))
                came_from[nxt] = current

    return []


def line_is_free(a: Point, b: Point, grid: np.ndarray) -> bool:
    """Check line segment in grid using sampled points."""
    length = dist(a, b)
    steps = max(2, int(length / (GRID_SIZE_PX * 0.5)))
    h, w = grid.shape
    for i in range(steps + 1):
        t = i / steps
        x = a[0] * (1 - t) + b[0] * t
        y = a[1] * (1 - t) + b[1] * t
        gx, gy = point_to_cell((x, y))
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False
        if grid[gy, gx] == 1:
            return False
    return True


def simplify_path(path: List[Point], grid: np.ndarray) -> List[Point]:
    """Remove unnecessary intermediate waypoints when straight line is free."""
    if len(path) <= 2:
        return path

    simplified = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_is_free(path[i], path[j], grid):
                break
            j -= 1
        simplified.append(path[j])
        i = j
    return simplified


# =========================
# TARGET SELECTION
# =========================

def group_detections(detections: List[Detection]) -> Tuple[List[Detection], Dict[str, Detection]]:
    blocks = [d for d in detections if d.kind == "block"]

    # If multiple fields of same color are detected, keep the highest confidence one.
    fields: Dict[str, Detection] = {}
    for d in detections:
        if d.kind != "field":
            continue
        if d.color not in fields or d.conf > fields[d.color].conf:
            fields[d.color] = d
    return blocks, fields


def choose_nearest_sortable_block(
    robot: RobotPose,
    blocks: List[Detection],
    fields: Dict[str, Detection],
) -> Optional[Detection]:
    sortable = [b for b in blocks if b.color in fields]
    if not sortable:
        return None
    return min(sortable, key=lambda b: dist(robot.center, b.center))


def get_lookahead_waypoint(robot: RobotPose, path: List[Point]) -> Optional[Point]:
    if not path:
        return None
    for p in path:
        if dist(robot.center, p) >= LOOKAHEAD_PX:
            return p
    return path[-1]


# =========================
# COMMAND SENDER
# =========================

class RobotCommander:
    def __init__(self, port: Optional[str], baud_rate: int):
        self.last_cmd = None
        self.last_time = 0.0
        self.ser = None

        if port is not None:
            if serial is None:
                raise ImportError("pyserial is not installed. Run: pip install pyserial")
            self.ser = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2.0)

    def send(self, cmd: str, force: bool = False) -> None:
        now = time.time()
        if not force and cmd == self.last_cmd and now - self.last_time < COMMAND_PERIOD_SEC:
            return
        if not force and now - self.last_time < COMMAND_PERIOD_SEC:
            return

        msg = (cmd + "\n").encode("utf-8")
        if self.ser is not None:
            self.ser.write(msg)
        else:
            # Debug mode: print only changes or periodic commands.
            if cmd != self.last_cmd or force:
                print(f"SEND: {cmd}")

        self.last_cmd = cmd
        self.last_time = now

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()


# =========================
# CONTROL LOGIC
# =========================

def command_to_follow(robot: RobotPose, waypoint: Point) -> str:
    dx = waypoint[0] - robot.x
    dy = waypoint[1] - robot.y
    target_angle = math.atan2(dy, dx)
    angle_error = normalize_angle(target_angle - robot.theta)

    if abs(math.degrees(angle_error)) > ANGLE_TOL_DEG:
        return CMD_LEFT if angle_error < 0 else CMD_RIGHT
    return CMD_FORWARD


def draw_debug(
    frame: np.ndarray,
    detections: List[Detection],
    robot: Optional[RobotPose],
    path: List[Point],
    state: SortState,
    target: Optional[Detection],
    carrying_color: Optional[str],
) -> np.ndarray:
    vis = frame.copy()

    # Draw detections.
    for d in detections:
        x1, y1, x2, y2 = map(int, d.xyxy)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 255, 255), 2)
        cv2.circle(vis, (int(d.center[0]), int(d.center[1])), 4, (255, 255, 255), -1)
        cv2.putText(vis, f"{d.color} {d.kind} {d.conf:.2f}", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # Draw current target.
    if target is not None:
        cv2.circle(vis, (int(target.center[0]), int(target.center[1])), 12, (0, 255, 255), 3)

    # Draw robot.
    if robot is not None:
        cx, cy = int(robot.x), int(robot.y)
        fx, fy = robot.front_point
        cv2.circle(vis, (cx, cy), 7, (0, 0, 255), -1)
        cv2.arrowedLine(vis, (cx, cy), (int(fx), int(fy)), (0, 0, 255), 3)
        cv2.circle(vis, (int(fx), int(fy)), 5, (0, 255, 255), -1)

    # Draw path.
    if len(path) >= 2:
        pts = [(int(x), int(y)) for x, y in path]
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(vis, a, b, (0, 255, 0), 2)
        for p in pts:
            cv2.circle(vis, p, 4, (0, 255, 0), -1)

    cv2.putText(vis, f"STATE: {state.name} | carrying: {carrying_color}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return vis


# =========================
# MAIN LOOP
# =========================

def main() -> None:
    detector = YoloDetector(MODEL_PATH)
    commander = RobotCommander(SERIAL_PORT, BAUD_RATE)
    cap = cv2.VideoCapture(CAMERA_SOURCE)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera source: {CAMERA_SOURCE}")

    state = SortState.SELECT_TARGET
    target_block: Optional[Detection] = None
    carrying_color: Optional[str] = None
    action_start_time = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame could not be read.")
                commander.send(CMD_STOP)
                break

            detections = detector.detect(frame)
            blocks, fields = group_detections(detections)
            robot = detect_robot_pose(frame)
            path: List[Point] = []

            if robot is None:
                commander.send(CMD_STOP)
                debug = draw_debug(frame, detections, None, path, state, target_block, carrying_color)
                cv2.putText(debug, "NO ARUCO ROBOT MARKER", (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("Sortify Path Finding", debug)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            if state == SortState.SELECT_TARGET:
                target_block = choose_nearest_sortable_block(robot, blocks, fields)
                carrying_color = None
                if target_block is None:
                    state = SortState.DONE
                    commander.send(CMD_STOP)
                else:
                    state = SortState.GO_TO_BLOCK

            elif state == SortState.GO_TO_BLOCK:
                if target_block is None:
                    state = SortState.SELECT_TARGET
                else:
                    grid = build_occupancy_grid(frame.shape, detections, target_block)
                    path = astar(grid, robot.center, target_block.center)

                    # Use the gripper/front point to decide pickup distance.
                    if dist(robot.front_point, target_block.center) <= TARGET_REACHED_PX:
                        commander.send(CMD_STOP, force=True)
                        state = SortState.GRAB
                        action_start_time = time.time()
                    else:
                        waypoint = get_lookahead_waypoint(robot, path)
                        if waypoint is None:
                            commander.send(CMD_STOP)
                        else:
                            commander.send(command_to_follow(robot, waypoint))

            elif state == SortState.GRAB:
                commander.send(CMD_GRAB, force=True)
                if time.time() - action_start_time > 1.0:
                    carrying_color = target_block.color if target_block is not None else None
                    state = SortState.GO_TO_FIELD

            elif state == SortState.GO_TO_FIELD:
                if carrying_color is None or carrying_color not in fields:
                    commander.send(CMD_STOP)
                    state = SortState.SELECT_TARGET
                else:
                    target_field = fields[carrying_color]
                    grid = build_occupancy_grid(frame.shape, detections, target_block=None)
                    path = astar(grid, robot.center, target_field.center)

                    if dist(robot.front_point, target_field.center) <= TARGET_REACHED_PX:
                        commander.send(CMD_STOP, force=True)
                        state = SortState.DROP
                        action_start_time = time.time()
                    else:
                        waypoint = get_lookahead_waypoint(robot, path)
                        if waypoint is None:
                            commander.send(CMD_STOP)
                        else:
                            commander.send(command_to_follow(robot, waypoint))

            elif state == SortState.DROP:
                commander.send(CMD_DROP, force=True)
                if time.time() - action_start_time > 1.0:
                    target_block = None
                    carrying_color = None
                    state = SortState.SELECT_TARGET

            elif state == SortState.DONE:
                commander.send(CMD_STOP)
                # If new blocks appear, continue again.
                if blocks:
                    state = SortState.SELECT_TARGET

            current_target = target_block
            if state == SortState.GO_TO_FIELD and carrying_color in fields:
                current_target = fields[carrying_color]

            debug = draw_debug(frame, detections, robot, path, state, current_target, carrying_color)
            cv2.imshow("Sortify Path Finding", debug)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                commander.send(CMD_STOP, force=True)

    finally:
        commander.send(CMD_STOP, force=True)
        commander.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
