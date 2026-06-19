"""
WP4 bridge: vision output -> robot command, with task state machine.

Per frame the vision team hands us a dict describing the scene:

    {
        "robot": {"xy": (cx, cy), "theta_deg": ...},   # from ArUco
        "blocks_by_color": {"red": [(x, y), ...], "blue": [...], ...},
        "fields_by_color": {"red": (x, y), "blue": (x, y), ...},
        "path_to_block":   [(x, y), ...],              # A* from main.py
        "path_to_field":   [(x, y), ...],              # A* from main.py
    }

The bridge keeps an internal state machine to drive the full pick-and-
place task. Each call returns at most one command for the robot, or None
if nothing should be sent this frame (robot still executing previous
command, or no actionable info available).

States:

    SEEKING_BLOCK   -> drive along path_to_block to reach the chosen cube
    GRABBING        -> close the gripper, briefly wait for the servo
    SEEKING_FIELD   -> drive along path_to_field to the matching colour zone
    RELEASING       -> open the gripper, briefly wait
    IDLE            -> no work to do this frame (or task complete)

State transitions happen on geometric conditions read from the vision
input — not on time, not on robot ACKs. This keeps the bridge stateless
in any deeper sense: even if we restart it mid-run, the next frame's
geometry alone determines what to do.

The one exception is GRABBING/RELEASING — those are time-windowed,
because while the gripper servo is moving we should not also be telling
the wheels to roll. We use the wall clock to gate them.

The robot is intentionally dumb: it just executes "TURN +030" / "MOVE +12.34"
/ "GRIP C" as fast as it can and pre-empts any in-flight movement when a
new command arrives. All planning lives here on the Mac.

Wire protocol (agreed with the robot firmware team):
    TURN +XXX        signed 3-digit angle in degrees (000-180). + = CW (right),
                     - = CCW (left). Examples: "TURN +030", "TURN -045".
    MOVE +XX.XX      signed 2.2 fixed-point distance in centimetres.
                     + = forward, - = backward.
                     Examples: "MOVE +12.34", "MOVE -05.00".
    GRIP C / GRIP O  close / open the gripper servo.
    STOP             emergency stop.

The MOVE units are *centimetres*, not motor-on milliseconds — the robot
firmware does the cm-to-time conversion via its own calibration constant.
We just hand it physical distance.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

Point = Tuple[float, float]


# --- tunables ---------------------------------------------------------

ANGLE_TOLERANCE_DEG = 8.0       # within this heading error, just drive
WAYPOINT_REACHED_PX = 25.0      # skip waypoints we've effectively hit
BLOCK_GRAB_RADIUS_PX = 40.0     # within this, attempt to grab the block
FIELD_RELEASE_RADIUS_PX = 60.0  # within this, attempt to release the block

# Pixels-to-centimetres scale of the overhead camera. Needs measuring:
# place an object of known width on the table, count the pixels it spans,
# divide. Until measured we assume ~10 px / cm, which is roughly right
# for a 1920-wide image of a ~50 cm table.
PX_PER_CM = 10.0

# Caps on a single command. Robot has no reverse; large steps are split
# across frames by the closed-loop bridge naturally.
MOVE_CM_MIN = 0.50              # ignore moves shorter than this — noise
MOVE_CM_MAX = 20.00             # cap one MOVE so we never outrun next frame
TURN_DEG_MAX = 90               # cap one TURN command

GRIP_SETTLE_SEC = 0.6           # let the servo finish before doing anything else

# Per-team agreement: pick blocks in this colour order.
COLOR_PRIORITY = ("red", "blue", "green")


# --- state machine ----------------------------------------------------

class State:
    SEEKING_BLOCK = "SEEKING_BLOCK"
    GRABBING = "GRABBING"
    SEEKING_FIELD = "SEEKING_FIELD"
    RELEASING = "RELEASING"
    IDLE = "IDLE"


@dataclass
class Bridge:
    state: str = State.SEEKING_BLOCK
    target_color: Optional[str] = None
    _action_started_at: float = field(default=0.0)
    _stop_sent: bool = False

    def reset(self) -> None:
        self.state = State.SEEKING_BLOCK
        self.target_color = None
        self._action_started_at = 0.0
        self._stop_sent = False

    def next_command(self, vision: dict) -> Optional[str]:
        """Decide the next command to send, or None to send nothing this frame."""
        robot = vision.get("robot")
        if not robot:
            return None
        robot_xy = robot.get("xy")
        theta = robot.get("theta_deg")
        if robot_xy is None or theta is None:
            return None

        if self.state == State.GRABBING:
            return self._tick_grabbing()
        if self.state == State.RELEASING:
            return self._tick_releasing()
        if self.state == State.SEEKING_BLOCK:
            return self._tick_seeking_block(vision, robot_xy, theta)
        if self.state == State.SEEKING_FIELD:
            return self._tick_seeking_field(vision, robot_xy, theta)
        return None  # IDLE

    # --- state handlers ----------------------------------------------

    def _tick_seeking_block(self, vision, robot_xy, theta) -> Optional[str]:
        # Pick a target colour if we don't have one yet.
        if self.target_color is None:
            self.target_color = _pick_target_color(vision)
            if self.target_color is None:
                self.state = State.IDLE
                return None

        block_xy = _nearest_block_of_color(vision, self.target_color, robot_xy)
        if block_xy is None:
            # That colour is gone or fully sorted -> try another colour.
            self.target_color = None
            return None

        # Close enough to attempt grab. Send STOP first to cancel any
        # in-flight MOVE/TURN from the streaming control loop, then on
        # the next frame transition into GRABBING and close the gripper.
        if _dist(robot_xy, block_xy) <= BLOCK_GRAB_RADIUS_PX:
            if not self._stop_sent:
                self._stop_sent = True
                return "STOP"
            self.state = State.GRABBING
            self._action_started_at = time.monotonic()
            self._stop_sent = False
            return "GRIP C"

        # Otherwise follow the planner's path to the block.
        path = vision.get("path_to_block") or [block_xy]
        return _drive_along_path(robot_xy, theta, path)

    def _tick_grabbing(self) -> Optional[str]:
        if time.monotonic() - self._action_started_at >= GRIP_SETTLE_SEC:
            self.state = State.SEEKING_FIELD
        return None  # servo is moving, hold position

    def _tick_seeking_field(self, vision, robot_xy, theta) -> Optional[str]:
        if self.target_color is None:
            # Shouldn't happen, but recover gracefully.
            self.state = State.SEEKING_BLOCK
            return None

        field_xy = (vision.get("fields_by_color") or {}).get(self.target_color)
        if field_xy is None:
            # Field gone out of view; keep holding, don't drive blindly.
            return None

        if _dist(robot_xy, field_xy) <= FIELD_RELEASE_RADIUS_PX:
            if not self._stop_sent:
                self._stop_sent = True
                return "STOP"
            self.state = State.RELEASING
            self._action_started_at = time.monotonic()
            self._stop_sent = False
            return "GRIP O"

        path = vision.get("path_to_field") or [field_xy]
        return _drive_along_path(robot_xy, theta, path)

    def _tick_releasing(self) -> Optional[str]:
        if time.monotonic() - self._action_started_at >= GRIP_SETTLE_SEC:
            # Done with this block. Look for the next one (any colour).
            self.target_color = None
            self.state = State.SEEKING_BLOCK
        return None


# --- helpers ----------------------------------------------------------

def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _normalize_deg(angle: float) -> float:
    a = (angle + 180.0) % 360.0 - 180.0
    if a == -180.0:
        a = 180.0
    return a


def _heading_to(src: Point, dst: Point) -> float:
    # Image coords: +y is down. atan2(dy, dx) matches the convention the
    # vision code uses when computing the marker's theta.
    return math.degrees(math.atan2(dst[1] - src[1], dst[0] - src[0]))


def _pick_target_waypoint(robot_xy: Point, path: Sequence[Point]) -> Optional[Point]:
    for wp in path:
        if _dist(robot_xy, wp) > WAYPOINT_REACHED_PX:
            return (float(wp[0]), float(wp[1]))
    return None


def _drive_along_path(robot_xy, theta, path) -> Optional[str]:
    target = _pick_target_waypoint(robot_xy, path)
    if target is None:
        return None
    error = _normalize_deg(_heading_to(robot_xy, target) - theta)
    if abs(error) > ANGLE_TOLERANCE_DEG:
        turn = max(-TURN_DEG_MAX, min(TURN_DEG_MAX, int(round(error))))
        return _format_turn(turn)
    distance_cm = _dist(robot_xy, target) / PX_PER_CM
    if distance_cm < MOVE_CM_MIN:
        return None
    distance_cm = min(MOVE_CM_MAX, distance_cm)
    return _format_move(distance_cm)


def _format_turn(deg: int) -> str:
    """Pack a signed degree into the wire format 'TURN +XXX' / 'TURN -XXX'."""
    sign = "+" if deg >= 0 else "-"
    return f"TURN {sign}{abs(deg):03d}"


def _format_move(cm: float) -> str:
    """Pack a signed cm distance into 'MOVE +XX.XX' / 'MOVE -XX.XX'."""
    cm = max(-99.99, min(99.99, cm))
    sign = "+" if cm >= 0 else "-"
    return f"MOVE {sign}{abs(cm):05.2f}"


def _pick_target_color(vision: dict) -> Optional[str]:
    blocks = vision.get("blocks_by_color") or {}
    fields = vision.get("fields_by_color") or {}
    for color in COLOR_PRIORITY:
        if blocks.get(color) and color in fields:
            return color
    return None


def _nearest_block_of_color(
    vision: dict, color: str, robot_xy: Point
) -> Optional[Point]:
    blocks = (vision.get("blocks_by_color") or {}).get(color) or []
    if not blocks:
        return None
    field_xy = (vision.get("fields_by_color") or {}).get(color)
    candidates = []
    for b in blocks:
        # Skip blocks already inside their field.
        if field_xy is not None and _dist(b, field_xy) <= FIELD_RELEASE_RADIUS_PX:
            continue
        candidates.append(b)
    if not candidates:
        return None
    candidates.sort(key=lambda b: _dist(robot_xy, b))
    return tuple(map(float, candidates[0]))
