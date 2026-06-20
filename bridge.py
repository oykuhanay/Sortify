"""
WP4 bridge: vision output -> robot command, with task state machine.

Per frame the vision team hands us a dict describing the scene. The schema
is intentionally a superset of what each model produces — fields the
keypoint model adds (gripper_pos) are used when available, with the
robot's center as fallback so the bridge still works on older builds.

    {
        "robot_center": (x, y),                # required; from ArUco marker
        "theta_deg": float,                    # required; from ArUco marker
        "gripper_pos": (x, y) | None,          # optional; from keypoint model
        "blocks_by_color": {"red": [(x, y), ...], "blue": [...], ...},
        "fields_by_color": {"red": (x, y), "blue": (x, y), ...},
        "path_to_block":   [(x, y), ...],      # A* from main.py
        "path_to_field":   [(x, y), ...],      # A* from main.py
    }

Decisions:

- Distances to "the block" / "the field" are measured from the GRIPPER
  position when available, falling back to the robot's center. This is
  what determines when to grab / release — the gripper is what actually
  touches the cube, not the marker.

- Each call returns at most ONE command. Streaming control is the loop's
  job — call next_command() every frame, send the result.

- Movements are small on purpose: 5 cm max forward, 30 deg max turn.
  Proportional: the closer the target, the smaller the step. This is what
  keeps the robot stable as it approaches.

- States:
    SEEKING_BLOCK   -> drive (TURN/MOVE) along path_to_block
    GRABBING        -> close gripper, hold while the servo moves
    SEEKING_FIELD   -> drive along path_to_field
    RELEASING       -> open gripper, hold while the servo moves
    BACKING_OFF     -> reverse 10 cm to physically clear the dropped cube
    IDLE            -> no work this frame (or task complete)

  Transitions are geometric (current frame's distances) for the driving
  states, and time-windowed for GRABBING / RELEASING / BACKING_OFF (the
  servo / motors need a moment to actually act before we move on).

Wire protocol (matches esp_firmware/command_echo/command_echo.ino):
    TURN +XXX        signed 3-digit angle. + = clockwise (right).
    TURN -XXX        - = counter-clockwise (left).
    MOVE +XX.XX      signed 2.2 fixed-point centimetres. + = forward.
    MOVE -XX.XX      - = backward.
    GRIP C / GRIP O  close / open the gripper servo.
    STOP             cut motor power immediately.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

Point = Tuple[float, float]


# --- tunables ---------------------------------------------------------

# Driving thresholds
# Tolerance generous on purpose: tiny TURN pulses can't break wheel
# stiction, so the robot reports "still off by 6°" forever and never moves
# forward. With 10° the robot drives whenever roughly aimed, then a longer
# move naturally corrects whatever drift is left.
ANGLE_TOLERANCE_DEG = 10.0      # under this heading error, just drive
WAYPOINT_REACHED_PX = 25.0      # skip waypoints we've already passed
# Generous on purpose: at 40 px the cube was already inside the jaws but
# the bridge still wanted to drive forward, so it kept trying to overshoot
# instead of closing. 80 px latches the grab as soon as the cube is solidly
# within reach; the throttle interval gives the wheels time to brake.
BLOCK_GRAB_RADIUS_PX = 80.0     # gripper this close to block -> grab
FIELD_RELEASE_RADIUS_PX = 90.0  # gripper this close to field -> release

# Pixels-to-centimetres scale of the overhead camera. Measure once with a
# ruler on the table. Until then ~10 px/cm is a reasonable guess for a
# 1920-wide image of a ~50 cm play area.
PX_PER_CM = 10.0

# Per-command size caps. Small + frequent beats large + sparse for stable
# closed-loop control. The robot's wheels turn the chassis fast enough that
# a 30-deg jerk throws the marker out of the camera's view; tiny steps with
# a tight outer cadence are dramatically smoother.
#
# TURN_DEG_MIN exists because the firmware turns ms = (deg/90)*1000, so a
# 5° pulse is ~55 ms and the motors don't even break stiction — the chassis
# sits in place reporting the same heading error frame after frame. 15°
# pulses are short enough not to overshoot but long enough to actually
# rotate the wheels.
MOVE_CM_MIN = 0.50              # ignore noise-level moves
# Per-state forward step caps. The block-approach cap is conservative
# because we have to stop within ±BLOCK_GRAB_RADIUS_PX of the cube — too
# big a step and we overshoot, knock the cube away, or miss the grab. The
# field-approach cap is larger because the field is a big open rectangle
# and there's nothing fragile to crash into between us and it.
MOVE_CM_MAX_BLOCK = 4.00        # max forward step while seeking the block
MOVE_CM_MAX_FIELD = 5.00        # max forward step while seeking the field
TURN_DEG_MIN = 10               # smallest turn that physically rotates the chassis
TURN_DEG_MAX = 10               # max turn per command
BACKOFF_CM = 10.00              # how far to reverse after releasing a cube

# Servo + reverse settle times — the bridge stays quiet during these so the
# robot can finish what we just told it to do without conflicting commands.
GRIP_SETTLE_SEC = 0.6
BACKOFF_SETTLE_SEC = 1.0

# Per-team agreement: which colour to chase first when several are visible.
COLOR_PRIORITY = ("red", "blue", "green")


# --- state machine ----------------------------------------------------
#
# Phase 1 (current): drive to the block, close the gripper, then stop.
#   INIT_OPEN -> SEEKING_BLOCK -> GRABBING -> HOLDING_DONE (no more commands)
#
# Phase 2 (later): drop into a field and back off for the next cube.
#   ... -> SEEKING_FIELD -> RELEASING -> BACKING_OFF -> SEEKING_BLOCK
# Those handlers are kept in this file ready to wire back in when phase 1
# is solid; phase 1 just never transitions into them.

class State:
    AWAITING_START = "AWAITING_START" # script up but user hasn't pressed START
    INIT_OPEN = "INIT_OPEN"           # send GRIP O once at startup
    SEEKING_BLOCK = "SEEKING_BLOCK"
    GRABBING = "GRABBING"
    HOLDING_DONE = "HOLDING_DONE"     # phase 1 terminal: cube captured, idle
    SEEKING_FIELD = "SEEKING_FIELD"   # phase 2 (unused for now)
    RELEASING = "RELEASING"           # phase 2
    BACKING_OFF = "BACKING_OFF"       # phase 2
    IDLE = "IDLE"


@dataclass
class Bridge:
    # We boot into AWAITING_START so the operator can physically line the
    # robot up (gripper centred over the cube etc.) before the state machine
    # starts driving. Press the START key in main.py's window to release.
    state: str = State.AWAITING_START
    target_color: Optional[str] = None
    gripper_open: bool = False        # mac-side cache of last gripper command
    _action_started_at: float = field(default=0.0)
    _stop_sent: bool = False
    _backoff_sent: bool = False

    def reset(self) -> None:
        self.state = State.AWAITING_START
        self.target_color = None
        self.gripper_open = False
        self._action_started_at = 0.0
        self._stop_sent = False
        self._backoff_sent = False

    def start(self) -> None:
        """Operator pressed START — leave AWAITING_START and run the flow."""
        if self.state == State.AWAITING_START:
            self.state = State.INIT_OPEN

    def next_command(self, vision: dict) -> Optional[str]:
        """Decide the next command to send, or None to send nothing this frame."""
        # AWAITING_START is the operator-controlled idle: send nothing until
        # the user presses START.
        if self.state == State.AWAITING_START:
            return None
        # INIT_OPEN runs even without a marker in view — fire it as soon as
        # the bridge wakes up so the gripper is ready by the time we see one.
        if self.state == State.INIT_OPEN:
            return self._tick_init_open()
        # HOLDING_DONE is a terminal idle: don't touch anything, ever.
        if self.state == State.HOLDING_DONE:
            return None

        robot_xy = vision.get("robot_center")
        theta = vision.get("theta_deg")
        if robot_xy is None or theta is None:
            return None

        # Gripper position falls back to robot center if no estimate is given.
        gripper_xy = vision.get("gripper_pos") or robot_xy

        if self.state == State.GRABBING:
            return self._tick_grabbing()
        if self.state == State.RELEASING:
            return self._tick_releasing()
        if self.state == State.BACKING_OFF:
            return self._tick_backing_off()
        if self.state == State.SEEKING_BLOCK:
            return self._tick_seeking_block(vision, robot_xy, theta, gripper_xy)
        if self.state == State.SEEKING_FIELD:
            return self._tick_seeking_field(vision, robot_xy, theta, gripper_xy)
        return None  # IDLE

    # --- state handlers ----------------------------------------------

    def _tick_init_open(self) -> Optional[str]:
        """First thing we do on startup: open the gripper so the cube can
        fit in when we eventually grab it. After one frame we move into
        SEEKING_BLOCK; no need to wait for the servo to finish because
        we'll be driving for a while before we reach a cube anyway."""
        self.gripper_open = True
        self.state = State.SEEKING_BLOCK
        return "GRIP O"

    def _tick_seeking_block(self, vision, robot_xy, theta, gripper_xy) -> Optional[str]:
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

        # Close enough to attempt grab — distance measured from the gripper,
        # not the robot center, because that's what actually touches the cube.
        if _dist(gripper_xy, block_xy) <= BLOCK_GRAB_RADIUS_PX:
            if not self._stop_sent:
                self._stop_sent = True
                return "STOP"
            self.state = State.GRABBING
            self._action_started_at = time.monotonic()
            self._stop_sent = False
            self.gripper_open = False
            return "GRIP C"

        # Otherwise follow the planner's A* path to the block.
        path = vision.get("path_to_block") or [block_xy]
        return _drive_along_path(robot_xy, theta, path, MOVE_CM_MAX_BLOCK)

    def _tick_grabbing(self) -> Optional[str]:
        # Once the servo has had time to close around the cube, head for the
        # matching colour field to drop it off.
        if time.monotonic() - self._action_started_at >= GRIP_SETTLE_SEC:
            self.state = State.SEEKING_FIELD
        return None  # servo is moving, hold position

    def _tick_seeking_field(self, vision, robot_xy, theta, gripper_xy) -> Optional[str]:
        if self.target_color is None:
            self.state = State.SEEKING_BLOCK
            return None

        field_xy = (vision.get("fields_by_color") or {}).get(self.target_color)
        if field_xy is None:
            # Field briefly out of view; keep holding, don't drive blindly.
            return None

        if _dist(gripper_xy, field_xy) <= FIELD_RELEASE_RADIUS_PX:
            if not self._stop_sent:
                self._stop_sent = True
                return "STOP"
            self.state = State.RELEASING
            self._action_started_at = time.monotonic()
            self._stop_sent = False
            self.gripper_open = True
            return "GRIP O"

        # Drive straight to the field. We deliberately ignore path_to_field
        # here: that path is planned from the block's centre (not the robot),
        # so its first waypoint sits next to the cube we're already holding
        # and the heading we'd compute from it is meaningless. The field is
        # an open coloured rectangle on flat cardboard — there's nothing
        # between us and it that needs planning around.
        return _drive_along_path(robot_xy, theta, [field_xy], MOVE_CM_MAX_FIELD)

    def _tick_releasing(self) -> Optional[str]:
        if time.monotonic() - self._action_started_at >= GRIP_SETTLE_SEC:
            # Servo done. Transition to BACKING_OFF — the cube is dropped
            # but the gripper is still around it; we have to physically move
            # away before chasing the next cube or we'll knock this one over.
            self.state = State.BACKING_OFF
            self._action_started_at = time.monotonic()
            self._backoff_sent = False
        return None

    def _tick_backing_off(self) -> Optional[str]:
        # First call in BACKING_OFF: send the reverse pulse.
        if not self._backoff_sent:
            self._backoff_sent = True
            return _format_move(-BACKOFF_CM)
        # Then wait long enough for the motors to actually reverse.
        if time.monotonic() - self._action_started_at >= BACKOFF_SETTLE_SEC:
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


def _drive_along_path(robot_xy, theta, path, move_cm_max: float) -> Optional[str]:
    """Emit either a TURN or a MOVE that nudges the robot toward the next
    A* waypoint. Step sizes shrink with distance to keep the approach stable.
    Caller supplies `move_cm_max` so block-approach (small steps, no overshoot)
    and field-approach (big steps, open ground) can have different ceilings."""
    target = _pick_target_waypoint(robot_xy, path)
    if target is None:
        return None
    error = _normalize_deg(_heading_to(robot_xy, target) - theta)
    if abs(error) > ANGLE_TOLERANCE_DEG:
        # Floor the magnitude at TURN_DEG_MIN so the wheels actually rotate
        # (the firmware's deg-to-ms conversion makes <10° pulses too short
        # to break stiction). Sign comes from `error`.
        mag = max(TURN_DEG_MIN, min(TURN_DEG_MAX, int(round(abs(error)))))
        turn = mag if error >= 0 else -mag
        return _format_turn(turn)
    distance_cm = _dist(robot_xy, target) / PX_PER_CM
    if distance_cm < MOVE_CM_MIN:
        return None
    distance_cm = min(move_cm_max, distance_cm)
    return _format_move(distance_cm)


def _format_turn(deg: int) -> str:
    """Pack a signed degree into 'TURN +XXX' / 'TURN -XXX'."""
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
        # Skip cubes already inside their target field.
        if field_xy is not None and _dist(b, field_xy) <= FIELD_RELEASE_RADIUS_PX:
            continue
        candidates.append(b)
    if not candidates:
        return None
    candidates.sort(key=lambda b: _dist(robot_xy, b))
    return tuple(map(float, candidates[0]))
