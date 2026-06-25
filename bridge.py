"""
WP4 bridge: vision output -> robot command, with task state machine.

Per frame the vision team hands us a dict describing the scene. The schema
is intentionally a superset of what each model produces — fields the
keypoint model adds (gripper_pos) are used when available, with the
robot's center as fallback so the bridge still works on older builds.

    {
        "robot_center": (x_cm, y_cm),          # required; world coords (cm)
        "theta_deg": float,                    # required; from ArUco marker
        "gripper_pos": (x_cm, y_cm) | None,    # optional; world coords (cm)
        "blocks_by_color": {"red": [(x_cm, y_cm), ...], "blue": [...], ...},
        "fields_by_color": {"red": (x_cm, y_cm), "blue": (x_cm, y_cm), ...},
        "path_to_block":   [(x_cm, y_cm), ...],
        "path_to_field":   [(x_cm, y_cm), ...],
    }

    Everything is in world centimetres now — main.py applies a homography
    so distances and angles match physical reality regardless of where on
    the playing field the robot is. Pixel-space measurements got fooled by
    perspective near the edges of the frame.

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
WAYPOINT_REACHED_CM = 2.5       # skip waypoints we've already passed (cm)
# Generous on purpose: at 4 cm the cube was already inside the jaws but
# the bridge still wanted to drive forward, so it kept trying to overshoot
# instead of closing. 8 cm latches the grab as soon as the cube is solidly
# within reach; the throttle interval gives the wheels time to brake.
BLOCK_GRAB_RADIUS_CM = 3.0      # gripper this close to block -> grab
FIELD_RELEASE_RADIUS_CM = 8.0   # gripper this close to field -> release

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
MOVE_CM_MIN = 0.40              # ignore noise-level moves
# Driving has two distance modes and an extra big-error turn mode on top.
#
# Distance modes — handoff at NEAR_THRESHOLD_CM, measured from the
# gripper to the target:
#   FAR  — robot is rolling toward a faraway target; big steps OK.
#   NEAR — gripper is close; small steps so we don't overshoot the cube.
#
# Turn modes — three:
#   HUGE  : heading error >= TURN_HUGE_ERROR_DEG. Used regardless of
#           distance, because a 150°-off robot would otherwise take
#           20+ pulses to face the cube.
#   FAR   : normal far-mode pulse.
#   NEAR  : we're close, so any pulse should be small to avoid spinning
#           past the cube and losing the marker out of frame.
# Per-state NEAR threshold. The block approach has to be precise (the
# cube has to end up inside the jaws), so we slow down well before
# arrival. The field approach doesn't — we already have the cube; the
# field is a big rectangle and any spot inside it counts. Smaller
# field-near threshold = robot keeps cruising until almost touching
# the field, which is what the operator asked for.
NEAR_THRESHOLD_CM       = 10.0          # default; SEEKING_BLOCK uses this
NEAR_THRESHOLD_CM_FIELD = 6.0           # SEEKING_FIELD: stay in FAR mode longer
MOVE_CM_MAX_FAR       = 3.00    # forward step when target is far away
MOVE_CM_MAX_NEAR      = 1.50    # forward step when we're close to target

# Default turn caps (SEEKING_BLOCK, INIT, etc.)
TURN_HUGE_ERROR_DEG   = 120.0   # above this heading error, use the big pulse
TURN_DEG_HUGE         = 25      # turn pulse when the heading error is huge
TURN_DEG_FAR          = 10      # turn pulse when target is far but heading isn't huge
TURN_DEG_NEAR         = 5       # turn pulse when we're close to the target
TURN_DEG_VERY_NEAR    = 3       # turn pulse when target is in the jaws — tiny so we don't kick the cube away
VERY_NEAR_THRESHOLD_CM = 7.0    # gripper-to-target distance for "very near" turn mode
TURN_DEG_MIN          = 3       # absolute floor — anything lower can't break stiction

# SEEKING_FIELD turn caps: faster than block but not crazy. 30°/15°
# was overshooting and oscillating left-right; 20°/12° still closes
# the angle quickly without spinning past it.
TURN_HUGE_ERROR_DEG_FIELD = 45.0
TURN_DEG_HUGE_FIELD       = 25
TURN_DEG_FAR_FIELD        = 12
BACKOFF_CM            = 6.00    # how far to reverse after releasing a cube

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
    # block_color -> dest field_color. Default identity (red->red etc).
    # Override via Bridge.set_color_routing({"red":"blue", ...}) for
    # cross-color sorting demos.
    color_routing: dict = field(default_factory=lambda: {
        "red": "red", "blue": "blue", "green": "green",
    })
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

    def set_color_routing(self, routing: dict) -> None:
        """Update block->field colour routing. Unknown keys are ignored;
        unspecified colours keep their previous mapping."""
        for k, v in (routing or {}).items():
            if k in ("red", "blue", "green") and v in ("red", "blue", "green"):
                self.color_routing[k] = v

    def _dest_color(self, block_color: Optional[str]) -> Optional[str]:
        if block_color is None:
            return None
        return self.color_routing.get(block_color, block_color)

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
        # IDLE — but if a new cube/field combo became visible (e.g. the
        # operator dropped a fresh cube on the field, or vision finally
        # picked up the green pair after we'd exhausted red) we should
        # leave IDLE and try again instead of just sitting there.
        if self.state == State.IDLE:
            if _pick_target_color(vision, self.color_routing) is not None:
                print("[BRIDGE] IDLE -> SEEKING_BLOCK (new candidate visible)")
                self.state = State.SEEKING_BLOCK
                self.target_color = None     # re-pick in the next tick
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
            self.target_color = _pick_target_color(vision, self.color_routing)
            if self.target_color is None:
                self.state = State.IDLE
                return None

        block_xy = _nearest_block_of_color(
            vision, self.target_color, robot_xy,
            dest_color=self._dest_color(self.target_color),
        )
        if block_xy is None:
            # Vision lost the cube. There are two flavours of this:
            #
            # 1) We already sent STOP because the cube was in range last
            #    frame. The disappearance is the gripper closing on it
            #    blocking the camera's view — that's exactly what we
            #    want, take the win and move on to GRABBING.
            #
            # 2) We were still approaching and the cube blinked out for
            #    a frame or two. Be patient: 15 frames of nothing before
            #    we give up on this target.
            if self._stop_sent:
                # Branch 1: jaws are closing over the cube. Commit.
                self.state = State.GRABBING
                self._action_started_at = time.monotonic()
                self._stop_sent = False
                self.gripper_open = False
                return "GRIP C"
            self._target_miss_frames = getattr(self, "_target_miss_frames", 0) + 1
            if self._target_miss_frames < 15:
                return None       # patience; just don't drive this tick
            self._target_miss_frames = 0
            self.target_color = None
            return None
        # Saw it this frame; reset the patience counter.
        self._target_miss_frames = 0

        # Close enough to attempt grab — distance measured from the gripper,
        # not the robot center, because that's what actually touches the cube.
        if _dist(gripper_xy, block_xy) <= BLOCK_GRAB_RADIUS_CM:
            if not self._stop_sent:
                self._stop_sent = True
                return "STOP"
            self.state = State.GRABBING
            self._action_started_at = time.monotonic()
            self._stop_sent = False
            self.gripper_open = False
            return "GRIP C"

        # Drive straight at the block. A* path_to_block was unreliable in
        # tests — its first waypoint sometimes sat behind the robot
        # because the pixel grid start cell wasn't exactly under the
        # robot, which made the bridge pick a waypoint that pointed the
        # wrong way and the robot would spin off in the opposite
        # direction. The block is a single target on open ground; nothing
        # we'd plan around between us and it.
        return _drive_along_path(robot_xy, theta, [block_xy], gripper_xy)

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

        # Cross-colour routing: the cube we picked up is target_color, but
        # we drop it on whatever field the operator routed it to.
        dest_color = self._dest_color(self.target_color)
        field_xy = (vision.get("fields_by_color") or {}).get(dest_color)
        if field_xy is None:
            # Field briefly out of view; keep holding, don't drive blindly.
            return None

        # Drop once the gripper is comfortably INSIDE the field
        # rectangle. We don't need to hit the centre — anywhere a few
        # cm in from the edge is fine — but the previous "any point
        # inside the bbox" check let the robot release on the field's
        # outer rim, so the cube ended up on the cardboard, not on
        # the colour. FIELD_RELEASE_INSET_CM pulls the trigger zone
        # inward so the cube lands well clear of the edge.
        FIELD_RELEASE_INSET_CM = 2.0
        field_box = (vision.get("field_boxes_by_color") or {}).get(dest_color)
        if field_box is not None:
            x1, y1, x2, y2 = field_box
            xmin, xmax = min(x1, x2), max(x1, x2)
            ymin, ymax = min(y1, y2), max(y1, y2)
            inset = FIELD_RELEASE_INSET_CM
            # Guard against pathological tiny bboxes — if the field is
            # smaller than 2*inset the inset would invert and nothing
            # would ever count as inside.
            if xmax - xmin > 2 * inset and ymax - ymin > 2 * inset:
                xmin += inset; xmax -= inset
                ymin += inset; ymax -= inset
            inside = (xmin <= gripper_xy[0] <= xmax
                      and ymin <= gripper_xy[1] <= ymax)
        else:
            inside = _dist(gripper_xy, field_xy) <= FIELD_RELEASE_RADIUS_CM

        if inside:
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
        return _drive_along_path(
            robot_xy, theta, [field_xy], gripper_xy,
            near_threshold_cm=NEAR_THRESHOLD_CM_FIELD,
            turn_huge_error_deg=TURN_HUGE_ERROR_DEG_FIELD,
            turn_deg_huge=TURN_DEG_HUGE_FIELD,
            turn_deg_far=TURN_DEG_FAR_FIELD,
        )

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
        if _dist(robot_xy, wp) > WAYPOINT_REACHED_CM:
            return (float(wp[0]), float(wp[1]))
    return None


def _drive_along_path(robot_xy, theta, path, gripper_xy=None,
                      near_threshold_cm: float = NEAR_THRESHOLD_CM,
                      turn_huge_error_deg: float = TURN_HUGE_ERROR_DEG,
                      turn_deg_huge: int = TURN_DEG_HUGE,
                      turn_deg_far: int = TURN_DEG_FAR) -> Optional[str]:
    """Emit either a TURN or a MOVE that nudges the robot toward the next
    waypoint. Switches between FAR and NEAR modes at `near_threshold_cm`:
    far targets get big pulses (cover ground), close ones get small
    pulses (don't overshoot the cube).

    `near` is measured from the GRIPPER to the target, not the marker
    centre — the gripper sits ~20 cm ahead of the marker, so using the
    marker would falsely report "far away" even when the jaws are
    practically over the cube.

    Per-state callers can lower `near_threshold_cm` so the field
    approach doesn't crawl: dropping off doesn't need cube-grab
    precision. They can also pass bigger turn caps for the same reason
    — SEEKING_FIELD doesn't care about overshoot, just about getting
    pointed at the rectangle fast."""
    target = _pick_target_waypoint(robot_xy, path)
    if target is None:
        return None
    # Distance the wheels actually need to roll. We always drive the
    # robot CENTRE forward (that's what TURN/MOVE move), but we judge
    # "are we close" from the gripper because the gripper is what
    # eventually has to be over the cube.
    distance_cm = _dist(robot_xy, target)
    proximity_xy = gripper_xy if gripper_xy is not None else robot_xy
    near = _dist(proximity_xy, target) <= near_threshold_cm

    # Heading error. When we're close to the cube, "robot pointed at
    # target" isn't enough — the gripper sits 20 cm ahead of the
    # marker, so a few degrees off-axis from the marker becomes a
    # gripper that's beside the cube instead of around it. In NEAR
    # mode we therefore measure the heading from the GRIPPER to the
    # target, which catches that lateral drift. The tolerance also
    # tightens to 4° so we re-aim before we ram the cube sideways.
    proximity_dist = _dist(proximity_xy, target)
    if near and gripper_xy is not None:
        error = _normalize_deg(_heading_to(gripper_xy, target) - theta)
        # In NEAR mode we measure from the gripper, and the closer we
        # get the more meaningless a few degrees of sideways drift
        # actually is — at 5 cm a 10° error is only 0.9 cm of lateral
        # offset, which is well inside the jaws. Tightening the
        # tolerance at that distance just makes the robot twitch
        # left-right forever instead of driving the last cm forward
        # and closing. So: linear loosening as we approach.
        #   25 cm  -> 4°      (tight; we still have room to re-aim)
        #    7 cm  -> 12°     (loose; just drive in)
        #    0 cm  -> 18°     (effectively pinned, MOVE only)
        if proximity_dist <= 7.0:
            # Map (0..7) -> (18..12)
            t = max(0.0, proximity_dist / 7.0)
            tolerance = 18.0 - 6.0 * t
        else:
            # Map (7..25) -> (12..4)
            t = max(0.0, min(1.0, (proximity_dist - 7.0) / (25.0 - 7.0)))
            tolerance = 12.0 - 8.0 * t
    else:
        error = _normalize_deg(_heading_to(robot_xy, target) - theta)
        tolerance = ANGLE_TOLERANCE_DEG
    if abs(error) > tolerance:
        # Four-tier turn cap:
        #   VERY NEAR (cube basically in the jaws): 3° nudges so we
        #     don't pivot the chassis enough to bat the cube sideways
        #     and end up chasing it left-right forever.
        #   HUGE error: big bite, doesn't matter how far.
        #   NEAR distance: small 5° pulses.
        #   Otherwise: caller-supplied far pulse.
        if proximity_dist <= VERY_NEAR_THRESHOLD_CM:
            turn_cap = TURN_DEG_VERY_NEAR
        elif abs(error) >= turn_huge_error_deg:
            turn_cap = turn_deg_huge
        elif near:
            turn_cap = TURN_DEG_NEAR
        else:
            turn_cap = turn_deg_far
        mag = max(TURN_DEG_MIN, min(turn_cap, int(round(abs(error)))))
        turn = mag if error >= 0 else -mag
        return _format_turn(turn)
    if distance_cm < MOVE_CM_MIN:
        return None
    move_cap = MOVE_CM_MAX_NEAR if near else MOVE_CM_MAX_FAR
    return _format_move(min(move_cap, distance_cm))


def _format_turn(deg: int) -> str:
    """Pack a signed degree into 'TURN +XXX' / 'TURN -XXX'."""
    sign = "+" if deg >= 0 else "-"
    return f"TURN {sign}{abs(deg):03d}"


def _format_move(cm: float) -> str:
    """Pack a signed cm distance into 'MOVE +XX.XX' / 'MOVE -XX.XX'."""
    cm = max(-99.99, min(99.99, cm))
    sign = "+" if cm >= 0 else "-"
    return f"MOVE {sign}{abs(cm):05.2f}"


def _pick_target_color(vision: dict, routing: Optional[dict] = None) -> Optional[str]:
    """Pick the highest-priority colour that has at least one cube NOT
    already sitting on its matching field. Without the bbox filter we'd
    forever re-pick `red` once the red cube is sorted: there's still a
    red block and a red field in the scene, so the old check passed,
    but `_nearest_block_of_color` would then filter the only candidate
    and return None, and we'd spin in circles instead of advancing to
    green."""
    blocks_by_color = vision.get("blocks_by_color") or {}
    fields_by_color = vision.get("fields_by_color") or {}
    field_boxes_by_color = vision.get("field_boxes_by_color") or {}
    for color in COLOR_PRIORITY:
        # Where would a cube of this colour actually be dropped? With
        # default identity routing this is just `color`; under
        # cross-colour routing it may be a different field.
        dest = (routing or {}).get(color, color)
        if dest not in fields_by_color:
            continue
        cubes = blocks_by_color.get(color) or []
        if not cubes:
            continue
        # Use the same containment logic as _nearest_block_of_color so
        # the two never disagree about whether a cube is sorted. We
        # check the ROUTED destination field's bbox, not the cube's
        # own colour, so a red cube routed to blue is "sorted" when
        # it's sitting on the blue field.
        field_box = field_boxes_by_color.get(dest)
        unsorted_exists = False
        for b in cubes:
            if field_box is not None:
                x1, y1, x2, y2 = field_box
                xmin, xmax = min(x1, x2), max(x1, x2)
                ymin, ymax = min(y1, y2), max(y1, y2)
                inset = 1.0
                if (xmin - inset <= b[0] <= xmax + inset
                        and ymin - inset <= b[1] <= ymax + inset):
                    continue
            unsorted_exists = True
            break
        if unsorted_exists:
            return color
    return None


def _nearest_block_of_color(
    vision: dict, color: str, robot_xy: Point,
    dest_color: Optional[str] = None,
) -> Optional[Point]:
    """Pick the nearest cube of `color` that isn't already sitting on
    its destination field. The 'already sorted' check uses the field's
    bounding BOX (with a small inward inset) rather than distance to
    the field's centre — fields are big rectangles and the cube can
    land anywhere on them. Under cross-colour routing, `dest_color`
    is where the cube goes (e.g. red->blue); defaults to `color`."""
    blocks = (vision.get("blocks_by_color") or {}).get(color) or []
    if not blocks:
        return None
    dest = dest_color or color
    field_box = (vision.get("field_boxes_by_color") or {}).get(dest)
    field_xy = (vision.get("fields_by_color") or {}).get(dest)
    candidates = []
    for b in blocks:
        # Primary filter: cube physically on its destination field.
        if field_box is not None:
            x1, y1, x2, y2 = field_box
            xmin, xmax = min(x1, x2), max(x1, x2)
            ymin, ymax = min(y1, y2), max(y1, y2)
            inset = 1.0   # 1 cm inward so cubes right on the edge still count as in
            if (xmin - inset <= b[0] <= xmax + inset
                    and ymin - inset <= b[1] <= ymax + inset):
                continue
        elif field_xy is not None and _dist(b, field_xy) <= FIELD_RELEASE_RADIUS_CM:
            # Fallback when bbox isn't available — older vision dicts.
            continue
        candidates.append(b)
    if not candidates:
        return None
    candidates.sort(key=lambda b: _dist(robot_xy, b))
    return tuple(map(float, candidates[0]))
