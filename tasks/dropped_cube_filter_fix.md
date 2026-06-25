# Fix: Bridge re-targets the cube it just dropped

## System Prompt (for the subagent)

You are a senior software engineer working on the Sortify robot project at `/Users/serhatuludag/Desktop/CENG424/Project/sortify-comm`. The state machine in `bridge.py` runs SEEKING_BLOCK → GRABBING → SEEKING_FIELD → RELEASING → BACKING_OFF → SEEKING_BLOCK in a loop. There's a bug at the wrap-around: after BACKING_OFF returns to SEEKING_BLOCK, the bridge sometimes re-selects the cube it just dropped.

## The Bug

In `bridge.py`, `_nearest_block_of_color` has this guard:

```python
field_xy = (vision.get("fields_by_color") or {}).get(color)
candidates = []
for b in blocks:
    if field_xy is not None and _dist(b, field_xy) <= FIELD_RELEASE_RADIUS_CM:
        continue   # Skip cubes already inside their target field.
    candidates.append(b)
```

The intent was: "if a cube is already in its matching field, don't chase it." Reasonable. But in practice:

- The cube is **placed near the field's edge**, not the centre. After the gripper opens, the cube sits roughly under where the jaws were.
- `FIELD_RELEASE_RADIUS_CM = 8.0`. If the gripper released 6 cm from field centre, that places the cube ~6 cm from field centre — INSIDE the threshold. Good case, gets filtered.
- But if the release was 7.9 cm from centre, the dropped cube sits ~7.9 cm away — also inside the threshold. Still good.
- If, however, the field detection bbox CENTRE is offset from where the cube actually ended up (because the field is rectangular and the bbox centre isn't the geometric centre of the cube's position), or if BACKING_OFF nudged the cube slightly with the jaws, the cube can end up >`FIELD_RELEASE_RADIUS_CM` from `field_xy`. Then it counts as a fresh red cube and gets re-targeted.
- Also: `fields_by_color[color]` is whatever bbox CENTRE YOLO reports. If the field is a big rectangle and the cube was dropped on a corner of it, the cube IS in the field visually but its world-cm distance to the field-centre is > radius.

Result: robot drives back to the cube it just dropped, picks it up again. Infinite loop.

## The Fix

We need a stronger "already sorted" signal than "close to field centre". Two complementary fixes:

### Fix A — Use bbox containment instead of distance to centre

`main.py` knows the field's `xyxy` rectangle. Pass that to the bridge (it currently passes only the centre point). In `_nearest_block_of_color`:

```python
field_box = (vision.get("field_boxes_by_color") or {}).get(color)
# field_box = (x1, y1, x2, y2) in world cm
for b in blocks:
    if field_box is not None and _box_contains(field_box, b):
        continue
    candidates.append(b)
```

Where `_box_contains(box, point)` returns True if point.x is between x1..x2 and point.y is between y1..y2. Add a small inset (~1 cm) so a cube right on the field edge still counts as inside.

The field is bigger than the grab radius, so this catches all cubes physically on the field — even those near the edge.

### Fix B — "Just dropped" memory

After RELEASING, the bridge knows where it dropped the cube (last `gripper_xy_cm`). Stash that in `self._last_drop_xy_cm` with a timestamp. In `_nearest_block_of_color`, skip any candidate within ~10 cm of the last-drop spot for the next ~15 seconds. This handles the "vision lost track of the field bbox for a frame" edge case.

Both fixes layer on top of each other safely.

## Files Affected

- `bridge.py`:
  - `_tick_releasing` → record drop position + timestamp
  - `_tick_seeking_block` → consult drop memory
  - `_nearest_block_of_color` → take `field_box` instead of just `field_xy`
  - Add `_box_contains` helper
  - New dataclass fields: `_last_drop_xy_cm: Optional[Point]`, `_last_drop_at: float`
- `main.py`:
  - Pass `field_boxes_by_color` in the vision dict (convert the YOLO xyxy pixel rectangle to world cm via `_to_world` for both corners)
  - Keep passing `fields_by_color` (centres) for the SEEKING_FIELD steering — that one's fine.

## Tunables

- `JUST_DROPPED_TIMEOUT_SEC = 15.0`
- `JUST_DROPPED_RADIUS_CM = 12.0`
- Field-inset for "is the cube on the field" check: 1.0 cm inward

## Done When

1. Robot picks up cube → drops on field → backs off → DOES NOT re-target the dropped cube.
2. If there's a SECOND red cube elsewhere on the cardboard, robot drives to that one.
3. If there are no other red cubes, bridge transitions through colour priority (red → blue → green) or to IDLE.
4. The fix doesn't break the case where you put a fresh cube ON the field manually — it stays ignored until removed, which is correct behaviour.

## Status

- [x] Done — both fixes implemented. `bridge.py` now takes `field_boxes_by_color` from the vision dict and filters via `_box_contains` (with `FIELD_BOX_INSET_CM = 1.0`); it also stashes `_last_drop_xy_cm`/`_last_drop_at` in `_tick_releasing` and `_nearest_block_of_color` skips any candidate within `JUST_DROPPED_RADIUS_CM = 12.0` for `JUST_DROPPED_TIMEOUT_SEC = 15.0`. `main.py` warps each field's `xyxy` through the homography and ships it in the vision dict.
