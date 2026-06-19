"""
WP4 Link 2+3 smoke test: simulated vision -> bridge state machine -> robot over BLE.

Replays a hand-crafted sequence of "vision snapshots" that walks the bridge
through every state (SEEKING_BLOCK -> GRABBING -> SEEKING_FIELD -> RELEASING
-> back to SEEKING_BLOCK with a new colour). Each command is sent over BLE
and ACKs are printed as they arrive from the robot.

This is the integration test for Links 2 + 3 end-to-end without needing
the vision team's model. The ESP firmware just needs to ACK commands
(see esp_firmware/command_echo/command_echo.ino).

Run:
    python3 bridge_smoketest.py
"""

import asyncio
import sys

from bridge import Bridge
from robot import Robot, RobotError


# Each item is a vision-snapshot dict the way the real main loop will produce.
# Geometry is contrived to exercise the state machine in order.
SIM_FRAMES = [
    # 1) Far from the red block -> bridge should TURN/MOVE toward it.
    {
        "robot": {"xy": (100.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [(400.0, 100.0)], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [(250.0, 100.0), (400.0, 100.0)],
        "path_to_field": [(500.0, 100.0), (600.0, 100.0)],
    },
    # 2) Closer to the red block -> shorter MOVE.
    {
        "robot": {"xy": (300.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [(400.0, 100.0)], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [(400.0, 100.0)],
        "path_to_field": [(500.0, 100.0), (600.0, 100.0)],
    },
    # 3) Within grab radius -> GRIP C, bridge enters GRABBING.
    {
        "robot": {"xy": (390.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [(400.0, 100.0)], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [(400.0, 100.0)],
        "path_to_field": [(500.0, 100.0), (600.0, 100.0)],
    },
    # 4) Still GRABBING (servo hasn't settled yet) -> bridge returns None.
    {
        "robot": {"xy": (390.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [(400.0, 100.0)], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [(400.0, 100.0)],
        "path_to_field": [(500.0, 100.0), (600.0, 100.0)],
    },
    # (After GRIP_SETTLE_SEC the bridge will move to SEEKING_FIELD on the next call.)
    # 5) On the way to the red field.
    {
        "robot": {"xy": (450.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [(400.0, 100.0)], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [(400.0, 100.0)],
        "path_to_field": [(550.0, 100.0), (600.0, 100.0)],
    },
    # 6) Within release radius -> GRIP O, bridge enters RELEASING.
    {
        "robot": {"xy": (595.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [(400.0, 100.0)], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [],
        "path_to_field": [(600.0, 100.0)],
    },
    # 7) Still RELEASING.
    {
        "robot": {"xy": (595.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [],
        "path_to_field": [],
    },
    # 8) Red done; now SEEKING_BLOCK for blue.
    {
        "robot": {"xy": (595.0, 100.0), "theta_deg": 0.0},
        "blocks_by_color": {"red": [], "blue": [(700.0, 300.0)]},
        "fields_by_color": {"red": (600.0, 100.0), "blue": (800.0, 400.0)},
        "path_to_block": [(650.0, 200.0), (700.0, 300.0)],
        "path_to_field": [(750.0, 350.0), (800.0, 400.0)],
    },
]


# Wait long enough between frames that GRABBING / RELEASING actually settles.
FRAME_INTERVAL_SEC = 0.4


async def main() -> int:
    bridge = Bridge()

    def on_msg(line: str) -> None:
        print(f"  < {line}")

    try:
        async with Robot(on_message=on_msg) as robot:
            print(f"Connected. Replaying {len(SIM_FRAMES)} simulated frames.\n")

            for i, vision in enumerate(SIM_FRAMES, 1):
                cmd = bridge.next_command(vision)
                print(f"frame {i:>2}: state={bridge.state}  "
                      f"target={bridge.target_color}  "
                      f"robot={vision['robot']['xy']}")
                if cmd is None:
                    print("           bridge: no-op")
                else:
                    print(f"           > {cmd}")
                    await robot.send(cmd)

                await asyncio.sleep(FRAME_INTERVAL_SEC)
                print()

            print(f"Done. Final bridge state: {bridge.state}")
            return 0
    except RobotError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
