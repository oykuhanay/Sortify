"""
Interactive WP4 robot console: type commands, watch ACKs, tune motor trims live.

Connects to the robot over BLE and gives you a prompt. Whatever you type
gets sent verbatim as a line to the robot; whatever the robot sends back
(ACKs, errors, debug) prints with a '<' prefix.

Heartbeat PONGs are filtered out of the visible output (they fire every
10 s in the background to keep the HM-10 link alive) so they don't
fight with the prompt while you're typing.

Live calibration shortcuts (the whole point of this REPL during tuning):

    r+        right motor trim +1.0  (sends 'TRIM R <new>')
    r-        right motor trim -1.0
    l+        left  motor trim +1.0
    l-        left  motor trim -1.0
    r+0.5     right motor trim by a custom step
    r=25.0    set right motor trim to exact value
    l=18.5    set left  motor trim to exact value
    t         print current trims (Mac-side cached)
    f         quick forward test pulse: 'MOVE +05.00'
    b         quick backward test pulse: 'MOVE -05.00'

Quick reference for the Sortify protocol:

    TURN +030       turn 30 degrees clockwise (right)
    TURN -045       turn 45 degrees counter-clockwise (left)
    MOVE +12.34     drive forward  12.34 cm
    MOVE -05.00     drive backward  5.00 cm
    GRIP C / O      close / open gripper
    STOP            emergency stop
    TRIM R 25.0     set right motor speed scaler
    TRIM L 18.5     set left  motor speed scaler

Type 'quit' or hit Ctrl-D to exit cleanly. Ctrl-C also works.

Run:
    python3 robot_repl.py
"""

import asyncio
import sys

from robot import Robot, RobotError


# Mac-side mirror of the trims currently programmed on the ESP. The defaults
# match the values Mert/Atakan have hard-coded in the firmware. The REPL
# updates these whenever it sends a TRIM command, so 't' can show what the
# robot should currently be using.
DEFAULT_TRIM_RIGHT = 80.00
DEFAULT_TRIM_LEFT = 22.00
TRIM_STEP_DEFAULT = 1.0
QUICK_MOVE_CM = 5.00


HELP_TEXT = """\
Commands you can type:

  Motion / gripper (raw protocol)
    TURN +030    TURN -045
    MOVE +12.34  MOVE -05.00
    GRIP C       GRIP O
    STOP

  Trim calibration (live tuning)
    r+           right trim +1.0
    r-           right trim -1.0
    l+           left  trim +1.0
    l-           left  trim -1.0
    r+0.5        right trim by custom step (also r-0.5)
    r=25.0       set right trim to exact value (also l=18.5)
    TRIM R 25.0  raw protocol form, same as r=25.0
    TRIM L 18.5  raw protocol form, same as l=18.5
    t            show current Mac-side trim values
    f            quick forward test pulse (MOVE +05.00)
    b            quick backward test pulse (MOVE -05.00)

  Session
    reconnect    reconnect if link dropped
    help / ?     show this help
    quit / q     exit
"""


class TrimState:
    """Mac-side cached copy of the trims currently on the robot."""

    def __init__(self) -> None:
        self.right = DEFAULT_TRIM_RIGHT
        self.left = DEFAULT_TRIM_LEFT

    def __str__(self) -> str:
        return f"trim R={self.right:.2f}  L={self.left:.2f}"


def parse_trim_shortcut(line: str, trims: TrimState):
    """Recognise the live-tuning shortcuts.

    Returns the protocol command to send (e.g. 'TRIM R 24.0') after
    mutating `trims`, or None if the line isn't a trim shortcut.
    """
    s = line.strip().lower()
    if not s:
        return None

    # 'r=25.0' / 'l=18.5' — absolute set
    if len(s) >= 3 and s[0] in ("r", "l") and s[1] == "=":
        try:
            val = float(s[2:])
        except ValueError:
            return None
        if s[0] == "r":
            trims.right = val
            return f"TRIM R {val:.2f}"
        trims.left = val
        return f"TRIM L {val:.2f}"

    # 'r+' / 'r-' / 'l+' / 'l-' — increment, with optional custom step
    if len(s) >= 2 and s[0] in ("r", "l") and s[1] in ("+", "-"):
        step_str = s[2:]
        if step_str == "":
            step = TRIM_STEP_DEFAULT
        else:
            try:
                step = float(step_str)
            except ValueError:
                return None
        if s[1] == "-":
            step = -step
        if s[0] == "r":
            trims.right += step
            return f"TRIM R {trims.right:.2f}"
        trims.left += step
        return f"TRIM L {trims.left:.2f}"

    return None


async def open_robot(on_msg) -> Robot:
    robot = Robot(on_message=on_msg)
    await robot.connect()
    return robot


async def main() -> int:
    trims = TrimState()

    def on_msg(line: str) -> None:
        # Filter heartbeat PONGs so they don't fight with the prompt.
        if line.strip().upper() == "PONG":
            return
        print(f"\n< {line}", flush=True)
        print("> ", end="", flush=True)

    print("Connecting to robot (looking for BT05)...")
    try:
        robot = await open_robot(on_msg)
    except RobotError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    print("Connected. Type 'help' for commands, 'quit' to exit.")
    print(f"Starting {trims}.\n")

    loop = asyncio.get_running_loop()

    async def send(command: str) -> None:
        print(f"  -> {command}", flush=True)
        try:
            await robot.send(command)
        except RobotError as e:
            raise e

    try:
        while True:
            try:
                line = await loop.run_in_executor(None, lambda: input("> "))
            except (EOFError, KeyboardInterrupt):
                print()
                break

            line = line.strip()
            if not line:
                continue

            cmd_lower = line.lower()
            if cmd_lower in ("quit", "exit", "q"):
                break
            if cmd_lower in ("help", "?"):
                print(HELP_TEXT)
                continue
            if cmd_lower == "t":
                print(f"  {trims}")
                continue
            if cmd_lower == "f":
                line_to_send = f"MOVE +{QUICK_MOVE_CM:05.2f}"
            elif cmd_lower == "b":
                line_to_send = f"MOVE -{QUICK_MOVE_CM:05.2f}"
            elif cmd_lower == "reconnect":
                print("Reconnecting...")
                try:
                    await robot.disconnect()
                except Exception:
                    pass
                try:
                    robot = await open_robot(on_msg)
                    print("Connected.")
                except RobotError as e:
                    print(f"reconnect failed: {e}", file=sys.stderr)
                continue
            else:
                trim_cmd = parse_trim_shortcut(line, trims)
                if trim_cmd is not None:
                    line_to_send = trim_cmd
                else:
                    line_to_send = line

            try:
                await send(line_to_send)
            except RobotError as e:
                print(f"send error: {e}. trying to reconnect...", file=sys.stderr)
                try:
                    await robot.disconnect()
                except Exception:
                    pass
                try:
                    robot = await open_robot(on_msg)
                    print("Reconnected. Retrying command.")
                    await send(line_to_send)
                except RobotError as e2:
                    print(f"reconnect failed: {e2}", file=sys.stderr)
    finally:
        print("Disconnecting...")
        try:
            await robot.disconnect()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    try:
        sys.exit(asyncio.run(main()))
    except KeyboardInterrupt:
        sys.exit(0)
