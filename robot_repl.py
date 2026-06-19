"""
Interactive WP4 robot console: type commands, watch ACKs.

Connects to the robot over BLE and gives you a prompt. Whatever you type
gets sent verbatim as a line to the robot; whatever the robot sends back
(ACKs, errors, debug) prints with a '<' prefix.

Heartbeat PONGs are filtered out of the visible output (they fire every
10 s in the background to keep the HM-10 link alive) so they don't
fight with the prompt while you're typing.

If the BLE link dies mid-session, the REPL will try to reconnect when
you press Enter on the next command.

Quick reference for the Sortify protocol:

    TURN +030     turn 30 degrees clockwise (right)
    TURN -045     turn 45 degrees counter-clockwise (left)
    MOVE 12.34    drive forward 12.34 cm (forward-only)
    GRIP C        close gripper
    GRIP O        open gripper
    STOP          emergency stop
    PING          ping (replies PONG, filtered from view)

Type 'quit' or hit Ctrl-D to exit cleanly. Ctrl-C also works.

Run:
    python3 robot_repl.py
"""

import asyncio
import sys

from robot import Robot, RobotError


HELP_TEXT = """\
Commands (type and press Enter):
  TURN +030   TURN -045
  MOVE 12.34
  GRIP C      GRIP O
  STOP
  PING
  reconnect   reconnect if link dropped
  help        show this help
  quit        exit
"""


async def open_robot(on_msg) -> Robot:
    robot = Robot(on_message=on_msg)
    await robot.connect()
    return robot


async def main() -> int:
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
    print("Connected. Type 'help' for commands, 'quit' to exit.\n")

    loop = asyncio.get_running_loop()

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
            if cmd_lower == "reconnect":
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

            try:
                await robot.send(line)
            except RobotError as e:
                print(f"send error: {e}. trying to reconnect...", file=sys.stderr)
                try:
                    await robot.disconnect()
                except Exception:
                    pass
                try:
                    robot = await open_robot(on_msg)
                    print("Reconnected. Retrying command.")
                    await robot.send(line)
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
