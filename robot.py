"""
WP4 robot driver: BLE link to the Sortify robot's HM-10 / ESP8266.

Usage:

    import asyncio
    from robot import Robot

    async def main():
        async with Robot() as robot:
            await robot.send("TURN +30")
            await robot.send("MOVE 200")

    asyncio.run(main())

Or, from inside an existing async loop:

    robot = Robot()
    await robot.connect()
    await robot.send("MOVE 200")
    await robot.disconnect()

Protocol (text, line-based, newline-terminated):

    TURN +30      turn 30 degrees clockwise (right)
    TURN -45      turn 45 degrees counter-clockwise (left)
    MOVE 200      drive forward 200 ms
    MOVE -100     drive backward 100 ms
    GRIP O        open gripper
    GRIP C        close gripper
    STOP          emergency stop

The robot's firmware parses these line by line and acts. Replies (if any)
arrive via Notify on the same characteristic and are delivered to an
optional callback.

Connection resilience:
    HM-10 modules drop the link after ~90 s of UART silence. macOS BLE
    also has a supervision timeout. The Robot class handles both:
    - sends a PING heartbeat every HEARTBEAT_SEC to keep the link warm
    - if a send fails or the link is found disconnected, transparently
      reconnects and retries the command once
"""

from __future__ import annotations

import asyncio
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner

DEFAULT_DEVICE_NAME = "BT05"
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
SCAN_TIMEOUT_SEC = 8.0
HEARTBEAT_SEC = 10.0       # how often to send PING to keep the link warm
RECONNECT_BACKOFF_SEC = 1.0


class RobotError(RuntimeError):
    pass


MessageCallback = Callable[[str], None]


class Robot:
    def __init__(
        self,
        device_name: str = DEFAULT_DEVICE_NAME,
        on_message: Optional[MessageCallback] = None,
        heartbeat: bool = True,
    ) -> None:
        self._device_name = device_name
        self._on_message = on_message
        self._heartbeat_enabled = heartbeat
        self._client: Optional[BleakClient] = None
        self._rx_buf = bytearray()
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._connect_lock = asyncio.Lock()

    async def connect(self) -> None:
        async with self._connect_lock:
            if self._client is not None and self._client.is_connected:
                return
            device = await BleakScanner.find_device_by_name(
                self._device_name, timeout=SCAN_TIMEOUT_SEC
            )
            if device is None:
                raise RobotError(f"BLE device '{self._device_name}' not found")
            client = BleakClient(device)
            await client.connect()
            if not client.is_connected:
                raise RobotError(f"failed to connect to '{self._device_name}'")
            await client.start_notify(CHAR_UUID, self._on_notify)
            self._client = client

            if self._heartbeat_enabled and self._heartbeat_task is None:
                self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

    async def disconnect(self) -> None:
        if self._heartbeat_task is not None:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except (asyncio.CancelledError, Exception):
                pass
            self._heartbeat_task = None

        if self._client is not None and self._client.is_connected:
            try:
                await self._client.stop_notify(CHAR_UUID)
            except Exception:
                pass
            try:
                await self._client.disconnect()
            except Exception:
                pass
        self._client = None

    async def send(self, command: str) -> None:
        """Send a command, reconnecting and retrying once if the link died."""
        try:
            await self._send_raw(command)
        except Exception:
            # Link probably died. Try to reconnect and resend once.
            await self._reconnect()
            await self._send_raw(command)

    async def _send_raw(self, command: str) -> None:
        if self._client is None or not self._client.is_connected:
            raise RobotError("not connected")
        line = command.strip() + "\n"
        await self._client.write_gatt_char(
            CHAR_UUID, line.encode("ascii"), response=False
        )

    async def _reconnect(self) -> None:
        # Tear down whatever we have, then connect fresh.
        try:
            if self._client is not None:
                try:
                    await self._client.disconnect()
                except Exception:
                    pass
        finally:
            self._client = None

        await asyncio.sleep(RECONNECT_BACKOFF_SEC)
        # Stop heartbeat while we rebuild — connect() will restart it.
        if self._heartbeat_task is not None:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except (asyncio.CancelledError, Exception):
                pass
            self._heartbeat_task = None
        await self.connect()

    async def _heartbeat_loop(self) -> None:
        try:
            while True:
                await asyncio.sleep(HEARTBEAT_SEC)
                if self._client is None or not self._client.is_connected:
                    # Link died silently between heartbeats. Try to recover.
                    try:
                        await self._reconnect()
                    except Exception:
                        # Will try again next tick.
                        continue
                try:
                    await self._send_raw("PING")
                except Exception:
                    # Send failed even though link looked up — drop & retry.
                    try:
                        await self._reconnect()
                    except Exception:
                        continue
        except asyncio.CancelledError:
            return

    def _on_notify(self, _sender, data: bytearray) -> None:
        self._rx_buf.extend(data)
        while b"\n" in self._rx_buf:
            line, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            text = line.decode("ascii", errors="replace").strip()
            if text and self._on_message is not None:
                self._on_message(text)

    async def __aenter__(self) -> "Robot":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        await self.disconnect()
