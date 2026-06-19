"""
WP4 Link 3 smoke test: Mac (Python) -> HM-10 (BT05) -> ESP8266 -> back.

What this proves:
    Mac can find the HM-10 over BLE by name, connect to it, write a
    PING command to characteristic FFE1, and receive a PONG reply via
    notifications.

If this prints "PONG", Link 3 is alive end-to-end in Python — the same
pipeline LightBlue used by hand, now driven by code.

Run:
    pip install bleak
    python3 ble_smoketest.py

Requirements on the ESP side:
    The ESP sketch must reply to "PING" with "PONG" over hardware UART
    at 9600 baud. The HM-10's TXD and RXD must be wired to the ESP's RX
    and TX respectively (see LEARNING_LOG.md Session 2 wiring table).
"""

import asyncio
import sys

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "BT05"
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
SCAN_TIMEOUT_SEC = 8.0
REPLY_TIMEOUT_SEC = 3.0


async def main() -> int:
    print(f"Scanning for '{DEVICE_NAME}' (timeout {SCAN_TIMEOUT_SEC:.0f}s)...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=SCAN_TIMEOUT_SEC)
    if device is None:
        print(f"ERROR: '{DEVICE_NAME}' not found. Is the HM-10 powered and advertising?", file=sys.stderr)
        return 1
    print(f"Found {device.name} at {device.address}")

    reply_received = asyncio.Event()
    reply_buf = bytearray()

    def on_notify(_sender, data: bytearray) -> None:
        reply_buf.extend(data)
        if b"PONG" in reply_buf:
            reply_received.set()

    async with BleakClient(device) as client:
        print("Connected.")
        await client.start_notify(CHAR_UUID, on_notify)

        print("> PING")
        await client.write_gatt_char(CHAR_UUID, b"PING\n", response=False)

        try:
            await asyncio.wait_for(reply_received.wait(), timeout=REPLY_TIMEOUT_SEC)
            print(f"< {reply_buf.decode('ascii', errors='replace').strip()}")
            print("OK — Link 3 round-trip works.")
            return 0
        except asyncio.TimeoutError:
            print(f"ERROR: no PONG within {REPLY_TIMEOUT_SEC:.0f}s. Got bytes: {bytes(reply_buf)!r}", file=sys.stderr)
            return 2
        finally:
            await client.stop_notify(CHAR_UUID)


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
