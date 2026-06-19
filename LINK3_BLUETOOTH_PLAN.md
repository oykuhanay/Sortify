# Link 3 — Wireless Communication Plan (Mac ↔ Robot)

**Author:** Serhat ULUDAĞ (WP4)
**Status:** Planning / pre-implementation
**Goal:** Prove wireless command channel between MacBook and the robot's microcontroller.

---

## 1. Why this document exists

This file is a **standalone roadmap** for getting wireless communication working between:

- A **MacBook** (running the vision/AI side of Sortify), and
- An **ESP8266 NodeMCU** development board (the robot's brain), via
- An **HM-10** Bluetooth module wired to the ESP board.

It's written so anyone (a teammate, a future-me, or another AI assistant) can read it and immediately understand:

1. What we're building
2. What every part does
3. What the wiring looks like, pin-by-pin
4. What concrete steps to follow, in order
5. How we'll know each step worked

**Read time:** ~15 minutes. **Total work time after reading:** roughly half a day, spread across ~4 sessions.

---

## 2. Glossary — every weird word explained

If you see any of these words below and don't know them, this is the lookup table:

| Word | What it actually means |
|---|---|
| **Bluetooth** | A way two devices talk to each other wirelessly over short distance (~10 m). |
| **BLE** | "Bluetooth Low Energy". A *newer flavor* of Bluetooth designed to send small messages with very little power. The HM-10 speaks BLE. |
| **HM-10** | The small blue PCB you bought. It is a "Bluetooth-to-wires translator." On one side it talks BLE through the air. On the other side it has 4 metal pins that talk to a microcontroller through wires. |
| **ESP8266** | The chip that does the actual computing on the robot side. It can run small programs you upload to it. It also has Wi-Fi (which we are not using right now). |
| **NodeMCU** | The full development board you bought. It has the ESP8266 chip plus a USB port, voltage regulator, and pin headers. When this doc says "ESP board" it means this NodeMCU. |
| **GPIO** | "General-Purpose Input/Output". A fancy name for *the metal pins on the side of the ESP board*. Each pin has a number (e.g., GPIO12, GPIO14). You tell the program: "use pin GPIO14 to send data" — that's what GPIO means. |
| **UART** | A simple way two chips talk over wires. Two wires: one to send (TX), one to receive (RX). The HM-10 and ESP board will talk over UART. |
| **TX / RX** | "Transmit" and "Receive". On every UART connection, **chip A's TX wire goes to chip B's RX wire**, and vice-versa. Like phone calls — your mouth (TX) connects to the other person's ear (RX). |
| **VCC** | The "+" side of power. For the HM-10 and ESP board, this is **3.3 volts**. |
| **GND** | "Ground." The "−" side of power. Both devices must share the same GND or nothing works. |
| **3.3V vs 5V logic** | Some chips use 3.3V signals, others 5V. Mixing them can fry the cheaper one. **Lucky for us, both the ESP board and HM-10 use 3.3V** — so we can connect them directly with no extra parts. |
| **Sketch** | The Arduino word for "a small program you upload to a microcontroller." Same thing as "code" or "firmware." |
| **Arduino IDE** | A free desktop app from arduino.cc. We use it to write sketches and upload them to the ESP board over USB. |
| **Serial Monitor** | A window inside the Arduino IDE that shows whatever the ESP board prints. Useful for debugging. |
| **bleak** | A Python library (like `cv2` was for the camera) that lets Python code talk to BLE devices. We'll `pip install` it later. |
| **PING / PONG** | Just two words we'll send over the wireless link to test it. The Mac sends "PING", the ESP replies "PONG". If both arrive, the link works. |

---

## 3. The big picture (one diagram)

```
   ┌──────────────┐                              ┌──────────────┐                          ┌──────────────┐
   │              │   Bluetooth Low Energy       │              │   UART, 4 wires          │              │
   │   MacBook    │  ◄─────────  air  ─────────► │    HM-10     │  ◄────────────────────►  │   ESP8266    │
   │   (Python)   │                              │  (BT module) │                          │   NodeMCU    │
   │              │                              │              │                          │  (the robot) │
   └──────────────┘                              └──────────────┘                          └──────────────┘
        │                                                                                          │
        │                                                                                          │
        │                                                                                          ▼
        │                                                                                  ┌──────────────┐
        │                                                                                  │ Motors,      │
        │                                                                                  │ servos,      │
        │                                                                                  │ gripper —    │
        │                                                                                  │ ADDED LATER  │
        │                                                                                  └──────────────┘
        │
        ▼
   ┌──────────────────────────────────────────┐
   │ The Mac runs a Python script.            │
   │ It says: "Connect to HM-10. Send PING."  │
   │ Then it waits to read the reply.         │
   └──────────────────────────────────────────┘
```

In words:

- The MacBook sends a message wirelessly to the **HM-10** over BLE.
- The HM-10 receives it and **passes the same message** through 4 wires to the **ESP board**.
- The ESP board runs a small program that reads the message, decides how to reply, and sends the reply back the same way.

The HM-10 itself is **not smart**. It's just a wireless extension cord. It does not "understand" the message. It just shuttles bytes between BLE-air and UART-wires.

---

## 4. What we already own vs what we still need

### Already bought
- ✅ ESP8266 NodeMCU V3 (32-pin variant)
- ✅ HM-10 Bluetooth 4.0 module (CC2541)

### Probably needed before starting
- ⬜ A USB cable that fits **one end into the NodeMCU** (Micro-USB on most NodeMCU V3s) and **the other end into the MacBook** (USB-A or USB-C depending on the Mac model). If your Mac only has USB-C, you may need a USB-C-to-Micro-USB cable or a USB-A-to-USB-C adapter.
- ⬜ **4 jumper wires** (female-to-female recommended, since both the HM-10 and NodeMCU have male header pins). Total length 10–20 cm.
- ⬜ (Optional but useful) A small breadboard, in case we want to mount things tidily. Not strictly required for these tests — we can connect HM-10 directly to NodeMCU pins with jumper wires.
- ⬜ (Optional) A free BLE scanner app on iPhone or Mac — e.g., **LightBlue** (App Store, free). Used only as a sanity check at one stage.

### Not needed for these tests
- ❌ Battery / external power — for all four stages, the NodeMCU is powered through USB by the Mac, and the HM-10 is powered from the NodeMCU's 3.3 V pin. Zero soldering. Zero batteries.
- ❌ Motor driver, motors, servo, chassis — those come *after* the Bluetooth link is proven.
- ❌ Level shifter / resistors — both chips are 3.3 V, so direct wiring is safe.

---

## 5. The pin map (memorize / pin to wall)

### HM-10 module (4 pins we'll use)

The HM-10 has 6 pins on it but we only use 4. Looking at the front of the module (the side with the chip and the antenna trace):

| Pin label on HM-10 | What it is | Connect to ESP board |
|---|---|---|
| **VCC** | + power, 3.3 V | ESP board's **3V3** pin |
| **GND** | − power, ground | ESP board's **GND** pin |
| **TXD** | HM-10's transmit (data leaving HM-10) | ESP board's **D6** pin (which is GPIO12 internally) |
| **RXD** | HM-10's receive (data entering HM-10) | ESP board's **D5** pin (which is GPIO14 internally) |
| STATE | (connection LED indicator — leave unconnected) | — |
| BRK | (break command — leave unconnected) | — |

### ESP8266 NodeMCU (the pins we'll use)

NodeMCU has labels printed on the board like `D0`, `D1`, `D5`, `3V3`, `GND`. Those are the labels you connect wires to. Internally, each `Dx` label corresponds to a different "GPIO number" (this is what your sketch uses):

| Label on board | Internal GPIO number | Used for |
|---|---|---|
| **3V3** | (power, not a GPIO) | Power for HM-10's VCC |
| **GND** | (ground, not a GPIO) | Ground for HM-10's GND |
| **D5** | GPIO14 | Software UART **TX** (ESP sends → HM-10 RXD) |
| **D6** | GPIO12 | Software UART **RX** (HM-10 TXD → ESP receives) |

> ⚠️ **Why D5 and D6 specifically?** Because some pins on the NodeMCU have special boot-time roles and *must not* be wired to anything that drives them at startup, or the board won't boot. **D5 and D6 (GPIO14 and GPIO12) are safe**, commonly used for software-UART projects, and easy to remember as a pair. Don't substitute random pins unless you know the consequences.

### The full wiring table — copy this when you wire it up

```
ESP8266 NodeMCU pin     →     HM-10 pin
────────────────────          ──────────
3V3                     →     VCC
GND                     →     GND
D5  (GPIO14, "TX")      →     RXD
D6  (GPIO12, "RX")      →     TXD
```

**Critical rule:** TX of one device connects to RX of the other. Never TX-to-TX.

### Visual: physical wiring diagram

```
           ESP8266 NodeMCU                              HM-10
        ┌────────────────────┐                     ┌────────────┐
        │                    │                     │            │
        │  3V3   ●───────────┼─────── red ────────►│ VCC        │
        │                    │                     │            │
        │  GND   ●───────────┼─────── black ──────►│ GND        │
        │                    │                     │            │
        │  D5    ●───────────┼─────── yellow ─────►│ RXD        │
        │  (TX)              │                     │            │
        │                    │                     │            │
        │  D6    ●◄──────────┼─────── green ──────·│ TXD        │
        │  (RX)              │                     │            │
        │                    │                     │            │
        │  USB ◄── to Mac    │                     └────────────┘
        └────────────────────┘
```

(Wire colors are suggestions — use whatever you have, but keep the function consistent: red=power, black=ground.)

---

## 6. The four-stage roadmap

We'll get to wireless PING/PONG in **four stages**. Each stage proves one thing. We don't move on until the current stage works.

### Why stages?
If you wire everything at once and it doesn't work, you have ~5 possible failure points and no way to tell which one is broken. By going stage by stage, each failure points to exactly one cause.

---

### Stage A — "Is the ESP board alive?"

**Goal of this stage:** Confirm that the MacBook can see the ESP board over USB, the Arduino IDE can upload a program to it, and the program runs.

**No HM-10 involved yet. No Bluetooth. No wires beyond the USB cable.**

#### What you do

1. **Install Arduino IDE** on the Mac (download from arduino.cc, the official site). Free.
2. **Add ESP8266 board support** in Arduino IDE settings. (Specific steps will be walked through during the actual session — there's a "Board Manager URL" you paste in.)
3. **Connect the NodeMCU to the Mac with the USB cable.** A small light on the NodeMCU should turn on.
4. **Upload the "Blink" sketch** — a built-in 5-line example sketch that turns the NodeMCU's onboard LED on and off every second.

#### How you know it worked
The small blue LED on the NodeMCU board flashes once per second. That's it.

#### If it doesn't work
- **No light at all on the board** → bad USB cable (charging-only cable, no data). Try another cable. This is the #1 cause of "doesn't work."
- **Light comes on but Arduino IDE can't find the board** → missing USB driver (CH340 or CP210x — depends on which NodeMCU clone you got). We install the driver.
- **Upload fails halfway** → USB hub problem. Plug directly into the Mac, no hub.

---

### Stage B — "Are the wires between HM-10 and ESP working?"

**Goal of this stage:** Confirm the 4 wires between the HM-10 and the NodeMCU are connected correctly and data can flow through them.

**Still no actual Bluetooth use yet.** We're just testing the local UART wires between the two chips.

#### What you do

1. **Power off** (unplug USB).
2. **Connect the 4 wires** as shown in Section 5.
3. **Plug USB back into Mac.** The HM-10's small red LED should start blinking — that means it has power and is advertising. (If not blinking: re-check VCC and GND.)
4. **Upload a slightly bigger sketch to the ESP board.** This sketch will:
   - Read anything that comes in from the HM-10's TXD wire,
   - Print it to the Arduino Serial Monitor (so you see it on the Mac),
   - And echo it back through the HM-10's RXD wire.
5. **Open the Arduino Serial Monitor** to watch.

#### How you know it worked
At this stage you can't yet test it from the Mac wirelessly (we haven't written that yet). What you *can* check:

- The HM-10 LED is blinking → power and ground wires correct.
- The ESP board boots without crashing into a reboot loop → no pin conflict.

The full "data goes through wires" test is finished in Stage D.

#### If it doesn't work
- **HM-10 LED never lights** → VCC or GND wire wrong, or HM-10 is dead.
- **ESP board boot loops or prints garbage in serial monitor** → likely TX/RX swapped (very common!), or wrong pins. Re-verify D5 = TX, D6 = RX.

---

### Stage C — "Can the Mac see the HM-10 over Bluetooth at all?"

**Goal of this stage:** Confirm that the HM-10 is broadcasting itself over Bluetooth and the Mac can detect it. This is the "wireless side" sanity check, separate from any Python or sketches.

**No code is written or run in this stage.** Pure point-and-click.

#### What you do

1. Power on the ESP board (which powers the HM-10). Confirm the HM-10's LED is blinking.
2. On Mac or iPhone: **install LightBlue** (free BLE scanner app, App Store).
3. Open LightBlue. It will start scanning for nearby BLE devices.
4. You should see a device named something like **"HMSoft"** or **"HM-10"** in the list. (Default name — can be changed later.)
5. Tap/click it. LightBlue connects to the HM-10. Some details show up (services, characteristics — those are BLE jargon you don't need to learn yet).
6. Look for a writable characteristic, send "hello" through it, and watch what comes back.

#### How you know it worked
- LightBlue lists the HM-10 → HM-10's BLE side works.
- LightBlue can connect to it → no pairing/auth issues.
- You can write "hello" and see it echoed back via the ESP board's serial monitor → **the entire pipeline works in test mode**.

#### If it doesn't work
- **LightBlue doesn't list the HM-10** → HM-10 not advertising. Check power again. Try moving phone/Mac closer.
- **Connects but writes don't reach the ESP** → wires are wrong or the sketch from Stage B isn't running. Re-flash.

---

### Stage D — "Can my Python script on the Mac talk to the ESP wirelessly?"

**Goal of this stage:** This is the actual Link 3 milestone. **A Python script on the Mac sends "PING" to the robot wirelessly. The robot replies "PONG". The Mac sees the reply.**

If this works, Link 3 is done in principle. Everything else we build later is "send different words through this same pipe."

#### What you do

1. In your project's `.venv`, run `pip install bleak`. Then `pip freeze > requirements.txt` to lock the new dependency.
2. **Write a small Python script** (`ble_smoketest.py`) that:
   - Scans for the HM-10 by name,
   - Connects to it,
   - Sends `PING\n`,
   - Waits up to a few seconds for a reply,
   - Prints whatever comes back.
3. **Update the ESP sketch** so that when it sees the text `PING` arrive over the HM-10, it replies with `PONG`. (Otherwise it just keeps echoing — also fine for testing.)
4. Run the Python script with the ESP board powered on.

#### How you know it worked
The terminal shows:
```
Connecting to HM-10...
Connected.
> PING
< PONG
```

That's it. **Wireless bidirectional communication proven.**

#### If it doesn't work
- **Python script can't find HM-10** → make sure the HM-10 isn't already connected to LightBlue (BLE devices typically allow one connection at a time).
- **Connects but no reply** → the ESP sketch's PING/PONG logic is wrong, or the line ending is off. We debug live.

---

## 7. After Stage D — what comes next

Once PING/PONG works, the rest of WP4 becomes much easier:

1. **Define the real command set** with the team (e.g., `MOVE F 100\n`, `TURN L 45\n`, `GRIP CLOSE\n`). I have a draft proposal.
2. **Wrap the BLE-talking code in a clean class** (`robot.py`) so the rest of the system uses `robot.send("MOVE_F", 100)` and doesn't worry about Bluetooth.
3. **Write the bridge** (`bridge.py`, Link 2): vision detections in → robot commands out.
4. **Hook up actual motors and servo** to the ESP board (this is more WP5 territory but I'll be involved).

These are *next-document* problems. Don't worry about them now. Get PING/PONG working first.

---

## 8. Decision log — choices made in this plan and why

| Decision | Choice | Why |
|---|---|---|
| Communication library on Mac | `bleak` | Pure Python, cross-platform, the proposal already specified it. |
| Pins on ESP for software UART | D5 (GPIO14) = TX, D6 (GPIO12) = RX | Safe boot pins, no special role at startup, common pair in tutorials. |
| Power for HM-10 | 3V3 pin of NodeMCU | Both chips run on 3.3V — direct connection, no level shifter needed. |
| Test message | `PING` / `PONG` | Human-readable, easy to debug, costs nothing. |
| Test order | Local USB blink → wires only → BLE scanner → Python | Each stage isolates one failure mode. Saves hours when something goes wrong. |
| MCU choice | ESP8266 NodeMCU (already bought) | Team already purchased — we work with what we have, even though proposal mentioned STM32/Arduino. |

---

## 9. Open questions still pending

These do not block any of the four stages above — but the team needs to answer them before we move past Stage D:

1. **Is ESP8266 the final MCU?** The proposal mentions STM32 / Arduino. If the team intended one of those, we redo the firmware part later (the wiring table changes, but the Mac side stays identical).
2. **Command protocol — ASCII vs binary?** WP4 (you and Mert Onur) own this decision.
3. **WP3↔WP4 data contract** — how vision passes detections to the bridge. Negotiated with Öykü and Serkan.

---

## 10. Reference links (sources used while writing this plan)

- [NodeMCU ESP8266 Pinout — components101](https://components101.com/development-boards/nodemcu-esp8266-pinout-features-and-datasheet)
- [ESP8266 GPIO Reference — Random Nerd Tutorials](https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/)
- [NodeMCU v3 high-resolution pinout — Mischianti](https://mischianti.org/nodemcu-v3-high-resolution-pinout-and-specs/)
- [HM-10 BLE Module Pinout — components101](https://components101.com/wireless/hm-10-bluetooth-module)
- [HM-10 Datasheet (Cornell)](https://people.ece.cornell.edu/land/courses/ece4760/PIC32/uart/HM10/DSD%20TECH%20HM-10%20datasheet.pdf)
- [HM-10 Tutorial — Martyn Currey](https://www.martyncurrey.com/hm-10-bluetooth-4ble-modules/)
- [bleak (Python BLE library)](https://github.com/hbldh/bleak)

---

*End of plan. Next update will be after Stage A is complete.*
