# Sortify WP4 — Learning Log

> **Purpose of this file:** A running record of what I (Serhat) built, why,
> and what I learned. Written so that I — or any future collaborator (human
> or AI) — can understand the project state and pick up where I left off
> without re-asking the basics.

---

## Project context (so anyone reading this knows what the project is)

- **Course:** CENG424 Embedded Computer Systems, IYTE, Spring 2026.
- **Project name:** Sortify — a low-cost autonomous color-sorting robot.
- **Architecture (centralized vision):**
  1. An overhead **smartphone camera** films a workspace.
  2. The video goes to a **PC**, which runs computer vision (OpenCV + YOLO).
  3. The PC computes commands and sends them to a **robot** over **Bluetooth**.
  4. The robot (STM32 or Arduino + DC motors + servo gripper) executes them.
- **Team (5 people, 7 work packages — WP1..WP7):**
  - Atakan SERÇE — WP2 (3D / mechanical), WP5 (robotics integration).
  - **Serhat ULUDAĞ (me)** — **WP4: Communication & Control Infrastructure** (with Mert Onur).
  - Öykü HANAY — WP3 (AI / vision).
  - Serkan AYDOĞDU — WP3 (AI / vision).
  - Mert Onur SELÇUKLU — WP4 (with me), WP2, WP5.

### My slice (WP4) in plain words

I am the **glue between three other groups**. I own three communication links:

```
[Phone camera] ──link 1──▶ [PC] ──link 2──▶ [PC bridge logic] ──link 3──▶ [Robot MCU]
                vision team's domain          MY BRIDGE                     robotics team's domain
                                                ↑↑↑
                                          this is WP4
```

- **Link 1 — Phone → PC video.** Get the phone's camera into Python on the PC.
- **Link 2 — Vision output → robot command.** Take "red object at (x,y)" from
  the vision team and turn it into "FORWARD 200ms" / "GRIP_CLOSE" etc.
- **Link 3 — PC → Robot over Bluetooth.** Send those commands as bytes to
  the robot's microcontroller.

The metric I'll be graded on most directly is **Communication Latency**.

---

## Session 1 — 2026-05-01 — Link 1 done

### What I built

| File | What it is | Permanent? |
| --- | --- | --- |
| `camera_smoketest.py` | A diagnostic script that proves the phone→PC video link works. Shows live video + measured FPS. | No — throwaway tool. Keep for now in case the link breaks. |
| `camera.py` | **The real WP4 deliverable.** A reusable Python module the vision team imports. Hides Iriun + OpenCV details, exposes a single `Camera` class with `get_frame()`. | Yes — this ships. |
| `demo_vision_consumer.py` | An example showing the vision team how to use `camera.py`. Documentation in code form. | No — scaffolding. Hand it to the vision team along with `camera.py`. |

Folder layout right now:

```
sortify-comm/
├── .python-version             ← pyenv pin: 3.12.7
├── .venv/                      ← virtual environment
├── camera.py                   ← the product
├── camera_smoketest.py         ← diagnostic
├── demo_vision_consumer.py     ← example
└── LEARNING_LOG.md             ← this file
```

### The setup chain (so I can rebuild this on another machine)

1. **Hardware:** iPhone connected to MacBook with a **USB-C ↔ Lightning** cable.
   USB chosen for development because eduroam (university Wi-Fi) blocks
   device-to-device discovery (mDNS/Bonjour), and the iPhone hotspot trick
   didn't work either due to an iOS quirk where the phone doesn't expose its
   own apps to USB-tethered clients.
2. **Software stack:**
   - Iriun Webcam app on iPhone (App Store).
   - Iriun Webcam desktop app on Mac (https://iriun.com).
   - Both must be running. Iriun Mac app turns the iPhone's camera into a
     **macOS virtual webcam** that any app (including OpenCV) can read.
3. **Python:**
   - System Python is 3.14 (too new for some libraries — OpenCV doesn't
     guarantee wheels for it yet).
   - I used **pyenv** to install **Python 3.12.7** locally for this folder
     (`pyenv local 3.12.7`).
   - Created a venv: `python3 -m venv .venv && source .venv/bin/activate`.
   - Installed: `pip install opencv-python` (pulls in NumPy too).
4. **Version control:**
   - `git init` in this folder, first commit, then `gh repo create sortify-comm --private --source=. --push`.
   - Repo: https://github.com/SerhatUludag32/sortify-comm (private).
   - Default branch: `master` (kept on purpose — don't rename to `main`).

### Dependency management — `requirements.txt` and `pip freeze`

**What it is:** `requirements.txt` is a list of every Python package this
project needs, with **exact version numbers** (e.g. `opencv-python==4.13.0.92`).
It's the "recipe" for the environment.

**Why it exists:** so anyone (teammate, future-me on a new laptop, the
grader) can recreate my exact environment with one command:
```bash
pip install -r requirements.txt
```
Without this file they'd have to guess which packages and versions I used,
and would likely install newer versions that behave differently — the
classic "works on my machine" trap.

**How to generate / update it:** `pip freeze` prints all installed packages
with versions. Redirect that into the file:
```bash
pip freeze > requirements.txt
```

**⚠️ Important — this file does NOT auto-update.** Every time I `pip install`
something new, the file becomes stale. The discipline:
```bash
pip install <new-package>
pip freeze > requirements.txt   # regenerate
git add requirements.txt
git commit -m "Add <new-package> dependency"
git push
```
If I forget the `pip freeze` step, my code uses the new package but
teammates won't get it when they `pip install -r requirements.txt`, and
their code will crash with `ImportError`. Easy mistake; remember it.

**When this will matter for WP4:** when I add `pyserial` (for the Bluetooth
serial link in link 3), `bleak` (if we go BLE), or anything else, regenerate
`requirements.txt`.

### How I run things now

```bash
cd ~/Desktop/CENG424/Project/sortify-comm
source .venv/bin/activate

# Sanity check the camera works:
python3 camera_smoketest.py

# Run the example consumer (shows what vision team's code will look like):
python3 demo_vision_consumer.py
```

Press **`q` while the video window has focus** to quit. (If `q` doesn't work,
you probably have Terminal focused — click the video window first.)

### What the camera_smoketest.py file does

A throwaway diagnostic. Opens camera index 1 (Iriun), reads frames in a tight
loop, draws an FPS overlay, shows the window. **Only used to confirm "is the
camera link working at all?"** Once `camera.py` works, this script's job is
done. I keep it because it's useful for debugging if the link breaks later.

### What the camera.py file does (THE IMPORTANT ONE)

This is the actual WP4 deliverable. It's an **interface** — a clean wall
between "how we get pictures from the phone" and "what we do with them."

**Public API the vision team uses:**

```python
from camera import Camera

with Camera() as cam:
    frame = cam.get_frame()   # numpy array, BGR, shape (H, W, 3)
    # ... vision team runs YOLO/OpenCV detection on `frame` ...
```

That is the whole surface. They never touch `cv2.VideoCapture`, never know
about Iriun, never deal with camera indices. If we later swap the phone for
a different camera, only this file changes — their code doesn't.

**Internals (the design decisions worth defending):**

- **Background reader thread.** OpenCV's `VideoCapture.read()` has an
  internal buffer. If the consumer is slow (e.g. AI takes 200 ms per frame),
  the buffer fills with old frames and `read()` returns increasingly stale
  ones. For robotics this is *bad* — you'd command the robot based on
  where the ball *used* to be. My fix: a daemon thread reads frames as fast
  as the camera produces them and always overwrites a single "latest frame"
  slot. `get_frame()` returns whatever is current — always <33 ms old at
  30 FPS. **Rule of thumb in robotics: fresh data > queued data, always.**
- **Lock around `_latest`.** Reader thread writes, consumer reads — that's
  a race. The lock makes it safe.
- **`.copy()` in `get_frame()`.** Returns a copy so the consumer can modify
  their frame (draw boxes, scale it) without the reader overwriting it
  underneath them.
- **Context manager (`__enter__` / `__exit__`).** Lets you write
  `with Camera() as cam:`. Guarantees `close()` runs even on exceptions.
  Without this, a crashed program leaves the camera locked until Iriun
  restarts.
- **Open-timeout in `__init__`.** Fails fast (3 s) if the camera opens but
  never produces frames. Better than hanging forever.
- **Daemon thread.** Dies automatically when the main program exits. Ctrl+C
  doesn't leave a zombie reader thread running.

**Things deliberately NOT implemented yet (premature otherwise):**

- No frame timestamps. (Add when closed-loop control needs to reject stale
  frames.)
- No reconnect logic if Iriun drops. (Add once I know the real failure
  modes.)
- Camera index hardcoded to 1. (Make configurable later.)

### What demo_vision_consumer.py does

A reference for the vision team. It uses `camera.py` exactly the way they
should. Where their YOLO call would go, there's just a comment. It's
"documentation in code form" — when I hand off to Öykü/Serkan I send them
`camera.py` and tell them "look at `demo_vision_consumer.py` to see the
pattern."

### Numbers I measured

- Capture resolution: **1920×1080**.
- Throughput on the consumer side (reading via `camera.py`): **~50 FPS**
  sustained. Way more than the project needs (the robot moves slowly enough
  that even 10 FPS would be fine). Big latency budget for layers 2 and 3.

### Gotchas I hit (so I/anyone don't redo them)

- **eduroam blocks Wi-Fi discovery between phone and laptop.** Don't waste
  time. Use USB during development.
- **iPhone hotspot to Mac doesn't work for Iriun either** — the phone
  doesn't expose its own services to USB-tethered Macs.
- **macOS lets only one app hold the camera at a time.** Quit Photo Booth
  / FaceTime / Zoom before running Python, or `cv2.VideoCapture` fails.
- **First Python camera open triggers a macOS permission prompt for
  Terminal.** Click Allow. If denied: System Settings → Privacy & Security
  → Camera → enable Terminal.
- **`q` only quits the OpenCV window when the window has focus**, not when
  Terminal does. This is a `cv2.waitKey` thing, not a bug.
- **The smoke test reported 30 FPS, the consumer demo reported 50 FPS.** Not
  a contradiction — the smoke test was bottlenecked by `cv2.imshow` blocking
  its read loop. The push-model camera reads in a background thread, so it
  actually captures at the camera's true rate. (This is exactly why the
  push model wins.)
- **Python 3.14 is too new** for guaranteed OpenCV wheels — pyenv-pinned to
  3.12.7 instead.

---

## What WP4 looks like overall (mental map)

```
┌──────────────────────────────────────────────────────────────────────────┐
│ camera.py        ← layer 1: phone → frames in Python.   ✅ DONE          │
│                    Hands frames to vision team.                          │
│                                                                          │
│ (vision team)    ← layer 2: frames → detections (red ball at x,y).       │
│                    Not my job. Öykü + Serkan.                            │
│                                                                          │
│ bridge.py        ← layer 3: detections → robot commands.   🔜 NEXT       │
│                    e.g. (robot at (100,200), ball at (300,400))          │
│                          → "TURN_LEFT 30°" then "FORWARD 200ms".         │
│                                                                          │
│ robot.py         ← layer 4: commands → Bluetooth bytes.                  │
│                    Sends "TURN_LEFT 30°" over BT to robot MCU.           │
└──────────────────────────────────────────────────────────────────────────┘
```

Layers 3 and 4 are the heart of WP4 and the bulk of the grade. Layer 1 was
the warm-up.

---

## Open decisions I still have to make (with the team)

These were flagged early; I haven't picked yet, and they cascade into how
layers 3 and 4 get built.

1. **HC-05 vs HM-10 Bluetooth module.**
   - HC-05 = classic Bluetooth, behaves like a serial cable. Trivial in
     Python (`pyserial`, ~5 lines). My recommendation for a first project.
   - HM-10 = BLE (Bluetooth Low Energy). Modern but harder — async,
     `bleak`, GATT characteristics, more code.
2. **STM32 vs Arduino on the robot side.** Arduino UNO + HC-05 = the
   absolute path of least resistance, lots of tutorials. STM32 is more
   capable but Mert/Atakan write that firmware, not me — their call.
3. **Open-loop vs closed-loop control.**
   - Open-loop: PC says "go forward 30 cm," robot does it blindly. Simple,
     drift accumulates.
   - Closed-loop: PC watches robot in camera, sends small corrections every
     ~100 ms. Robust but tight latency budget, harder protocol. Proposal
     hints at closed-loop ("If the robot deviates from the planned path,
     corrective commands are sent" — §4.10).
4. **Command protocol shape.** ASCII (`"FWD,200\n"`) vs binary
   (`0x01 0xC8`). I lean ASCII — easy to debug (you can literally type
   commands into a serial terminal to test the robot).
5. **WP3↔WP4 interface contract.** What exactly does the vision team hand
   me each frame? List of `(color, x, y)` tuples? Robot pose
   `(x, y, theta)`? Pixels or cm? At what rate? **This is a conversation I
   still need to have with Öykü and Serkan** — until it's pinned down, I
   can't fully design `bridge.py`.

---

## Session 2 — 2026-06-12 — Link 3 PING/PONG works wirelessly (Stages A–D)

This session followed `LINK3_BLUETOOTH_PLAN.md` end-to-end and proved
Mac ↔ HM-10 ↔ ESP8266 round-trip wirelessly. **All four stages green.**

### Hardware in hand

- ESP8266 NodeMCU **V3 32-pin (Lolin variant)** — wider board than the
  standard 30-pin NodeMCU. Important: **this variant does not have D6, D7,
  D8 labels.** It has D0, D2, D4, D5, D9, D10, D12, D13, D14, D15, D16, plus
  TX/RX/EN/MOSI/SCLK/MISO on the other rail. Any tutorial that says
  "use D5 and D6" needs translation on this board.
- HM-10 module labelled `EN VCC GND TXD RXD STATE` (6 pins, used 4).
  Default name advertised as **`BT05`**, not `HMSoft` → this is a CC2541
  clone, functionally identical to HM-10. BLE varsayılan baud **9600**.
- 60-row full-size breadboard, male-male + female-female jumper kit.

### Final wiring (the one that worked)

```
HM-10 pin    →    NodeMCU pin           Notes
─────────         ─────────────         ─────────────────────────────
VCC          →    3.3V                  power
GND          →    GND                   ground
RXD          →    TX  (GPIO1)           left rail, just under EN
TXD          →    RX  (GPIO3)           left rail, just under TX
STATE, EN    →    not connected
```

**This uses the ESP's hardware UART0** — the same UART the USB serial bridge
uses. That's why uploads need the HM-10 data lines unplugged (see gotchas).

### Stage-by-stage what happened

**Stage A — ESP alive (Blink).** Smooth once two paper cuts cleared:
1. Arduino IDE didn't have ESP8266 board support → added URL
   `https://arduino.esp8266.com/stable/package_esp8266com_index.json` in
   Preferences, installed `esp8266 by ESP8266 Community 3.1.2` via Boards
   Manager.
2. First USB cable was charge-only — board powered but `ls /dev/cu.*` showed
   no new port. Swapped to a data-capable Micro-USB cable, `/dev/cu.usbserial-110`
   appeared immediately. No CH340 driver install needed for this clone.
3. Board name in IDE: **"NodeMCU 1.0 (ESP-12E Module)"** works fine even
   though our board is V3 — no V3 entry exists, and "1.0" is compatible.

**Stage B — HM-10 wired to ESP.** Wired all 4 lines on the breadboard,
plugged USB, all three LEDs correct (NodeMCU red + blue blink, HM-10 red
fast blink = advertising). No heat, no smell.

**Stage C — Mac sees HM-10 over BLE.** Installed **LightBlue** from Mac App
Store. Many "Unnamed" devices appeared. Sorted by signal — strongest
"Unnamed" turned out to be **BT05** (not HMSoft as the plan assumed).
Connected, saw service **`FFE0`** with characteristic **`FFE1`**
(Readable + Writeable + Supports Notification). Cached BT05 UUID:
`6902B06B-95D5-9A7F-B00F-8D86713AB08D` — note: **macOS-local UUID, not
portable across machines**; Python `bleak` will discover its own.

**Stage D — Bidirectional PING/PONG.** Took several debug rounds before it
worked. See the gotchas section below — that's where the real lessons are.
Final result: Mac wrote `0x50494E470A` ("PING\n") to FFE1, ESP replied
`0x504F4E47` ("PONG") within 70 ms. **Stages A–D all green.**

### Numbers measured

- BLE round-trip latency (Mac → ESP → Mac, full PING/PONG): **~70 ms**
  per the LightBlue timestamps (21:13:59.373 sent → 21:13:59.443 received).
  Plenty of headroom for ~10 Hz closed-loop robot control.

### Gotchas I burned hours on (so future-me skips them)

**Hours sunk on SoftwareSerial that turned out to be a dead end.** The plan
specified D5 (GPIO14) = TX, D6 (GPIO12) = RX via `SoftwareSerial`. On this
specific board:
- D6 is not even labelled on the V3 32-pin variant. Substituted D2 (GPIO4)
  as RX, kept D5 as TX.
- With `SoftwareSerial bt(4, 14)`, ESP transmitted nothing detectable to
  HM-10. Confirmed by LightBlue subscribing to FFE1 and seeing only the
  factory default `0x0102030405000000...` value on Read — meaning HM-10
  had received nothing over UART to relay.
- Tried `SoftwareSerial(13, 14)` (RX=D7=GPIO13) — same dead silence.
- Tried baud rates 9600, 19200, 38400, 57600, 115200 — none worked.

**The diagnostic that cracked it: HM-10 loopback.** Unplugged HM-10's TXD
and RXD from the ESP, jumpered them directly to each other on the module.
Wrote `0x54455354` ("TEST") to FFE1 from LightBlue → got `0x54455354` back
on Notify within 80 ms. **This proved the HM-10 module + BLE side were
perfect**, so the failure had to be on the ESP-side UART or wiring.

**Switching to hardware UART (Serial / GPIO1+GPIO3) fixed it instantly.**
Wired HM-10 RXD → ESP TX pin (left rail, under EN). One-shot test sketch
that did `Serial.print("HELLO")` every second → LightBlue immediately
showed `0x48454C4C4F` on Notify. Added HM-10 TXD → ESP RX, full PING/PONG
worked on the next upload.

**Why probably?** SoftwareSerial on ESP8266 is finicky — it's bit-banged
under interrupts and the WiFi stack steals timing. Hardware UART is
rock-solid. **Default to hardware UART for any future ESP8266 ↔ peripheral
UART work.** Only reach for SoftwareSerial if a second UART is genuinely
needed.

**Upload requires temporarily disconnecting HM-10 from TX/RX.** Since the
HM-10 sits on GPIO1/GPIO3 — the same pins the USB bootloader uses to flash
the chip — upload fails with `Failed to connect to ESP8266: Timed out
waiting for packet header` when HM-10 data lines are connected. Workflow:
1. Unplug HM-10 RXD and TXD jumpers (leave VCC/GND).
2. Upload.
3. Replug HM-10 RXD/TXD.
4. Reset or power-cycle.
Tedious but unavoidable on this board (no boot mode switch).

**Serial Monitor must be closed before Upload.** Otherwise the IDE can't
grab the port — upload hangs or fails. Open Serial Monitor only after
upload finishes.

**Boot-time garbage on Serial Monitor is normal.** The ESP ROM bootloader
prints at 74880 baud before the sketch's `Serial.begin(9600)` kicks in.
Looks like `{$l��|�d�...` for the first ~1 second after reset. Ignore it,
wait for the next clean line.

**Iriun virtual webcam preview goes black while Python script runs.**
Not a bug — macOS only lets one app hold the camera. The script has it,
Iriun's own preview can't draw frames. Quitting the script frees it back.
Already noted in Session 1 but worth repeating: **this is system working
correctly, not broken**.

### ESP sketch we ended on (committed to `esp_firmware/pingpong/pingpong.ino`)

```cpp
String buf = "";

void setup() {
  Serial.begin(9600);
  delay(2000);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.indexOf("PING") >= 0) {
        Serial.print("PONG");
      }
      buf = "";
    } else {
      buf += c;
      if (buf.length() > 32) buf = "";
    }
  }
}
```

Note: uses `Serial` (hardware UART0). Same pins as USB serial — so any
debug `Serial.println(...)` would also leak out to the HM-10. Kept the
sketch quiet for that reason. Next iteration: use `Serial1.println(...)`
(hardware UART1, TX-only on GPIO2/D4) for debug, leaving UART0 clean for
HM-10 traffic.

### What I handed off to the vision team this session

Pushed the camera module to Öykü's repo (`oykuhanay/Sortify`) under
`camera/`: `camera.py`, `demo_vision_consumer.py`, `requirements.txt`,
and a Turkish `README.md` covering install, usage, design rationale, and
known gotchas. Commit `4d3eea9` on main. WP4 → WP3 handoff for Link 1 is
formally done.

### Then we ported PING/PONG off LightBlue into Python — same session

LightBlue had only proved the wireless pipe by hand. To make Link 3 an
actual deliverable, wrote `ble_smoketest.py` with `bleak`:

- Scans for device name `BT05` (no hardcoded UUID — `bleak` discovers
  the macOS-local address fresh each run).
- Connects, subscribes to notifications on characteristic `FFE1`, writes
  `PING\n`, waits up to 3 s for a notification containing `PONG`.
- Exits 0 on success, prints the reply line.

`pip install bleak` (3.0.2) — pulls in `pyobjc-core`,
`pyobjc-framework-corebluetooth`, etc. on macOS. `pip freeze >
requirements.txt`.

First run on the live rig output exactly:

```
Scanning for 'BT05' (timeout 8s)...
Found BT05 at 6902B06B-95D5-9A7F-B00F-8D86713AB08D
Connected.
> PING
< PONG
OK — Link 3 round-trip works.
```

That's Link 3 done programmatically. Same hardware as the LightBlue test,
same ESP sketch, but now driven by code we own — the actual deliverable.

**One small gotcha during this:** LightBlue must be fully quit (Cmd+Q)
before running the Python test. macOS BLE only allows one app at a time
to hold a connection to a peripheral; if LightBlue still held BT05,
`bleak`'s `BleakClient.__aenter__` would have failed or hung.
**Symptom to watch for:** the HM-10's red LED stays solid instead of
blinking — solid = still connected to something. Blinking = advertising,
ready for Python.

---

## Next session — wrap BLE in a robot.py class, design the protocol

LightBlue proved the wireless pipe. Now move PING/PONG into Python with
`bleak` so the project actually has a programmatic Link 3 deliverable.

1. `pip install bleak`, `pip freeze > requirements.txt`, commit.
2. Write `ble_smoketest.py`:
   - Scan for device named `BT05` (do NOT hardcode the
     `6902B06B-...` UUID — that's macOS-local, fresh discovery each run).
   - Connect, write `PING\n` to characteristic `FFE1`, subscribe to
     notifications on same characteristic, print whatever comes back.
   - Disconnect cleanly.
3. Same upload-dance reminder: unplug HM-10 TX/RX before reflashing the ESP,
   replug after.
4. Once `PING → PONG` works in Python, wrap it in a `robot.py` class:
   `robot.send("PING")` / `robot.on_message(callback)`. That's the public
   API the bridge will consume.
5. Then design the real command protocol with the team. My draft is still
   ASCII, line-based — `MOVE F 200\n`, `TURN L 45\n`, `GRIP CLOSE\n`.
   Pin shape with team before writing the parser.

When picking up next time, just say: *"let's do link 3 in Python"*.

---

## Glossary (for me, future-me, and any AI agent reading this cold)

- **Iriun Webcam:** a third-party app pair (phone + Mac) that turns an
  iPhone into a virtual webcam macOS can use. We use it because the proposal
  picked it and it's free + supports iPhone.
- **OpenCV (`cv2`):** the Python library that reads frames from cameras and
  provides image-processing primitives. `cv2.VideoCapture(index)` opens a
  camera; `cap.read()` returns one frame as a NumPy array.
- **NumPy array:** how OpenCV represents an image — a 3D grid of numbers
  shaped `(height, width, 3)` where the 3 is BGR color channels (note:
  Blue-Green-Red, not RGB — OpenCV is weird).
- **Push vs pull camera model:** see `camera.py` design notes above.
- **Daemon thread:** a Python background thread that doesn't block the
  program from exiting.
- **pyenv:** lets me have multiple Python versions installed and pick one
  per folder.
- **venv:** an isolated copy of Python's package directory for this project.
  Anything I `pip install` here doesn't pollute system Python.
- **WP / Work Package:** the proposal divides the project into 7 numbered
  tasks. WP4 is mine.
