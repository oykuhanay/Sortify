# Web Dashboard for Sortify

## System Prompt (for the subagent)

You are a senior software engineer working on the Sortify robot project at `/Users/serhatuludag/Desktop/CENG424/Project/sortify-comm`. The project's WP4 owner (Serhat) integrates vision (YOLO), ArUco markers, A* path planning, BLE robot driver, and an ESP8266-based robot. Today's task: design and implement a localhost web dashboard.

**Strict constraints — do not violate these:**

1. **Do not break the existing flow.** `main.py` must keep working stand-alone (camera + bridge + BLE) even if the dashboard isn't running. The dashboard is OPT-IN.
2. **No new heavy dependencies.** Use Python stdlib HTTP server + a single HTML file + plain JS+fetch. No Flask, no FastAPI, no React, no build step. The constraint is "demo-installable on a fresh Mac in under 60 seconds".
3. **No threading hell.** The dashboard must run cleanly alongside the OpenCV main loop. Either run it in a background daemon thread that ONLY reads/writes shared state, or as a separate process that talks to `main.py` over a tiny file/socket protocol. Prefer threads — simpler.
4. **MJPEG streaming for video.** Don't try WebRTC. A simple multipart/x-mixed-replace MJPEG endpoint is what every dashboard like this uses, and it Just Works in a browser `<img>` tag.
5. **All state changes go through the bridge.** The dashboard sends commands like "set target color = red" or "set tunables: gripper_forward=21"; the bridge / main.py state mutate. The dashboard is a thin view+controller, no business logic in JS.

## What the Dashboard Must Do

### Top of page — Live status strip
- Current bridge state (AWAITING_START, SEEKING_BLOCK, etc.)
- Current target colour
- Most recent command sent to the robot (e.g. "TURN +010")
- Robot connected? Last heartbeat? (red/green dot)
- Frame timestamp / FPS

### Main panel — Live MJPEG video
The same frame OpenCV is rendering (with cyan ring, theta arrow, paths, etc.) streamed to the browser at maybe 10-15 FPS. Browser renders it in an `<img>` with auto-refresh via `multipart/x-mixed-replace; boundary=frame`.

### Left panel — Flow control
- START button (sends SPACE-equivalent)
- RESET button (sends R-equivalent)
- STOP button (sends emergency STOP)
- Target colour selector: dropdown red/blue/green/auto (auto = current priority logic)

### Right panel — Live tune
Sliders + number inputs, mirrors the WASD keys but lets non-keyboard users tune:
- GRIPPER_FORWARD_CM (slider 0–40 cm, step 0.5)
- GRIPPER_RIGHT_CM (slider -10 to +10 cm, step 0.5)
- PARALLAX_FACTOR (slider 0–0.5, step 0.005)
- CAMERA_NADIR_CM (two number inputs, x and y)
- "Save tunables.json" button
- "Reset to defaults" button

### Bottom panel — Manual command box
A text input + Send button. Typed text goes raw to the BLE robot via `_send_to_robot`. Useful for `GRIP O`, `MOVE +5.00`, etc.

### Command-style sentences
Above the tune panel, a small "macro" area:
- Buttons like "Move red cube to red field", "Move blue cube to blue field", "Sort everything" — these set the bridge's `target_color` and start the flow.

## API Design

Run the HTTP server on `127.0.0.1:8080`. Routes:

- `GET /` → HTML page (single file, see below)
- `GET /stream.mjpg` → MJPEG live stream
- `GET /status.json` → current bridge state + target + last command + tunables + robot connected
- `POST /control` → JSON body: `{"action": "start"}` / `{"action": "reset"}` / `{"action": "stop"}` / `{"action": "set_target", "color": "red"}`
- `POST /tune` → JSON body: any subset of `{"gripper_forward_cm": ..., "gripper_right_cm": ..., "parallax_factor": ..., "camera_nadir_cm": [x, y]}`. Merges into state.
- `POST /save_tunables` → triggers `_save_tunables(state)`
- `POST /command` → raw command passthrough: `{"command": "GRIP O"}` → calls `_send_to_robot`

All responses are JSON except `/` and `/stream.mjpg`.

## Implementation Plan

1. **`web_dashboard.py`** — new file. Defines:
   - `start_dashboard(state, bridge, send_command)` — launches the HTTP server in a daemon thread, takes references to the shared state dict, bridge, and the send function.
   - Internally uses `http.server.ThreadingHTTPServer` from stdlib.
   - The MJPEG endpoint reads the latest frame from a `state["latest_jpeg"]` byte buffer that `_process_frame` writes after each frame. Use a lock around it.
2. **`main.py`**:
   - Import the dashboard and call `start_dashboard(state, _bridge, _send_to_robot)` after `_start_robot_thread()`. Wrap in try/except so if the dashboard fails it doesn't kill the demo.
   - At the end of `_process_frame`, encode the rendered frame as JPEG (~q=70) and stash it in `state["latest_jpeg"]`. Don't do this if no dashboard is connected (track a `state["dashboard_subscribers"]` counter to skip the JPEG encode when nobody's watching).
3. **`dashboard.html`** — single file embedded as a Python string inside `web_dashboard.py`, or served from disk. Plain HTML, plain JS, polled `fetch("/status.json")` every 250 ms, sliders POST to `/tune` on input.
4. **Smoke test**: open `http://127.0.0.1:8080`, see the live feed, push a slider, watch the cyan ring move on both the OpenCV window AND the dashboard.

## Layout Sketch

```
+---------------------------------------------------------------+
| state: SEEKING_BLOCK   target: red   > TURN +010   robot ●    |
+---------------------------------------------------------------+
|                                                               |
|                 [   live MJPEG video here   ]                 |
|                                                               |
+-----------------+-------------------------+-------------------+
| Flow            | Macros                  | Tune              |
|  [START]        |  [Red cube → Red field] |  fwd ____ 20.0    |
|  [RESET]        |  [Blue cube → Blue]     |  rt  ____  0.0    |
|  [STOP]         |  [Sort everything]      |  parallax ___ 0.18|
|  target: red ▾  |                         |  nadir [50,35]    |
|                 |                         |  [Save] [Defaults]|
+-----------------+-------------------------+-------------------+
| Manual command: [GRIP O           ] [Send]                    |
+---------------------------------------------------------------+
```

## Deliverables

- `web_dashboard.py` — server + embedded HTML
- `main.py` — `start_dashboard` call + JPEG-stashing in `_process_frame`
- Update `HOW_TO.md` with: "Open `http://127.0.0.1:8080` once `main.py` is running."

## Done When

1. `python3 main.py` still works without the dashboard if anything HTTP fails.
2. Browser shows live video and state.
3. Sliders mutate the same state the WASD keys do.
4. Buttons trigger the same bridge transitions SPACE/R do.
5. Closing the browser doesn't crash anything; reopening reconnects.

## Status

- [x] Done
