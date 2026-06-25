"""Localhost web dashboard for Sortify.

A tiny stdlib-only HTTP server that lets you watch the OpenCV frame in a
browser, see the bridge's live state, tune offsets without leaning over
the keyboard, and fire raw BLE commands. Opt-in: if any of this fails
the demo keeps running.

Usage from main.py:

    import web_dashboard
    web_dashboard.start_dashboard(state, _bridge, _send_to_robot,
                                  save_tunables=lambda: _save_tunables(state),
                                  reset_tunables=lambda: _reset_tunables(state))

Design notes:

- ThreadingHTTPServer in a daemon thread. The handlers ONLY read/write
  the shared `state` dict and call the supplied callbacks; no business
  logic lives in here.
- MJPEG streaming reads `state["latest_jpeg"]` under a lock. main.py is
  responsible for stashing the encoded frame; we just shovel bytes.
- A `state["dashboard_subscribers"]` counter lets main.py skip the JPEG
  encode when nobody is connected.
- All POST endpoints expect JSON, all responses are JSON except `/` and
  `/stream.mjpg`.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional


# Module-level handles populated by start_dashboard so the request
# handlers can reach the shared state without per-instance plumbing.
_STATE: Optional[dict] = None
_BRIDGE = None
_SEND: Optional[Callable[[str], None]] = None
_SAVE_TUNABLES: Optional[Callable[[], None]] = None
_RESET_TUNABLES: Optional[Callable[[], None]] = None
_RECONNECT_ROBOT: Optional[Callable[[], str]] = None
_FRAME_LOCK = threading.Lock()


# ---- the HTML page ---------------------------------------------------

# Single self-contained HTML page. Plain JS + fetch; no build step, no
# external CDN. Polls /status.json every 250ms and POSTs slider changes
# debounced through the browser's `input` event.
_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Sortify Dashboard</title>
<style>
  :root {
    --bg: #0f1115;
    --panel: #181b22;
    --border: #2a2f3a;
    --fg: #e6e8ef;
    --muted: #8b93a7;
    --accent: #38bdf8;
    --ok: #22c55e;
    --bad: #ef4444;
    --warn: #f59e0b;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 0;
    background: var(--bg); color: var(--fg);
    font: 14px/1.4 ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif;
  }
  header {
    display: flex; align-items: center; gap: 18px;
    padding: 10px 16px; background: var(--panel);
    border-bottom: 1px solid var(--border);
    flex-wrap: wrap;
  }
  header .pill {
    background: #232735; border: 1px solid var(--border);
    padding: 4px 10px; border-radius: 999px;
    font-variant-numeric: tabular-nums;
  }
  header .pill b { color: var(--accent); margin-left: 4px; }
  .dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 6px; vertical-align: -1px; }
  .dot.ok { background: var(--ok); box-shadow: 0 0 8px var(--ok); }
  .dot.bad { background: var(--bad); box-shadow: 0 0 8px var(--bad); }
  .dot.warn { background: var(--warn); }

  main { display: grid; grid-template-columns: 1fr; gap: 12px; padding: 12px; }
  .video-wrap {
    background: #000; border: 1px solid var(--border); border-radius: 6px;
    display: flex; align-items: center; justify-content: center; min-height: 320px;
    overflow: hidden;
  }
  .video-wrap img { max-width: 100%; max-height: 70vh; display: block; }
  .grid {
    display: grid; gap: 12px;
    grid-template-columns: repeat(3, minmax(0, 1fr));
  }
  @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  .panel {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 6px; padding: 12px;
  }
  .panel h2 {
    margin: 0 0 10px; font-size: 13px; color: var(--muted);
    text-transform: uppercase; letter-spacing: .08em;
  }
  button, select, input[type=text] {
    background: #232735; color: var(--fg);
    border: 1px solid var(--border); border-radius: 4px;
    padding: 6px 10px; font: inherit;
  }
  button { cursor: pointer; }
  button:hover { border-color: var(--accent); }
  button.primary { background: #1f4d6b; border-color: #2e7aa6; }
  button.danger { background: #5a1d1d; border-color: #a23b3b; }
  .row { display: flex; gap: 8px; align-items: center; margin-bottom: 8px; flex-wrap: wrap; }
  .row label { color: var(--muted); min-width: 110px; }
  .row input[type=range] { flex: 1; min-width: 120px; }
  .row .val {
    font-variant-numeric: tabular-nums; min-width: 64px; text-align: right;
    color: var(--accent);
  }
  .cmdbox { display: flex; gap: 8px; }
  .cmdbox input { flex: 1; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
  .macros button { width: 100%; margin-bottom: 6px; text-align: left; }
  .log {
    font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    font-size: 12px; color: var(--muted);
    max-height: 100px; overflow: auto; white-space: pre-wrap;
    background: #11141a; border: 1px solid var(--border); border-radius: 4px;
    padding: 8px;
  }
</style>
</head>
<body>
  <header>
    <div class="pill">state <b id="s-state">-</b></div>
    <div class="pill">target <b id="s-target">-</b></div>
    <div class="pill">last <b id="s-cmd">-</b></div>
    <div class="pill"><span id="s-dot" class="dot bad"></span><b id="s-robot">robot ?</b></div>
    <button id="reconnect-btn" onclick="reconnect()" style="margin-left:6px">Reconnect BLE</button>
    <div class="pill">fps <b id="s-fps">-</b></div>
    <div class="pill" style="margin-left:auto">t <b id="s-time">-</b></div>
  </header>

  <main>
    <div class="video-wrap">
      <img id="video" src="/stream.mjpg" alt="live feed">
    </div>

    <div class="grid">
      <section class="panel">
        <h2>Flow</h2>
        <div class="row">
          <button class="primary" onclick="ctl('start')">START (space)</button>
          <button onclick="ctl('reset')">RESET (r)</button>
          <button class="danger" onclick="ctl('stop')">STOP</button>
        </div>
        <div class="row">
          <label>target colour</label>
          <select id="target-sel" onchange="setTarget(this.value)">
            <option value="auto">auto (priority)</option>
            <option value="red">red</option>
            <option value="blue">blue</option>
            <option value="green">green</option>
          </select>
        </div>
      </section>

      <section class="panel macros">
        <h2>Macros</h2>
        <button onclick="macro('red')">Move red cube to red field</button>
        <button onclick="macro('blue')">Move blue cube to blue field</button>
        <button onclick="macro('green')">Move green cube to green field</button>
        <button onclick="macro('auto')">Sort everything (auto)</button>
      </section>

      <section class="panel">
        <h2>Live Tune</h2>
        <div class="row">
          <label>fwd (cm)</label>
          <input id="t-fwd" type="range" min="0" max="40" step="0.5" oninput="tune('gripper_forward_cm', +this.value)">
          <span class="val" id="v-fwd">-</span>
        </div>
        <div class="row">
          <label>right (cm)</label>
          <input id="t-rt" type="range" min="-10" max="10" step="0.5" oninput="tune('gripper_right_cm', +this.value)">
          <span class="val" id="v-rt">-</span>
        </div>
        <div class="row">
          <label>parallax</label>
          <input id="t-par" type="range" min="0" max="0.5" step="0.005" oninput="tune('parallax_factor', +this.value)">
          <span class="val" id="v-par">-</span>
        </div>
        <div class="row">
          <label>nadir x,y (cm)</label>
          <input id="t-nx" type="text" style="width:60px" onchange="tuneNadir()">
          <input id="t-ny" type="text" style="width:60px" onchange="tuneNadir()">
        </div>
        <div class="row">
          <button onclick="saveTun()">Save tunables.json</button>
          <button onclick="resetTun()">Reset to defaults</button>
        </div>
      </section>
    </div>

    <section class="panel">
      <h2>Manual command</h2>
      <div class="cmdbox">
        <input id="cmd" type="text" placeholder="e.g. GRIP O, MOVE +5.00, TURN +010, STOP"
               onkeydown="if(event.key==='Enter')sendCmd()">
        <button class="primary" onclick="sendCmd()">Send</button>
      </div>
      <div style="margin-top:8px" class="log" id="log"></div>
    </section>
  </main>

<script>
  const $ = id => document.getElementById(id);
  const log = (msg) => {
    const el = $('log');
    const t = new Date().toLocaleTimeString();
    el.textContent = `[${t}] ${msg}\n` + el.textContent;
    if (el.textContent.length > 4000) el.textContent = el.textContent.slice(0, 4000);
  };

  let _editingTun = false;
  let _editTimer = null;
  const markEditing = () => {
    _editingTun = true;
    clearTimeout(_editTimer);
    _editTimer = setTimeout(() => _editingTun = false, 1500);
  };

  async function postJSON(url, body) {
    try {
      const r = await fetch(url, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body || {})
      });
      const data = await r.json().catch(() => ({}));
      if (!r.ok) log(`ERR ${url}: ${data.error || r.status}`);
      return data;
    } catch (e) {
      log(`ERR ${url}: ${e}`);
    }
  }

  function ctl(action) { postJSON('/control', {action}).then(() => log(`ctl ${action}`)); }
  function setTarget(color) {
    postJSON('/control', {action: 'set_target', color}).then(() => log(`target=${color}`));
  }
  function macro(color) {
    postJSON('/control', {action: 'set_target', color}).then(() => {
      postJSON('/control', {action: 'start'}).then(() => log(`macro ${color}`));
    });
  }
  function tune(key, value) {
    markEditing();
    postJSON('/tune', {[key]: value});
  }
  function tuneNadir() {
    markEditing();
    const x = parseFloat($('t-nx').value);
    const y = parseFloat($('t-ny').value);
    if (!isNaN(x) && !isNaN(y)) postJSON('/tune', {camera_nadir_cm: [x, y]});
  }
  function saveTun()  { postJSON('/save_tunables').then(() => log('tunables saved')); }
  function resetTun() { postJSON('/reset_tunables').then(() => log('tunables reset')); }
  async function reconnect() {
    // BLE pair-up takes a few seconds; disable the button + show a
    // working state so we don't fire two reconnects in parallel.
    const btn = $('reconnect-btn');
    if (!btn) return;
    btn.disabled = true;
    const oldText = btn.textContent;
    btn.textContent = 'Reconnecting...';
    try {
      const r = await postJSON('/reconnect_robot', {});
      log(`reconnect: ${r && r.status ? r.status : 'ok'}`);
    } catch (e) {
      log(`reconnect err: ${e}`);
    } finally {
      btn.disabled = false;
      btn.textContent = oldText;
    }
  }
  function sendCmd() {
    const v = $('cmd').value.trim();
    if (!v) return;
    postJSON('/command', {command: v}).then(() => log(`> ${v}`));
    $('cmd').value = '';
  }

  async function poll() {
    try {
      const r = await fetch('/status.json');
      const s = await r.json();
      $('s-state').textContent = s.state || '-';
      $('s-target').textContent = s.target_color || 'auto';
      $('s-cmd').textContent = s.last_command || '-';
      $('s-fps').textContent = (s.fps != null ? s.fps.toFixed(1) : '-');
      $('s-time').textContent = s.frame_age_s != null ? s.frame_age_s.toFixed(2) + 's' : '-';
      const robotOk = !!s.robot_connected;
      $('s-dot').className = 'dot ' + (robotOk ? 'ok' : 'bad');
      $('s-robot').textContent = robotOk ? 'robot connected' : 'robot offline';
      if (!_editingTun && s.tunables) {
        const t = s.tunables;
        $('t-fwd').value = t.gripper_forward_cm;  $('v-fwd').textContent = (+t.gripper_forward_cm).toFixed(1);
        $('t-rt').value  = t.gripper_right_cm;    $('v-rt').textContent  = (+t.gripper_right_cm).toFixed(1);
        $('t-par').value = t.parallax_factor;     $('v-par').textContent = (+t.parallax_factor).toFixed(3);
        if (document.activeElement !== $('t-nx')) $('t-nx').value = (+t.camera_nadir_cm[0]).toFixed(1);
        if (document.activeElement !== $('t-ny')) $('t-ny').value = (+t.camera_nadir_cm[1]).toFixed(1);
      }
      if (s.target_color && document.activeElement !== $('target-sel')) {
        $('target-sel').value = s.target_color;
      }
    } catch (e) { /* server might be reloading; ignore */ }
  }
  setInterval(poll, 250);
  poll();
</script>
</body>
</html>
"""


# ---- helpers --------------------------------------------------------

def _bump_subs(delta: int) -> None:
    """Track MJPEG viewers so main.py can skip the JPEG encode when
    nobody is watching. Cheap, just an int + the GIL."""
    if _STATE is None:
        return
    _STATE["dashboard_subscribers"] = max(0, _STATE.get("dashboard_subscribers", 0) + delta)


def _robot_connected() -> bool:
    """Reads the live BLE link status from shared state. main.py updates
    `state['robot_connected']` every frame by inspecting the underlying
    BleakClient, so this catches silent disconnects too (not just 'we
    haven't seen the Robot() object yet')."""
    if _STATE is None:
        return False
    return bool(_STATE.get("robot_connected"))


def _build_status() -> dict:
    """Snapshot the bits of state the dashboard renders."""
    s = _STATE or {}
    bridge_state = getattr(_BRIDGE, "state", None) if _BRIDGE else None
    target = getattr(_BRIDGE, "target_color", None) if _BRIDGE else None
    frame_ts = s.get("latest_jpeg_ts")
    frame_age = (time.monotonic() - frame_ts) if frame_ts else None
    return {
        "state": bridge_state,
        "target_color": target,
        "gripper_open": getattr(_BRIDGE, "gripper_open", None) if _BRIDGE else None,
        "last_command": s.get("last_command_str"),
        "fps": s.get("fps"),
        "frame_age_s": frame_age,
        "robot_connected": _robot_connected(),
        "tunables": {
            "gripper_forward_cm": s.get("tun_gripper_forward_cm"),
            "gripper_right_cm":   s.get("tun_gripper_right_cm"),
            "parallax_factor":    s.get("tun_parallax_factor"),
            "camera_nadir_cm":    list(s.get("tun_camera_nadir_cm", (0.0, 0.0))),
        },
    }


# ---- handler ---------------------------------------------------------

class _Handler(BaseHTTPRequestHandler):
    # Quiet the default access log — it screams over each MJPEG frame
    # and drowns the operator's own [WP4] prints.
    def log_message(self, fmt, *args):
        return

    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict:
        n = int(self.headers.get("Content-Length", "0") or 0)
        if n <= 0:
            return {}
        raw = self.rfile.read(n)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return {}

    # ---- GET ----
    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/" or path == "/index.html":
            body = _HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/status.json":
            try:
                self._send_json(_build_status())
            except Exception as e:
                self._send_json({"error": str(e)}, 500)
            return

        if path == "/stream.mjpg":
            self._stream_mjpeg()
            return

        self.send_error(404, "not found")

    # ---- POST ----
    def do_POST(self):
        path = self.path.split("?", 1)[0]
        body = self._read_json()
        try:
            if path == "/control":
                self._send_json(self._handle_control(body))
            elif path == "/tune":
                self._send_json(self._handle_tune(body))
            elif path == "/save_tunables":
                if _SAVE_TUNABLES:
                    _SAVE_TUNABLES()
                self._send_json({"ok": True})
            elif path == "/reset_tunables":
                if _RESET_TUNABLES:
                    _RESET_TUNABLES()
                self._send_json({"ok": True})
            elif path == "/reconnect_robot":
                # Force a fresh BLE pair-up. This blocks until the bleak
                # loop finishes (up to ~15 s) so the operator sees a real
                # status string instead of an optimistic "ok".
                if _RECONNECT_ROBOT is None:
                    self._send_json({"error": "reconnect not wired"}, 500)
                else:
                    status = _RECONNECT_ROBOT()
                    self._send_json({"ok": True, "status": status})
            elif path == "/command":
                raw = str(body.get("command", "")).strip()
                if not raw:
                    self._send_json({"error": "empty command"}, 400)
                    return
                # ESP firmware only knows uppercase verbs. The dashboard
                # is operator-facing, so accept any case and uppercase the
                # VERB while keeping any quoted text intact. Easiest: just
                # uppercase the first token (verb) and the GRIP letter.
                # For our wire protocol (TURN/MOVE/GRIP/STOP/TRIM/PING)
                # uppercasing the whole line is safe — there's no
                # case-sensitive payload.
                cmd = raw.upper()
                if _SEND:
                    _SEND(cmd)
                if _STATE is not None:
                    _STATE["last_command_str"] = cmd
                self._send_json({"ok": True, "sent": cmd})
            else:
                self.send_error(404, "not found")
        except Exception as e:
            self._send_json({"error": str(e)}, 500)

    def _handle_control(self, body: dict) -> dict:
        action = str(body.get("action", "")).lower()
        if action == "start":
            if _BRIDGE:
                _BRIDGE.start()
            return {"ok": True, "state": getattr(_BRIDGE, "state", None)}
        if action == "reset":
            if _BRIDGE:
                _BRIDGE.reset()
            return {"ok": True, "state": getattr(_BRIDGE, "state", None)}
        if action == "stop":
            # Emergency stop: tell the robot to cut motors AND drop the
            # bridge back to AWAITING_START so it doesn't immediately
            # issue another MOVE.
            if _SEND:
                _SEND("STOP")
            if _BRIDGE:
                _BRIDGE.reset()
            if _STATE is not None:
                _STATE["last_command_str"] = "STOP"
            return {"ok": True}
        if action == "set_target":
            color = body.get("color")
            if color is None:
                return {"error": "missing color"}
            color = str(color).lower()
            if color == "auto":
                # Auto = let the bridge pick by priority on next tick.
                if _BRIDGE:
                    _BRIDGE.target_color = None
                    _BRIDGE.target_locked = False
                return {"ok": True, "target_color": None}
            if color not in ("red", "green", "blue"):
                return {"error": f"unknown color {color!r}"}
            if _BRIDGE:
                _BRIDGE.target_color = color
                # Operator-set target: don't let the state machine silently
                # swap it for something else if the cube briefly vanishes.
                _BRIDGE.target_locked = True
            return {"ok": True, "target_color": color}
        return {"error": f"unknown action {action!r}"}

    def _handle_tune(self, body: dict) -> dict:
        if _STATE is None:
            return {"error": "no state"}
        # Whitelist + cast each key so a malformed POST can't poison the
        # tunables dict with unexpected types.
        changed = {}
        if "gripper_forward_cm" in body:
            _STATE["tun_gripper_forward_cm"] = float(body["gripper_forward_cm"])
            changed["gripper_forward_cm"] = _STATE["tun_gripper_forward_cm"]
        if "gripper_right_cm" in body:
            _STATE["tun_gripper_right_cm"] = float(body["gripper_right_cm"])
            changed["gripper_right_cm"] = _STATE["tun_gripper_right_cm"]
        if "parallax_factor" in body:
            _STATE["tun_parallax_factor"] = float(body["parallax_factor"])
            changed["parallax_factor"] = _STATE["tun_parallax_factor"]
        if "camera_nadir_cm" in body:
            v = body["camera_nadir_cm"]
            if isinstance(v, (list, tuple)) and len(v) == 2:
                _STATE["tun_camera_nadir_cm"] = (float(v[0]), float(v[1]))
                changed["camera_nadir_cm"] = list(_STATE["tun_camera_nadir_cm"])
        return {"ok": True, "changed": changed}

    # ---- MJPEG ----
    def _stream_mjpeg(self):
        """Multipart/x-mixed-replace stream of the latest JPEG. The
        browser's <img> tag just keeps rendering each part as it arrives;
        no JS or WebRTC required."""
        boundary = b"frame"
        try:
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
        except Exception:
            return

        _bump_subs(+1)
        last_ts = 0.0
        try:
            while True:
                jpeg = None
                ts = 0.0
                with _FRAME_LOCK:
                    if _STATE is not None:
                        jpeg = _STATE.get("latest_jpeg")
                        ts = _STATE.get("latest_jpeg_ts") or 0.0
                if jpeg is not None and ts != last_ts:
                    last_ts = ts
                    try:
                        self.wfile.write(b"--" + boundary + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    except Exception:
                        break
                # ~30 Hz upper bound; actual rate follows the producer.
                time.sleep(1.0 / 30.0)
        finally:
            _bump_subs(-1)


# ---- entry point -----------------------------------------------------

def start_dashboard(state: dict,
                    bridge,
                    send_command: Callable[[str], None],
                    save_tunables: Optional[Callable[[], None]] = None,
                    reset_tunables: Optional[Callable[[], None]] = None,
                    reconnect_robot: Optional[Callable[[], str]] = None,
                    host: str = "127.0.0.1",
                    port: int = 8080) -> Optional[ThreadingHTTPServer]:
    """Launch the HTTP dashboard in a daemon thread.

    Returns the server object so the caller can stash/shutdown it if
    they care; returns None if startup failed. Failures are non-fatal —
    main.py wraps this in try/except so the demo still runs.
    """
    global _STATE, _BRIDGE, _SEND, _SAVE_TUNABLES, _RESET_TUNABLES, _RECONNECT_ROBOT
    _STATE = state
    _BRIDGE = bridge
    _SEND = send_command
    _SAVE_TUNABLES = save_tunables
    _RESET_TUNABLES = reset_tunables
    _RECONNECT_ROBOT = reconnect_robot
    # Initial counters / placeholders so handlers don't have to
    # special-case "first request before main.py has produced a frame".
    state.setdefault("latest_jpeg", None)
    state.setdefault("latest_jpeg_ts", None)
    state.setdefault("dashboard_subscribers", 0)
    state.setdefault("last_command_str", None)
    state.setdefault("fps", None)
    state.setdefault("robot_connected", None)
    state["frame_lock"] = _FRAME_LOCK

    try:
        server = ThreadingHTTPServer((host, port), _Handler)
    except OSError as e:
        print(f"[WP4] Dashboard could not bind {host}:{port}: {e}")
        return None

    t = threading.Thread(target=server.serve_forever,
                         name="sortify-dashboard",
                         daemon=True)
    t.start()
    print(f"[WP4] Dashboard listening at http://{host}:{port}")
    return server
