#!/usr/bin/env python3
"""Web UI for XR3220 bus servos (Feetech protocol). Stdlib only + servo.py.

  GET  /              -> control page
  GET  /api/status    -> {"servos": [{id,pos,speed,load,volts,temp_C,mA,
                                      moving,torque,online}, ...]}
  POST /api/move      -> {"id": int, "position": 0-4095, "time_ms": int}
  POST /api/torque    -> {"id": int, "on": bool}
  POST /api/set-id    -> {"id": int, "new_id": 0-253}   (only from ID 1)

A background thread scans IDs 0-253; newly detected servos appear as tiles.
Run:  python3 server.py [--port 8001] [--serial /dev/ttyUSB0]
"""

import argparse
import glob
import json
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

from servo import BusServo, ServoError

PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Servo Control</title>
<style>
  :root { --bg:#0f1216; --card:#1a1f27; --fg:#e8ecf1; --dim:#8a94a3;
          --acc:#4da3ff; --ok:#3ecf8e; --warn:#ffb454; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:15px/1.5 system-ui, sans-serif; padding:24px; }
  .wrap { max-width:920px; margin:0 auto; }
  .card { background:var(--card); border-radius:14px; padding:28px;
          width:440px; box-shadow:0 4px 24px #0008; }
  .card.off { opacity:.45; }
  h1 { font-size:18px; margin:0 0 20px; letter-spacing:.4px; }
  h1 .idtag { color:var(--acc); }
  .offdot { float:right; font-size:12px; color:var(--warn); }
  .dial { width:160px; height:160px; border-radius:50%; margin:0 auto 14px;
          position:relative;
          background:conic-gradient(var(--acc) var(--a,0deg), #262d38 0); }
  .dial::after { content:""; position:absolute; inset:13px;
          border-radius:50%; background:var(--card); }
  .dial .lbl { position:absolute; inset:0; display:flex; z-index:1;
          flex-direction:column; align-items:center; justify-content:center; }
  .dial .lbl b { font-size:28px; font-variant-numeric:tabular-nums; }
  .dial .lbl span { color:var(--dim); font-size:12px; }
  .row { display:flex; gap:10px; align-items:center; margin-top:12px;
         flex-wrap:wrap; }
  .idctl { display:inline-flex; gap:8px; align-items:center; }
  .idctl.off { visibility:hidden; }
  input[type=range] { flex:1; accent-color:var(--acc); height:28px; }
  input[type=number] { width:74px; background:#10151c; color:var(--fg);
          border:1px solid #2c3644; border-radius:8px; padding:8px; }
  input.newId { width:54px; }
  label { color:var(--dim); font-size:13px; }
  .stats { display:flex; gap:12px; justify-content:center; flex-wrap:wrap;
          color:var(--dim); font-size:13px; margin-top:6px;
          font-variant-numeric:tabular-nums; }
  .stats b { color:var(--fg); }
  button { background:#2a3340; color:var(--fg); border:0; border-radius:8px;
          padding:9px 14px; cursor:pointer; font-size:14px; }
  button:hover { background:#354052; }
  button.on { background:var(--ok); color:#04160d; }
  .presets button { flex:1; padding:9px 4px; }
  #conn { position:fixed; bottom:10px; left:0; right:0; font-size:12px;
          text-align:center; color:var(--dim); }
  #conn.bad { color:var(--warn); }
  .tabbar { display:flex; gap:8px; margin-bottom:18px; }
  .tabbtn { padding:9px 24px; border-radius:8px; }
  .tabbtn.active { background:var(--acc); color:#041019; }
  .pane { display:none; }
  .pane.active { display:flex; flex-wrap:wrap; gap:20px;
                 justify-content:center; }
  #editor { width:100%; }
  #none { color:var(--dim); padding:40px; background:var(--card);
          border-radius:14px; text-align:center; line-height:1.8; }
  #none b { color:var(--fg); font-size:16px; display:block; }
  #code { width:100%; height:240px; background:#0b0e13; color:#cfe3ff;
          font:13px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace;
          border:1px solid #2c3644; border-radius:8px; padding:12px;
          resize:vertical; tab-size:4; white-space:pre; }
  #out { background:#0b0e13; color:#9fb4d0; max-height:170px; overflow:auto;
         font:12px/1.45 ui-monospace, Menlo, monospace; padding:10px;
         border-radius:8px; margin:12px 0 0; white-space:pre-wrap;
         min-height:38px; }
  #runBtn { background:var(--ok); color:#04160d; }
  #scrState { color:var(--dim); font-size:13px; }
</style>
</head>
<body>
<div class="wrap">
<div class="tabbar">
  <button class="tabbtn active" data-t="paneTiles">Servos</button>
  <button class="tabbtn" data-t="paneScript">Script</button>
</div>

<div class="pane active" id="paneTiles">
  <div id="none"><b>Scanning for servos&hellip;</b>
    no servo answered yet &mdash; check power &amp; wiring</div>
</div>

<div class="pane" id="paneScript">
  <div class="card" id="editor">
    <h1>Script</h1>
    <textarea id="code" spellcheck="false"></textarea>
    <div class="row">
      <button id="runBtn">&#9654; Run</button>
      <button id="stopBtn">&#9632; Stop</button>
      <span id="scrState">idle</span>
    </div>
    <pre id="out"></pre>
  </div>
</div>
</div>
<div id="conn">connecting&hellip;</div>

<template id="tileTpl">
<div class="card">
  <h1>XR3220 <span class="idtag"></span><span class="offdot" hidden>offline</span></h1>
  <div class="dial"><div class="lbl">
    <b class="posVal">-</b><span>target / 0&ndash;4095</span>
  </div></div>
  <div class="stats">
    <span>ID <b class="sid">-</b></span>
    <span>actual <b class="actPos">-</b></span>
    <span><b class="volts">-</b> V</span>
    <span><b class="temp">-</b>&deg;C</span>
    <span class="moving">idle</span>
  </div>
  <div class="row">
    <input type="range" class="pos" min="0" max="4095" value="0">
    <input type="number" class="posNum" min="0" max="4095" value="0">
  </div>
  <div class="row">
    <label>move&nbsp;time&nbsp;(ms)</label>
    <input type="number" class="time" min="0" max="30000" value="800" step="100">
    <span class="idctl" title="connect only ONE servo when changing ID">
      <label>new&nbsp;ID</label>
      <input type="number" class="newId" min="0" max="253" value="2">
      <button class="setIdBtn">Set</button>
    </span>
  </div>
  <div class="row">
    <button class="tq on">Torque: ON</button>
  </div>
  <div class="row presets">
    <button data-p="0">0&deg;</button>
    <button data-p="1024">90&deg;</button>
    <button data-p="2048">180&deg;</button>
    <button data-p="3072">270&deg;</button>
    <button data-p="4095">360&deg;</button>
  </div>
</div>
</template>

<script>
const DEFAULT_CODE = `# available helpers:
#   move(id, pos, time_ms)   pos(id)      volts(id)   temp(id)
#   speed(id)                torque(id, on)          wheel(id, speed)
#   servo_mode(id)           sleep(s)     running()   ids()
#   print(...)               time (module)
for _ in range(3):
    for sid in ids():
        move(sid, 2048, 800)
    sleep(1.0)
    for sid in ids():
        move(sid, 1024, 800)
    sleep(1.0)
print("done:", [pos(s) for s in ids()])
`;

const $ = id => document.getElementById(id);
const tiles = {};   // id -> {el, q(class)->element}
let sending = {};   // id -> bool
let pending = {};   // id -> position

async function api(path, body) {
  const r = await fetch(path, body === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)});
  if (!r.ok) throw new Error((await r.json()).error || r.statusText);
  return r.json();
}

function connBad(e) {
  const c = document.getElementById('conn');
  c.textContent = 'servo error: ' + e.message;
  c.className = 'bad';
}

function makeTile(id) {
  const el = document.getElementById('tileTpl').content.cloneNode(true)
                     .querySelector('.card');
  el.dataset.id = id;
  const q = c => el.querySelector('.' + c);
  q('idtag').textContent = '#' + id;
  q('sid').textContent = id;
  if (id !== 1) q('idctl').classList.add('off');   // Set ID only on ID 1

  const show = p => {
    q('posVal').textContent = p;
    el.querySelector('.dial').style.setProperty('--a', p / 4095 * 360 + 'deg');
  };
  const sendMove = async p => {
    if (sending[id]) { pending[id] = p; return; }
    sending[id] = true;
    try { await api('/api/move', {id, position: p, time_ms: +q('time').value}); }
    catch (e) { connBad(e); }
    sending[id] = false;
    if (pending[id] !== undefined) {
      const n = pending[id]; delete pending[id]; sendMove(n);
    }
  };
  const setTarget = (p, fire) => {
    p = Math.max(0, Math.min(4095, p | 0));
    q('pos').value = p; q('posNum').value = p; show(p);
    if (fire) sendMove(p);
  };

  q('pos').oninput   = e => setTarget(+e.target.value, true);
  q('posNum').onchange = e => setTarget(+e.target.value, true);
  el.querySelectorAll('.presets button').forEach(b =>
    b.onclick = () => setTarget(+b.dataset.p, true));

  q('tq').onclick = async () => {
    const on = !q('tq').classList.contains('on');
    try {
      await api('/api/torque', {id, on});
      q('tq').classList.toggle('on', on);
      q('tq').textContent = 'Torque: ' + (on ? 'ON' : 'OFF');
    } catch (e) { connBad(e); }
  };

  const btn = q('setIdBtn');
  if (btn) btn.onclick = async () => {
    const newId = +q('newId').value;
    try {
      const r = await api('/api/set-id', {id, new_id: newId});
      if (!r.ok) connBad(new Error(r.error || 'ID change not confirmed'));
    } catch (e) { connBad(e); }
  };

  document.getElementById('paneTiles').appendChild(el);
  tiles[id] = {el, q, setTarget};
}

function updateTile(id, s) {
  const t = tiles[id];
  t.el.classList.toggle('off', !s.online);
  t.q('offdot').hidden = s.online;
  if (!s.online) return;
  t.q('actPos').textContent = s.pos;
  t.q('volts').textContent = s.volts.toFixed(1);
  t.q('temp').textContent = s.temp_C;
  t.q('moving').textContent = s.moving ? 'moving' : 'idle';
  t.q('tq').classList.toggle('on', s.torque);
  t.q('tq').textContent = 'Torque: ' + (s.torque ? 'ON' : 'OFF');
  // keep UI synced to hardware unless user is editing the number box
  if (document.activeElement !== t.q('posNum'))
    t.setTarget(s.pos, false);
}

async function poll() {
  try {
    const s = await api('/api/status');
    const seen = new Set();
    for (const srv of s.servos) {
      seen.add(srv.id);
      if (!tiles[srv.id]) makeTile(srv.id);
      updateTile(srv.id, srv);
    }
    for (const id of Object.keys(tiles))
      if (!seen.has(+id)) { tiles[id].el.remove(); delete tiles[id]; }
    document.getElementById('none').style.display =
      s.servos.length ? 'none' : '';
    if (s.script) {
      $('scrState').textContent = s.script.running ? 'running' : 'idle';
      $('runBtn').disabled = s.script.running;
      const out = $('out');
      out.textContent = s.script.output;
      out.scrollTop = out.scrollHeight;
    }
    const c = document.getElementById('conn');
    c.textContent = 'connected - ' + s.servos.length + ' servo(s)';
    c.className = '';
  } catch (e) { connBad(e); }
  setTimeout(poll, 400);
}

// ---- tabs ----
document.querySelectorAll('.tabbtn').forEach(b => b.onclick = () => {
  document.querySelectorAll('.tabbtn').forEach(x =>
    x.classList.toggle('active', x === b));
  document.querySelectorAll('.pane').forEach(p =>
    p.classList.toggle('active', p.id === b.dataset.t));
});

// ---- editor ----
const codeEl = $('code');
codeEl.value = localStorage.getItem('servoScript') || DEFAULT_CODE;
codeEl.addEventListener('input',
  () => localStorage.setItem('servoScript', codeEl.value));
codeEl.addEventListener('keydown', e => {
  if (e.key === 'Tab') {
    e.preventDefault();
    codeEl.setRangeText('    ', codeEl.selectionStart,
                        codeEl.selectionEnd, 'end');
  }
});
$('runBtn').onclick = async () => {
  try { await api('/api/script/run', {code: codeEl.value}); }
  catch (e) { connBad(e); }
};
$('stopBtn').onclick = async () => {
  try { await api('/api/script/stop', {}); }
  catch (e) { connBad(e); }
};

poll();
</script>
</body>
</html>
"""


class State:
    bus = None
    lock = threading.Lock()
    servos = {}          # id -> {"online": bool, "misses": int}
    want_port = None     # preferred port (--serial); falls back to any ttyUSB*
    baud = 1000000
    port = None          # port actually in use
    script_stop = threading.Event()
    script_running = False
    script_output = []


class ScriptStopped(Exception):
    pass


def script_out(text):
    State.script_output.append(str(text))
    del State.script_output[:-200]


def script_env():
    """Helper API exposed to user scripts."""
    ev = State.script_stop

    def check():
        if ev.is_set():
            raise ScriptStopped()

    def sleep(s):
        end = time.time() + float(s)
        while time.time() < end:
            check()
            time.sleep(min(0.05, end - time.time()))

    def call(name, *a, **k):
        check()
        with State.lock:
            if State.bus is None:
                raise ServoError("serial port disconnected")
            return getattr(State.bus, name)(*a, **k)

    return {
        "move": lambda sid, p, t=0: call("move", sid, p, time_ms=t),
        "pos": lambda sid: call("get_position", sid),
        "volts": lambda sid: call("get_voltage", sid),
        "temp": lambda sid: call("get_temp", sid),
        "speed": lambda sid: call("get_speed", sid),
        "load": lambda sid: call("get_load", sid),
        "current": lambda sid: call("get_current", sid),
        "torque": lambda sid, on: call("torque", sid, bool(on)),
        "wheel": lambda sid, v: call("wheel", sid, v),
        "servo_mode": lambda sid: call("servo_mode", sid),
        "status": lambda sid: call("status", sid),
        "ids": lambda: sorted(k for k, v in State.servos.items()
                              if v["online"]),
        "sleep": sleep,
        "running": lambda: not ev.is_set(),
        "print": lambda *a: script_out(" ".join(map(str, a))),
        "time": time,
    }


def run_script(code):
    if State.script_running:
        raise ValueError("a script is already running")
    State.script_running = True
    State.script_stop.clear()
    State.script_output.clear()

    def worker():
        try:
            exec(compile(code, "<servo-script>", "exec"), script_env())
            script_out("--- finished ---")
        except ScriptStopped:
            script_out("--- stopped ---")
        except Exception:
            script_out(traceback.format_exc())
        finally:
            State.script_running = False

    threading.Thread(target=worker, daemon=True).start()


def open_bus():
    """Try preferred port first, then any /dev/ttyUSB*. None if all fail."""
    cands = ([State.want_port] if State.want_port else []) + \
            sorted(glob.glob("/dev/ttyUSB*"))
    for p in dict.fromkeys(cands):
        try:
            bus = BusServo(p, State.baud)
            State.port = p
            return bus
        except (serial.SerialException, OSError):
            continue
    return None


def scanner():
    """Continuously ping IDs 0-253; update State.servos. Reconnects the
    serial port automatically if the USB cable re-enumerates."""
    while True:
        if State.bus is None:
            State.bus = open_bus()
            if State.bus is None:
                time.sleep(1)
                continue
        for i in range(254):
            with State.lock:
                try:
                    # ~2 s sweep; online servos get one retry at a slower
                    # timeout so a single slow reply doesn't drop them
                    found = State.bus.ping(i, timeout_ms=7)
                    s = State.servos.get(i)
                    if not found and s and s["online"]:
                        found = State.bus.ping(i, timeout_ms=50)
                except (serial.SerialException, OSError):
                    State.bus = None
                    for s in State.servos.values():
                        s["online"] = False
                    break
            s = State.servos.get(i)
            if found:
                State.servos[i] = {"online": True, "misses": 0}
            elif s:
                s["misses"] += 1
                if s["misses"] >= 3:
                    s["online"] = False
                if s["misses"] >= 10:     # gone for good: drop the tile
                    del State.servos[i]
            time.sleep(0)  # yield so user commands interleave
        time.sleep(0.3)


class Handler(BaseHTTPRequestHandler):
    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self):
        n = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(n) or b"{}")

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/status":
            out = []
            with State.lock:
                ids = sorted(State.servos)
                for sid in ids:
                    info = {"id": sid, "online": State.servos[sid]["online"]}
                    if info["online"] and State.bus:
                        try:
                            info.update(State.bus.status(sid))
                            p = State.bus.read(sid, 40, 1)  # torque enable
                            info["torque"] = bool(p[0]) if p else None
                        except (ServoError, serial.SerialException, OSError):
                            info["online"] = False
                    out.append(info)
            self._json(200, {
                "servos": out,
                "port": State.port,
                "script": {
                    "running": State.script_running,
                    "output": "".join(
                        line + "\n" for line in
                        State.script_output[-200:])[-8000:],
                },
            })
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        try:
            d = self._read_body()
            sid = int(d.get("id", 1))
            if self.path == "/api/move":
                pos, t = int(d["position"]), int(d.get("time_ms", 0))
                if not 0 <= pos <= 4095 or not 0 <= t <= 30000:
                    raise ValueError("position 0..4095, time_ms 0..30000")
                with State.lock:
                    if State.bus is None:
                        raise ServoError("serial port disconnected")
                    State.bus.move(sid, pos, time_ms=t)
                self._json(200, {"ok": True})

            elif self.path == "/api/torque":
                with State.lock:
                    if State.bus is None:
                        raise ServoError("serial port disconnected")
                    State.bus.torque(sid, bool(d["on"]))
                self._json(200, {"ok": True})

            elif self.path == "/api/set-id":
                new_id = int(d["new_id"])
                if not 0 <= new_id <= 253:
                    raise ValueError("new_id must be 0..253")
                if sid != 1:
                    raise ValueError("only the ID=1 servo can be renumbered")
                with State.lock:
                    if State.bus is None:
                        raise ServoError("serial port disconnected")
                    State.bus.set_id(new_id, old_id=sid)
                    ok = State.bus.ping(new_id, wait=True)
                    if ok:
                        State.servos[1] = {"online": False, "misses": 99}
                        State.servos[new_id] = {"online": True, "misses": 0}
                self._json(200, {"ok": ok, "id": new_id if ok else sid})

            elif self.path == "/api/script/run":
                run_script(str(d["code"]))
                self._json(200, {"ok": True})

            elif self.path == "/api/script/stop":
                State.script_stop.set()
                self._json(200, {"ok": True})

            else:
                self._json(404, {"error": "not found"})
        except (KeyError, ValueError, ServoError,
                serial.SerialException, OSError) as e:
            self._json(400, {"error": str(e)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--serial", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1000000)
    args = ap.parse_args()

    State.want_port = args.serial
    State.baud = args.baud
    State.bus = open_bus()      # may be None; scanner keeps retrying
    threading.Thread(target=scanner, daemon=True).start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"http://localhost:{args.port}  "
          f"(serial: {State.port or 'waiting for /dev/ttyUSB*'})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if State.bus:
            State.bus.close()


if __name__ == "__main__":
    main()
