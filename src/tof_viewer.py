#!/usr/bin/env python3
"""Live browser viewer for the VL53L5CX array.  Runs ON THE ROBOT.

    python3 src/tof_viewer.py                 # then open http://x3:8090/
    python3 src/tof_viewer.py --sim           # no hardware, synthetic frames
    python3 src/tof_viewer.py --freq 15       # 8x8 caps at 15 Hz

Deliberately standalone: it opens the sensor itself and serves its own page,
so it neither needs nor disturbs x3_server.  Do not run it at the same time as
anything else that opens the ToF -- one I2C device, one owner.

WHAT TO CHECK FIRST IS ORIENTATION.  Put a hand in ONE corner of the field of
view and watch which corner of the grid goes near.  Top-left of the grid must
be up and to the robot's LEFT.  If it is not, find the --transpose/--flip-h/
--flip-v combination that fixes it and carry those same three values into the
ROS node's params; the viewer and the node share tof_geometry's ordering code,
so a setting that looks right here IS right there.

The side panel shows each ROW's mean range against the range that row should
read if it were hitting a flat floor, given the mount height and pitch.  That
is the calibration check: on a flat floor with nothing in view, measured and
predicted should track.  A row reading consistently SHORT is seeing an
obstacle; the whole grid reading short means the pitch or height is wrong.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, 'yahboomcar_bringup'))

from yahboomcar_bringup.tof_geometry import (  # noqa: E402
    FOV_DEG, VALID_STATUS, DEFAULT_MAX_RANGE_M, floor_slant_ranges, reorder,
    valid_mask)

PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>VL53L5CX viewer</title>
<style>
  :root { color-scheme: dark; }
  body { background:#0d1117; color:#c9d1d9; font-family:ui-monospace,Menlo,Consolas,monospace;
         margin:0; padding:16px; }
  h1 { font-size:15px; font-weight:600; margin:0 0 2px; color:#e6edf3; }
  .sub { font-size:11px; color:#7d8590; margin-bottom:14px; }
  .wrap { display:flex; gap:22px; align-items:flex-start; flex-wrap:wrap; }
  table { border-collapse:separate; border-spacing:3px; }
  td { width:58px; height:50px; text-align:center; vertical-align:middle;
       border-radius:5px; font-size:13px; font-variant-numeric:tabular-nums;
       color:#08090c; font-weight:600; transition:background-color .07s linear; }
  td.bad { background:#161b22 !important; color:#30363d; font-weight:400; }
  td .st { display:block; font-size:9px; opacity:.55; font-weight:400; }
  .panel { font-size:12px; line-height:1.85; min-width:260px; }
  .panel b { color:#e6edf3; font-weight:600; }
  .k { color:#7d8590; display:inline-block; min-width:104px; }
  .axes { font-size:11px; color:#7d8590; margin:8px 0 0; }
  .rows { margin-top:12px; border-top:1px solid #21262d; padding-top:8px; }
  .rows div { display:flex; justify-content:space-between; gap:10px; }
  .warn { color:#d29922; } .err { color:#f85149; } .ok { color:#3fb950; }
  .bar { height:5px; background:#21262d; border-radius:3px; overflow:hidden; margin-top:3px; }
  .bar i { display:block; height:100%; background:#388bfd; }
  #conn { font-size:11px; }
</style></head><body>
<h1>VL53L5CX &mdash; live</h1>
<div class="sub">top-left of the grid = UP and to the robot's LEFT &nbsp;|&nbsp;
  grey = rejected (status not in {5,9}, or out of range)</div>
<div class="wrap">
  <div>
    <table id="g"></table>
    <div class="axes">columns: robot LEFT &rarr; RIGHT &nbsp;&nbsp; rows: UP &rarr; DOWN
      (down-range rows are nearest the floor)</div>
  </div>
  <div class="panel">
    <div><span class="k">connection</span><b id="conn">connecting...</b></div>
    <div><span class="k">frames</span><b id="n">0</b></div>
    <div><span class="k">rate</span><b id="hz">-</b></div>
    <div><span class="k">valid zones</span><b id="v">-</b></div>
    <div class="bar"><i id="vb" style="width:0%"></i></div>
    <div><span class="k">nearest</span><b id="mn">-</b></div>
    <div><span class="k">farthest</span><b id="mx">-</b></div>
    <div><span class="k">resolution</span><b id="res">-</b></div>
    <div><span class="k">mount</span><b id="mount">-</b></div>
    <div class="rows" id="rows"></div>
  </div>
</div>
<script>
const N_MAX_DEFAULT = 2500;
let maxRange = N_MAX_DEFAULT, n = 8, frames = 0, t0 = null;

// Perceptually ordered ramp: near = warm, far = cool. Distinguishable in
// greyscale too, so a screenshot still reads correctly.
function colour(mm) {
  const t = Math.max(0, Math.min(1, mm / maxRange));
  const stops = [[255,236,179],[252,176,64],[233,105,44],[186,53,90],
                 [104,52,139],[38,54,120],[14,26,48]];
  const x = t * (stops.length - 1), i = Math.min(stops.length - 2, Math.floor(x)), f = x - i;
  const a = stops[i], b = stops[i+1];
  return `rgb(${Math.round(a[0]+(b[0]-a[0])*f)},${Math.round(a[1]+(b[1]-a[1])*f)},${Math.round(a[2]+(b[2]-a[2])*f)})`;
}

function build(size) {
  const t = document.getElementById('g'); t.innerHTML = '';
  for (let r = 0; r < size; r++) {
    const tr = document.createElement('tr');
    for (let c = 0; c < size; c++) {
      const td = document.createElement('td');
      td.id = `c${r}_${c}`; td.textContent = '-';
      tr.appendChild(td);
    }
    t.appendChild(tr);
  }
  n = size;
}

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  const conn = document.getElementById('conn');
  ws.onopen  = () => { conn.textContent = 'live'; conn.className = 'ok'; };
  ws.onclose = () => { conn.textContent = 'disconnected - retrying'; conn.className = 'err';
                       setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.meta) {
      maxRange = m.meta.max_range_mm; build(m.meta.n);
      document.getElementById('res').textContent =
        `${m.meta.n}x${m.meta.n} @ ${m.meta.freq} Hz, ${m.meta.fov}° FoV`;
      document.getElementById('mount').textContent =
        `${m.meta.height_mm} mm up, ${m.meta.pitch}° down`;
      return;
    }
    frames++;
    if (t0 === null) t0 = performance.now();
    for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) {
      const i = r * n + c, td = document.getElementById(`c${r}_${c}`);
      const ok = m.keep[i];
      td.className = ok ? '' : 'bad';
      td.style.backgroundColor = ok ? colour(m.dist[i]) : '';
      td.innerHTML = (ok ? m.dist[i] : '&mdash;') +
                     `<span class="st">s${m.status[i]}</span>`;
    }
    const el = (id, v) => document.getElementById(id).textContent = v;
    el('n', frames);
    const dt = (performance.now() - t0) / 1000;
    el('hz', dt > 0.5 ? (frames / dt).toFixed(1) + ' Hz' : '-');
    el('v', `${m.valid} / ${n * n}`);
    document.getElementById('vb').style.width = (100 * m.valid / (n * n)) + '%';
    el('mn', m.valid ? m.min + ' mm' : '-');
    el('mx', m.valid ? m.max + ' mm' : '-');
    const rows = document.getElementById('rows');
    rows.innerHTML = '<div><b>row</b><b>measured</b><b>flat floor</b></div>' +
      m.rows.map(r => {
        const meas = r.mean === null ? '&mdash;' : r.mean + ' mm';
        const pred = r.floor === null ? 'above horizon' : r.floor + ' mm';
        let cls = '';
        if (r.mean !== null && r.floor !== null)
          cls = Math.abs(r.mean - r.floor) > 0.25 * r.floor ? 'warn' : 'ok';
        return `<div><span class="k">${r.i}</span><span class="${cls}">${meas}</span><span style="color:#7d8590">${pred}</span></div>`;
      }).join('');
  };
}
build(8); connect();
</script></body></html>
"""


def build_meta(args, tof):
    return {"meta": {"n": args.resolution, "freq": tof.ranging_freq_hz,
                     "fov": FOV_DEG, "max_range_mm": int(args.max_range * 1000),
                     "height_mm": int(round(args.height * 1000)),
                     "pitch": args.pitch}}


def floor_prediction(args):
    """Range each row would read off a flat floor, or None above the horizon."""
    n = args.resolution
    rows, ranges = floor_slant_ranges(args.height, args.pitch, n)
    pred = [None] * n
    for r, d in zip(rows, ranges):
        pred[int(r)] = int(round(d * 1000))
    return pred


def frame_payload(args, dist, status, pred):
    n = args.resolution
    keep = valid_mask(dist, status, n, max_range_m=args.max_range,
                      transpose=args.transpose, flip_h=args.flip_h,
                      flip_v=args.flip_v)
    d = reorder(dist, n, args.transpose, args.flip_h, args.flip_v)
    s = reorder(status, n, args.transpose, args.flip_h, args.flip_v)
    k = np.asarray(keep, dtype=bool).ravel()
    d = np.asarray(d, dtype=np.float64).ravel()
    s = np.asarray(s, dtype=np.int32).ravel()

    rows = []
    for r in range(n):
        sl = slice(r * n, (r + 1) * n)
        rk = k[sl]
        rows.append({"i": r,
                     "mean": int(round(float(d[sl][rk].mean()))) if rk.any() else None,
                     "floor": pred[r]})
    return {"dist": [int(x) for x in d],
            "status": [int(x) for x in s],
            "keep": [bool(x) for x in k],
            "valid": int(k.sum()),
            "min": int(d[k].min()) if k.any() else 0,
            "max": int(d[k].max()) if k.any() else 0,
            "rows": rows}


async def main_async(args):
    try:
        from aiohttp import web, WSMsgType
    except ImportError:
        print("aiohttp not installed. On the robot:  pip3 install --user aiohttp")
        return 1

    from drivers_x3 import VL53L5CXArray
    tof = VL53L5CXArray(i2c_bus=args.bus, i2c_addr=args.addr,
                        resolution=args.resolution,
                        ranging_freq_hz=args.freq,
                        max_range_m=args.max_range, sim_mode=args.sim,
                        transpose=args.transpose, flip_h=args.flip_h,
                        flip_v=args.flip_v)
    if not tof.available:
        print("no sensor found (use --sim to exercise the viewer without one)")
        return 1

    pred = floor_prediction(args)
    latest = {"payload": None}
    clients = set()

    async def sampler():
        # One reader for the sensor regardless of how many browsers are open:
        # the ULD is not reentrant and two readers race for the same frame.
        loop = asyncio.get_running_loop()
        while True:
            ready = await loop.run_in_executor(None, tof.data_ready)
            if not ready:
                await asyncio.sleep(0.004)
                continue
            frame = await loop.run_in_executor(None, tof.read_frame)
            if frame is None:
                await asyncio.sleep(0.01)
                continue
            latest["payload"] = frame_payload(args, frame[0], frame[1], pred)
            dead = []
            for ws in clients:
                try:
                    await ws.send_json(latest["payload"])
                except Exception:
                    dead.append(ws)
            for ws in dead:
                clients.discard(ws)

    async def index(_request):
        return web.Response(text=PAGE, content_type="text/html")

    async def websocket(request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        await ws.send_json(build_meta(args, tof))
        if latest["payload"] is not None:
            await ws.send_json(latest["payload"])
        clients.add(ws)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.ERROR:
                    break
        finally:
            clients.discard(ws)
        return ws

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.port)
    await site.start()
    print(f"VL53L5CX viewer on http://0.0.0.0:{args.port}/   (Ctrl-C to stop)")
    print(f"  from the laptop:  http://x3:{args.port}/")
    task = asyncio.ensure_future(sampler())
    try:
        while True:
            await asyncio.sleep(3600)
    finally:
        task.cancel()
        await runner.cleanup()
        tof.cleanup()
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0x29)
    p.add_argument("--resolution", type=int, default=8, choices=(4, 8))
    p.add_argument("--freq", type=int, default=10, help="Hz (8x8 caps at 15)")
    p.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE_M)
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--height", type=float, default=0.0640,
                   help="aperture height above the floor, m (URDF: 0.0640)")
    p.add_argument("--pitch", type=float, default=15.0,
                   help="nose-down pitch, deg (URDF: 15)")
    p.add_argument("--transpose", action="store_true")
    p.add_argument("--flip-h", action="store_true")
    p.add_argument("--flip-v", action="store_true")
    p.add_argument("--sim", action="store_true")
    args = p.parse_args()
    try:
        return asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print()
        return 0


if __name__ == "__main__":
    sys.exit(main())
