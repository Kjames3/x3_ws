#!/usr/bin/env python3
"""Measure what the GUI WebSocket delivers to this machine: per-camera fps,
frame size and resolution, plus total bytes/s (JSON telemetry included).

    python3 src/tools/viewers/stream_bench.py --host orin --seconds 20 [--profile high]

--profile sends set_stream_profile first (the server keeps it until changed).
"""
import argparse, asyncio, json, time, collections
import numpy as np, cv2, websockets

ROLES = ('front', 'rear', 'left', 'right')

async def main(a):
    async with websockets.connect(f"ws://{a.host}:8081", max_size=None) as ws:
        if a.profile:
            await ws.send(json.dumps({"type": "set_stream_profile", "profile": a.profile}))
            await asyncio.sleep(3)   # let the rear pipeline reopen
        n = collections.Counter(); b = collections.Counter(); res = {}; gaps = collections.defaultdict(list)
        last = {}; total = 0; t0 = time.monotonic()
        while (now := time.monotonic()) - t0 < a.seconds:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=1)
            except asyncio.TimeoutError:
                continue
            total += len(msg)
            if isinstance(msg, bytes) and msg[:4] == b'CAM1':
                role = ROLES[msg[4]]
                n[role] += 1; b[role] += len(msg)
                if role in last: gaps[role].append(now - last[role])
                last[role] = now
                if role not in res:
                    img = cv2.imdecode(np.frombuffer(msg[6:], np.uint8), cv2.IMREAD_UNCHANGED)
                    res[role] = f"{img.shape[1]}x{img.shape[0]}" if img is not None else '?'
        dt = time.monotonic() - t0
        print(f"total {total*8/dt/1e6:.2f} Mb/s over {dt:.0f} s")
        for r in ROLES:
            if n[r]:
                g = np.array(gaps[r]) * 1000
                print(f"  {r:5s} {res[r]:>9s} {n[r]/dt:5.1f} fps  {b[r]/n[r]/1024:5.1f} KB/frame  "
                      f"{b[r]*8/dt/1e6:5.2f} Mb/s  gap p95 {np.percentile(g,95):4.0f} ms")

p = argparse.ArgumentParser(); p.add_argument('--host', default='orin')
p.add_argument('--seconds', type=float, default=20); p.add_argument('--profile')
asyncio.run(main(p.parse_args()))
