#!/usr/bin/env python3
"""Record the lower ToF array's bare-floor depth per zone -> config JSON.

server_x3 flags a lower-array zone as an obstacle when it reads SHORTER than
this baseline by more than a threshold.  That is far more sensitive than a
height test: the 8x8 zones blend a small object with the floor around it, so
a 3 cm block reads only 1-1.7 cm tall, but it shortens the zones that see it
by 1.6-4.1 cm against a floor that repeats to about a millimetre.

Record on a CLEAR, flat floor with the robot standing still.  Re-record after
touching the bracket or moving to a very different floor.  Reads the server's
WebSocket readout, so x3_server keeps the Teensy port and can stay running:

    python3 src/tof_floor_baseline.py --host x3 --seconds 30
    sudo systemctl restart x3_server      # the server loads it at startup
"""
import argparse
import asyncio
import json
import os
import sys
import time

import numpy as np

DEFAULT_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                           'config', 'tof_floor_baseline.json')
VALID_STATUS = (5, 9)


async def collect(host, port, seconds, sensor):
    import websockets
    dist, ok, seen = [], [], set()
    async with websockets.connect(f'ws://{host}:{port}', max_size=None) as ws:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = await asyncio.wait_for(ws.recv(), 5)
            if isinstance(msg, bytes):
                continue
            data = json.loads(msg)
            tof = (data.get('readout') or data).get('tof') or {}
            frame = tof.get(sensor)
            if not frame or frame.get('stale') or frame['seq'] in seen:
                continue
            seen.add(frame['seq'])
            dist.append(frame['dist'])
            ok.append([s in VALID_STATUS for s in frame['status']])
    return np.asarray(dist, dtype=np.float64), np.asarray(ok, dtype=bool)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=8081)
    p.add_argument('--seconds', type=float, default=30.0)
    p.add_argument('--sensor', default='lower', choices=('lower',))
    p.add_argument('--output', default=DEFAULT_OUT)
    args = p.parse_args()

    dist, ok = asyncio.run(collect(args.host, args.port, args.seconds, args.sensor))
    if len(dist) < 50:
        print(f'only {len(dist)} frames received; is x3_server running?')
        return 1
    valid_frac = ok.mean(axis=0)
    masked = np.where(ok, dist, np.nan)
    depth = np.nanmedian(masked, axis=0)
    # Robust spread: 1.4826 * MAD estimates a standard deviation.
    sd = 1.4826 * np.nanmedian(np.abs(masked - depth), axis=0)
    usable = valid_frac >= 0.9
    record = {
        'sensor': args.sensor,
        'recorded': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'frames': int(len(dist)),
        'depth_mm': [round(float(v), 1) if u else None for v, u in zip(depth, usable)],
        'sd_mm': [round(float(v), 2) if u else None for v, u in zip(sd, usable)],
        'valid_frac': [round(float(v), 3) for v in valid_frac],
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(record, f, indent=1)
    print(f'{len(dist)} frames, {int(usable.sum())}/64 zones usable -> {args.output}')
    print(f'depth {np.nanmin(depth[usable]):.0f}..{np.nanmax(depth[usable]):.0f} mm, '
          f'sd median {np.nanmedian(sd[usable]):.2f} mm, max {np.nanmax(sd[usable]):.2f} mm')
    return 0


if __name__ == '__main__':
    sys.exit(main())
