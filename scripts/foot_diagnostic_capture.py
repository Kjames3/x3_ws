#!/usr/bin/env python3
"""Read-only WebSocket capture of foot diagnostic telemetry; sends no commands."""
import argparse
import asyncio
from collections import Counter
import json
from pathlib import Path
import re
import time

import websockets


async def capture(args):
    label = re.sub(r'[^a-zA-Z0-9_-]', '_', args.label)
    directory = Path(args.output) / (time.strftime('%Y%m%d-%H%M%S') + '-' + label)
    directory.mkdir(parents=True, exist_ok=False)
    counts = Counter()
    packets = feet_packets = 0
    seen = set()
    async with websockets.connect(f'ws://{args.host}:8081', max_size=8*1024*1024) as ws:
        print('Connected. Stand still; capture starts in 3 seconds.', flush=True)
        for n in (3, 2, 1):
            print(n, flush=True)
            until = time.monotonic()+1
            while time.monotonic() < until:
                try:
                    await asyncio.wait_for(ws.recv(), timeout=max(.001, until-time.monotonic()))
                except asyncio.TimeoutError:
                    break
        print(f'START — {args.seconds:g} seconds. Keep robot stationary; slowly extend and withdraw one foot.', flush=True)
        start = time.monotonic()
        with (directory/'telemetry.ndjson').open('w') as out:
            while time.monotonic()-start < args.seconds:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=2)
                except asyncio.TimeoutError:
                    continue
                if not isinstance(raw,str):
                    continue
                message = json.loads(raw)
                if message.get('type') != 'readout':
                    continue
                diag = message.get('foot_diagnostic')
                if diag is None:
                    counts['missing_diagnostic'] += 1
                    continue
                out.write(json.dumps(dict(elapsed_s=time.monotonic()-start, diagnostic=diag,
                                          robot_pose=message.get('robot_pose')),allow_nan=False)+'\n')
                key = diag.get('seq')
                if key in seen:
                    continue
                seen.add(key)
                packets += 1
                feet_packets += bool(diag.get('feet'))
                counts[diag.get('status','unknown')] += 1
        summary = dict(unique_packets=packets, packets_with_feet=feet_packets,
                       statuses=dict(counts), motors_commanded=False,
                       note='Coverage only, not detection accuracy or safety validation.')
        (directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print('STOP — capture complete.', flush=True)
        print(json.dumps(summary,indent=2))
        print(f'Saved: {directory}')
        if not feet_packets:
            print('No foot regions captured. Inspect status/rejection reasons before a longer run.')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='x3')
    p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--label',required=True)
    p.add_argument('--output',default='evidence/foot-diagnostic')
    args=p.parse_args()
    if not 0 < args.seconds <= 300:
        p.error('--seconds must be in (0, 300]')
    asyncio.run(capture(args))
