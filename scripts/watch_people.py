#!/usr/bin/env python3
"""Print what the OAK person detector and the C3 tracker see, until Ctrl+C.

  python3 scripts/watch_people.py            # on the robot (x3_server running)
  python3 scripts/watch_people.py x3         # from another machine

Each line: time, per OAK person box its centre column u (0..479, centre ~247),
width, and box depth z; then each C3 ring's forward/left position in metres.
Every line is also appended to artifacts/watch_people.ndjson.
"""
import asyncio
import json
import sys
import time
from pathlib import Path

import websockets

HOST = sys.argv[1] if len(sys.argv) > 1 else 'localhost'
LOG = Path(__file__).resolve().parents[1] / 'artifacts' / 'watch_people.ndjson'
PERIOD_S = 0.5


async def main():
    async with websockets.connect(f'ws://{HOST}:8081', max_size=None) as ws:
        print(f'Watching {HOST}. Ctrl+C to stop; also logging to {LOG}')
        last = 0.0
        with LOG.open('a') as log:
            while True:
                msg = await ws.recv()
                if not isinstance(msg, str):
                    continue
                d = json.loads(msg)
                if 'oak_detections' not in d or time.time() - last < PERIOD_S:
                    continue
                last = time.time()
                people = [x for x in (d.get('oak_detections') or []) if x.get('label') == 'person']
                rings = d.get('c3_tracks') or []
                log.write(json.dumps({'t': last, 'people': people, 'c3': rings}) + '\n')
                boxes = ' | '.join(
                    f"u={(p['bbox'][0] + p['bbox'][2]) / 2:3.0f} w={p['bbox'][2] - p['bbox'][0]:3d} "
                    f"z={(p.get('xyz_m') or {}).get('z', float('nan')):.2f}" for p in people) or 'no person'
                c3 = ' '.join(f"ring fwd {r['fwd']:.2f} left {r['left']:+.2f}" for r in rings)
                print(time.strftime('%H:%M:%S'), boxes, '||', c3 or 'no ring', flush=True)


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\nstopped')
