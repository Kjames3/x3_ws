#!/usr/bin/env python3
"""Watch the live C3 tracker for N seconds and summarise coverage. Run on the laptop.

  python3 scripts/c3_live_window.py [--host x3] [--seconds 60] [--label pose]

Stay in the camera's view for the whole window. Reports, from the server's
telemetry: detection packets, packets with a person box, packets that gave a
ring (c3_stats.diag deltas), duplicate boxes/tracks, merged seg boxes, and a
5 s timeline. Saves the raw samples to /tmp/c3win-<label>.ndjson.
"""
import argparse
import asyncio
import collections
import json
import time

import websockets


async def record(host, seconds, path):
    rows, system = [], {}
    async with websockets.connect(f'ws://{host}:8081', max_size=None) as ws:
        t0 = time.time()
        while time.time() - t0 < seconds:
            m = await ws.recv()
            if isinstance(m, bytes):
                continue
            d = json.loads(m)
            if d.get('system'):          # slow telemetry, sent apart from c3_stats
                system = d['system']
            if not d.get('c3_stats'):
                continue
            rows.append({'t': time.time() - t0, 'st': d['c3_stats'], 'tr': d.get('c3_tracks') or [],
                         'oak': [{k: o.get(k) for k in ('label', 'conf', 'bbox', 'xyz_m', 'merged_from')}
                                 for o in d.get('oak_detections') or []],
                         'sys': {k: system.get(k) for k in ('cpu_total', 'proc_cpu', 'gpu_pct')}})
    with open(path, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    return rows


def summarise(rows):
    a, b = rows[0]['st'], rows[-1]['st']
    dur = rows[-1]['t'] - rows[0]['t']
    d = {k: b['diag'][k] - a['diag'][k] for k in b['diag'] if k != 'latency_ms_avg'}
    upd = b['updates'] - a['updates']
    pp = max(1, d['person_packets'])
    print(f"{dur:.0f} s, {upd} detection packets ({upd / dur:.1f} per s)")
    print(f"person in packet {100 * d['person_packets'] / max(1, upd):.0f}% | "
          f"ring given a person {100 * d['ring_packets'] / pp:.0f}% | "
          f"ring overall {100 * d['ring_packets'] / max(1, upd):.0f}% | "
          f"boxes with depth {100 * d['meas'] / max(1, d['dets']):.0f}% | "
          f"boxes per person packet {d['dets'] / pp:.2f}")
    print(f"resets: stale pose {d['reset_pose']}, no depth {d['reset_depth']}, packet gap {d['reset_gap']}")
    people = [sum(o['label'] == 'person' for o in r['oak']) for r in rows]
    print('person boxes per sample', sorted(collections.Counter(people).items()),
          '| rings per sample', sorted(collections.Counter(len(r['tr']) for r in rows).items()))
    merged = sum(1 for r in rows for o in r['oak'] if (o.get('merged_from') or 1) > 1)
    print(f'samples with a merged box: {merged}')
    cpu = [r['sys']['cpu_total'] for r in rows if r['sys'].get('cpu_total') is not None]
    proc = [r['sys']['proc_cpu'] for r in rows if r['sys'].get('proc_cpu') is not None]
    if cpu:
        print(f'CPU total {sum(cpu) / len(cpu):.0f}% | server process {sum(proc) / max(1, len(proc)):.0f}%')
    for k in range(0, int(dur) + 1, 5):
        seg = [r for r in rows if k <= r['t'] < k + 5]
        if not seg:
            continue
        zs = [o['xyz_m']['z'] for r in seg for o in r['oak'] if o['label'] == 'person' and o['xyz_m']]
        print(f"{k:3d}-{k + 5:<3d}s person {100 * sum(any(o['label'] == 'person' for o in r['oak']) for r in seg) / len(seg):3.0f}%"
              f"  ring {100 * sum(bool(r['tr']) for r in seg) / len(seg):3.0f}%"
              f"  depth {min(zs):.1f}-{max(zs):.1f} m" if zs else
              f"{k:3d}-{k + 5:<3d}s person   0%  ring {100 * sum(bool(r['tr']) for r in seg) / len(seg):3.0f}%")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='x3')
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--label', default='run')
    args = ap.parse_args()
    path = f'/tmp/c3win-{args.label}.ndjson'
    print(f'recording {args.seconds:g} s from {args.host} ...', flush=True)
    rows = asyncio.run(record(args.host, args.seconds, path))
    if len(rows) < 2:
        raise SystemExit('no c3_stats in telemetry: is x3_server running with the C3 tracker?')
    summarise(rows)
    print(f'raw samples: {path}')


if __name__ == '__main__':
    main()
