#!/usr/bin/env python3
"""Compare legacy polling and timestamped MLP inputs on recorded C1 depth/odom.

Source ROS Humble first. No camera, inference download, ROS graph, or robot motion.
These are output/coverage comparisons, not ground-truth velocity accuracy.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
import json
from pathlib import Path

import numpy as np
from c1_dataset import load_pair
from c3_replay import make_v3, load_odom, odom_at, depth_metres, intrinsics, TICK_NS


def replay(directory, output, seconds):
    rows = json.loads((directory / 'pair-index.json').read_text())
    odom = load_odom(directory)
    models = {'legacy': make_v3(directory), 'timed': make_v3(directory)}
    frames = [row['depth']['stamp_ns'] for row in rows]
    start = frames[0]
    end = min(frames[-1], start + int(seconds * 1e9))
    records = {name: [] for name in models}
    counts = {name: dict(ticks=0, processed=0, output_ticks=0, outputs=0,
                         duplicates=0, missing_pose=0) for name in models}
    j, loaded, last_timed = 0, None, None
    gaps = []
    for tick in range(start, end + 1, TICK_NS):
        while j + 1 < len(frames) and frames[j + 1] <= tick:
            j += 1
        cap = frames[j]
        if loaded != j:
            arrays, row = load_pair(directory, j)
            depth = depth_metres(arrays, row)
            cents = models['legacy']._extract_depth_centroids(None, depth, intrinsics(row))
            loaded = j
        for name, est in models.items():
            c = counts[name]
            c['ticks'] += 1
            if tick - cap > 500_000_000:
                est._reset_timed_tracks()
                continue
            if name == 'timed' and cap == last_timed:
                c['duplicates'] += 1
                continue
            pose = odom_at(odom, cap if name == 'timed' else tick)
            if pose is None:
                c['missing_pose'] += 1
                est._reset_timed_tracks()
                continue
            kwargs = {}
            if name == 'timed':
                if last_timed is not None:
                    gaps.append((cap - last_timed) / 1e9)
                last_timed = cap
                kwargs['capture_t'] = (cap - start) / 1e9
            estimates, _ = est._step(cents, *pose, **kwargs)
            c['processed'] += 1
            c['output_ticks'] += bool(estimates)
            c['outputs'] += len(estimates)
            records[name].append({'tick_s': (tick - start) / 1e9,
                                  'capture_s': (cap - start) / 1e9,
                                  'estimates': estimates})
    output.mkdir(parents=True, exist_ok=True)
    for name, recs in records.items():
        (output / (name + '.json')).write_text(json.dumps(recs))
        speeds = [e['speed'] for r in recs for e in r['estimates']]
        counts[name].update(
            speed_p50=float(np.median(speeds)) if speeds else None,
            speed_p95=float(np.percentile(speeds, 95)) if speeds else None,
            fraction_outputs_above_015=float(np.mean(np.array(speeds) > .15)) if speeds else None)
    summary = {'dataset': str(directory), 'seconds': (end-start)/1e9,
               'note': 'No independent velocity truth; output populations differ. No person tagging.',
               'arms': counts, 'timing_stats': {k: models['timed'].timing_stats[k]
                                for k in ('window_wait', 'gap_resets')},
               'selected_capture_gap_p50_s': float(np.median(gaps)) if gaps else None,
               'selected_capture_gap_max_s': max(gaps) if gaps else None}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('dataset', type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--seconds', type=float, default=30.)
    args = ap.parse_args()
    replay(args.dataset, args.output, args.seconds)
