#!/usr/bin/env python3
"""Does raising the person-measurement depth limit past 4.0 m help or hurt arm C?

  python3 src/c3_depth_cap_test.py RUN_DIR... --output OUT [--caps 4.0,6.0]

Replays the cached pose and merged-seg detections (c3_seg_vs_pose.py must have
run) with c3_person_tracker.DEPTH_RANGE_M's upper limit at each cap. Scoring
uses ONE reference for every cap: the cached yolo11x boxes with positions
recomputed at the largest cap (c3-reference-far.json), and a 4 m+ range bin.
With the stock 4.0 m reference the scorer cannot see past 4 m and counts
correct far tracking as non-person output.

Two things to read: the 4 m+ bin (what the longer limit buys) and the bins
under 4 m (what it costs: background 4-6 m behind a person now counts as valid
depth inside their box). Offline only.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import c3_replay as rp  # noqa: E402
import c3_score as cs  # noqa: E402
import c3_person_tracker as pt  # noqa: E402
from c1_dataset import load_pair  # noqa: E402

DETECTIONS = {'pose': 'c3-det-pose.json', 'seg-merged': 'c3-det-seg-merged.json'}
FAR_REFERENCE = 'c3-reference-far.json'


def write_far_reference(run, cap):
    """c3-reference.json's boxes with positions recomputed out to `cap`."""
    ref = json.loads((run / 'c3-reference.json').read_text())
    pt.DEPTH_RANGE_M = (0.5, cap)
    for i, frame in enumerate(ref['frames']):
        frame['person'] = None
        if not frame['boxes']:
            continue
        arrays, row = load_pair(run, i)
        ofc = row.get('odom_from_camera')
        best = max(frame['boxes'], key=lambda b: b['conf'])
        m = pt.person_measurement(rp.depth_metres(arrays, row), best['xyxy'])
        if m is None or ofc is None:
            continue
        fx, fy, cx, cy = rp.intrinsics(row)[:4]
        p = [(m['u'] - cx) * m['z'] / fx, (m['v'] - cy) * m['z'] / fy, m['z']]
        xy, _ = pt.camera_point_to_odom(p, ofc)
        frame['person'] = dict(x=float(xy[0]), y=float(xy[1]), z=m['z'],
                               valid_fraction=m['valid_fraction'])
    ref['position'] += f'; depth range 0.5-{cap:g} m'
    (run / FAR_REFERENCE).write_text(json.dumps(ref) + '\n')
    n = sum(f['person'] is not None and f['person']['z'] > 4.0 for f in ref['frames'])
    print(f'{run.name}: far reference, {n} frames past 4 m', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--caps', default='4.0,6.0')
    args = ap.parse_args()
    caps = [float(c) for c in args.caps.split(',')]
    runs = sorted(r for r in args.runs if (r / 'c3-reference.json').exists())

    for run in runs:
        write_far_reference(run, max(caps))
    cs.REFERENCE_FILE = FAR_REFERENCE
    cs.BINS = rp.RANGE_BINS            # adds the 4 m+ bin

    result = {}
    for cap in caps:
        pt.DEPTH_RANGE_M = (0.5, cap)
        for fe, name in DETECTIONS.items():
            label = f'{fe} @ {cap:g} m'
            report = cs.score(runs, args.output / f'{fe}-cap{cap:g}', detections_name=name)
            result[label] = cs.aggregate(report)['C']
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'depth_cap.json').write_text(json.dumps(result, indent=1) + '\n')

    labels = list(result)
    keys = sorted(set().union(*(result[k] for k in labels)))
    lines = ['| metric | ' + ' | '.join(labels) + ' |', '|---' * (len(labels) + 1) + '|']
    for k in keys:
        cells = []
        for label in labels:
            v = result[label].get(k)
            cells.append('--' if v is None else ', '.join(
                f'{m}={x:.3g}' if isinstance(x, float) else f'{m}={x}' for m, x in v.items()))
        lines.append(f'| {k} | ' + ' | '.join(cells) + ' |')
    (args.output / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
