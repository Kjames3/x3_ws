#!/usr/bin/env python3
"""Seg vs pose as the C3 person front end, on identical recordings.

  python3 src/c3_seg_vs_pose.py RUN_DIR... --output OUT

For every run: cache yolo26n-pose boxes and yolo26n-seg boxes + silhouettes
(c3_replay detect), build the yolo11x reference (c3_score prepare), then score
arm C twice -- pose boxes with torso depth, seg silhouettes with silhouette
depth -- with the unchanged c3_score metrics (recall, false motion, speed,
stop/go delay). Arms A/B do not depend on the front end and are identical in
both reports.

Adds a taped-distance check the reference cannot give (it shares the depth
sensor): runs named stand-<d>m-rN / stand-side-<d>m-rN have the person's toes
on a mark <d> m from the camera lens, so arm C's forward distance has a known
target. Torso depth sits ~0.1-0.15 m behind the toes for both front ends, so
compare bias between front ends, not against zero.
Offline only; nothing reaches the robot or the CBF.
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import c3_replay as rp  # noqa: E402
import c3_score as cs  # noqa: E402

FRONT_ENDS = {
    'pose': dict(model='models/yolo26n-pose.pt', name='c3-det-pose.json', masks=False),
    'seg': dict(model='models/yolo26n-seg.pt', name='c3-det-seg.json', masks=True),
}
# The live OAK's nn_conf. The replay default (0.35) admits the lab's
# towel-on-chair phantom for both models, which is not what would run live.
CONF = 0.5
STAND_RE = re.compile(r'^stand-(side-)?(\d+(?:\.\d+)?)m-r\d+$')


def stand_accuracy(run_out, run_name):
    """Forward distance of arm C's measured, confirmed outputs vs the tape."""
    m = STAND_RE.match(run_name)
    path = run_out / 'armC_tracks.csv'
    if not m or not path.exists():
        return None
    target = float(m.group(2))
    with path.open() as f:
        rows = [r for r in csv.DictReader(f)
                if r['confirmed'] == 'True' and r['measured'] == 'True']
    if not rows:
        return dict(target_m=target, side_on=bool(m.group(1)), n=0)
    # The person is the track with the most measured outputs; other ids are
    # phantoms or re-acquisitions, counted but kept out of bias/jitter.
    ids = [r['id'] for r in rows]
    person = max(set(ids), key=ids.count)
    z = np.array([float(r['z']) for r in rows if r['id'] == person])
    return dict(target_m=target, side_on=bool(m.group(1)), n=int(z.size),
                bias_m=float(np.median(z) - target),
                jitter_m=float(np.percentile(z, 84) - np.percentile(z, 16)) / 2,
                track_ids=len(set(ids)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    runs = sorted(r for r in args.runs if (r / 'pair-index.json').exists())

    for run in runs:
        for fe in FRONT_ENDS.values():
            if not (run / fe['name']).exists():
                print(f'{run.name}: detect {fe["model"]}', flush=True)
                rp.detect(run, fe['model'], CONF, fe['name'], fe['masks'])
        cs.prepare(run)

    result = {}
    for label, fe in FRONT_ENDS.items():
        out = args.output / label
        report = cs.score(runs, out, detections_name=fe['name'])
        result[label] = dict(
            aggregate_C=cs.aggregate(report)['C'],
            stand={r.name: stand_accuracy(out / r.name, r.name) for r in runs
                   if STAND_RE.match(r.name)})
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'seg_vs_pose.json').write_text(json.dumps(result, indent=1) + '\n')

    # Side-by-side table of every arm C metric both front ends produced.
    keys = sorted(set(result['pose']['aggregate_C']) | set(result['seg']['aggregate_C']))
    lines = ['| metric | pose | seg |', '|---|---|---|']
    for k in keys:
        cells = []
        for label in FRONT_ENDS:
            v = result[label]['aggregate_C'].get(k)
            cells.append('--' if v is None else ', '.join(
                f'{m}={x:.3g}' if isinstance(x, float) else f'{m}={x}' for m, x in v.items()))
        lines.append(f'| {k} | {cells[0]} | {cells[1]} |')
    lines += ['', '| stand run | pose bias / jitter / ids | seg bias / jitter / ids |', '|---|---|---|']
    for name in sorted(result['pose']['stand']):
        cells = []
        for label in FRONT_ENDS:
            s = result[label]['stand'].get(name)
            cells.append('no output' if not s or not s['n'] else
                         f"{s['bias_m']:+.3f} m / {s['jitter_m']:.3f} m / {s['track_ids']}")
        lines.append(f'| {name} | {cells[0]} | {cells[1]} |')
    (args.output / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
