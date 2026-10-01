#!/usr/bin/env python3
"""People cut by the image edge: how wrong is box depth, and what does it do to arm C?

  python3 src/c3_edge_test.py RUN_DIR... --output OUT

Needs the caches from c3_seg_vs_pose.py and c3_calibration.py. Three parts:

1. Depth vs the tape line. On crossing-<d>m runs the person walks a line <d> m
   from the lens, so measured depth should be <d>. Reported for the yolo11x
   reference, pose (box torso depth) and merged seg (silhouette depth), split
   by whether the box touches the left/right image edge.

2. Fair scoring. c3_score's reference uses box depth too, so it is wrong in
   the same edge frames and agrees with pose there. Here the reference keeps
   only frames whose box is fully inside the image (c3-reference-inside.json),
   and its speed is fitted on those frames only. Edge frames are then unscored,
   so the scorer's non-person counts are meaningless here and are dropped.

3. pose-noedge: pose detections with edge-touching boxes removed, so the
   tracker coasts through entry and exit instead of taking a wrong depth.

Arm C runs with the live ARM_C_CONFIG. Offline only.
"""
import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import c3_score as cs  # noqa: E402
from c3_live import ARM_C_CONFIG  # noqa: E402

EDGE_PX = 3              # a box within this of the left/right border is clipped
OFF_M = 0.4              # depth this far from the tape line counts as wrong
INSIDE_REFERENCE = 'c3-reference-inside.json'
FRONT_ENDS = {'pose': 'c3-det-pose.json', 'pose-noedge': 'c3-det-pose-noedge.json',
              'seg-merged': 'c3-det-seg-merged.json'}
MEAS = {'pose': 'c3-meas-pose.json', 'seg-merged': 'c3-meas-seg-merged.json'}
CROSS_RE = re.compile(r'^cross(?:ing)?-(\d+(?:\.\d+)?)m-r\d+$')


def image_width(run):
    return json.loads((run / 'pair-index.json').read_text())[0]['depth']['width']


def clipped(xyxy, width):
    return xyxy[0] <= EDGE_PX or xyxy[2] >= width - EDGE_PX


def tape_depth(run, width, acc):
    """Part 1: (source, line, edge|inside) -> [depth - line]."""
    m = CROSS_RE.match(run.name)
    if not m:
        return
    line = float(m.group(1))
    ref = json.loads((run / 'c3-reference.json').read_text())['frames']
    sources = {'reference': [([f['person']['z']] if f['person'] else [],
                              [max(f['boxes'], key=lambda b: b['conf'])['xyxy']] if f['boxes'] else [])
                             for f in ref]}
    for fe, meas_name in MEAS.items():
        if not (run / meas_name).exists():
            continue
        meas = json.loads((run / meas_name).read_text())
        det = json.loads((run / FRONT_ENDS[fe]).read_text())['frames']
        sources[fe] = [([x['z'] for x in mf['meas']], [b['xyxy'] for b in df['boxes']])
                       for mf, df in zip(meas, det)]
    for name, frames in sources.items():
        for zs, boxes in frames:
            if len(zs) != 1 or len(boxes) != 1:   # unambiguous box <-> depth only
                continue
            acc[(name, line, 'edge' if clipped(boxes[0], width) else 'inside')].append(zs[0] - line)


def write_inside_reference(run, width):
    ref = json.loads((run / 'c3-reference.json').read_text())
    dropped = 0
    for f in ref['frames']:
        if f['person'] is not None and clipped(max(f['boxes'], key=lambda b: b['conf'])['xyxy'], width):
            f['person'] = None
            dropped += 1
    ref['position'] += '; frames with an edge-clipped box removed'
    (run / INSIDE_REFERENCE).write_text(json.dumps(ref) + '\n')
    return dropped


def write_noedge(run, width):
    det = json.loads((run / FRONT_ENDS['pose']).read_text())
    n_in = n_out = 0
    for f in det['frames']:
        n_in += len(f['boxes'])
        f['boxes'] = [b for b in f['boxes'] if not clipped(b['xyxy'], width)]
        n_out += len(f['boxes'])
    det['edge_filter'] = dict(edge_px=EDGE_PX)
    (run / FRONT_ENDS['pose-noedge']).write_text(json.dumps(det) + '\n')
    return n_in, n_out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    runs = sorted(r for r in args.runs if (r / 'c3-reference.json').exists())

    acc = defaultdict(list)
    for run in runs:
        w = image_width(run)
        tape_depth(run, w, acc)
        dropped = write_inside_reference(run, w)
        n_in, n_out = write_noedge(run, w)
        print(f'{run.name}: prepared, reference -{dropped} edge frames, pose boxes {n_in} -> {n_out}',
              flush=True)

    lines = ['## 1. Depth vs the tape line on crossings', '',
             f'| line m | source | inside: n, off > {OFF_M} m, median | edge: n, off > {OFF_M} m, median |',
             '|---|---|---|---|']
    tape = {}
    for line in sorted({k[1] for k in acc}):
        for name in ('reference', 'pose', 'seg-merged'):
            cells = []
            for where in ('inside', 'edge'):
                a = np.array(acc.get((name, line, where), []))
                tape[f'{line:g} | {name} | {where}'] = dict(
                    n=int(a.size), off_pct=100 * float(np.mean(np.abs(a) > OFF_M)) if a.size else None,
                    median_m=float(np.median(a)) if a.size else None)
                cells.append('--' if not a.size else
                             f'{a.size}, {100 * np.mean(np.abs(a) > OFF_M):.0f}%, {np.median(a):+.2f} m')
            lines.append(f'| {line:g} | {name} | {cells[0]} | {cells[1]} |')

    cs.REFERENCE_FILE = INSIDE_REFERENCE
    result = {}
    for label, det_name in FRONT_ENDS.items():
        report = cs.score(runs, args.output / label, config_c=dict(ARM_C_CONFIG),
                          detections_name=det_name)
        result[label] = {k: v for k, v in cs.aggregate(report)['C'].items() if 'nonperson' not in k}
    labels = list(result)
    lines += ['', '## 2-3. Arm C vs the inside-only reference (live tracker settings)', '',
              '| metric | ' + ' | '.join(labels) + ' |', '|---' * (len(labels) + 1) + '|']
    for k in sorted(set().union(*(result[fe] for fe in labels))):
        cells = []
        for fe in labels:
            v = result[fe].get(k)
            cells.append('--' if v is None else ', '.join(
                f'{m}={x:.3g}' if isinstance(x, float) else f'{m}={x}' for m, x in v.items()))
        lines.append(f'| {k} | ' + ' | '.join(cells) + ' |')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'edge_test.json').write_text(json.dumps(dict(tape=tape, scores=result), indent=1) + '\n')
    (args.output / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
