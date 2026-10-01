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
   pose-soft<s>: edge-touching boxes kept, but with <s> m added to their
   measurement sigma, so the tracker uses them weakly.

4. Tracker output vs the tape line on crossings, split the same way (edge
   frames = the pose box is clipped). This is the only edge-frame truth: the
   inside-only reference in part 2 cannot score those frames.

Arm C runs with the live ARM_C_CONFIG. Offline only.
"""
import argparse
import csv
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
SOFT_SIGMAS_M = (0.5, 1.0)
FRONT_ENDS = {'pose': 'c3-det-pose.json', 'pose-noedge': 'c3-det-pose-noedge.json',
              **{f'pose-soft{s:g}': f'c3-det-pose-soft{s:g}.json' for s in SOFT_SIGMAS_M},
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


def write_soft(run, width, sigma):
    det = json.loads((run / FRONT_ENDS['pose']).read_text())
    for f in det['frames']:
        for b in f['boxes']:
            if clipped(b['xyxy'], width):
                b['extra_sigma_m'] = sigma
    det['edge_filter'] = dict(edge_px=EDGE_PX, extra_sigma_m=sigma)
    (run / FRONT_ENDS[f'pose-soft{sigma:g}']).write_text(json.dumps(det) + '\n')


def track_vs_tape(run, width, out_dir, acc):
    """Part 4: (front end, line, edge|inside) -> [track depth - line], plus frame counts."""
    m = CROSS_RE.match(run.name)
    if not m:
        return
    line = float(m.group(1))
    where = {}     # depth stamp -> 'edge' | 'inside', frames with exactly one pose box
    for f in json.loads((run / FRONT_ENDS['pose']).read_text())['frames']:
        if len(f['boxes']) == 1:
            where[f['depth_stamp_ns']] = 'edge' if clipped(f['boxes'][0]['xyxy'], width) else 'inside'
    for fe in FRONT_ENDS:
        path = out_dir / fe / run.name / 'armC_tracks.csv'
        if not path.exists():
            continue
        errs = defaultdict(list)   # frame -> |depth - line| of every confirmed output
        with path.open() as fh:
            for r in csv.DictReader(fh):
                t = int(r['t_ns'])
                if r['confirmed'] == 'True' and t in where:
                    errs[t].append(abs(float(r['z']) - line))
        for t, w in where.items():
            acc[(fe, line, w, 'frames')].append(1)
            if t in errs:
                # One person walks the line, so every confirmed output should be
                # on it: the worst one catches a second track at a wrong depth.
                acc[(fe, line, w, 'err')].append(min(errs[t]))
                acc[(fe, line, w, 'worst')].append(max(errs[t]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--front-ends', default=','.join(FRONT_ENDS),
                    help='comma list to score (default: all)')
    args = ap.parse_args()
    runs = sorted(r for r in args.runs if (r / 'c3-reference.json').exists())

    acc = defaultdict(list)
    for run in runs:
        w = image_width(run)
        tape_depth(run, w, acc)
        dropped = write_inside_reference(run, w)
        n_in, n_out = write_noedge(run, w)
        for sigma in SOFT_SIGMAS_M:
            write_soft(run, w, sigma)
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
    for label in args.front_ends.split(','):
        det_name = FRONT_ENDS[label]
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
    acc4 = defaultdict(list)
    for run in runs:
        track_vs_tape(run, image_width(run), args.output, acc4)
    lines += ['', '## 4. Tracker output vs the tape line on crossings', '',
              f'| line m | front end | inside: tracked, any track off > {OFF_M} m, median abs | '
              f'edge: tracked, any track off > {OFF_M} m, median abs |', '|---|---|---|---|']
    track = {}
    for line in sorted({k[1] for k in acc4}):
        for fe in labels:
            cells = []
            for where in ('inside', 'edge'):
                n = len(acc4.get((fe, line, where, 'frames'), []))
                a = np.array(acc4.get((fe, line, where, 'err'), []))
                worst = np.array(acc4.get((fe, line, where, 'worst'), []))
                track[f'{line:g} | {fe} | {where}'] = dict(
                    frames=n, tracked_pct=100 * a.size / n if n else None,
                    off_pct=100 * float(np.mean(worst > OFF_M)) if a.size else None,
                    median_abs_m=float(np.median(a)) if a.size else None)
                cells.append('--' if not n or not a.size else
                             f'{100 * a.size / n:.0f}%, {100 * np.mean(worst > OFF_M):.0f}%, {np.median(a):.2f} m')
            lines.append(f'| {line:g} | {fe} | {cells[0]} | {cells[1]} |')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'edge_test.json').write_text(json.dumps(
        dict(tape=tape, scores=result, track_vs_tape=track), indent=1) + '\n')
    (args.output / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
