#!/usr/bin/env python3
"""Summarize OAK detection logs written by x3_server --oak-det-log.

  python3 src/oak_pose_summary.py LOG.ndjson [LOG2.ndjson ...] [--from S] [--to S]
                                  [--timeline 5] [--notes notes.txt]

Per log: model, duration, NN packet rate, person detections per minute, and
(for pose models) the fraction of person detections with each body part
visible (>= 0.5) per distance bin. Distance is the detection's box depth
(xyz_m.z). --from/--to (seconds from the log's first packet) cut one test
segment out of a longer log; --timeline prints per-window rows so segments
can be found (e.g. which 10 s you stood at the 1.2 m mark). --notes takes the
"HH:MM:SS label" lines written during the test (same clock as the robot) and
prints one row per segment; a segment runs until the next note. Notes outside
the log's time span are ignored, so one notes file can cover several runs.
"""
import argparse
import json
from pathlib import Path

import numpy as np

VIS = 0.5
PARTS = {'head': [0], 'shoulders': [5, 6], 'hips': [11, 12], 'knees': [13, 14], 'ankles': [15, 16]}
BINS = [(0.0, 1.0), (1.0, 1.5), (1.5, 2.2), (2.2, 3.0), (3.0, 4.5), (4.5, 99.0)]


def load(path, t_from=None, t_to=None):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    if not rows:
        return rows
    t0 = rows[0]['t_mono']
    for r in rows:
        r['t'] = r['t_mono'] - t0
    return [r for r in rows if (t_from is None or r['t'] >= t_from) and (t_to is None or r['t'] <= t_to)]


def persons(rows):
    for r in rows:
        for d in r['dets']:
            if d.get('label') == 'person':
                yield r, d


def depth(d):
    return (d.get('xyz_m') or {}).get('z')


def part_visible(d, idx):
    kp = d.get('keypoints')
    return kp is not None and all(kp[i][2] >= VIS for i in idx)


def summarize(rows):
    if len(rows) < 2:
        return {'packets': len(rows)}
    dur = rows[-1]['t'] - rows[0]['t']
    dets = list(persons(rows))
    out = {'model': rows[0].get('model'), 'duration_s': round(dur, 1), 'packets': len(rows),
           'nn_hz': round((len(rows) - 1) / dur, 2) if dur > 0 else None,
           'packets_with_person': round(np.mean([any(d.get('label') == 'person' for d in r['dets'])
                                                 for r in rows]), 3),
           'person_dets_per_min': round(len(dets) / dur * 60, 1) if dur > 0 else None,
           'no_depth': sum(depth(d) is None for _, d in dets)}
    has_kp = any('keypoints' in d for _, d in dets)
    if has_kp:
        table = {}
        for lo, hi in BINS:
            sel = [d for _, d in dets if depth(d) is not None and lo <= depth(d) < hi]
            if sel:
                table[f'{lo:g}-{hi:g}m'] = dict(n=len(sel), **{
                    p: round(float(np.mean([part_visible(d, i) for d in sel])), 2) for p, i in PARTS.items()})
        out['visibility'] = table
    return out


def timeline(rows, window):
    if not rows:
        return
    has_kp = any('keypoints' in d for _, d in persons(rows))
    print(f'{"t_s":>6} {"pkts":>5} {"person%":>8} {"z_med":>6}' + ('  hips  ankles' if has_kp else ''))
    t, end = rows[0]['t'], rows[-1]['t']
    while t < end:
        w = [r for r in rows if t <= r['t'] < t + window]
        ds = [d for _, d in persons(w)]
        zs = [depth(d) for d in ds if depth(d) is not None]
        line = (f'{t:6.0f} {len(w):5d} {np.mean([any(x.get("label") == "person" for x in r["dets"]) for r in w]) if w else 0:8.0%}'
                f' {np.median(zs) if zs else float("nan"):6.2f}')
        if has_kp and ds:
            line += f'  {np.mean([part_visible(d, PARTS["hips"]) for d in ds]):4.0%}  {np.mean([part_visible(d, PARTS["ankles"]) for d in ds]):6.0%}'
        print(line)
        t += window


def note_segments(rows, notes_path):
    """[(label, start_wall, end_wall)] from 'HH:MM:SS label' lines, anchored to
    the log's first wall time; times before it roll over to the next day."""
    import datetime as dt
    if not rows:
        return []
    first = dt.datetime.fromtimestamp(rows[0]['t_wall'])
    marks = []
    for line in Path(notes_path).read_text().splitlines():
        if not line.strip():
            continue
        hms, label = line.strip().split(' ', 1)
        # The day that puts the note nearest the log start (runs cross midnight).
        t = min((dt.datetime.combine(first.date() + dt.timedelta(days=k), dt.time.fromisoformat(hms))
                 for k in (-1, 0, 1)), key=lambda c: abs((c - first).total_seconds()))
        marks.append((t.timestamp(), label))
    marks.sort()
    start, end = rows[0]['t_wall'], rows[-1]['t_wall']
    # Notes from another run in the same file fall outside this log; drop them.
    marks = [m for m in marks if start - 60 <= m[0] < end]
    return [(lab, a, b) for (a, lab), (b, _) in zip(marks, marks[1:] + [(end, None)])]


def segment_table(rows, notes_path):
    has_kp = any('keypoints' in d for _, d in persons(rows))
    head = f'{"segment":20s} {"dur":>4} {"Hz":>5} {"person%":>7} {"z_med":>5} {"conf":>5}'
    print(head + ('  head shoulders hips knees ankles' if has_kp else ''))
    for label, a, b in note_segments(rows, notes_path):
        seg = [r for r in rows if a <= r['t_wall'] < b]
        if not seg:
            print(f'{label:20s} no packets')
            continue
        ds = [d for _, d in persons(seg)]
        zs = [depth(d) for d in ds if depth(d) is not None]
        line = (f'{label:20s} {b - a:4.0f} {len(seg) / (b - a):5.2f} '
                f'{np.mean([any(d.get("label") == "person" for d in r["dets"]) for r in seg]):7.0%} '
                f'{np.median(zs) if zs else float("nan"):5.2f} '
                f'{np.median([d["conf"] for d in ds]) if ds else float("nan"):5.2f}')
        if has_kp and ds:
            line += ''.join(f' {np.mean([part_visible(d, i) for d in ds]):{max(4, len(p))}.0%}'
                            for p, i in PARTS.items())
        print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', type=Path, nargs='+')
    ap.add_argument('--from', dest='t_from', type=float)
    ap.add_argument('--to', dest='t_to', type=float)
    ap.add_argument('--timeline', type=float, metavar='SECONDS')
    ap.add_argument('--notes', type=Path, help='"HH:MM:SS label" segment notes')
    args = ap.parse_args()
    for p in args.logs:
        rows = load(p, args.t_from, args.t_to)
        print(f'\n== {p.name}')
        print(json.dumps(summarize(rows), indent=1))
        if args.notes:
            segment_table(load(p), args.notes)
        if args.timeline:
            timeline(rows, args.timeline)


if __name__ == '__main__':
    main()
