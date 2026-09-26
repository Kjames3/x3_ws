#!/usr/bin/env python3
"""Score short-horizon position prediction of the C3 arms against the reference.

  python3 src/c3_predict_score.py EVAL_DIR CAPTURE_DIR [--output OUT.json]

EVAL_DIR holds <run>/arm{A,B,C}_tracks.csv from c3_replay.py; CAPTURE_DIR holds
<run>/c3-reference.json. At every time an arm has a confirmed output matched to
the reference person (c3_score rules), each predictor extrapolates to t + h and
is compared with the reference person position at t + h:

  hold  - current position, zero velocity (the "no motion model" floor)
  cv    - position + velocity * h (the arm's own velocity)

Arm A's velocity frame is not documented in velocity_estimator.py, so it is
scored twice: as odom ('cv') and rotated by the odom yaw ('cv_rot'); the robot
is parked in these captures, so yaw is constant and one of the two is right.
The reference shares the depth sensor with every arm: this measures prediction
consistency, not ground-truth accuracy. Diagnostic only.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c3_score as cs  # noqa: E402

HORIZONS_S = (0.5, 1.0)


def load_tracks(csv_path):
    import csv
    by_t = {}
    with csv_path.open() as f:
        for r in csv.DictReader(f):
            if r['confirmed'] != 'True':
                continue
            by_t.setdefault(int(r['t_ns']), []).append(
                dict(x=float(r['x']), y=float(r['y']), vx=float(r['vx'] or 0), vy=float(r['vy'] or 0)))
    return by_t


def odom_yaw(capture):
    """Constant parked-robot yaw, only needed for arm A's 'cv_rot'."""
    try:
        import c3_replay as rp
        odom = rp.load_odom(capture)
        return float(np.median([o[3] for o in odom]))
    except Exception:
        return None


def ref_at(t_ref, pos, t):
    k = int(np.argmin(np.abs(t_ref - t)))
    if abs(t_ref[k] - t) > cs.TIME_MATCH_NS or pos[k] is None:
        return None
    return pos[k]


def score_run(eval_run, capture, arms):
    t_ref, pos, _ = cs.reference_track(capture)
    yaw = odom_yaw(capture) if 'A' in arms else None
    errs = {}
    for arm in arms:
        by_t = load_tracks(eval_run / f'arm{arm}_tracks.csv')
        for t, outs in by_t.items():
            p = ref_at(t_ref, pos, t)
            if p is None or not outs:
                continue
            d = [math.hypot(o['x'] - p['x'], o['y'] - p['y']) for o in outs]
            j = int(np.argmin(d))
            if d[j] > cs.MATCH_M:
                continue
            o = outs[j]
            preds = {'hold': (0.0, 0.0), 'cv': (o['vx'], o['vy'])}
            if arm == 'A' and yaw is not None:
                c, s = math.cos(yaw), math.sin(yaw)
                preds['cv_rot'] = (c * o['vx'] - s * o['vy'], s * o['vx'] + c * o['vy'])
            for h in HORIZONS_S:
                q = ref_at(t_ref, pos, t + int(h * 1e9))
                if q is None:
                    continue
                for name, (vx, vy) in preds.items():
                    e = math.hypot(o['x'] + vx * h - q['x'], o['y'] + vy * h - q['y'])
                    errs.setdefault((arm, name, h), []).append(e)
    return errs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('eval_dir', type=Path)
    ap.add_argument('capture_dir', type=Path)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    table = {}
    for eval_run in sorted(p for p in args.eval_dir.iterdir() if (p / 'armC_tracks.csv').exists()):
        capture = args.capture_dir / eval_run.name
        if not (capture / 'c3-reference.json').exists():
            continue
        scen = cs.scenario_of(eval_run)
        for key, e in score_run(eval_run, capture, 'ABC').items():
            table.setdefault(scen, {}).setdefault('%s/%s/%.1fs' % key, []).extend(e)
    report = {scen: {k: dict(n=len(v), p50=float(np.median(v)), p90=float(np.percentile(v, 90)))
                     for k, v in sorted(rows.items())}
              for scen, rows in sorted(table.items())}
    text = json.dumps(report, indent=1)
    if args.output:
        args.output.write_text(text + '\n')
    for scen, rows in report.items():
        print(f'\n== {scen}')
        for k, r in rows.items():
            print(f'  {k:18s} n={r["n"]:5d}  p50={r["p50"]:.3f}  p90={r["p90"]:.3f}')


if __name__ == '__main__':
    main()
