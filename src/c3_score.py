#!/usr/bin/env python3
"""Score C3 arms A/B/C against an independent person reference.

  python3 src/c3_score.py prepare RUN_DIR...        # arm-C boxes + reference
  python3 src/c3_score.py score RUN_DIR... --output OUT [--config-c JSON]

The reference is YOLO11x (not arm C's yolo26n) plus the same robust torso
depth, placed in odom with the recorded camera TF. It is not ground truth: its
position shares the depth sensor with every arm, so position/speed agreement is
a consistency measure. Recall, false motion on non-person outputs and
event delays against it are the usable signals. Runs are parked-robot C3
captures named <scenario>-rN (empty, cross-*, approach, startstop, stand-*).
"""
import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import c3_replay as rp  # noqa: E402
import c3_person_tracker as pt  # noqa: E402
from c1_dataset import load_pair  # noqa: E402

REFERENCE_MODEL = 'models/yolo11x.pt'
REFERENCE_CONF = 0.5
MATCH_M = 0.5            # output within this of the reference person = person output
TIME_MATCH_NS = 60_000_000
SPEED_WINDOW_S = 0.5     # reference speed: centred linear fit over +/- this
FALSE_SPEED = 0.15       # plan threshold for a false-motion event
BINS = rp.RANGE_BINS[:3]


def prepare(run):
    if not (run / 'c3-detections.json').exists():
        rp.detect(run)
    ref_path = run / 'c3-reference.json'
    if ref_path.exists():
        return
    rp.detect(run, REFERENCE_MODEL, REFERENCE_CONF, 'c3-reference.json')
    ref = json.loads(ref_path.read_text())
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
    ref['position'] = 'torso median depth in the highest-confidence box, odom via recorded TF'
    ref_path.write_text(json.dumps(ref) + '\n')


def reference_track(run):
    frames = json.loads((run / 'c3-reference.json').read_text())['frames']
    t = np.array([f['depth_stamp_ns'] for f in frames], dtype=np.int64)
    pos = [f['person'] for f in frames]
    speed = np.full(len(t), np.nan)
    for i, p in enumerate(pos):
        if p is None:
            continue
        idx = [j for j in range(len(t)) if pos[j] is not None and abs(t[j] - t[i]) <= SPEED_WINDOW_S * 1e9]
        if len(idx) < 5:
            continue
        tt = (t[idx] - t[i]) / 1e9
        vx = np.polyfit(tt, [pos[j]['x'] for j in idx], 1)[0]
        vy = np.polyfit(tt, [pos[j]['y'] for j in idx], 1)[0]
        speed[i] = math.hypot(vx, vy)
    return t, pos, speed


def load_arm(csv_path):
    by_t = defaultdict(list)
    with csv_path.open() as f:
        for r in csv.DictReader(f):
            if r['confirmed'] != 'True':
                continue
            by_t[int(r['t_ns'])].append(dict(x=float(r['x']), y=float(r['y']), z=float(r['z']),
                                             speed=float(r['speed']),
                                             measured=r['measured'] == 'True'))
    return by_t


def eval_times(run, arm, by_t):
    stamps = [r['depth']['stamp_ns'] for r in json.loads((run / 'pair-index.json').read_text())]
    if arm != 'A':
        return stamps
    return list(range(stamps[0], stamps[-1] + 1, rp.TICK_NS))


def score_arm(run, arm, by_t, ref):
    t_ref, pos, ref_speed = ref
    rows = []   # per evaluation time
    for t in eval_times(run, arm, by_t):
        k = int(np.argmin(np.abs(t_ref - t)))
        person = pos[k] if abs(t_ref[k] - t) <= TIME_MATCH_NS else None
        outs = by_t.get(t, [])
        matched, others = None, outs
        if person is not None and outs:
            d = [math.hypot(o['x'] - person['x'], o['y'] - person['y']) for o in outs]
            j = int(np.argmin(d))
            if d[j] <= MATCH_M:
                matched = outs[j]
                others = outs[:j] + outs[j + 1:]
        rows.append(dict(t=t, person=person, ref_speed=ref_speed[k] if person else np.nan,
                         matched=matched, others=others))
    return rows


def summarize(rows, scenario, duration_s):
    res = {}
    for lo, hi in BINS:
        pr = [r for r in rows if r['person'] and lo <= r['person']['z'] < hi]
        if not pr:
            continue
        b = dict(n=len(pr), recall=float(np.mean([r['matched'] is not None for r in pr])))
        sp = np.array([r['matched']['speed'] for r in pr if r['matched']])
        if scenario == 'stand' and sp.size:
            b.update(static_p50=float(np.median(sp)), static_p95=float(np.percentile(sp, 95)),
                     static_frac_gt_0_15=float(np.mean(sp > FALSE_SPEED)))
        walk = [(r['matched']['speed'], r['ref_speed']) for r in pr
                if r['matched'] and not np.isnan(r['ref_speed']) and r['ref_speed'] > 0.3]
        if scenario != 'stand' and walk:
            w = np.array(walk)
            b.update(walk_n=len(w), walk_median_abs_err=float(np.median(np.abs(w[:, 0] - w[:, 1]))),
                     walk_median_ratio=float(np.median(w[:, 0] / w[:, 1])))
        res[f'person_{lo:g}-{hi:g}m'] = b
    # Predicted-only outputs (a track coasting after its person left view) are
    # stale predictions, a separate error class from measured false motion.
    for label, keep in (('nonperson', True), ('nonperson_predicted', False)):
        other = [o for r in rows for o in r['others'] if o['measured'] == keep]
        fm = [o for o in other if o['speed'] > FALSE_SPEED]
        res[label] = dict(outputs=len(other), false_motion=len(fm),
                          false_motion_per_min=len(fm) / (duration_s / 60),
                          frac_gt_0_15=len(fm) / len(other) if other else 0.0)
    if scenario == 'startstop':
        res['events'] = stop_go_delays(rows)
    return res


def stop_go_delays(rows, horizon_s=2.0):
    """Delays from reference stop/restart to the matched output crossing 0.15 m/s."""
    t = np.array([r['t'] for r in rows]) / 1e9
    rs = np.array([r['ref_speed'] for r in rows])
    armv = np.array([r['matched']['speed'] if r['matched'] else np.nan for r in rows])
    still = rs < 0.15
    events = []
    i = 0
    while i < len(t):
        if still[i] and i > 0 and np.nanmax(rs[max(0, i - 15):i], initial=0) > 0.4:
            j = i
            while j < len(t) and (still[j] or np.isnan(rs[j])):
                j += 1
            if t[min(j, len(t) - 1)] - t[i] >= 1.0:
                events.append(('stop', t[i]))
                if j < len(t):
                    events.append(('go', t[j]))
            i = j
        i += 1
    out = defaultdict(list)
    for kind, te in events:
        w = (t >= te - 0.3) & (t <= te + horizon_s) & ~np.isnan(armv)
        hit = (armv < FALSE_SPEED) if kind == 'stop' else (armv > FALSE_SPEED)
        k = np.where(w & hit)[0]
        out[kind].append(float(t[k[0]] - te) if k.size else None)
    return {k: dict(n=len(v), missed=sum(x is None for x in v),
                    median_s=float(np.median([x for x in v if x is not None])) if any(x is not None for x in v) else None)
            for k, v in out.items()}


def scenario_of(run):
    name = run.name.rsplit('-r', 1)[0]
    return 'stand' if name.startswith('stand') else name.split('-')[0]


def score(runs, output, config_c=None):
    report = {}
    for run in runs:
        prepare(run)
        summary = rp.run(run, output / run.name, None, config_c)
        ref = reference_track(run)
        report[run.name] = {}
        for arm in 'ABC':
            by_t = load_arm(output / run.name / f'arm{arm}_tracks.csv')
            rows = score_arm(run, arm, by_t, ref)
            report[run.name][arm] = summarize(rows, scenario_of(run), summary['duration_s'])
        report[run.name]['reference_person_frames'] = int(sum(p is not None for p in ref[1]))
        print(run.name, 'done', flush=True)
    report['_config'] = dict(tracker_b=summary['tracker_config'], tracker_c=summary['tracker_config_c'],
                             reference=REFERENCE_MODEL, reference_conf=REFERENCE_CONF,
                             match_m=MATCH_M, speed_window_s=SPEED_WINDOW_S)
    (output / 'score.json').write_text(json.dumps(report, indent=1) + '\n')
    return report


def aggregate(report):
    """Pool per-run results into one table per arm (frame-weighted)."""
    agg = defaultdict(lambda: defaultdict(list))
    for run, arms in report.items():
        if run.startswith('_'):
            continue
        scen = scenario_of(Path(run))
        for arm in 'ABC':
            for key, val in arms[arm].items():
                if key == 'events':
                    for kind, e in val.items():
                        agg[arm][f'{kind}_delay'].append(e)
                    continue
                agg[arm][(scen, key)].append(val)
    table = {}
    for arm, items in agg.items():
        t = {}
        for key, vals in items.items():
            if isinstance(key, str):   # event delays
                n = sum(v['n'] for v in vals); miss = sum(v['missed'] for v in vals)
                meds = [v['median_s'] for v in vals if v['median_s'] is not None]
                t[key] = dict(n=n, missed=miss, median_of_run_medians_s=float(np.median(meds)) if meds else None)
                continue
            scen, sub = key
            if sub.startswith('nonperson'):
                t[f'{scen}/{sub}'] = dict(outputs=sum(v['outputs'] for v in vals),
                                          false_per_min=float(np.mean([v['false_motion_per_min'] for v in vals])),
                                          frac_gt_0_15=sum(v['false_motion'] for v in vals) / max(1, sum(v['outputs'] for v in vals)))
            else:
                n = sum(v['n'] for v in vals)
                row = dict(n=n, recall=sum(v['recall'] * v['n'] for v in vals) / n)
                for m in ('static_frac_gt_0_15', 'static_p95', 'walk_median_abs_err', 'walk_median_ratio'):
                    got = [v[m] for v in vals if m in v]
                    if got:
                        row[m] = float(np.median(got))
                t[f'{scen}/{sub}'] = row
        table[arm] = t
    return table


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='mode', required=True)
    p = sub.add_parser('prepare'); p.add_argument('runs', type=Path, nargs='+')
    s = sub.add_parser('score'); s.add_argument('runs', type=Path, nargs='+')
    s.add_argument('--output', type=Path, required=True)
    s.add_argument('--config-c', type=json.loads, default=None)
    a = sub.add_parser('aggregate'); a.add_argument('score_json', type=Path)
    args = parser.parse_args()
    if args.mode == 'aggregate':
        print(json.dumps(aggregate(json.loads(args.score_json.read_text())), indent=1))
        return
    if args.mode == 'prepare':
        for run in args.runs:
            prepare(run)
            print(run.name, 'prepared', flush=True)
    else:
        score(args.runs, args.output, args.config_c)


if __name__ == '__main__':
    main()
