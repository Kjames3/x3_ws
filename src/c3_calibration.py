#!/usr/bin/env python3
"""Is the C3 tracker's stated uncertainty honest? Measurement only.

  python3 src/c3_calibration.py RUN_DIR... --output OUT

Replays arm C (cached pose and merged-seg detections from c3_seg_vs_pose.py)
under two tracker configs -- the replay default and the live ARM_C_CONFIG --
keeping each output's covariance, and asks two questions:

A. Standing runs (stand-*): the person does not move, so the spread of the
   track about its own mean is its real position noise, free of any reference.
   Compared with the stated sigma; 'cover95' is the share of frames inside the
   stated 95 % ellipse (honest = 95). Reported speed should be 0.

B. Walking runs: from each confirmed, measured output predict h seconds ahead
   with constant velocity and the filter's own process noise, and compare with
   the yolo11x reference position at t + h. 'cover95' as above; 'nees' is the
   mean squared error in units of the stated covariance (honest = 2 in 2-D).
   h = 0 is the filtered position itself.

Caveats: the reference shares the depth sensor (position agreement is partly
shared error) and has its own jitter, which pushes B's coverage below the true
value; A has no such problem but only covers a stationary person.
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import c3_replay as rp  # noqa: E402
import c3_score as cs  # noqa: E402
import c3_person_tracker as pt  # noqa: E402
from c1_dataset import load_pair  # noqa: E402
from c3_live import ARM_C_CONFIG  # noqa: E402

DETECTIONS = {'pose': 'c3-det-pose.json', 'seg-merged': 'c3-det-seg-merged.json'}
CONFIGS = {'default': dict(pt.DEFAULT_CONFIG), 'live': dict(ARM_C_CONFIG)}
HORIZONS_S = (0.0, 0.5, 1.0)
CHI2_95 = 5.991          # chi-square, 2 dof
WARMUP_S = 2.0           # standing runs: skip walking onto the mark
MATCH_M = 0.5
TIME_MATCH_NS = 60_000_000


def measurements(run, det_name):
    """Per-frame arm C measurements, cached: tracker configs only change the covariance floor."""
    cache = run / det_name.replace('c3-det-', 'c3-meas-')
    if cache.exists():
        return json.loads(cache.read_text())
    det = json.loads((run / det_name).read_text())
    masks = np.load(run / det['masks_file']) if det.get('masks_file') else None
    rows = json.loads((run / 'pair-index.json').read_text())
    frames = []
    for i in range(len(rows)):
        arrays, row = load_pair(run, i)
        depth_m = rp.depth_metres(arrays, row)
        ofc = row.get('odom_from_camera')
        fx, fy, cx, cy = rp.intrinsics(row)[:4]
        meas = []
        for b in det['frames'][i]['boxes'] if ofc is not None else []:
            sil = None
            if masks is not None and 'mask' in b:
                sil = np.unpackbits(masks[b['mask']])[:depth_m.size].reshape(depth_m.shape).astype(bool)
            m = pt.person_measurement(depth_m, b['xyxy'], mask=sil)
            if m is None:
                continue
            p = [(m['u'] - cx) * m['z'] / fx, (m['v'] - cy) * m['z'] / fy, m['z']]
            xy, rot = pt.camera_point_to_odom(p, ofc)
            meas.append(dict(xy=[float(xy[0]), float(xy[1])], rot=np.asarray(rot).tolist(),
                             z=m['z'], spread=m['spread_m']))
        frames.append(dict(t_ns=row['depth']['stamp_ns'], has_pose=ofc is not None, meas=meas))
    cache.write_text(json.dumps(frames) + '\n')
    return frames


def replay(frames, config):
    trk = pt.KalmanTracker(config)
    floor = trk.config['meas_sigma_floor_m']
    out = []
    for f in frames:
        if not f['has_pose']:
            trk.reset()
            continue
        meas = [(np.array(m['xy']),
                 pt.rotate_cov_to_odom(pt.measurement_cov_camera(m['z'], floor, m['spread']),
                                       np.array(m['rot'])))
                for m in f['meas']]
        for o in trk.update(f['t_ns'] / 1e9, meas):
            if o['confirmed'] and o['measured']:
                out.append(dict(o, t_ns=f['t_ns']))
    return out


def standing(outs):
    if not outs:
        return None
    ids = [o['id'] for o in outs]
    person = max(set(ids), key=ids.count)
    t0 = outs[0]['t_ns']
    o = [x for x in outs if x['id'] == person and x['t_ns'] - t0 >= WARMUP_S * 1e9]
    if len(o) < 30:
        return None
    xy = np.array([[x['x'], x['y']] for x in o])
    err = xy - xy.mean(axis=0)
    cov = np.array([np.asarray(x['cov'])[:2, :2] for x in o])
    nees = np.einsum('ni,nij,nj->n', err, np.linalg.inv(cov), err)
    vel = np.array([[x['vx'], x['vy']] for x in o])
    vcov = np.array([np.asarray(x['cov'])[2:, 2:] for x in o])
    vnees = np.einsum('ni,nij,nj->n', vel, np.linalg.inv(vcov), vel)
    return dict(n=len(o),
                actual_mm=1000 * float(np.sqrt((err ** 2).sum(axis=1).mean())),
                stated_mm=1000 * float(np.sqrt(np.trace(cov, axis1=1, axis2=2).mean())),
                cover95=100 * float((nees <= CHI2_95).mean()),
                speed_rms=float(np.sqrt((vel ** 2).sum(axis=1).mean())),
                speed_stated=float(np.sqrt(np.trace(vcov, axis1=1, axis2=2).mean())),
                speed_cover95=100 * float((vnees <= CHI2_95).mean()))


def walking(outs, ref, accel_sigma):
    """Per-horizon lists of (squared error m^2, nees, stated variance m^2)."""
    t_ref, pos, _ = ref
    res = {h: [] for h in HORIZONS_S}
    by_t = defaultdict(list)
    for o in outs:
        by_t[o['t_ns']].append(o)
    for t, cands in by_t.items():
        k = int(np.argmin(np.abs(t_ref - t)))
        if abs(t_ref[k] - t) > TIME_MATCH_NS or pos[k] is None:
            continue
        d = [math.hypot(o['x'] - pos[k]['x'], o['y'] - pos[k]['y']) for o in cands]
        j = int(np.argmin(d))
        if d[j] > MATCH_M:
            continue          # not the person: association errors are scored elsewhere
        o = cands[j]
        p_full = np.asarray(o['cov'])
        for h in HORIZONS_S:
            kk = int(np.argmin(np.abs(t_ref - (t + h * 1e9))))
            if abs(t_ref[kk] - (t + h * 1e9)) > TIME_MATCH_NS or pos[kk] is None:
                continue
            f = np.hstack([np.eye(2), h * np.eye(2)])
            g = np.array([[h * h / 2, 0], [0, h * h / 2]])
            p = f @ p_full @ f.T + accel_sigma ** 2 * g @ g.T
            e = np.array([o['x'] + h * o['vx'] - pos[kk]['x'], o['y'] + h * o['vy'] - pos[kk]['y']])
            res[h].append((float(e @ e), float(e @ np.linalg.solve(p, e)), float(np.trace(p))))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    runs = sorted(r for r in args.runs if (r / 'c3-reference.json').exists())

    stand = defaultdict(list)                       # (label, distance) -> [per-run dict]
    walk = defaultdict(lambda: defaultdict(list))   # (label, scenario) -> horizon -> rows
    for run in runs:
        scen = cs.scenario_of(run)
        ref = cs.reference_track(run) if scen not in ('stand', 'empty') else None
        for fe, det_name in DETECTIONS.items():
            if not (run / det_name).exists():
                continue
            frames = measurements(run, det_name)
            for cname, cfg in CONFIGS.items():
                label = f'{fe} / {cname}'
                outs = replay(frames, cfg)
                if scen == 'stand':
                    s = standing(outs)
                    if s:
                        stand[(label, run.name.rsplit('-r', 1)[0])].append(s)
                elif ref is not None:
                    for h, rows in walking(outs, ref, cfg['accel_sigma_mps2']).items():
                        walk[(label, scen)][h] += rows
        print(run.name, 'done', flush=True)

    lines = ['## A. Standing: real spread vs stated (honest cover95 = 95)', '',
             '| front end / config | runs | n | actual mm | stated mm | stated/actual | cover95 % '
             '| speed rms m/s | speed stated m/s | speed cover95 % |', '|---' * 10 + '|']
    table = {'standing': {}, 'walking': {}}
    for (label, dist), rs in sorted(stand.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        w = np.array([r['n'] for r in rs], dtype=float)
        avg = {k: float(np.average([r[k] for r in rs], weights=w)) for k in rs[0] if k != 'n'}
        table['standing'][f'{dist} | {label}'] = dict(avg, n=int(w.sum()), runs=len(rs))
        lines.append(f"| {dist} · {label} | {len(rs)} | {int(w.sum())} | {avg['actual_mm']:.0f} | "
                     f"{avg['stated_mm']:.0f} | {avg['stated_mm'] / max(avg['actual_mm'], 1e-9):.1f}x | "
                     f"{avg['cover95']:.0f} | {avg['speed_rms']:.3f} | {avg['speed_stated']:.3f} | "
                     f"{avg['speed_cover95']:.0f} |")
    lines += ['', '## B. Walking: prediction vs reference (honest cover95 = 95, nees = 2)', '',
              '| scenario · front end / config | horizon s | n | rms error m | stated sigma m | cover95 % | nees |',
              '|---' * 7 + '|']
    for (label, scen), hs in sorted(walk.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        for h in HORIZONS_S:
            a = np.array(hs[h])
            if not len(a):
                continue
            row = dict(n=len(a), rms_m=float(np.sqrt(a[:, 0].mean())),
                       stated_m=float(np.sqrt(a[:, 2].mean())),
                       cover95=100 * float((a[:, 1] <= CHI2_95).mean()), nees=float(a[:, 1].mean()))
            table['walking'][f'{scen} | {label} | {h:g}'] = row
            lines.append(f"| {scen} · {label} | {h:g} | {row['n']} | {row['rms_m']:.3f} | "
                         f"{row['stated_m']:.3f} | {row['cover95']:.0f} | {row['nees']:.1f} |")
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'calibration.json').write_text(json.dumps(
        dict(configs=CONFIGS, horizons_s=HORIZONS_S, **table), indent=1) + '\n')
    (args.output / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
