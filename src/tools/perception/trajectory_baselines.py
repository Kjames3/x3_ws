#!/usr/bin/env python3
"""Pedestrian trajectory-prediction baselines + ADE/FDE harness.

Replays /pedestrian_states from recorded runs (bag/*.db3 under each run dir),
puts every track in the odom frame with the robot's real pose (the same
transform orca_cbf_compare uses), resamples each track to a fixed rate, cuts
history/future windows and scores simple predictors at several horizons.

Ground truth is the tracker's own future positions, so the numbers measure
"how well can you forecast what the tracker will report", noise included. The
point is a bar for a learned model (GRU etc.) to beat, not an absolute accuracy.

Predictors
  stationary   last position held
  cv_est       constant velocity from the estimator's own velocity at t0
  cv_fit       constant velocity from a least-squares line over the last FIT_S
  cv_damped    cv_fit with exponential speed decay (tau DAMP_TAU)
  ctrv         constant speed + turn rate fitted over the history

--export writes the windows (history, future, run index) as .npz for training.

  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/tools/perception/trajectory_baselines.py artifacts/cbf-2026-10-1*/walk-* \
      --out artifacts/traj-baselines-2026-11-08
"""
import argparse
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1] / 'nav'))
sys.path.insert(0, str(HERE.parents[2]))

RATE_HZ = 10.0
GAP_S = 0.35          # a track with a hole longer than this is split
HIST_S = 1.0
HORIZONS = (0.5, 1.0, 2.0, 3.0)
FIT_S = 0.5
DAMP_TAU = 1.5
MOVING_MS = 0.3       # "moving" windows: mean ground-truth speed over the horizon above this


# ───────────────────────────── track extraction ─────────────────────────────

def tracks_from_run(run):
    """{id: array[N, 5] (t, x, y, vx, vy)} in the odom frame."""
    out = {}
    for t, people in run['wped']:
        for i, p, v in people:
            out.setdefault(i, []).append((t, p[0], p[1], v[0], v[1]))
    return {i: np.array(sorted(r)) for i, r in out.items()}


def dedupe(tracks, tol=0.02):
    """Drop tracks that duplicate another id sample-for-sample (seen in busy-r2)."""
    keep, ids = {}, sorted(tracks, key=lambda i: -len(tracks[i]))
    for i in ids:
        a = tracks[i]
        dup = False
        for j, b in keep.items():
            if len(a) == len(b) and np.allclose(a[:, :3], b[:, :3], atol=tol):
                dup = True
                break
        if not dup:
            keep[i] = a
    return keep


def segments(track, rate=RATE_HZ, gap=GAP_S):
    """Split at holes and resample each piece to a uniform grid."""
    t = track[:, 0]
    cuts = np.where(np.diff(t) > gap)[0] + 1
    for piece in np.split(track, cuts):
        if len(piece) < 2:
            continue
        g = np.arange(piece[0, 0], piece[-1, 0] + 1e-9, 1.0 / rate)
        yield np.column_stack([g] + [np.interp(g, piece[:, 0], piece[:, k]) for k in range(1, 5)])


def windows(seg, hist_n, fut_n):
    """(history[hist_n, 4: x y vx vy], future[fut_n, 2]) pairs, stride 1."""
    n = len(seg)
    for k in range(hist_n - 1, n - fut_n):
        yield seg[k - hist_n + 1:k + 1, 1:5], seg[k + 1:k + 1 + fut_n, 1:3]


# ──────────────────────────────── predictors ────────────────────────────────

def _fit_vel(hist, dt, fit_n):
    h = hist[-fit_n:, :2]
    tt = np.arange(len(h)) * dt
    tt = tt - tt.mean()
    d = (tt ** 2).sum()
    return (tt[:, None] * (h - h.mean(0))).sum(0) / d if d > 0 else np.zeros(2)


def predict(name, hist, fut_n, dt):
    p0 = hist[-1, :2]
    steps = np.arange(1, fut_n + 1)[:, None] * dt
    fit_n = max(2, int(round(FIT_S / dt)) + 1)
    if name == 'stationary':
        return np.repeat(p0[None], fut_n, 0)
    if name == 'cv_est':
        return p0 + steps * hist[-1, 2:4]
    if name == 'cv_fit':
        return p0 + steps * _fit_vel(hist, dt, fit_n)
    if name == 'cv_damped':
        v = _fit_vel(hist, dt, fit_n)
        return p0 + v * DAMP_TAU * (1 - np.exp(-steps / DAMP_TAU))
    if name == 'ctrv':
        # speed from the recent fit, turn rate from the heading change between
        # the two halves of the history
        v = _fit_vel(hist, dt, fit_n)
        s = float(np.hypot(*v))
        half = len(hist) // 2
        if s < 0.15 or half < 2:
            return p0 + steps * v
        v1, v2 = _fit_vel(hist[:half + 1], dt, half + 1), _fit_vel(hist[half:], dt, len(hist) - half)
        if np.hypot(*v1) < 0.15:
            return p0 + steps * v
        w = (math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0]) + math.pi) % (2 * math.pi) - math.pi
        w = float(np.clip(w / (half * dt), -1.5, 1.5))
        th = math.atan2(v[1], v[0]) + w * steps[:, 0]
        # integrate heading at constant speed
        out = np.cumsum(np.column_stack([np.cos(th), np.sin(th)]) * s * dt, 0)
        return p0 + out
    raise ValueError(name)


PREDICTORS = ('stationary', 'cv_est', 'cv_fit', 'cv_damped', 'ctrv')


# ───────────────────────────────── harness ──────────────────────────────────

def collect(paths, rate=RATE_HZ, hist_s=HIST_S, horizons=HORIZONS):
    import orca_cbf_compare as occ
    hist_n = int(round(hist_s * rate)) + 1
    fut_n = int(round(max(horizons) * rate))
    H, F, R, runs = [], [], [], []
    for p in paths:
        try:
            run = occ.to_world(occ.load_run(p))
        except Exception as e:  # noqa: BLE001  (bad / empty bag)
            print('skip %s: %s' % (p, e), file=sys.stderr)
            continue
        ri = len(runs)
        runs.append(Path(p).name if Path(p).name != 'bag' else Path(p).parent.name)
        for tr in dedupe(tracks_from_run(run)).values():
            for seg in segments(tr, rate):
                for h, f in windows(seg, hist_n, fut_n):
                    H.append(h)
                    F.append(f)
                    R.append(ri)
    if not H:
        raise SystemExit('no windows: tracks too short for %.1f s history + %.1f s future'
                         % (hist_s, max(horizons)))
    return np.array(H), np.array(F), np.array(R), runs


def score(H, F, rate=RATE_HZ, horizons=HORIZONS, predictors=PREDICTORS):
    dt = 1.0 / rate
    fut_n = F.shape[1]
    preds = {n: np.array([predict(n, h, fut_n, dt) for h in H]) for n in predictors}
    res = {}
    for hz in horizons:
        k = int(round(hz * rate))
        gt = F[:, :k]
        disp = np.linalg.norm(gt[:, -1] - H[:, -1, :2], axis=1)
        moving = disp / hz > MOVING_MS
        res[hz] = {'n': int(len(F)), 'n_moving': int(moving.sum())}
        for n, P in preds.items():
            e = np.linalg.norm(P[:, :k] - gt, axis=2)
            ade, fde = e.mean(1), e[:, -1]
            row = {}
            for tag, m in (('all', np.ones(len(F), bool)), ('moving', moving)):
                if m.any():
                    row[tag] = dict(ade=float(ade[m].mean()), fde=float(fde[m].mean()),
                                    fde_p90=float(np.percentile(fde[m], 90)))
            res[hz][n] = row
    return res


def report(res, runs, n_windows):
    lines = ['runs: %d   windows: %d' % (len(runs), n_windows)]
    for hz, r in res.items():
        lines.append('\nhorizon %.1f s   (%d windows, %d moving > %.1f m/s)' % (hz, r['n'], r['n_moving'], MOVING_MS))
        lines.append('  %-11s %22s   %22s' % ('', 'all: ADE  FDE  FDEp90', 'moving: ADE  FDE  FDEp90'))
        for n in PREDICTORS:
            a, m = r[n].get('all'), r[n].get('moving')
            f = lambda d: '%5.2f %5.2f %6.2f' % (d['ade'], d['fde'], d['fde_p90']) if d else '     -'
            lines.append('  %-11s %22s   %22s' % (n, f(a), f(m)))
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='run dirs (containing bag/*.db3); globs ok')
    ap.add_argument('--out', help='write report.txt / results.json here')
    ap.add_argument('--export', help='write windows to this .npz for training')
    ap.add_argument('--rate', type=float, default=RATE_HZ)
    ap.add_argument('--hist', type=float, default=HIST_S)
    a = ap.parse_args()
    paths = sorted({p for g in a.runs for p in (glob.glob(g) or [g])
                    if list(Path(p).glob('bag/*.db3')) or p.endswith('.db3')})
    H, F, R, runs = collect(paths, a.rate, a.hist)
    res = score(H, F, a.rate)
    txt = report(res, runs, len(H))
    print(txt)
    if a.out:
        o = Path(a.out)
        o.mkdir(parents=True, exist_ok=True)
        (o / 'report.txt').write_text(txt + '\n')
        per_run = {runs[i]: int((R == i).sum()) for i in range(len(runs))}
        (o / 'results.json').write_text(json.dumps(dict(
            runs=per_run, rate=a.rate, hist_s=a.hist, moving_ms=MOVING_MS,
            results={str(k): v for k, v in res.items()}), indent=1))
    if a.export:
        np.savez_compressed(a.export, hist=H, fut=F, run=R, runs=np.array(runs), rate=a.rate)
        print('exported %d windows -> %s' % (len(H), a.export))


if __name__ == '__main__':
    main()
