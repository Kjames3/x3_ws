#!/usr/bin/env python3
"""Extractor recall on real recordings, with the camera as the reference.

    python3 src/tools/perception/real_crowd_eval.py artifacts/person-lidar-2026-11-03/* \
        [--variants base,peaks] [--every 1]

Truth per Livox frame = the C3 camera people measured at that time (lidar_persons.ndjson,
cleaned as in lidar_person_dataset), inside the camera's reliable region
(VIEW_RANGE_M, +-VIEW_HALF_DEG).  Camera people closer than CAM_DUP_M to each other are
taken as one (a duplicated track, not two people).  This reference does not come from
either extractor, so a pair the base extractor merges still counts as two people.

Each variant's candidates are assigned one-to-one to the camera people (Hungarian,
gate CAM_MATCH_M), so one candidate between two people cannot count for both.  Reported:
recall by distance to the nearest other camera person (any range), recall by the number
of camera people in the frame, candidates per frame, and in-view candidates more than
1 m from every camera person (clutter, or a person the camera missed).

Limits: the camera itself loses people in tight groups (occluded, or two people as one
box), so the < 0.6 m bin is the camera's pairs, not every pair; and its positions carry
~0.1-0.3 m of depth error, hence the 0.5 m gate.
"""
import argparse
import functools
import math
import multiprocessing as mp
import pathlib
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import lidar_person_dataset as lpd  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

CAM_MATCH_M = lpd.CAM_MATCH_M
CAM_DUP_M = 0.3
NN_BINS = ((0.0, 0.6), (0.6, 1.0), (1.0, 1.5), (1.5, 1e9))
VARIANTS = {'base': pc.extract, 'peaks': functools.partial(pc.extract, peaks=True)}


def dedupe(cam):
    """Merge camera people closer than CAM_DUP_M (greedy, mean position)."""
    out = []
    for p in cam:
        for q in out:
            if math.hypot(p[0] - q[0][0], p[1] - q[0][1]) < CAM_DUP_M:
                q[1].append(p)
                q[0] = np.mean(q[1], axis=0)
                break
        else:
            out.append([np.asarray(p, float), [p]])
    return np.array([q[0] for q in out]).reshape(-1, 2), len(cam) - len(out)


def assign(truth, cands):
    """Boolean per truth row: matched one-to-one within CAM_MATCH_M."""
    hit = np.zeros(len(truth), bool)
    if not len(truth) or not len(cands):
        return hit
    d = np.hypot(truth[:, None, 0] - cands[None, :, 0], truth[:, None, 1] - cands[None, :, 1])
    r, c = linear_sum_assignment(np.where(d < CAM_MATCH_M, d, 1e3))
    hit[r[d[r, c] < CAM_MATCH_M]] = True
    return hit


def run(args):
    run_dir, names, every = args
    rows, frames = [], []
    n_dup = 0
    for k, f in enumerate(lpd.iter_frames(run_dir)):
        if k % every or f['rec'] is None:
            continue
        cam, nd = dedupe(f['cam'])
        n_dup += nd
        cf, sf = math.cos(f['fa']), math.sin(f['fa'])
        dx, dy = cam[:, 0] - f['fx'], cam[:, 1] - f['fy']
        fwd, left = cf * dx + sf * dy, -sf * dx + cf * dy
        rng, brg = np.hypot(fwd, left), np.degrees(np.arctan2(left, fwd))
        view = (np.abs(brg) < lpd.VIEW_HALF_DEG) & (rng > lpd.VIEW_RANGE_M[0]) & (rng < lpd.VIEW_RANGE_M[1])
        if len(cam) > 1:
            d = np.hypot(cam[:, None, 0] - cam[None, :, 0], cam[:, None, 1] - cam[None, :, 1])
            np.fill_diagonal(d, np.inf)
            nn = d.min(1)
        else:
            nn = np.full(len(cam), 1e3)
        truth = cam[view]
        fr = dict(n_cam=int(view.sum()))
        for name in names:
            c = VARIANTS[name](f['wx'], f['wy'], f['z'], origin=(f['ox'], f['oy']), range_max=lpd.RANGE_MAX_M)
            hit = assign(truth, c[:, :2])
            # in-view candidates far from every camera person
            cdx, cdy = c[:, 0] - f['fx'], c[:, 1] - f['fy']
            cfw, clf = cf * cdx + sf * cdy, -sf * cdx + cf * cdy
            cr, cb = np.hypot(cfw, clf), np.degrees(np.arctan2(clf, cfw))
            cv = (np.abs(cb) < lpd.VIEW_HALF_DEG) & (cr > lpd.VIEW_RANGE_M[0]) & (cr < lpd.VIEW_RANGE_M[1])
            if len(cam):
                far = np.hypot(c[:, None, 0] - cam[None, :, 0], c[:, None, 1] - cam[None, :, 1]).min(1) > 1.0
            else:
                far = np.ones(len(c), bool)
            fr[name] = dict(n=len(c), clutter_view=int((cv & far).sum()), hit=hit)
        frames.append(fr)
        for i, (n_, r_) in enumerate(zip(nn[view], rng[view])):
            rows.append(dict(run=pathlib.Path(run_dir).name, nn=float(n_), range=float(r_),
                             group=int(view.sum()),
                             **{name: bool(fr[name]['hit'][i]) for name in names}))
    return pathlib.Path(run_dir).name, rows, frames, n_dup


def pct(sel, name):
    return f'{100 * np.mean([r[name] for r in sel]):.1f}% /{len(sel)}' if sel else '--'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--variants', default='base,peaks')
    ap.add_argument('--every', type=int, default=1)
    a = ap.parse_args()
    names = a.variants.split(',')
    with mp.Pool(min(len(a.runs), mp.cpu_count())) as pool:
        res = pool.map(run, [(r, names, a.every) for r in a.runs])
    rows = [r for _, rr, _, _ in res for r in rr]
    frames = [f for _, _, ff, _ in res for f in ff]
    w = 15
    print(f'{len(frames)} frames, {len(rows)} camera people in view, '
          f'{sum(r[3] for r in res)} duplicate camera tracks merged')
    print('\n' + f'{"":40s}' + ''.join(f'{n:>{w}s}' for n in names))

    def line(label, f):
        print(f'{label:40s}' + ''.join(f'{f(n):>{w}s}' for n in names))
    line('recall, all camera people', lambda n: pct(rows, n))
    for lo, hi in NN_BINS:
        tag = 'alone (> 1.5 m)' if lo >= 1.5 else f'{lo}-{hi} m'
        line(f'recall, nearest other {tag}', lambda n, lo=lo, hi=hi: pct([r for r in rows if lo <= r['nn'] < hi], n))
    for g in (1, 2, 3):
        sel = [r for r in rows if (r['group'] >= g if g == 3 else r['group'] == g)]
        line(f'recall, {g}{"+" if g == 3 else ""} people in view', lambda n, sel=sel: pct(sel, n))
    for lo, hi in ((0.8, 2.0), (2.0, 3.0), (3.0, 3.8)):
        line(f'recall, range {lo}-{hi} m', lambda n, lo=lo, hi=hi: pct([r for r in rows if lo <= r['range'] < hi], n))
    line('candidates / frame', lambda n: f'{np.mean([f[n]["n"] for f in frames]):.2f}')
    line('in-view candidates > 1 m from camera', lambda n: f'{np.mean([f[n]["clutter_view"] for f in frames]):.3f}')
    print('\nper run (all camera people):')
    for name_run, rr, ff, _ in res:
        close = [r for r in rr if r['nn'] < 1.0]
        print(f'  {name_run:14s} {len(ff):5d} frames  mean {np.mean([f["n_cam"] for f in ff]) if ff else 0:.2f} in view  '
              + '  '.join(f'{n} {pct(rr, n)} (nn<1 m {pct(close, n)})' for n in names))


if __name__ == '__main__':
    main()
