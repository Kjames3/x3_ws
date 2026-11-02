#!/usr/bin/env python3
"""Compare person-candidate extractors on the same data, every check at once.

    python3 src/tools/perception/extractor_compare.py artifacts/person-lidar-2026-10-30/* \
        --own artifacts/lidar-person-classifier-2026-11-01/own.npz \
        [--sit ~/datasets/sit/Cafeteria_1 ...] [--variants base,peaks] [--set NAME=VALUE ...]

Variants: 'base' = person_candidates.extract as deployed, 'peaks' = the same with the
head/shoulder peak split (extract(..., peaks=True)).  --set overrides a
person_candidates constant for the 'peaks' variant only (tuning sweeps).

Checks, all with identical inputs per variant:
  synthetic   synthetic_people scenes (same seed): recall by distance to the nearest
              other person, and duplicates (extra candidates on one person) per person
  clutter     candidates per real frame, and those > 1 m from every labelled real
              person (what a classifier then has to reject)
  real        labelled real people (own.npz) the base extractor found: share the
              variant still finds, and extra candidates on them
  timing      ms per real frame, one process (the robot runs the extractor at 10 Hz)
  sit         SiT pedestrians with >= 10 points: recall by nearest-other distance,
              duplicates (--sit clips only; slower)
"""
import argparse
import functools
import multiprocessing as mp
import pathlib
import sys
import time

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import synthetic_people as sp  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

NN_BINS = sp.NN_BINS
_FRAMES = None


def variant(name, overrides=()):
    if name == 'base':
        return pc.extract
    if name == 'peaks':
        return functools.partial(pc.extract, peaks=True)
    raise ValueError(name)


def _apply(overrides):
    for k, v in overrides:
        setattr(pc, k, v)


def _scene(args):
    name, overrides, job = args
    _apply(overrides)
    return sp.run_scene(job, extract=variant(name))


def synthetic(name, overrides, jobs, pool):
    rows = [r for res in pool.imap_unordered(_scene, [(name, overrides, j) for j in jobs], chunksize=8)
            for r in res]
    rows = [r for r in rows if r['outcome'] != 'hidden']
    out = {}
    for lo, hi in NN_BINS:
        sel = [r for r in rows if lo <= r['nn'] < hi]
        out[(lo, hi)] = (np.mean([r['outcome'] == 'found' for r in sel]), len(sel))
    iso = [r for r in rows if r['nn'] >= 1.5]
    far = [r for r in rows if r['nn'] >= 1.0 and r['range'] >= 6]
    out['far'] = (np.mean([r['outcome'] == 'found' for r in far]) if far else float('nan'), len(far))
    out['dups'] = np.mean([r['dups'] for r in rows])
    out['dups_iso'] = np.mean([r['dups'] for r in iso])
    return out


def _real_one(args):
    name, overrides, (xyz, _t, real) = args
    _apply(overrides)
    ex = variant(name)
    t0 = time.perf_counter()
    c = ex(xyz[:, 0], xyz[:, 1], xyz[:, 2], origin=(0.0, 0.0), range_max=sp.RANGE_MAX_M)
    dt = time.perf_counter() - t0
    if len(real) and len(c):
        d = np.hypot(c[:, None, 0] - real[None, :, 0], c[:, None, 1] - real[None, :, 1])
        clutter = int((d.min(1) > 1.0).sum())
        hit = (d < sp.MATCH_M).any(0)
        extra = np.maximum(0, (d < sp.DUP_M).sum(0) - 1)
    else:
        clutter = len(c)
        hit = np.zeros(len(real), bool)
        extra = np.zeros(len(real), int)
    return len(c), clutter, hit, extra, dt


def real(name, overrides, frames, pool):
    res = pool.map(_real_one, [(name, overrides, f) for f in frames], chunksize=8)
    return dict(cands=np.mean([r[0] for r in res]), clutter=np.mean([r[1] for r in res]),
                hit=[r[2] for r in res], extra=np.concatenate([r[3] for r in res]))


def timing(name, overrides, frames, n=150):
    _apply(overrides)
    ex = variant(name)
    ts = []
    for xyz, _t, _r in frames[:n]:
        t0 = time.perf_counter()
        ex(xyz[:, 0], xyz[:, 1], xyz[:, 2], origin=(0.0, 0.0), range_max=8.0)   # live range
        ts.append(time.perf_counter() - t0)
    ts = np.array(ts) * 1000
    return float(np.median(ts)), float(np.percentile(ts, 95))


# ---------------------------------------------------------------- SiT

def _sit_frame(args):
    name, overrides, clip, fi, floor = args
    _apply(overrides)
    import sit_pcd
    import sit_person_dataset as spd
    try:
        p = sit_pcd.read_pcd(f'{clip}/velo/concat/data/{fi}.pcd')
        boxes = spd._boxes_ego(sit_pcd.read_labels(f'{clip}/label_3d/{fi}.txt'),
                               sit_pcd.read_pose(f'{clip}/ego_trajectory/{fi}.txt'))
    except FileNotFoundError:
        return []
    x, y = p['x'].astype(np.float64), p['y'].astype(np.float64)
    z = p['z'].astype(np.float64) - floor
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (np.abs(x) + np.abs(y) > 0.05)
    x, y, z = x[ok], y[ok], z[ok]
    ped = [b for b in boxes if b[0] == 'Pedestrian' and np.hypot(b[4], b[5]) < 11.5]
    keep = []
    for b in ped:
        n = ((np.abs(x - b[4]) < 0.6) & (np.abs(y - b[5]) < 0.6) & (np.abs(z - (b[6] - floor)) < b[1] / 2)).sum()
        if n >= 10:
            keep.append(b)
    if not keep:
        return []
    cands = variant(name)(x, y, z, range_max=12.0)
    allp = np.array([[b[4], b[5]] for b in boxes if b[0] in ('Pedestrian', 'Pedestrain_sitting')])
    rows = []
    for b in keep:
        o = np.hypot(allp[:, 0] - b[4], allp[:, 1] - b[5])
        nn = float(np.sort(o)[1]) if len(o) > 1 else 99.0
        found = any(spd._in_box(c[0], c[1], b, spd.MATCH_PAD_M) for c in cands)
        near = int((np.hypot(cands[:, 0] - b[4], cands[:, 1] - b[5]) < sp.DUP_M).sum()) if len(cands) else 0
        rows.append((nn, found, max(0, near - 1)))
    return rows


def sit(name, overrides, clips, every, pool):
    import glob
    import sit_person_dataset as spd
    jobs = []
    for clip in clips:
        n = len(glob.glob(f'{clip}/velo/concat/data/*.pcd'))
        floor = spd._floor(clip)
        jobs += [(name, overrides, clip, fi, floor) for fi in range(0, n, every)]
    rows = [r for res in pool.imap_unordered(_sit_frame, jobs, chunksize=4) for r in res]
    out = {}
    for lo, hi in NN_BINS:
        sel = [r for r in rows if lo <= r[0] < hi]
        out[(lo, hi)] = (np.mean([r[1] for r in sel]) if sel else float('nan'), len(sel))
    out['dups'] = np.mean([r[2] for r in rows]) if rows else float('nan')
    return out


# ---------------------------------------------------------------- main

def _parse_set(items):
    out = []
    for it in items:
        k, v = it.split('=')
        if not hasattr(pc, k):
            raise SystemExit(f'person_candidates has no {k}')
        out.append((k, type(getattr(pc, k))(float(v))))
    return tuple(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--own', required=True)
    ap.add_argument('--sit', nargs='*', default=[])
    ap.add_argument('--sit-every', type=int, default=5)
    ap.add_argument('--variants', default='base,peaks')
    ap.add_argument('--set', nargs='*', default=[], help='NAME=VALUE overrides for the peaks variant')
    ap.add_argument('--scenes', type=int, default=4000)
    ap.add_argument('--every', type=int, default=5)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    own = dict(np.load(a.own, allow_pickle=True))
    frames = []
    for run in a.runs:
        frames += sp.load_frames(run, a.every, own)
    rng = np.random.default_rng(a.seed)
    pick = rng.integers(len(frames), size=a.scenes)
    jobs = [(frames[i][0], frames[i][1], frames[i][2], a.seed * 1_000_003 + s) for s, i in enumerate(pick)]
    real_frames = frames[::2]
    sets = _parse_set(a.set)
    print(f'{len(frames)} background frames, {a.scenes} synthetic scenes, {len(real_frames)} real frames'
          + (f', peaks overrides {dict(sets)}' if sets else ''))
    res = {}
    with mp.Pool() as pool:
        for name in a.variants.split(','):
            ov = sets if name != 'base' else ()
            r = dict(syn=synthetic(name, ov, jobs, pool), real=real(name, ov, real_frames, pool),
                     time=timing(name, ov, real_frames))
            if a.sit:
                r['sit'] = sit(name, ov, [str(pathlib.Path(c).expanduser()) for c in a.sit], a.sit_every, pool)
            _apply([(k, getattr(pc, k)) for k, _ in ov])   # (forked workers carry their own)
            res[name] = r
    base_hit = res.get('base', {}).get('real', {}).get('hit')
    names = list(res)
    w = 13
    print('\n' + f'{"":34s}' + ''.join(f'{n:>{w}s}' for n in names))

    def line(label, f):
        print(f'{label:34s}' + ''.join(f'{f(res[n]):>{w}s}' for n in names))
    for lo, hi in NN_BINS:
        tag = 'alone (> 1.5 m)' if lo >= 1.5 else f'{lo}-{hi} m'
        line(f'synthetic recall, nearest {tag}', lambda r, k=(lo, hi): f'{100 * r["syn"][k][0]:.1f}% /{r["syn"][k][1]}')
    line('synthetic recall, >= 6 m, nn >= 1 m', lambda r: f'{100 * r["syn"]["far"][0]:.1f}% /{r["syn"]["far"][1]}')
    line('duplicates per person (all)', lambda r: f'{r["syn"]["dups"]:.3f}')
    line('duplicates per person (alone)', lambda r: f'{r["syn"]["dups_iso"]:.3f}')
    line('real: candidates / frame', lambda r: f'{r["real"]["cands"]:.2f}')
    line('real: clutter (> 1 m from people)', lambda r: f'{r["real"]["clutter"]:.2f}')
    if base_hit is not None:
        bh = np.concatenate(base_hit)

        def kept(r):
            h = np.concatenate(r['real']['hit'])
            return f'{100 * h[bh].mean():.1f}% /{bh.sum()}' if bh.any() else '--'
        line('real people base found, still found', kept)
    line('real: extra candidates per person', lambda r: f'{r["real"]["extra"].mean():.3f}')
    line('ms / frame (8 m, median)', lambda r: f'{r["time"][0]:.2f}')
    line('ms / frame (8 m, p95)', lambda r: f'{r["time"][1]:.2f}')
    if a.sit:
        for lo, hi in NN_BINS:
            tag = 'alone' if lo >= 1.5 else f'{lo}-{hi} m'
            line(f'SiT recall, nearest {tag}', lambda r, k=(lo, hi): f'{100 * r["sit"][k][0]:.1f}% /{r["sit"][k][1]}')
        line('SiT duplicates per person', lambda r: f'{r["sit"]["dups"]:.3f}')


if __name__ == '__main__':
    main()
