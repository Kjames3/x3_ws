#!/usr/bin/env python3
"""Offline evaluation of camera-seeded lidar person candidates (seated people at desks).

    python3 src/tools/perception/camera_seed_eval.py --model MODEL.joblib \
        --overrides label_overrides.json --jobs 24 -o results.txt RUN_DIR [RUN_DIR ...]

Base arm: person_candidates.extract as deployed, labelled and featured exactly as
lidar_person_dataset.process_run does (tracklets + overrides, cluster_features on the
candidate box).  Seeded arm: the base candidates plus person_candidates.seeded_candidates
for each measured C3 camera person of the frame (deduplicated against the base ones).

Seeded candidates are never labelled by the camera (they sit on a camera person by
construction, so the auto-labeller would always call them people).  Their ground truth
is: every seeded candidate of an EMPTY run (EMPTY_RUNS: nobody in the camera's view) is
clutter, and fixed-XY hand overrides (label_overrides.json entries without near_camera)
label the rest where they apply; everything else is 'camera-confirmed, unlabelled'.

Per run:
  cam      measured camera-person instances (person x frame)
  found    instances with a candidate within MATCH_M of the camera XY (base / seeded arm);
           'ray' = base candidate on the camera's line of sight instead (the camera's range
           over-reads seated people by 0.5-1 m, so MATCH_M undercounts the base arm there)
  >0.5     instances whose best matching candidate scores > 0.5
  dup      seeds whose body was suppressed by a base candidate within SEED_DEDUP_M
           (and how many of those suppressors score > 0.5)
  add/fr   seeded candidates added per frame; FA = clutter-labelled ones scoring > 0.5
  clutter  base non-person (label 0) candidates per frame and their false alarms (> 0.5);
           in empty runs: all base candidates in the camera's view
  ms       seeded_candidates time per frame with seeds (median / p95), extract for scale

Controls: every fixed-XY override (r_m <= 0.6) is used as a seed on every frame of its
time window, with no dedup.  'sitting' entries are occupied seats, 'not_person' ones
empty chairs / furniture.  If an empty chair scores like an occupied one, the lidar shape
adds no confirmation to a camera detection.
"""
import argparse
import json
import math
import os
import pathlib
import sys
import time
from concurrent.futures import ProcessPoolExecutor

# one OpenMP thread per worker: HistGradientBoosting's predict on a few rows per frame
# with 32 threads in each of N processes spends its time spinning
os.environ.setdefault('OMP_NUM_THREADS', '1')

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import lidar_person_dataset as lpd  # noqa: E402
from yahboomcar_bringup import cluster_features as cf  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

MATCH_M = 0.6
EMPTY_RUNS = {'empty-r1'}       # 11-20 lab: nobody in front of the robot (person stayed behind it)
CONTROL_MAX_R = 0.6


def category(run):
    if run.startswith('seated-') or run.startswith('apt-seated-') or (
            run.startswith('apt-desk2-') and 'empty' not in run):
        return 'seated'
    if run == 'empty-r1' or run.startswith('apt-desk2-empty'):
        return 'empty'
    return 'standing'


def _fixed_overrides(run, overrides):
    return [o for o in overrides if o['run'] == run and not o.get('near_camera')]


def _gt(run, xy, rel_t, overrides):
    """Ground truth of seeded candidates: 1 person/sitting, 0 not person, -1 unlabelled."""
    lab = np.full(len(xy), -1, dtype=np.int8)
    if run in EMPTY_RUNS:
        lab[:] = 0
        return lab
    for o in _fixed_overrides(run, overrides):
        if o.get('r_m', 0.5) > 1.0 and o['label'] != 'not_person':
            continue
        m = np.hypot(xy[:, 0] - o['x'], xy[:, 1] - o['y']) < o.get('r_m', 0.5)
        m &= (rel_t >= o.get('t_from', -np.inf)) & (rel_t < o.get('t_to', np.inf))
        lab[m] = 0 if o['label'] == 'not_person' else (1 if o['label'] in ('person', 'sitting') else -1)
    return lab


def _ray(cands_xy, origin, seed):
    """Mask of candidates on the camera's line of sight to seed (seeded_candidates' corridor)."""
    rc = math.hypot(seed[0] - origin[0], seed[1] - origin[1])
    if not len(cands_xy) or rc < 1e-6:
        return np.zeros(len(cands_xy), bool)
    u = ((seed[0] - origin[0]) / rc, (seed[1] - origin[1]) / rc)
    dx, dy = cands_xy[:, 0] - origin[0], cands_xy[:, 1] - origin[1]
    s, lat = dx * u[0] + dy * u[1], -dx * u[1] + dy * u[0]
    return (np.abs(lat) < pc.SEED_LAT_M) & (s > rc - pc.SEED_BEFORE_M) & (s < rc + pc.SEED_AFTER_M)


def _score(clf, feats):
    if not len(feats):
        return np.zeros(0)
    return clf.predict_proba(np.asarray(feats))[:, 1]


def eval_run(run_dir, overrides, model_path):
    import joblib
    mdl = joblib.load(model_path)
    assert list(mdl['features']) == list(cf.NAMES), 'model features != cluster_features.NAMES'
    clf = mdl['model']
    run_dir = pathlib.Path(run_dir)
    run = run_dir.name
    base = lpd.process_run(run_dir, overrides)
    if base is None:
        return None
    pb = _score(clf, base['X'])
    t0 = float(base['t'].min())
    by_t = {}
    for i, t in enumerate(base['t']):
        by_t.setdefault(float(t), []).append(i)
    controls = [o for o in _fixed_overrides(run, overrides) if o.get('r_m', 0.5) <= CONTROL_MAX_R]
    inst = []                    # per camera instance
    seeded = []                  # per seeded row: p_trim, p_box, gt, range
    dups = []                    # suppressor scores
    ctl = []                     # (control index, found, p_trim)
    seed_ms, extract_ms = [], []
    n_frames = 0
    n_frames_seed = 0
    for fi, f in enumerate(lpd.iter_frames(run_dir)):
        n_frames += 1
        wx, wy, z = f['wx'], f['wy'], f['z']
        origin = (f['ox'], f['oy'])
        rel = f['end'] - t0
        bi = np.array(by_t.get(float(f['end']), []), dtype=np.int64)
        bxy = base['xy'][bi].astype(np.float64) if len(bi) else np.zeros((0, 2))
        if fi % 10 == 0:
            t1 = time.perf_counter()
            pc.extract(wx, wy, z, origin=origin, range_max=lpd.RANGE_MAX_M)
            extract_ms.append(1e3 * (time.perf_counter() - t1))
        cam = f['cam']
        if len(cam):
            n_frames_seed += 1
            t1 = time.perf_counter()
            rows, idx, status = pc.seeded_candidates(wx, wy, z, cam, origin=origin, existing=bxy,
                                                     return_points=True)
            seed_ms.append(1e3 * (time.perf_counter() - t1))
            ft, fb = [], []
            for r, pi in zip(rows, idx):
                ft.append(cf.features(wx[pi], wy[pi], z[pi], origin=origin))
                m = cf.crop(wx, wy, z, r)
                fb.append(cf.features(wx[m], wy[m], z[m], origin=origin))
            pt, pbx = _score(clf, ft), _score(clf, fb)
            gt = _gt(run, rows[:, :2].astype(np.float64), np.full(len(rows), rel), overrides)
            for k in range(len(rows)):
                seeded.append((pt[k], pbx[k], gt[k], math.hypot(rows[k, 0] - origin[0], rows[k, 1] - origin[1])))
            for c, (st, k) in zip(cam, status):
                d = np.hypot(bxy[:, 0] - c[0], bxy[:, 1] - c[1]) if len(bxy) else np.zeros(0)
                near = d < MATCH_M
                ray = _ray(bxy, origin, c)
                b_p = pb[bi[near]].max() if near.any() else -1.0
                r_p = pb[bi[ray]].max() if ray.any() else -1.0
                s_p, s_pb, s_near, body_d, code = -1.0, -1.0, False, np.nan, 0
                if st == 'ok':
                    code = 1
                    s_p, s_pb = float(pt[k]), float(pbx[k])
                    body_d = math.hypot(rows[k, 0] - c[0], rows[k, 1] - c[1])
                    s_near = body_d < MATCH_M
                elif st == 'dup':
                    code = 2
                    s_p = s_pb = float(pb[bi[k]])
                    body_d = math.hypot(bxy[k, 0] - c[0], bxy[k, 1] - c[1])
                    dups.append(s_p)
                inst.append((near.any(), b_p, ray.any(), r_p, code, s_near, s_p, s_pb,
                             math.hypot(c[0] - origin[0], c[1] - origin[1]), body_d))
        for ci, o in enumerate(controls):
            if not (o.get('t_from', -np.inf) <= rel < o.get('t_to', np.inf)):
                continue
            rows, idx, _ = pc.seeded_candidates(wx, wy, z, [(o['x'], o['y'])], origin=origin,
                                                return_points=True)
            if not len(rows):
                ctl.append((ci, False, -1.0, -1.0))
                continue
            pi = idx[0]
            p_t = _score(clf, [cf.features(wx[pi], wy[pi], z[pi], origin=origin)])[0]
            m = cf.crop(wx, wy, z, rows[0])
            p_b = _score(clf, [cf.features(wx[m], wy[m], z[m], origin=origin)])[0]
            ctl.append((ci, True, float(p_t), float(p_b)))
    y, iv = base['y'], base['in_view']
    if run in EMPTY_RUNS:
        neg = iv.copy()
    else:
        neg = y == 0
    return dict(run=run, cat=category(run), n_frames=n_frames, n_frames_seed=n_frames_seed,
                n_base=len(y), base_neg=int(neg.sum()), base_neg_fa=int((pb[neg] > 0.5).sum()),
                inst=np.array(inst, dtype=np.float64).reshape(-1, 10),
                seeded=np.array(seeded, dtype=np.float64).reshape(-1, 4),
                dups=np.array(dups), seed_ms=np.array(seed_ms), extract_ms=np.array(extract_ms),
                controls=[dict(o) for o in controls], ctl=np.array(ctl, dtype=np.float64).reshape(-1, 4))


def _pct(a, b):
    return f'{100.0 * a / b:5.1f}' if b else '   --'


def report(results, out):
    lines = []
    w = lines.append
    w('Camera-seeded lidar person candidates, offline (camera_seed_eval.py)')
    w(f'params: SEED_LAT_M {pc.SEED_LAT_M} BEFORE/AFTER {pc.SEED_BEFORE_M}/{pc.SEED_AFTER_M} '
      f'UPPER_Z {pc.SEED_UPPER_Z_M} GAP {pc.SEED_GAP_M} DEPTH {pc.SEED_DEPTH_M} CORE {pc.SEED_CORE_M} '
      f'DEDUP {pc.SEED_DEDUP_M}; match {MATCH_M} m')
    w('')
    w('(a)(b)(d) camera-person instances (person x frame), % of them:')
    w('  base   a deployed-extract candidate within 0.6 m of the camera XY; >.5 = best such scores > 0.5')
    w('  ray    a base candidate anywhere on the camera ray (the seeded search corridor); >.5 likewise')
    w('  body   seeded_candidates found a body on the ray: a new row (trimmed-point features) or a base')
    w('         candidate within 0.5 m of it (dup, its own score); >.5 likewise.  box = new rows featured')
    w('         on their XY box (cluster_features.crop) instead')
    w('  seed   base + NEW seeded rows, within 0.6 m of the camera XY (what a 0.55 m camera gate could use)')
    w('  new    a new seeded row was added;  d50 = median distance body -> camera XY (m)')
    hdr = (f'  {"run":26s} {"cat":8s} {"frames":>6s} {"cam":>5s} | {"base":>5s} {">.5":>5s} | {"ray":>5s} {">.5":>5s} |'
           f' {"body":>5s} {">.5":>5s} {"box":>5s} | {"seed":>5s} {">.5":>5s} | {"new":>5s} {"d50":>4s} | {"dup":>4s} {"dup>.5":>6s} |'
           f' {"add/fr":>6s} {"FA/fr":>6s} | {"clut/fr":>7s} {"FA/fr":>6s}')
    w(hdr)
    tot = {}
    for r in results:
        I, S = r['inst'], r['seeded']
        n = len(I)
        nf = max(r['n_frames'], 1)
        if not n:
            I = np.zeros((0, 10))
        b_f = I[:, 0] > 0
        b_ok = b_f & (I[:, 1] > 0.5)
        ray_f = I[:, 2] > 0
        ray_ok = ray_f & (I[:, 3] > 0.5)
        new = I[:, 4] == 1
        body = I[:, 4] > 0
        body_ok = body & (I[:, 6] > 0.5)
        body_box = body & (I[:, 7] > 0.5)
        s_f = b_f | (new & (I[:, 5] > 0))
        best = np.maximum(I[:, 1], np.where(new & (I[:, 5] > 0), I[:, 6], -1))
        d50 = np.nanmedian(I[body, 9]) if body.any() else np.nan
        clut = S[:, 2] == 0 if len(S) else np.zeros(0, bool)
        fa_seed = int((clut & (S[:, 0] > 0.5)).sum()) if len(S) else 0
        nd = len(r['dups'])
        w(f'  {r["run"]:26s} {r["cat"]:8s} {r["n_frames"]:6d} {n:5d} | {_pct(b_f.sum(), n)} {_pct(b_ok.sum(), n)} |'
          f' {_pct(ray_f.sum(), n)} {_pct(ray_ok.sum(), n)} | {_pct(body.sum(), n)} {_pct(body_ok.sum(), n)} {_pct(body_box.sum(), n)} |'
          f' {_pct(s_f.sum(), n)} {_pct((s_f & (best > 0.5)).sum(), n)} | {_pct(new.sum(), n)} {d50:4.2f} |'
          f' {nd:4d} {int((r["dups"] > 0.5).sum()):6d} |'
          f' {len(S) / nf:6.3f} {fa_seed / nf:6.3f} | {r["base_neg"] / nf:7.2f} {r["base_neg_fa"] / nf:6.3f}')
        t = tot.setdefault(r['cat'], np.zeros(14))
        t += [nf, n, b_f.sum(), b_ok.sum(), ray_f.sum(), ray_ok.sum(), body.sum(), body_ok.sum(),
              s_f.sum(), (s_f & (best > 0.5)).sum(), len(S), fa_seed, r['base_neg'], r['base_neg_fa']]
    w('')
    w('  totals per category')
    for cat, t in tot.items():
        nf, n = t[0], t[1]
        w(f'  {cat:8s} frames {int(nf):6d} cam {int(n):6d} | base {_pct(t[2], n)} >.5 {_pct(t[3], n)} | ray {_pct(t[4], n)}'
          f' >.5 {_pct(t[5], n)} | body {_pct(t[6], n)} >.5 {_pct(t[7], n)} | seed {_pct(t[8], n)} >.5 {_pct(t[9], n)}'
          f' | added {t[10] / nf:.3f}/fr, clutter FA {t[11] / nf:.3f}/fr | base clutter {t[12] / nf:.2f}/fr FA {t[13] / nf:.3f}/fr')
    w('')
    w('(b) seeded candidates by ground truth: count, scored >0.5 (trimmed / box features)')
    for r in results:
        S = r['seeded']
        if not len(S):
            continue
        parts = []
        for g, name in ((1, 'person'), (0, 'clutter'), (-1, 'unlabelled')):
            m = S[:, 2] == g
            if m.any():
                parts.append(f'{name} {int(m.sum())}: {_pct((S[m, 0] > 0.5).sum(), m.sum())}% / {_pct((S[m, 1] > 0.5).sum(), m.sum())}%')
        w(f'  {r["run"]:26s} ' + ' | '.join(parts))
    w('')
    w('Controls: fixed seat XY seeded every frame of its window (no dedup): % frames with a body, % >0.5 (trimmed / box)')
    for r in results:
        C = r['ctl']
        for ci, o in enumerate(r['controls']):
            m = C[:, 0] == ci if len(C) else np.zeros(0, bool)
            if not m.any():
                continue
            c = C[m]
            win = ''
            if 't_from' in o or 't_to' in o:
                win = f' t[{o.get("t_from", 0)},{o.get("t_to", "end")})'
            w(f'  {r["run"]:26s} {o["label"]:10s} ({o["x"]:+.2f},{o["y"]:+.2f}){win:14s} frames {int(m.sum()):5d}'
              f' body {_pct((c[:, 1] > 0).sum(), m.sum())} >.5 {_pct((c[:, 2] > 0.5).sum(), m.sum())} /'
              f' {_pct((c[:, 3] > 0.5).sum(), m.sum())}   {o.get("note", "")[:60]}')
    w('')
    sm = np.concatenate([r['seed_ms'] for r in results if len(r['seed_ms'])] or [np.zeros(1)])
    em = np.concatenate([r['extract_ms'] for r in results if len(r['extract_ms'])] or [np.zeros(1)])
    w(f'(e) runtime (laptop, one process per run): seeded_candidates per frame with seeds'
      f' median {np.median(sm):.2f} ms p95 {np.percentile(sm, 95):.2f} ms (n={len(sm)});'
      f' extract median {np.median(em):.1f} ms p95 {np.percentile(em, 95):.1f} ms')
    text = '\n'.join(lines)
    print(text)
    if out:
        pathlib.Path(out).write_text(text + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--model', required=True)
    ap.add_argument('--overrides', required=True)
    ap.add_argument('--jobs', type=int, default=1)
    ap.add_argument('-o', '--out')
    ap.add_argument('--save-npz', help='per-run arrays, for digging')
    a = ap.parse_args()
    overrides = json.load(open(a.overrides))
    with ProcessPoolExecutor(max_workers=a.jobs) as ex:
        res = list(ex.map(eval_run, a.runs, [overrides] * len(a.runs), [a.model] * len(a.runs)))
    res = [r for r in res if r is not None]
    order = {'seated': 0, 'standing': 1, 'empty': 2}
    res.sort(key=lambda r: (order[r['cat']], r['run']))
    report(res, a.out)
    if a.save_npz:
        np.savez_compressed(a.save_npz, **{f'{r["run"]}__{k}': r[k] for r in res
                                           for k in ('inst', 'seeded', 'dups', 'ctl', 'seed_ms')})


if __name__ == '__main__':
    main()
