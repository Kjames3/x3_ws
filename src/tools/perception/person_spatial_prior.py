#!/usr/bin/env python3
"""Per-map person spatial prior from recorded lidar person tracks (offline).

    python3 src/tools/perception/person_spatial_prior.py RUN_DIR [RUN_DIR ...] \
        --map RealLab3 --out artifacts/person-prior-YYYY-MM-DD

Each RUN_DIR is a scripts/record_person_lidar.sh capture: lidar_persons.ndjson
(tracker frames, odom frame) + bag/*.db3 (for /tf map->odom and odom->base).
Runs without map->odom are skipped; check alignment first with
src/tools/nav/bag_map_alignment.py (a parked robot's AMCL never updates, so a
constant map->odom proves nothing by itself).

Accumulated per map cell (map grid, --cell m):
  person     confirmed LIDAR-owned track positions (results[].odom_x/y). Camera
             positions are not used: runs before da7d8ae carry the raw-odom bug.
  clutter    lidar person candidates >= --clear m from every confirmed track,
             counted only inside the camera's view (front, +-HFOV/2, < --cam-range),
             where C3 could have confirmed a person had there been one.
  exp_lidar  frames the cell was within --lidar-range of the robot.
  exp_cam    frames the cell was inside the camera view.
Rates are counts / exposure, so the result is not a visibility map. No occlusion
modelling: a cell behind a wall still counts as exposed (conservative for
clutter rate, optimistic for person rate).

THIS IS A CONFIDENCE PRIOR ONLY. Seated people look like static clutter; never
let it remove a detection or anything the CBF sees.
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path

_R = _Path(__file__).resolve()
for _p in (_R.parents[2], _R.parents[1] / 'nav', _R.parents[1] / 'calibration'):
    if str(_p) not in _sys.path:
        _sys.path.insert(0, str(_p))

import argparse
import glob
import json
import math
import os

import numpy as np

from bag_map_alignment import MAPS_DIR, T2, tf_series
from map_prior import MapPrior


def interp_pose(st, val, t):
    """Linear interpolation of (x, y, yaw) by stamp; clamps at the ends."""
    if len(st) == 1:
        return val[0]
    i = int(np.clip(np.searchsorted(st, t), 1, len(st) - 1))
    t0, t1 = st[i - 1], st[i]
    a = 0.0 if t1 <= t0 else float(np.clip((t - t0) / (t1 - t0), 0, 1))
    (x0, y0, h0), (x1, y1, h1) = val[i - 1], val[i]
    dh = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
    return (x0 + a * (x1 - x0), y0 + a * (y1 - y0), h0 + a * dh)


def fit_parked_pose(db, prior, seed, n_scans=30, a_span=1.0, a_aspan=25.0):
    """Grid-search ONE map->base pose for a parked robot from /scan in the base frame."""
    import bag_signals as bs
    from bag_map_alignment import scans
    from map_prior import STATIC
    static = bs.static_tf_tree(db)
    pts = []
    for s in scans(db, n_scans):
        Tb = bs.lookup(static, 'base_footprint', s.header.frame_id)
        r = np.asarray(s.ranges, float)
        ang = s.angle_min + s.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > s.range_min) & (r < 12)
        pts.append((Tb @ np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), 0 * r[ok], 1 + 0 * r[ok]]))[:2])
    P = np.vstack([np.hstack(pts), np.ones(sum(p.shape[1] for p in pts))])

    def score(x, y, th):
        return float((prior.label((T2(x, y, th) @ P)[:2].T) == STATIC).mean())
    best = (score(*seed), *seed)
    for span, step, aspan, astep in ((a_span, 0.1 if a_span <= 1.0 else 0.2, a_aspan, 1.0 if a_aspan <= 25 else 2.0), (0.25, 0.05, 3.0, 0.5), (0.06, 0.02, 1.0, 0.25)):
        cx, cy, cth = best[1:]
        for dth in np.arange(-aspan, aspan + 1e-9, astep):
            for dx in np.arange(-span, span + 1e-9, step):
                for dy in np.arange(-span, span + 1e-9, step):
                    s_ = score(cx + dx, cy + dy, cth + math.radians(dth))
                    if s_ > best[0]:
                        best = (s_, cx + dx, cy + dy, cth + math.radians(dth))
    return best[1], best[2], best[3], best[0]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--map', default='RealLab3')
    ap.add_argument('--cell', type=float, default=0.25)
    ap.add_argument('--lidar-range', type=float, default=8.0)
    ap.add_argument('--cam-range', type=float, default=4.0)
    ap.add_argument('--cam-hfov', type=float, default=80.0, help='deg; conservative for the front OAK')
    ap.add_argument('--clear', type=float, default=0.6, help='m from a confirmed track to count as clutter')
    ap.add_argument('--min-exposure', type=float, default=50, help='frames; rates below this are NaN')
    ap.add_argument('--parked', action='store_true',
                    help='robot did not move: fit ONE map->base pose per run from /scan (searched around '
                         '--seed or the bag\'s first AMCL pose) and ignore map->odom. Use when odom yaw '
                         'drifts while parked (2026-11-20: /imu/data had a 0.17 deg/s bias).')
    ap.add_argument('--seed', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--search-m', type=float, default=1.0, help='--parked: +- search around the seed (m)')
    ap.add_argument('--search-deg', type=float, default=25.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)

    prior = MapPrior.from_yaml(str(MAPS_DIR / f'{a.map}.yaml'))
    lf = prior.lf
    W, H = int(math.ceil(lf.w * lf.res / a.cell)), int(math.ceil(lf.h * lf.res / a.cell))
    grids = {k: np.zeros((H, W), np.float64) for k in ('person', 'clutter', 'exp_lidar', 'exp_cam')}
    cy, cx = np.mgrid[0:H, 0:W]
    cell_x, cell_y = lf.ox + (cx + 0.5) * a.cell, lf.oy + (cy + 0.5) * a.cell

    def to_cell(xy):
        ix = np.floor((xy[:, 0] - lf.ox) / a.cell).astype(int)
        iy = np.floor((xy[:, 1] - lf.oy) / a.cell).astype(int)
        ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        return ix[ok], iy[ok]

    used = []
    for run in a.runs:
        dbs = glob.glob(f'{run}/bag/*.db3')
        nd = os.path.join(run, 'lidar_persons.ndjson')
        if not dbs or not os.path.exists(nd):
            print(f'skip {run}: missing bag or ndjson')
            continue
        mst, mval = tf_series(dbs[0], 'map', 'odom')
        ost, oval = tf_series(dbs[0], 'odom', 'base_footprint')
        if not len(ost) or (not len(mst) and not (a.parked and a.seed)):
            print(f'skip {run}: no map->odom or odom->base_footprint')
            continue
        fit = None
        if a.parked:
            if a.seed:
                seed = (a.seed[0], a.seed[1], math.radians(a.seed[2]))
            else:
                m0 = T2(*mval[0]) @ T2(*oval[0])
                seed = (m0[0, 2], m0[1, 2], math.atan2(m0[1, 0], m0[0, 0]))
            fit = fit_parked_pose(dbs[0], prior, seed, a_span=a.search_m, a_aspan=a.search_deg)
            print(f'{run}: parked pose ({fit[0]:.2f}, {fit[1]:.2f}, {math.degrees(fit[2]):.1f} deg), '
                  f'wall fit {100 * fit[3]:.1f}%')
            T_map_base = T2(*fit[:3])
        n_frames = n_person = n_clutter = 0
        with open(nd) as fh:
            for line in fh:
                fr = json.loads(line)
                t = fr['stamp']
                if fit is not None:
                    Tm = T_map_base @ np.linalg.inv(T2(*interp_pose(ost, oval, t)))
                else:
                    Tm = T2(*interp_pose(mst, mval, t))
                rx, ry, rh = interp_pose(ost, oval, t)
                bx, by, _ = Tm @ np.array([rx, ry, 1.0])
                bh = rh + math.atan2(Tm[1, 0], Tm[0, 0])
                dx, dy = cell_x - bx, cell_y - by
                d = np.hypot(dx, dy)
                grids['exp_lidar'] += d < a.lidar_range
                rel = np.abs(np.arctan2(np.sin(np.arctan2(dy, dx) - bh), np.cos(np.arctan2(dy, dx) - bh)))
                cam = (d < a.cam_range) & (rel < math.radians(a.cam_hfov / 2))
                grids['exp_cam'] += cam
                conf = [(r['odom_x'], r['odom_y']) for r in fr.get('results', []) if r.get('confirmed')]
                conf_m = (Tm @ np.vstack([np.array(conf).T, np.ones(len(conf))]))[:2].T if conf else np.zeros((0, 2))
                ix, iy = to_cell(conf_m)
                np.add.at(grids['person'], (iy, ix), 1)
                n_person += len(ix)
                cands = fr.get('candidates') or []
                if cands:
                    c = np.array([[cc[0], cc[1]] for cc in cands])
                    cm = (Tm @ np.vstack([c.T, np.ones(len(c))]))[:2].T
                    far = np.ones(len(cm), bool)
                    if len(conf_m):
                        far = np.min(np.hypot(cm[:, None, 0] - conf_m[None, :, 0],
                                              cm[:, None, 1] - conf_m[None, :, 1]), 1) >= a.clear
                    ix, iy = to_cell(cm[far])
                    inview = cam[iy, ix]
                    np.add.at(grids['clutter'], (iy[inview], ix[inview]), 1)
                    n_clutter += int(inview.sum())
                n_frames += 1
        used.append({'parked_pose': None if fit is None else [fit[0], fit[1], math.degrees(fit[2]), fit[3]],
                     'run': run, 'frames': n_frames, 'person_cells': n_person, 'clutter_in_view': n_clutter})
        print(f'{run}: {n_frames} frames, {n_person} person hits, {n_clutter} in-view clutter candidates')

    os.makedirs(a.out, exist_ok=True)
    with np.errstate(invalid='ignore', divide='ignore'):
        p_rate = np.where(grids['exp_lidar'] >= a.min_exposure, grids['person'] / grids['exp_lidar'], np.nan)
        c_rate = np.where(grids['exp_cam'] >= a.min_exposure, grids['clutter'] / grids['exp_cam'], np.nan)
    np.savez_compressed(os.path.join(a.out, f'person_prior_{a.map}.npz'), person_rate=p_rate, clutter_rate=c_rate,
                        origin=np.array([lf.ox, lf.oy]), cell=a.cell, **grids)
    meta = {'map': a.map, 'cell_m': a.cell, 'origin': [lf.ox, lf.oy], 'shape': [H, W], 'runs': used,
            'params': {k: getattr(a, k) for k in ('lidar_range', 'cam_range', 'cam_hfov', 'clear', 'min_exposure')},
            'status': 'pipeline proof; not for the tracker (see docstring)'}
    with open(os.path.join(a.out, 'summary.json'), 'w') as fh:
        json.dump(meta, fh, indent=2)
    render(prior, lf, a.cell, p_rate, c_rate, grids, os.path.join(a.out, f'person_prior_{a.map}.png'))
    print(f'wrote {a.out}')


def render(prior, lf, cell, p_rate, c_rate, grids, path):
    import cv2
    base = np.where(lf.occupied, 0, np.where(lf.known_free, 235, 160)).astype(np.uint8)
    base = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    up = lambda g: cv2.resize(np.nan_to_num(g, nan=0).astype(np.float32), (lf.w, lf.h),  # noqa: E731
                              interpolation=cv2.INTER_NEAREST)
    panels = []
    for g, cmap, label in ((p_rate, cv2.COLORMAP_INFERNO, 'person rate'),
                           (c_rate, cv2.COLORMAP_VIRIDIS, 'clutter rate (camera view)'),
                           (grids['exp_lidar'], cv2.COLORMAP_BONE, 'lidar exposure (frames)')):
        v = up(g)
        hi = np.percentile(v[v > 0], 99) if (v > 0).any() else 1
        col = cv2.applyColorMap((255 * np.clip(v / hi, 0, 1)).astype(np.uint8), cmap)
        m = (v > 0)[..., None]
        img = np.where(m, (0.75 * col + 0.25 * base).astype(np.uint8), base)
        img = cv2.flip(img, 0)  # map y up
        cv2.putText(img, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        panels.append(img)
    out = np.hstack(panels)
    scale = 1600 / out.shape[1]
    if scale < 1:
        out = cv2.resize(out, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, out)


if __name__ == '__main__':
    main()
