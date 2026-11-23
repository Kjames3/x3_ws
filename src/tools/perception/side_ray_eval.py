#!/usr/bin/env python3
"""Offline check of the bearing-only ray search (person_candidates.ray_candidates).

    python3 src/tools/perception/side_ray_eval.py RUN_DIR [RUN_DIR ...] [--every N]

RUN_DIR is a record_person_lidar.sh run (bag/ with /livox/lidar + /odom, and
lidar_persons.ndjson).  Two modes, picked per run:

  side  the run has side/side_rays.ndjson (x3_server with X3_SIDE_PERSONS=1 while
        recording): the recorded side-camera rays are re-run against the bag's Livox
        frames exactly as livox_scan_node does (rays <= 0.3 s old at the frame end).
  c3    no side recording (every run before 2026-11-22): each measured C3 person becomes
        a bearing-only ray from the Livox origin -- its range is thrown away -- and the
        whole ray (0.4-6 m) is searched with no box top (top_tan NaN: the head-height test
        off, the hardest case).  Front field of view only, but the same lidar code.

Reference: the live tracker's confirmed tracks in lidar_persons.ndjson (the 11-22 runs
were recorded with X3_SEEDED_TRACKER=1: desk 98% / walk 95% confirmed).  For each ray,
the reference is the nearest confirmed track within REF_LAT_M of it.

Per run:
  rays      ray instances (person x frame)
  found     a body was found on the ray
  ref       a confirmed track lies on the ray
  agree     found AND within AGREE_M of that track (the right body)
  wrong     found but > AGREE_M from the track (picked something else on the ray)
  missed    track on the ray, nothing found
  no-ref    found with no track on the ray (unverified: may be a person the tracker
            lacks, or clutter)
  off-person  extra rays cast at bearings >= OFF_SEP_DEG from every camera person and
            track (front +-60 deg in 15 deg steps, every Nth frame): share that found a
            'body'.  In the empty runs that is a false-pick rate outright; elsewhere it
            counts furniture and unseen people alike, so it is an upper bound.
  ms        ray_candidates time per frame (median / p95)
"""
import argparse
import bisect
import json
import math
import pathlib
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'src' / 'tools' / 'perception'),
                str(ROOT / 'src' / 'yahboomcar_bringup')]
import lidar_person_dataset as lpd  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

REF_LAT_M = 0.5
AGREE_M = 0.5
OFF_SEP_DEG = 25.0
OFF_BEARINGS_DEG = np.arange(-60, 61, 15)
SENSOR_Z_M = pc.SENSOR_Z_M
SIDE_MAX_AGE_S = 0.3


def _on_ray(o, u, pts, lat_m=REF_LAT_M, s_rng=pc.RAY_RANGE_M):
    """Index of the nearest point (rows x, y) within lat_m of the ray, or None."""
    best = None
    for i, (x, y) in enumerate(pts):
        dx, dy = x - o[0], y - o[1]
        s, lat = dx * u[0] + dy * u[1], -dx * u[1] + dy * u[0]
        if abs(lat) < lat_m and s_rng[0] - 0.3 < s < s_rng[1] + 0.3 and (best is None or s < best[0]):
            best = (s, i)
    return None if best is None else best[1]


def _confirmed(rec):
    if rec is None:
        return np.zeros((0, 2))
    return np.array([(r['odom_x'], r['odom_y']) for r in rec.get('results') or []
                     if r.get('confirmed')]).reshape(-1, 2)


def _side_rays(run):
    p = pathlib.Path(run) / 'side' / 'side_rays.ndjson'
    if not p.exists():
        return None
    recs = [json.loads(l) for l in open(p) if l.strip()]
    rows = []        # (stamp, cam, ray row, tid)
    for r in recs:
        for ray in r.get('rays', []):
            if ray.get('ready'):
                rows.append((r['stamp'], r['cam'], [ray[k] for k in pc.RAY_IN_FIELDS], ray['tid']))
    rows.sort(key=lambda t: t[0])
    return rows


def evaluate(run, every=3):
    run = pathlib.Path(run)
    side = _side_rays(run)
    mode = 'side' if side is not None else 'c3'
    st = {k: 0 for k in ('rays', 'found', 'ref', 'agree', 'wrong', 'missed', 'no_ref',
                         'off_rays', 'off_found')}
    per_cam = {}
    ms = []
    side_t = [t for t, *_ in side] if side else []
    for fi, f in enumerate(lpd.iter_frames(run)):
        o = (f['ox'], f['oy'])
        ref = _confirmed(f['rec'])
        rays, cams = [], []
        if mode == 'c3':
            for cx, cy in f['cam']:
                r = math.hypot(cx - o[0], cy - o[1])
                if r > 1e-6:
                    rays.append([o[0], o[1], SENSOR_Z_M, (cx - o[0]) / r, (cy - o[1]) / r,
                                 pc.RAY_RANGE_M[0], pc.RAY_RANGE_M[1], float('nan')])
                    cams.append('c3')
        else:
            end = f['end']
            i0 = bisect.bisect_left(side_t, end - SIDE_MAX_AGE_S)
            i1 = bisect.bisect_right(side_t, end + 0.15)
            latest = {}
            for t, cam, row, tid in side[i0:i1]:
                if cam not in latest or t > latest[cam][0]:
                    latest[cam] = (t, [])
                if t == latest[cam][0]:
                    latest[cam][1].append(row)
            for cam, (_, rs) in latest.items():
                rays += rs
                cams += [cam] * len(rs)
        n_main = len(rays)
        if fi % every == 0:
            fwd = [math.atan2(y - o[1], x - o[0]) for x, y in np.vstack([f['cam'], ref])]
            for b in OFF_BEARINGS_DEG:
                a = f['fa'] + math.radians(b)
                if all(abs(math.remainder(a - q, 2 * math.pi)) > math.radians(OFF_SEP_DEG) for q in fwd):
                    rays.append([o[0], o[1], SENSOR_Z_M, math.cos(a), math.sin(a),
                                 pc.RAY_RANGE_M[0], pc.RAY_RANGE_M[1], float('nan')])
        if not rays:
            continue
        t0 = time.perf_counter()
        out = pc.ray_candidates(f['wx'], f['wy'], f['z'], rays)
        ms.append((time.perf_counter() - t0) * 1e3)
        hit = {int(r[-1]): r for r in out}
        for k, ray in enumerate(rays):
            body = hit.get(k)
            if k >= n_main:
                st['off_rays'] += 1
                st['off_found'] += body is not None
                continue
            c = per_cam.setdefault(cams[k], {'rays': 0, 'found': 0, 'agree': 0})
            st['rays'] += 1
            c['rays'] += 1
            u = (ray[3], ray[4])
            j = _on_ray((ray[0], ray[1]), u, ref)
            if body is not None:
                st['found'] += 1
                c['found'] += 1
            if j is None:
                st['no_ref'] += body is not None
                continue
            st['ref'] += 1
            if body is None:
                st['missed'] += 1
            elif math.hypot(body[0] - ref[j, 0], body[1] - ref[j, 1]) < AGREE_M:
                st['agree'] += 1
                c['agree'] += 1
            else:
                st['wrong'] += 1
    st['ms'] = (float(np.median(ms)) if ms else 0.0, float(np.percentile(ms, 95)) if ms else 0.0)
    return mode, st, per_cam


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--every', type=int, default=3, help='off-person rays on every Nth frame')
    a = ap.parse_args()
    pct = lambda n, d: f"{100.0 * n / d:5.1f}%" if d else '   - '
    print(f"{'run':28s} {'mode':4s} {'rays':>6s} {'found':>7s} {'ref':>6s} {'agree':>7s} "
          f"{'wrong':>7s} {'missed':>7s} {'no-ref':>7s} {'off-person':>12s} {'ms':>11s}")
    for run in a.runs:
        mode, s, per_cam = evaluate(run, a.every)
        print(f"{pathlib.Path(run).name:28s} {mode:4s} {s['rays']:6d} {pct(s['found'], s['rays'])} "
              f"{s['ref']:6d} {pct(s['agree'], s['ref'])} {pct(s['wrong'], s['ref'])} "
              f"{pct(s['missed'], s['ref'])} {s['no_ref']:7d} "
              f"{pct(s['off_found'], s['off_rays'])} ({s['off_rays']:4d}) "
              f"{s['ms'][0]:4.1f}/{s['ms'][1]:4.1f}")
        if mode == 'side':
            for cam, c in sorted(per_cam.items()):
                print(f"    {cam:6s} rays {c['rays']}, found {pct(c['found'], c['rays'])}, "
                      f"agree with a track {c['agree']}")


if __name__ == '__main__':
    main()
