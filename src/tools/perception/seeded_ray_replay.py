#!/usr/bin/env python3
"""Replay a run through the lidar person tracker: base / +seeded / +seeded with the camera
anchor snapped to the nearest candidate on its ray (2026-11-22 desk over-read test).

    python3 src/tools/perception/seeded_ray_replay.py RUN_DIR [RUN_DIR ...]

Robot origin per frame comes from the bag (lidar_person_dataset.iter_frames)."""
import bisect, copy, json, math, pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'src/tools/perception'), str(ROOT / 'src/yahboomcar_bringup')]
import lidar_person_dataset as lpd
from lidar_person_tracker import LidarPersonTracker, merge_seeded
SF = ['x', 'y', 'x_lo', 'x_hi', 'y_lo', 'y_hi', 'z_lo', 'z_hi', 'n']
LAT_M, BEFORE_M, AFTER_M = 0.35, 1.2, 0.5


def snap(camera, cands, origin):
    out = []
    for c in camera:
        c = dict(c)
        ox, oy = origin
        rc = math.hypot(c['odom_x'] - ox, c['odom_y'] - oy)
        if rc > 1e-6:
            ux, uy = (c['odom_x'] - ox) / rc, (c['odom_y'] - oy) / rc
            best = None
            for k in cands:
                dx, dy = k['x'] - ox, k['y'] - oy
                s, lat = dx * ux + dy * uy, -dx * uy + dy * ux
                if abs(lat) < LAT_M and rc - BEFORE_M < s < rc + AFTER_M and (best is None or s < best[0]):
                    best = (s, k)
            if best:
                c['odom_x'], c['odom_y'] = best[1]['x'], best[1]['y']
        out.append(c)
    return out


for run in sys.argv[1:]:
    run = pathlib.Path(run)
    L = [json.loads(l) for l in open(run / 'lidar_persons.ndjson')]
    org = sorted((f['end'], (f['ox'], f['oy'])) for f in lpd.iter_frames(run))
    ts = [t for t, _ in org]
    def origin(t):
        i = bisect.bisect_left(ts, t)
        j = min((k for k in (i - 1, i) if 0 <= k < len(ts)), key=lambda k: abs(ts[k] - t))
        return org[j][1] if abs(ts[j] - t) < 0.1 else None
    for arm in ('base', 'seeded', 'seeded+ray'):
        T = LidarPersonTracker(); prev = None; cam = conf = near = 0; ids = set()
        for f in L:
            now = f['stamp']; c = [dict(zip(f['fields'], r)) for r in f['candidates']]
            if arm != 'base':
                s = f.get('seeded')
                if s: prev = (s, now)
                if prev: c = merge_seeded(c, prev[0], SF, prev[1], now)
            camera = f['camera']
            o = origin(now)
            if arm == 'seeded+ray' and camera and o:
                camera = snap(camera, c, o)
            R = [r for r in T.update(now, c, camera, f['packet_age']) if r['confirmed']]
            ids |= {r['id'] for r in R}
            if f['camera']:
                cam += 1; conf += bool(R)
                # "on the person": on the camera ray within the search window (the camera XY over-reads)
                if o and R:
                    a = f['camera'][0]
                    near += bool(snap([dict(a, odom_x=a['odom_x'], odom_y=a['odom_y'])],
                                      [{'x': r['odom_x'], 'y': r['odom_y']} for r in R], o)[0]['odom_x'] != a['odom_x']
                                 or any(math.hypot(r['odom_x'] - a['odom_x'], r['odom_y'] - a['odom_y']) < 0.6 for r in R))
        print(f"{run.name:24s} {arm:11s} camera frames {cam:4d}  any confirmed {100*conf/max(cam,1):5.1f}%"
              f"  on the person's ray {100*near/max(cam,1):5.1f}%  births {T.stats['births']:3d}  ids {len(ids)}")
