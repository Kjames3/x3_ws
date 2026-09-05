"""Stage 4: convergence basin, 3D against 2D, plus per-registration cost.

Accuracy turned out not to be the discriminator -- both methods land within a
few mm whenever they converge at all.  What separates them is how bad an
initial guess they tolerate, which is what actually decides whether a localizer
survives a kidnapped robot, a wheel slip or a bad AMCL seed.  So this sweeps
displacement until each one breaks, using the smallest realistic query (a
single 0.142 s cloud) to avoid crediting either with accumulated data.
"""

import json
import os
import time

import numpy as np
from scipy.spatial import cKDTree

import icp
from budget import LASER_Z, SLAB, build_map_2d
from study import SEED, build_map, load
from viewpoint import NOISE_M, visible_from

HERE = os.path.dirname(os.path.abspath(__file__))
ONE_CLOUD = 2970
CLOUD_PERIOD_MS = 1000.0 / 7.18
LADDER = [(0.2, 5), (0.5, 10), (0.8, 15), (1.0, 20), (1.2, 25), (1.4, 30),
          (2.0, 40), (2.5, 50)]


def main():
    rng = np.random.default_rng(SEED)
    pts = load()
    idx = rng.permutation(len(pts))
    half = len(idx) // 2
    map_pts, pool = pts[idx[:half]], pts[idx[half:]]
    ref, ref_n = build_map(map_pts)
    ref2, ref2_n = build_map_2d(map_pts, SLAB, LASER_Z)

    out = {'query_points': ONE_CLOUD, 'map_voxels_3d': int(len(ref)),
           'map_voxels_2d': int(len(ref2)), 'basin': [], 'timing_ms': {}}
    print('basin, query = one cloud (%d pts), max_dist 1.0 m' % ONE_CLOUD)
    print('%14s %10s %9s | %10s %9s'
          % ('displacement', '3d trans', '3d rot', '2d trans', '2d rot'))
    for d, yaw in LADDER:
        v = np.array([d / np.sqrt(2), d / np.sqrt(2), 0.0])
        local = visible_from(pool, v)
        local = local + rng.normal(0, NOISE_M, local.shape)
        sub = local[rng.choice(len(local), min(ONE_CLOUD, len(local)),
                               replace=False)]
        T_true = icp.make_transform([0, 0, np.radians(yaw)], v)
        unyaw = icp.make_transform([0, 0, -np.radians(yaw)], [0, 0, 0])

        T3, _ = icp.icp(icp.transform(sub, unyaw), ref, ref_n, max_dist=1.0)
        t3, r3 = icp.pose_error(T3, T_true)
        near = np.abs(sub[:, 2] - LASER_Z) < SLAB
        flat = sub[near].copy()
        flat[:, 2] = 0.0
        T2, _ = icp.icp(icp.transform(flat, unyaw), ref2, ref2_n,
                        max_dist=1.0, planar=True)
        t2, r2 = icp.pose_error(T2, T_true)
        out['basin'].append({'displacement_m': d, 'yaw_deg': yaw,
                             'trans_err_3d': round(t3, 4), 'rot_err_3d': round(r3, 3),
                             'ok_3d': bool(t3 < 0.1), 'trans_err_2d': round(t2, 4),
                             'rot_err_2d': round(r2, 3), 'ok_2d': bool(t2 < 0.1)})
        print('%6.1f m/%2d deg %9.4f %9.3f %s | %9.4f %9.3f %s'
              % (d, yaw, t3, r3, 'ok ' if t3 < .1 else 'BAD',
                 t2, r2, 'ok ' if t2 < .1 else 'BAD'))

    # Cost. The KD-tree is built once per map, not per cloud.
    t0 = time.perf_counter()
    cKDTree(ref)
    out['timing_ms']['kdtree_build'] = round((time.perf_counter() - t0) * 1e3, 1)
    v = np.array([0.3, 0.2, 0.0])
    local = visible_from(pool, v)
    print('\nper-registration cost (this machine, 20 iterations max):')
    for n in (ONE_CLOUD, 10668, 42673):
        sub = local[rng.choice(len(local), min(n, len(local)), replace=False)]
        ts = []
        for _ in range(5):
            t0 = time.perf_counter()
            icp.icp(sub, ref, ref_n, max_dist=0.6, max_iter=20)
            ts.append(time.perf_counter() - t0)
        ms = float(np.median(ts)) * 1e3
        out['timing_ms']['n_%d' % n] = round(ms, 1)
        print('  %6d pts: %6.1f ms  (%.0f%% of the %.0f ms cloud period)'
              % (n, ms, 100 * ms / CLOUD_PERIOD_MS, CLOUD_PERIOD_MS))

    with open(os.path.join(HERE, 'basin.json'), 'w') as f:
        json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
