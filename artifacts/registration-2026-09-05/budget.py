"""Stage 3: realistic point budgets, and 3D against the 2D slice it replaces.

Stage 2 gave every query ~78k points -- a view complete in every angular bin,
about 2.5 sweep cycles' worth.  A real sweep delivers ~2970 points per cloud
and ~43k per 2 s pass, so stage 2 measured the geometry, not the sensor.

This stage crosses displacement with the point budget the mount can actually
deliver in a given time, and runs the same cases through a 2D slice at the
laser plane -- what AMCL sees today -- so the two are compared on identical
returns, identical map, identical solver.  A 3D win has to be demonstrated
against that baseline, not assumed from the fact that the data is 3D.
"""

import json
import os

import numpy as np

from icp import (estimate_normals_2d, icp, make_transform, pose_error,
                 transform, voxel_downsample)
from study import SEED, VOXEL, build_map, load
from viewpoint import NOISE_M, visible_from

HERE = os.path.dirname(os.path.abspath(__file__))
SLAB = 0.05             # +-5 cm about the laser plane == a 2D scan
LASER_Z = 0.313

BUDGETS = [('one cloud', 0.142, 2970),
           ('quarter pass', 0.5, 10668),
           ('half pass', 1.0, 21336),
           ('full pass', 2.0, 42673)]
CASES = [(0.10, 0.0, 2), (0.50, 0.0, 10), (1.00, 1.00, 30)]


def build_map_2d(points, slab, laser_z):
    """A 2D scan-match map: the laser-plane slab, flattened, with in-plane
    wall normals.  Flattening is what makes it 2D -- keeping z would let the
    slab's own thickness masquerade as structure."""
    near = points[np.abs(points[:, 2] - laser_z) < slab].copy()
    near[:, 2] = 0.0
    grid = voxel_downsample(near, VOXEL)
    normals, planarity = estimate_normals_2d(grid)
    keep = planarity > 0.35
    return grid[keep], normals[keep]


def run(query, ref, ref_n, T_true, max_dist=0.6, planar=False):
    T_est, info = icp(query, ref, ref_n, max_dist=max_dist, planar=planar)
    te, re = pose_error(T_est, T_true)
    eig = np.linalg.eigvalsh(info['hessian'])
    return te, re, float(eig[-1] / max(eig[0], 1e-12)), info


def main():
    rng = np.random.default_rng(SEED)
    pts = load()
    idx = rng.permutation(len(pts))
    half = len(idx) // 2
    map_pts, pool = pts[idx[:half]], pts[idx[half:]]
    ref, ref_n = build_map(map_pts)

    # The 2D baseline gets a map made the same way, from the same returns,
    # restricted to the laser plane -- i.e. exactly a 2D occupancy scan match.
    ref2, ref2_n = build_map_2d(map_pts, SLAB, LASER_Z)

    out = {'slab_m': SLAB, 'map_voxels_3d': int(len(ref)),
           'map_voxels_2d': int(len(ref2)), 'rows': []}
    print('map: %d planar voxels (3D) vs %d (2D slice at z=%.3f +-%.2f)\n'
          % (len(ref), len(ref2), LASER_Z, SLAB))
    print('%-13s %-14s %8s %10s %9s %8s %6s'
          % ('budget', 'displacement', 'pts', 'trans err', 'rot err', 'cond', 'mode'))

    for name, seconds, n_budget in BUDGETS:
        for dx, dy, yaw in CASES:
            v = np.array([dx, dy, 0.0])
            local = visible_from(pool, v)
            local = local + rng.normal(0, NOISE_M, local.shape)
            take = min(n_budget, len(local))
            sub = local[rng.choice(len(local), take, replace=False)]
            T_true = make_transform([0, 0, np.radians(yaw)], v)
            unyaw = make_transform([0, 0, -np.radians(yaw)], [0, 0, 0])

            q3 = transform(sub, unyaw)
            te, re, cond, info = run(q3, ref, ref_n, T_true)
            row = {'budget': name, 'seconds': seconds, 'dx': dx, 'dy': dy,
                   'yaw_deg': yaw, 'mode': '3d', 'points': int(len(q3)),
                   'trans_err_m': round(te, 4), 'rot_err_deg': round(re, 3),
                   'condition': round(cond, 1),
                   'converged': bool(te < 0.10 and re < 3.0)}
            out['rows'].append(row)
            print('%-13s %.2f,%.2f/%2ddeg %8d %10.4f %9.3f %8.0f %6s %s'
                  % (name, dx, dy, yaw, len(q3), te, re, cond, '3d',
                     'ok' if row['converged'] else 'FAILED'))

            # 2D: same returns, restricted to the laser plane -- what a level
            # 2D scan collects.  `sub` is in the sensor frame, whose origin is
            # the viewpoint at FLOOR level, so the laser plane sits at
            # z = LASER_Z here, not z = 0.  Selecting |z| < SLAB instead grabs
            # floor returns and matches them against a map slab 0.31 m higher.
            near = np.abs(sub[:, 2] - LASER_Z) < SLAB
            flat = sub[near].copy()
            flat[:, 2] = 0.0
            q2 = transform(flat, unyaw)
            if len(q2) < 30:
                print('%-13s %.2f,%.2f/%2ddeg %8d %10s %9s %8s %6s'
                      % (name, dx, dy, yaw, len(q2), '-', '-', '-', '2d'))
                out['rows'].append({'budget': name, 'dx': dx, 'dy': dy,
                                    'yaw_deg': yaw, 'mode': '2d',
                                    'points': int(len(q2)),
                                    'converged': False, 'note': 'too few returns'})
                continue
            te2, re2, cond2, _ = run(q2, ref2, ref2_n, T_true, planar=True)
            row2 = {'budget': name, 'seconds': seconds, 'dx': dx, 'dy': dy,
                    'yaw_deg': yaw, 'mode': '2d', 'points': int(len(q2)),
                    'trans_err_m': round(te2, 4), 'rot_err_deg': round(re2, 3),
                    'condition': round(cond2, 1),
                    'converged': bool(te2 < 0.10 and re2 < 3.0)}
            out['rows'].append(row2)
            print('%-13s %.2f,%.2f/%2ddeg %8d %10.4f %9.3f %8.0f %6s %s'
                  % (name, dx, dy, yaw, len(q2), te2, re2, cond2, '2d',
                     'ok' if row2['converged'] else 'FAILED'))
        print()

    with open(os.path.join(HERE, 'budget.json'), 'w') as f:
        json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
