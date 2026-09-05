"""Offline feasibility: can a tilting-lidar sweep register well enough to localize?

Uses the parked 30 s / 45 deg/s continuous capture (deskewed, base_footprint).

WHAT THIS CAN AND CANNOT ANSWER
-------------------------------
The robot was stationary for every capture on disk, so there is no real
displacement to recover and no odometry drift to fight.  What that permits is
still the decisive question: registration ACCURACY and the CONVERGENCE BASIN,
measured against a known ground truth by perturbing a cloud by a known
transform and asking ICP to recover it.  If a sweep cannot recover a synthetic
0.2 m offset against a map of the same room, it certainly cannot localize.

Two honest limitations:
  * The npz stores concatenated points with no per-cloud boundary, so a
    single-pass cloud is SIMULATED by subsampling the aggregate to one pass's
    point count.  Because the robot is parked and the scene static, every pass
    observes the same geometry, so this reproduces the spatial sampling that
    determines observability.  It does NOT reproduce within-pass temporal
    correlation or residual deskew error, both of which can only get worse.
  * One scene, one pose.  A furnished apartment room is a favourable case:
    a corridor or a bare hall has far less structure.

Map and query are drawn from DISJOINT halves of the points, so ICP is never
matching a measurement against itself.
"""

import json
import os

import numpy as np

from icp import (estimate_normals, icp, make_transform, pose_error, transform,
                 voxel_downsample)

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'continuous-2026-09-05', 'continuous_45_points.npz')
VOXEL = 0.05                 # matches the 2D map and the octree resolution
N_CLOUDS, SECONDS = 218, 30.345
PERIOD_S = 4.0               # full ping-pong at 45 deg/s over +-45 deg
RANGE_MAX = 6.0
SEED = 20260905


def load():
    p = np.load(SRC)['points']
    d = np.hypot(p[:, 0], p[:, 1])
    # Same cap the deployed processor now applies.
    return p[(d > 0.15) & (d < RANGE_MAX) & np.isfinite(p).all(axis=1)]


def build_map(points, planarity_min=0.35):
    grid = voxel_downsample(points, VOXEL)
    normals, planarity = estimate_normals(grid)
    keep = planarity > planarity_min
    return grid[keep], normals[keep]


def points_per(seconds, total, rng):
    """How many returns one `seconds` of sweeping contributes."""
    return int(round(total * seconds / SECONDS))


def run_case(query, ref, ref_n, perturbations, rng):
    out = []
    for trans, yaw_deg in perturbations:
        # Perturb the query, then ask ICP to undo it.
        T_applied = make_transform([0, 0, np.radians(yaw_deg)],
                                   [trans, 0.0, 0.0])
        moved = transform(query, T_applied)
        T_est, info = icp(moved, ref, ref_n, max_dist=max(0.5, 2 * trans))
        te, re = pose_error(T_est, np.linalg.inv(T_applied))
        eig = np.linalg.eigvalsh(info['hessian'])
        out.append({
            'perturb_m': trans, 'perturb_deg': yaw_deg,
            'trans_err_m': round(te, 4), 'rot_err_deg': round(re, 3),
            'converged': bool(te < 0.05 and re < 2.0),
            'inliers': info['inliers'], 'rmse_m': round(info['rmse'], 4),
            'hessian_eig_min': float(eig[0]), 'hessian_eig_max': float(eig[-1]),
            'condition': float(eig[-1] / max(eig[0], 1e-12)),
        })
    return out


def main():
    rng = np.random.default_rng(SEED)
    pts = load()
    idx = rng.permutation(len(pts))
    half = len(idx) // 2
    map_pts, query_pool = pts[idx[:half]], pts[idx[half:]]

    ref, ref_n = build_map(map_pts)
    result = {'source': os.path.relpath(SRC, HERE), 'n_points': int(len(pts)),
              'map_voxels': int(len(ref)), 'voxel_m': VOXEL,
              'seed': SEED, 'cases': {}}
    print('map: %d planar voxels from %d points (%.0f cm)'
          % (len(ref), len(map_pts), VOXEL * 100))

    perturbations = [(0.05, 1), (0.10, 2), (0.20, 5), (0.40, 10),
                     (0.80, 20), (1.50, 30)]

    # How much sweeping the query cloud represents.
    budgets = [('one cloud (0.14 s)', SECONDS / N_CLOUDS),
               ('quarter pass (0.5 s)', 0.5),
               ('half pass (1.0 s)', 1.0),
               ('full pass (2.0 s)', PERIOD_S / 2),
               ('full cycle (4.0 s)', PERIOD_S)]

    for name, seconds in budgets:
        n = points_per(seconds, len(query_pool) * 2, rng)
        n = min(n, len(query_pool))
        query = query_pool[rng.choice(len(query_pool), n, replace=False)]
        rows = run_case(query, ref, ref_n, perturbations, rng)
        result['cases'][name] = {'seconds': round(seconds, 3),
                                 'query_points': int(n), 'runs': rows}
        ok = [r for r in rows if r['converged']]
        basin = max([r['perturb_m'] for r in ok], default=0.0)
        best = rows[0]
        print('\n%-22s %6d pts | basin <= %.2f m | at 0.10 m: '
              'trans %.3f m rot %.2f deg | cond %.0f'
              % (name, n, basin, rows[1]['trans_err_m'], rows[1]['rot_err_deg'],
                 rows[1]['condition']))
        for r in rows:
            print('    perturb %.2f m /%3.0f deg -> %.4f m %6.3f deg  %s'
                  % (r['perturb_m'], r['perturb_deg'], r['trans_err_m'],
                     r['rot_err_deg'], 'ok' if r['converged'] else 'FAILED'))
        del best

    with open(os.path.join(HERE, 'study.json'), 'w') as f:
        json.dump(result, f, indent=1)


if __name__ == '__main__':
    main()
