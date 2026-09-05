"""Stage 2: register from a DISPLACED viewpoint, which is the localization case.

Stage 1 (study.py) registered a sweep against a map built from the same parked
pose.  It converged from 1.5 m / 30 deg to sub-millimetre, which flatters the
method: with a shared viewpoint every surface is seen at the same incidence and
nothing is occluded, so a perfect global optimum exists and ICP walks to it.
That measures self-consistency, not localization.

Here the query is synthesized from a viewpoint the robot never occupied:

  1. Split the returns into disjoint halves.  Half A builds the map; half B is
     the raw material for the query, so query points are different physical
     measurements than map points.
  2. From a displaced viewpoint v, keep only the nearest return per angular
     bin of half B -- hidden-surface removal.  Surfaces the new pose cannot
     see are dropped, and near objects occlude far ones, as a real sensor
     would experience them.
  3. Apply the 6 m range cap from v, not from the original pose.
  4. Register that partial view against the map and compare to the known v.

The honest ceiling on this: the map only contains surfaces the ORIGINAL pose
saw, so a displaced view can never reveal genuinely new geometry, and holes
grow with displacement.  That caps credible displacement at roughly 1-1.5 m in
this room and is why the sweep below stops there.  It biases toward optimism,
so a failure here is conclusive while a success is necessary-but-not-
sufficient.  Only a driving capture settles it.
"""

import json
import os

import numpy as np

from icp import estimate_normals, icp, make_transform, pose_error, transform
from study import RANGE_MAX, SEED, build_map, load

HERE = os.path.dirname(os.path.abspath(__file__))
BIN_DEG = 0.35          # angular cell for hidden-surface removal
NOISE_M = 0.007         # measured unperturbed registration RMSE (stage 1)


def visible_from(points, viewpoint, bin_deg=BIN_DEG, range_max=RANGE_MAX):
    """Nearest return per angular bin as seen from `viewpoint`."""
    rel = points - viewpoint
    r = np.linalg.norm(rel, axis=1)
    ok = (r > 0.15) & (r < range_max)
    rel, r = rel[ok], r[ok]
    az = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]))
    el = np.degrees(np.arcsin(np.clip(rel[:, 2] / r, -1, 1)))
    key = (np.floor(az / bin_deg).astype(np.int64) * 100000
           + np.floor(el / bin_deg).astype(np.int64))
    order = np.lexsort((r, key))
    key_sorted = key[order]
    first = np.r_[True, key_sorted[1:] != key_sorted[:-1]]
    return rel[order[first]]      # in the sensor frame at `viewpoint`


def main():
    rng = np.random.default_rng(SEED)
    pts = load()
    idx = rng.permutation(len(pts))
    half = len(idx) // 2
    ref, ref_n = build_map(pts[idx[:half]])
    pool = pts[idx[half:]]

    result = {'bin_deg': BIN_DEG, 'noise_m': NOISE_M, 'map_voxels': int(len(ref)),
              'runs': []}
    print('map: %d planar voxels; query synthesized by occlusion from half B\n'
          % len(ref))
    print('%8s %8s %9s %9s %9s %8s %9s'
          % ('offset', 'yaw', 'pts', 'trans err', 'rot err', 'rmse', 'cond'))

    for dx, dy, yaw in [(0.0, 0.0, 0), (0.10, 0.0, 0), (0.25, 0.0, 5),
                        (0.50, 0.0, 10), (0.50, 0.50, 15), (1.00, 0.0, 20),
                        (1.00, 1.00, 30), (1.50, 0.0, 30)]:
        v = np.array([dx, dy, 0.0])
        local = visible_from(pool, v)
        if len(local) < 500:
            continue
        local = local + rng.normal(0, NOISE_M, local.shape)
        # Express the view in a body frame yawed by `yaw`, so ICP must recover
        # rotation as well as translation.
        T_true = make_transform([0, 0, np.radians(yaw)], v)
        query = transform(local, make_transform([0, 0, -np.radians(yaw)],
                                                [0, 0, 0]))
        T_est, info = icp(query, ref, ref_n, max_dist=0.6)
        te, re = pose_error(T_est, T_true)
        eig = np.linalg.eigvalsh(info['hessian'])
        cond = float(eig[-1] / max(eig[0], 1e-12))
        row = {'dx': dx, 'dy': dy, 'yaw_deg': yaw, 'query_points': int(len(query)),
               'trans_err_m': round(te, 4), 'rot_err_deg': round(re, 3),
               'rmse_m': round(info['rmse'], 4), 'condition': round(cond, 1),
               'converged': bool(te < 0.10 and re < 3.0)}
        result['runs'].append(row)
        print('%6.2f,%.2f %6d deg %8d %9.4f %9.3f %8.4f %9.0f  %s'
              % (dx, dy, yaw, len(query), te, re, info['rmse'], cond,
                 'ok' if row['converged'] else 'FAILED'))

    with open(os.path.join(HERE, 'viewpoint.json'), 'w') as f:
        json.dump(result, f, indent=1)


if __name__ == '__main__':
    main()
