"""The registration study on REAL multi-pose data (drive_capture.npz).

Five stations, driven by hand around an apartment room on 2026-09-05, each with
~68 deskewed clouds captured while parked. Headings span nearly 300 deg
(12.5, -37.0, -156.9, -37.8, +136.7), so this finally supplies the thing the
parked study could not: genuine viewpoint change, with real occlusion, real
incidence-angle change, and a map containing surfaces the query pose never saw.

Leave-one-station-out: the map for station i is built from the OTHER four,
placed in a common frame by odometry. The query is one of station i's clouds,
in its own body frame. So query and map share no measurement, no viewpoint and
no sweep.

Three things are measured:

  A. AGREEMENT -- initialise at the odometry pose and record how far ICP moves.
     Small means registration and odometry agree; it does not prove either is
     right, since odometry is the only reference available and it drifts.
  B. REPEATABILITY -- all ~68 clouds of a station are the same true pose, so
     the spread of their recovered poses is registration precision, measured
     rather than assumed. This needs no ground truth at all and is the most
     trustworthy number here.
  C. BASIN -- perturb the initial guess until it stops coming back.
"""

import json
import os

import numpy as np

import icp
from study import VOXEL, build_map

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'drive_capture.npz')
RANGE_MAX = 6.0
STILL_EPS = 0.02
MIN_STILL_S = 7.0


def station_poses(odom):
    """Recover each station's pose from the still segments of the odom trace.

    The capture stamped odom with monotonic time and clouds with ROS time, so
    the two cannot be joined directly; the sweeps are the still periods, in
    order. (The capture script now records poses explicitly -- this recovery is
    for the first run's file.)
    """
    segs, i = [], 0
    while i < len(odom):
        j = i
        while (j + 1 < len(odom)
               and np.linalg.norm(odom[j + 1, 1:3] - odom[i, 1:3]) < STILL_EPS):
            j += 1
        if odom[j, 0] - odom[i, 0] >= MIN_STILL_S:
            seg = odom[i:j + 1]
            yaw = np.arctan2(np.sin(seg[:, 3]).mean(), np.cos(seg[:, 3]).mean())
            segs.append([seg[:, 1].mean(), seg[:, 2].mean(), yaw])
        i = j + 1
    return np.array(segs)


def pose_to_T(x, y, yaw):
    return icp.make_transform([0, 0, yaw], [x, y, 0.0])


def clip(points):
    d = np.hypot(points[:, 0], points[:, 1])
    return points[(d > 0.15) & (d < RANGE_MAX) & np.isfinite(points).all(axis=1)]


def main():
    data = np.load(SRC)
    pts, cid, meta = data['points'].astype(float), data['cloud_id'], data['cloud_meta']
    poses = station_poses(data['odom'])
    n_st = len(poses)
    cloud_station = {int(m[0]): int(m[1]) for m in meta}
    station_of_point = np.array([cloud_station[c] for c in cid])

    out = {'stations': poses.tolist(), 'n_stations': n_st, 'results': []}
    print('stations (odom): ' + ', '.join(
        '(%.2f,%.2f,%.0fdeg)' % (p[0], p[1], np.degrees(p[2])) for p in poses))

    for i in range(n_st):
        T_true = pose_to_T(*poses[i])
        # Map: the other four stations, placed by odometry.
        other = []
        for j in range(n_st):
            if j == i:
                continue
            other.append(icp.transform(clip(pts[station_of_point == j]),
                                       pose_to_T(*poses[j])))
        ref, ref_n = build_map(np.concatenate(other))

        ids = sorted({int(m[0]) for m in meta if int(m[1]) == i})
        recovered = []
        for c in ids:
            q = clip(pts[cid == c])
            if len(q) < 300:
                continue
            T, info = icp.icp(q, ref, ref_n, init=T_true, max_dist=0.6)
            te, re = icp.pose_error(T, T_true)
            recovered.append([T[0, 3], T[1, 3],
                              np.arctan2(T[1, 0], T[0, 0]), te, re,
                              info['rmse']])
        rec = np.array(recovered)

        # B: repeatability -- spread of the per-cloud solutions.
        spread_xy = float(np.hypot(*(rec[:, :2] - rec[:, :2].mean(0)).T).std())
        spread_yaw = float(np.degrees(np.std(rec[:, 2])))

        # C: basin, using one representative cloud.
        q = clip(pts[cid == ids[len(ids) // 2]])
        basin = []
        for d, yaw in [(0.2, 5), (0.5, 10), (1.0, 20), (1.5, 30), (2.0, 40)]:
            T_init = pose_to_T(poses[i][0] + d / np.sqrt(2),
                               poses[i][1] + d / np.sqrt(2),
                               poses[i][2] + np.radians(yaw))
            T, _ = icp.icp(q, ref, ref_n, init=T_init, max_dist=1.0)
            te, re = icp.pose_error(T, T_true)
            basin.append({'offset_m': d, 'yaw_deg': yaw,
                          'trans_err_m': round(te, 4), 'rot_err_deg': round(re, 3),
                          'ok': bool(te < 0.15 and re < 4.0)})

        row = {'station': i, 'clouds': len(rec), 'map_voxels': int(len(ref)),
               'agreement_trans_p50_m': round(float(np.median(rec[:, 3])), 4),
               'agreement_trans_p95_m': round(float(np.quantile(rec[:, 3], .95)), 4),
               'agreement_rot_p50_deg': round(float(np.median(rec[:, 4])), 3),
               'repeatability_xy_std_m': round(spread_xy, 4),
               'repeatability_yaw_std_deg': round(spread_yaw, 3),
               'rmse_p50_m': round(float(np.median(rec[:, 5])), 4),
               'basin': basin}
        out['results'].append(row)
        ok = max([b['offset_m'] for b in basin if b['ok']], default=0.0)
        print('\nstation %d: %d clouds, map %d voxels' % (i, len(rec), len(ref)))
        print('  A agreement with odom : %.3f m p50, %.3f m p95, %.2f deg p50'
              % (row['agreement_trans_p50_m'], row['agreement_trans_p95_m'],
                 row['agreement_rot_p50_deg']))
        print('  B repeatability       : %.1f mm xy std, %.3f deg yaw std'
              % (1000 * spread_xy, spread_yaw))
        print('  C basin               : converged up to %.1f m offset' % ok)
        print('  residual RMSE         : %.3f m' % row['rmse_p50_m'])

    with open(os.path.join(HERE, 'real_study.json'), 'w') as f:
        json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
