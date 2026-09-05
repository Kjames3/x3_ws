"""Pose-graph optimization over the five drive-capture stations.

The leave-one-out study failed not because registration is weak but because its
map was four stations placed by RAW ODOMETRY, which drifts 0.15-0.88 m across
one room -- a smeared double image. The fix is the standard one: measure the
relative pose between stations by registration, treat those as edges, optimise
the node poses, then rebuild the map from the optimised poses.

Planar SE(2) throughout. The chassis is a mecanum base on a flat floor and
every station's cloud is already upright (floor z~0 at all five), so the
out-of-plane DOF carry no information worth estimating and including them only
adds gauge freedom.

Edges are GATED, which matters more than the optimiser. Pair 3->4 (34% overlap)
has a false global minimum 5.27 m and 154.7 deg from the truth that fits BETTER
than the correct pose, so trusting lowest cost is not safe. An edge is admitted
only if it overlaps well, aligns tightly, and agrees with its own reverse
registration -- that last check is what catches a confident wrong answer, since
a false minimum is rarely symmetric.
"""

import json
import os

import numpy as np
from scipy.spatial import cKDTree

import icp
from real_study import clip, pose_to_T, station_poses
from study import build_map

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'drive_capture.npz')
SUBSAMPLE = 30000
INLIER_GATE = 0.15          # correspondence distance defining "overlapping"
MIN_OVERLAP = 0.50          # fraction of query points with a correspondence
MAX_INLIER_RMSE = 0.06      # metres
MAX_REVERSE_DISAGREE = 0.10  # m, forward vs reverse registration


def se2(T):
    """SE(3) transform -> (x, y, yaw)."""
    return np.array([T[0, 3], T[1, 3], np.arctan2(T[1, 0], T[0, 0])])


def se2_to_T(p):
    return icp.make_transform([0, 0, p[2]], [p[0], p[1], 0.0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s], [s, c]])


def register(src, ref, ref_n, init):
    T, info = icp.icp(src, ref, ref_n, init=init, max_dist=0.8)
    moved = icp.transform(src, T)
    d, i = cKDTree(ref).query(moved, k=1, distance_upper_bound=INLIER_GATE)
    ok = np.isfinite(d)
    if ok.sum() < 100:
        return T, 0.0, np.inf
    res = np.einsum('ij,ij->i', moved[ok] - ref[i[ok]], ref_n[i[ok]])
    return T, float(ok.mean()), float(np.sqrt((res ** 2).mean()))


def build_edges(clouds, odom_poses, rng):
    maps = {}
    for i, c in clouds.items():
        maps[i] = build_map(c)
    edges, rejected = [], []
    n = len(clouds)
    for i in range(n):
        for j in range(i + 1, n):
            src = clouds[j][rng.choice(len(clouds[j]),
                                       min(SUBSAMPLE, len(clouds[j])), False)]
            dst = clouds[i][rng.choice(len(clouds[i]),
                                       min(SUBSAMPLE, len(clouds[i])), False)]
            init = np.linalg.inv(se2_to_T(odom_poses[i])) @ se2_to_T(odom_poses[j])
            T_f, ov_f, rm_f = register(src, *maps[i], init)
            T_r, ov_r, rm_r = register(dst, *maps[j], np.linalg.inv(T_f))
            # A trustworthy edge must survive being measured backwards.
            disagree = float(np.linalg.norm(
                se2(T_f @ T_r)[:2]))
            overlap, rmse = min(ov_f, ov_r), max(rm_f, rm_r)
            rec = {'i': i, 'j': j, 'overlap': round(overlap, 3),
                   'inlier_rmse': round(rmse, 4),
                   'reverse_disagree_m': round(disagree, 4),
                   'z': se2(T_f).tolist()}
            if (overlap >= MIN_OVERLAP and rmse <= MAX_INLIER_RMSE
                    and disagree <= MAX_REVERSE_DISAGREE):
                edges.append(rec)
            else:
                why = []
                if overlap < MIN_OVERLAP:
                    why.append('overlap %.2f' % overlap)
                if rmse > MAX_INLIER_RMSE:
                    why.append('rmse %.3f' % rmse)
                if disagree > MAX_REVERSE_DISAGREE:
                    why.append('reverse %.3f m' % disagree)
                rec['rejected_because'] = ', '.join(why)
                rejected.append(rec)
    return edges, rejected


def optimize(poses, edges, iters=30):
    """Gauss-Newton on SE(2); node 0 is the fixed gauge."""
    x = poses.copy()
    n = len(x)
    for _ in range(iters):
        H = np.zeros((3 * n, 3 * n))
        b = np.zeros(3 * n)
        for e in edges:
            i, j, z = e['i'], e['j'], np.asarray(e['z'])
            Ri, Rz = rot(x[i, 2]), rot(z[2])
            dt = x[j, :2] - x[i, :2]
            err = np.empty(3)
            err[:2] = Rz.T @ (Ri.T @ dt - z[:2])
            err[2] = wrap(x[j, 2] - x[i, 2] - z[2])
            dRi = np.array([[-np.sin(x[i, 2]), np.cos(x[i, 2])],
                            [-np.cos(x[i, 2]), -np.sin(x[i, 2])]])
            A = np.zeros((3, 3))
            A[:2, :2] = -Rz.T @ Ri.T
            A[:2, 2] = Rz.T @ dRi @ dt
            A[2, 2] = -1.0
            B = np.zeros((3, 3))
            B[:2, :2] = Rz.T @ Ri.T
            B[2, 2] = 1.0
            # Weight by inlier count proxy: tighter alignments constrain more.
            w = 1.0 / max(e['inlier_rmse'], 1e-3) ** 2
            si, sj = slice(3 * i, 3 * i + 3), slice(3 * j, 3 * j + 3)
            H[si, si] += w * A.T @ A
            H[si, sj] += w * A.T @ B
            H[sj, si] += w * B.T @ A
            H[sj, sj] += w * B.T @ B
            b[si] += w * A.T @ err
            b[sj] += w * B.T @ err
        H[:3, :3] += np.eye(3) * 1e9        # fix node 0
        dx = np.linalg.solve(H + np.eye(3 * n) * 1e-9, -b).reshape(n, 3)
        x += dx
        x[:, 2] = wrap(x[:, 2])
        if np.linalg.norm(dx) < 1e-9:
            break
    return x


def sharpness(clouds, poses, rng):
    """Occupied 5 cm voxels of the merged map, plus mean NN distance between
    stations. A smeared map fills MORE voxels for the same surfaces."""
    merged = np.concatenate([icp.transform(clouds[i], se2_to_T(poses[i]))
                             for i in clouds])
    keys = np.floor(merged / 0.05).astype(np.int64)
    voxels = len(np.unique(keys, axis=0))
    a = icp.transform(clouds[0], se2_to_T(poses[0]))
    b = np.concatenate([icp.transform(clouds[i], se2_to_T(poses[i]))
                        for i in clouds if i != 0])
    sub = a[rng.choice(len(a), 20000, False)]
    d, _ = cKDTree(b[rng.choice(len(b), 120000, False)]).query(sub, k=1)
    return voxels, float(np.median(d))


def main():
    rng = np.random.default_rng(0)
    data = np.load(SRC)
    pts, cid, meta = data['points'].astype(float), data['cloud_id'], data['cloud_meta']
    odom_poses = station_poses(data['odom'])
    cs = {int(m[0]): int(m[1]) for m in meta}
    sp = np.array([cs[c] for c in cid])
    clouds = {i: clip(pts[sp == i]) for i in range(len(odom_poses))}

    edges, rejected = build_edges(clouds, odom_poses, rng)
    print('edges accepted: %d of %d possible' % (len(edges), 10))
    for e in edges:
        print('  %d-%d overlap %.0f%% rmse %.3f m reverse %.3f m'
              % (e['i'], e['j'], 100 * e['overlap'], e['inlier_rmse'],
                 e['reverse_disagree_m']))
    for e in rejected:
        print('  REJECTED %d-%d: %s' % (e['i'], e['j'], e['rejected_because']))
    # Edge COUNT does not imply connectivity: this capture produced 6 edges
    # over 5 stations, which passes any count test, while station 3 had zero of
    # them and silently kept its raw odometry pose. Check the component.
    adj = {i: set() for i in range(len(odom_poses))}
    for e in edges:
        adj[e['i']].add(e['j'])
        adj[e['j']].add(e['i'])
    seen, stack = {0}, [0]
    while stack:
        for v in adj[stack.pop()]:
            if v not in seen:
                seen.add(v)
                stack.append(v)
    isolated = [i for i in range(len(odom_poses)) if i not in seen]
    if isolated:
        print('\nWARNING: stations %s are DISCONNECTED from the graph. They '
              'keep their raw odometry pose and are NOT localized; any map '
              'built from them is still smeared.' % isolated)

    opt = optimize(odom_poses, edges)
    print('\n%8s %26s %26s %10s' % ('station', 'odometry', 'optimised', 'moved'))
    for i in range(len(opt)):
        d = np.linalg.norm(opt[i, :2] - odom_poses[i, :2])
        print('%8d %10.3f %7.3f %5.1fdeg %10.3f %7.3f %5.1fdeg %9.3f m'
              % (i, odom_poses[i, 0], odom_poses[i, 1], np.degrees(odom_poses[i, 2]),
                 opt[i, 0], opt[i, 1], np.degrees(opt[i, 2]), d))

    v0, d0 = sharpness(clouds, odom_poses, rng)
    v1, d1 = sharpness(clouds, opt, rng)
    print('\nmap sharpness (lower is sharper):')
    print('  odometry  : %7d voxels, median cross-station NN %.4f m' % (v0, d0))
    print('  optimised : %7d voxels, median cross-station NN %.4f m' % (v1, d1))
    print('  change    : %+.1f%% voxels, %+.1f%% NN'
          % (100 * (v1 - v0) / v0, 100 * (d1 - d0) / d0))

    # Residuals: how self-consistent is the optimised graph?
    res = []
    for e in edges:
        i, j, z = e['i'], e['j'], np.asarray(e['z'])
        rel = np.linalg.inv(se2_to_T(opt[i])) @ se2_to_T(opt[j])
        res.append(np.linalg.norm(se2(rel)[:2] - z[:2]))
    print('\nedge residual after optimisation: p50 %.4f m, max %.4f m'
          % (np.median(res), np.max(res)))

    with open(os.path.join(HERE, 'posegraph.json'), 'w') as f:
        json.dump({'odom': odom_poses.tolist(), 'optimised': opt.tolist(),
                   'edges': edges, 'rejected': rejected,
                   'sharpness': {'odom_voxels': v0, 'opt_voxels': v1,
                                 'odom_nn_m': d0, 'opt_nn_m': d1},
                   'edge_residual_p50_m': float(np.median(res))}, f, indent=1)


if __name__ == '__main__':
    main()
