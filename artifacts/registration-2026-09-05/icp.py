"""Point-to-plane ICP in numpy, with the 6x6 information matrix exposed.

Written rather than pulled from open3d/small_gicp (neither is installed on this
machine) because the question this study asks is not "what pose came out" but
"which degrees of freedom did the sweep actually constrain".  That answer lives
in the eigenvalues of J^T J, which the library APIs do not hand back.

Convention: transforms are 4x4, applied as `x @ R.T + t`.  Rotations are solved
in the small-angle linearisation and composed exactly, so large initial errors
are recovered by iterating rather than by trusting the linearisation.
"""

import numpy as np
from scipy.spatial import cKDTree


def voxel_downsample(points, voxel):
    """Centroid per occupied voxel.  Keeps the surface, drops the density bias
    that would otherwise let a densely-sampled near wall outvote the room."""
    keys = np.floor(points / voxel).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True,
                               return_counts=True)
    summed = np.zeros((len(counts), 3))
    np.add.at(summed, inv, points)
    return summed / counts[:, None]


def estimate_normals(points, k=16):
    """PCA normal per point.  Also returns planarity (1 - l0/l1), used to drop
    points on clutter where a plane is not a meaningful local model."""
    tree = cKDTree(points)
    _, idx = tree.query(points, k=min(k, len(points)))
    nbrs = points[idx]
    centred = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', centred, centred) / centred.shape[1]
    vals, vecs = np.linalg.eigh(cov)
    normals = vecs[:, :, 0]
    planarity = 1.0 - vals[:, 0] / np.maximum(vals[:, 1], 1e-12)
    return normals, planarity


def rotation(rvec):
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        return np.eye(3)
    axis = rvec / theta
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def transform(points, T):
    return points @ T[:3, :3].T + T[:3, 3]


def make_transform(rvec, t):
    T = np.eye(4)
    T[:3, :3] = rotation(np.asarray(rvec, float))
    T[:3, 3] = t
    return T


def estimate_normals_2d(points, k=12):
    """In-plane normals from 2D PCA on (x, y).

    A 2D scan matcher's normals are the WALL normals, which are horizontal.
    Running 3D PCA over a thin horizontal slab instead returns the slab's own
    vertical normal for every point, and point-to-plane then measures only
    vertical displacement -- which x, y and yaw cannot move.  That produces a
    baseline that fails at 0.1 m, which is a bug, not evidence about 2D.
    """
    xy = points[:, :2]
    tree = cKDTree(xy)
    _, idx = tree.query(xy, k=min(k, len(xy)))
    nbrs = xy[idx]
    centred = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', centred, centred) / centred.shape[1]
    vals, vecs = np.linalg.eigh(cov)
    n = np.zeros((len(points), 3))
    n[:, :2] = vecs[:, :, 0]
    planarity = 1.0 - vals[:, 0] / np.maximum(vals[:, 1], 1e-12)
    return n, planarity


def icp(source, target, target_normals, init=None, max_iter=50,
        max_dist=0.5, tol=1e-5, planar=False):
    """Register `source` onto `target`.  Returns (T, info).

    `info['hessian']` is J^T J at the solution: its eigenvalues say which DOF
    the geometry constrained and which it merely tolerated.

    `planar=True` solves only (yaw, tx, ty), the 3 DOF a 2D scan matcher has.
    Without it the roll/pitch/tz columns of a planar problem are identically
    zero and the 6x6 solve is singular.
    """
    active = [2, 3, 4] if planar else [0, 1, 2, 3, 4, 5]
    tree = cKDTree(target)
    T = np.eye(4) if init is None else init.copy()
    prev = None
    hessian = np.zeros((6, 6))
    inliers = 0
    for _ in range(max_iter):
        moved = transform(source, T)
        dist, idx = tree.query(moved, k=1, distance_upper_bound=max_dist)
        ok = np.isfinite(dist)
        if ok.sum() < 50:
            break
        p, q = moved[ok], target[idx[ok]]
        n = target_normals[idx[ok]]
        residual = np.einsum('ij,ij->i', p - q, n)
        # Huber weights: a sweep sees through doorways and past the map edge,
        # so a hard inlier count alone still lets a few long pairings dominate.
        scale = 1.4826 * np.median(np.abs(residual)) + 1e-9
        w = np.where(np.abs(residual) < 2 * scale, 1.0,
                     2 * scale / np.abs(residual))
        J = np.hstack([np.cross(p, n), n])[:, active]
        Jw = J * w[:, None]
        hessian = J.T @ Jw
        try:
            solved = np.linalg.solve(hessian, -Jw.T @ residual)
        except np.linalg.LinAlgError:
            break
        delta = np.zeros(6)
        delta[active] = solved
        T = make_transform(delta[:3], delta[3:]) @ T
        inliers = int(ok.sum())
        step = np.linalg.norm(delta)
        if prev is not None and abs(prev - step) < tol and step < tol:
            break
        prev = step
    return T, {'hessian': hessian, 'inliers': inliers,
               'rmse': float(np.sqrt(np.mean(residual ** 2)))}


def pose_error(T_est, T_true):
    """(translation error in m, rotation error in deg) of est relative to true."""
    err = np.linalg.inv(T_true) @ T_est
    trace = np.clip((np.trace(err[:3, :3]) - 1) / 2, -1, 1)
    return float(np.linalg.norm(err[:3, 3])), float(np.degrees(np.arccos(trace)))
