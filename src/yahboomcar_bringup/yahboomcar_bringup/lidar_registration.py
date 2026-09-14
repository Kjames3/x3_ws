"""Diagnostic planar localization using 3D point-to-plane correspondences.

No ROS or hardware access. Match rejection is deliberately conservative; it
is not a guarantee against perceptual aliasing. A map-aligned initial guess is
required. No global relocalization or covariance claim is made.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy.spatial import cKDTree


def pose_matrix(x=0., y=0., yaw=0.):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0., x], [s, c, 0., y],
                     [0., 0., 1., 0.], [0., 0., 0., 1.]])


def transform(p, T):
    return p @ T[:3, :3].T + T[:3, 3]


def component(edges, anchor=0):
    seen, pending = {anchor}, [anchor]
    while pending:
        u = pending.pop()
        for e in edges:
            v = e['j'] if e['i'] == u else e['i'] if e['j'] == u else None
            if v is not None and v not in seen:
                seen.add(v); pending.append(v)
    return sorted(seen)


@dataclass(frozen=True)
class Limits:
    max_points: int = 3000
    max_iterations: int = 30
    correspondence_m: float = .40
    inlier_m: float = .15
    min_points: int = 150
    min_overlap: float = .60
    max_plane_rmse_m: float = .06
    max_nn_rmse_m: float = .10
    max_correction_m: float = .30
    max_correction_yaw_rad: float = np.deg2rad(15.)
    max_condition: float = 1000.
    max_runtime_s: float = .12


class Matcher:
    def __init__(self, points, normals, limits=None):
        self.limits = limits or Limits()
        p, n = np.asarray(points, float), np.asarray(normals, float)
        if p.ndim != 2 or p.shape[1] != 3 or n.shape != p.shape:
            raise ValueError('map points/normals must be matching Nx3 arrays')
        norm = np.linalg.norm(n, axis=1)
        if len(p) < self.limits.min_points or not np.isfinite(p).all() or not np.isfinite(n).all() or np.any(norm < .5):
            raise ValueError('invalid or insufficient map geometry')
        self.points, self.normals = p, n / norm[:, None]
        self.tree = cKDTree(p)

    def match(self, points, initial):
        started = time.perf_counter(); L = self.limits
        T = np.asarray(initial, float).copy()
        result = {'accepted': False, 'reasons': [], 'iterations': 0}
        def finish(reasons):
            result.update(reasons=reasons, accepted=not reasons,
                          runtime_s=time.perf_counter()-started)
            return T, result
        # Only upright planar priors are meaningful to this constrained solver.
        if T.shape != (4, 4) or not np.isfinite(T).all():
            return finish(['invalid_prior'])
        initial = T.copy()
        yaw = np.arctan2(T[1, 0], T[0, 0])
        if not np.allclose(T, pose_matrix(T[0, 3], T[1, 3], yaw), atol=1e-5):
            return finish(['nonplanar_prior'])
        p = np.asarray(points, float)
        if p.ndim != 2 or p.shape[1] != 3:
            return finish(['invalid_cloud'])
        p = p[np.isfinite(p).all(axis=1)]
        r = np.linalg.norm(p, axis=1)
        p = p[(r > .15) & (r < 6.)]
        if len(p) > L.max_points:
            p = p[np.linspace(0, len(p)-1, L.max_points).astype(int)]
        result['query_points'] = len(p)
        if len(p) < L.min_points:
            return finish(['insufficient_points'])
        converged = False
        for iteration in range(L.max_iterations):
            if time.perf_counter()-started > L.max_runtime_s:
                return finish(['runtime_budget'])
            moved = transform(p, T)
            d, idx = self.tree.query(moved, distance_upper_bound=L.correspondence_m)
            ok = np.isfinite(d)
            if ok.sum() < L.min_points:
                return finish(['insufficient_correspondences'])
            a, b, n = moved[ok], self.points[idx[ok]], self.normals[idx[ok]]
            residual = np.einsum('ij,ij->i', a-b, n)
            # Rotate around the query origin, avoiding map-origin-dependent conditioning.
            rel = a - T[:3, 3]
            J = np.column_stack((-rel[:, 1]*n[:, 0]+rel[:, 0]*n[:, 1], n[:, 0], n[:, 1]))
            scale = max(.005, 1.4826 * np.median(np.abs(residual)))
            w = np.minimum(1., 2*scale/np.maximum(np.abs(residual), 1e-12))
            H = J.T @ (J*w[:, None])
            diag = np.sqrt(np.maximum(np.diag(H), 1e-20))
            condition = float(np.linalg.cond(H/diag[:, None]/diag[None, :]))
            if not np.isfinite(condition) or condition > L.max_condition or np.min(np.diag(H)) < 1e-8:
                return finish(['degenerate_geometry'])
            try: delta = np.linalg.solve(H, -J.T @ (w*residual))
            except np.linalg.LinAlgError: return finish(['degenerate_geometry'])
            yaw += delta[0]
            T = pose_matrix(T[0, 3]+delta[1], T[1, 3]+delta[2], yaw)
            result['iterations'] = iteration+1
            if np.linalg.norm(delta[1:]) < .0005 and abs(delta[0]) < .0002:
                converged = True; break
        moved = transform(p, T)
        d, idx = self.tree.query(moved, distance_upper_bound=L.inlier_m)
        ok = np.isfinite(d); overlap = float(ok.mean())
        reasons = []
        if ok.sum() < L.min_points: return finish(['insufficient_inliers'])
        n = self.normals[idx[ok]]
        residual = np.einsum('ij,ij->i', moved[ok]-self.points[idx[ok]], n)
        plane = float(np.sqrt(np.mean(residual**2)))
        nn = float(np.sqrt(np.mean(d[ok]**2)))
        # Recheck observability on the final, tighter inlier set.
        rel = moved[ok]-T[:3, 3]
        J = np.column_stack((-rel[:, 1]*n[:, 0]+rel[:, 0]*n[:, 1], n[:, 0], n[:, 1]))
        H = J.T @ J; diag = np.sqrt(np.maximum(np.diag(H), 1e-20))
        condition = float(np.linalg.cond(H/diag[:, None]/diag[None, :]))
        correction = float(np.linalg.norm(T[:2, 3]-initial[:2, 3]))
        angle = float(abs(np.arctan2(np.sin(yaw-np.arctan2(initial[1, 0], initial[0, 0])),
                                    np.cos(yaw-np.arctan2(initial[1, 0], initial[0, 0])))))
        if not converged: reasons.append('not_converged')
        if overlap < L.min_overlap: reasons.append('low_overlap')
        if plane > L.max_plane_rmse_m: reasons.append('plane_residual')
        if nn > L.max_nn_rmse_m: reasons.append('nn_residual')
        if correction > L.max_correction_m or angle > L.max_correction_yaw_rad: reasons.append('prior_disagreement')
        if not np.isfinite(condition) or condition > L.max_condition or np.min(np.diag(H)) < 1e-8: reasons.append('degenerate_geometry')
        if time.perf_counter()-started > L.max_runtime_s: reasons.append('runtime_budget')
        result.update(overlap=overlap, plane_rmse_m=plane, nn_rmse_m=nn,
                      correction_m=correction, correction_yaw_rad=angle,
                      condition=condition)
        return finish(reasons)
