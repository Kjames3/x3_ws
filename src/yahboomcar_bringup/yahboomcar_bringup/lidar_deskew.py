"""Numpy geometry and bounded interpolation for the X3 tilting lidar."""
import numpy as np


def interpolate_pitch(samples, times, max_gap, require_settled=True):
    """Samples are (seconds, radians, settled); never extrapolate."""
    a = np.asarray(samples, dtype=float)
    if len(a) < 2 or not np.isfinite(a).all():
        raise ValueError('insufficient tilt history')
    if np.any(np.diff(a[:, 0]) <= 0):
        raise ValueError('nonmonotonic tilt history')
    if times[0] < a[0, 0] or times[-1] > a[-1, 0]:
        raise ValueError('tilt does not bracket acquisition')
    lo = max(0, np.searchsorted(a[:, 0], times[0], side='right') - 1)
    hi = min(len(a) - 1, np.searchsorted(a[:, 0], times[-1]))
    window = a[lo:hi + 1]
    if np.any(np.diff(window[:, 0]) > max_gap):
        raise ValueError('tilt sample gap exceeds limit')
    if require_settled and not np.all(window[:, 2]):
        raise ValueError('mount moved during acquisition')
    # The joint has a bounded Y axis, so linear angle interpolation is
    # equivalent to shortest-arc SLERP over its physical travel.
    return np.interp(times, a[:, 0], a[:, 1])


def slerp(q0, q1, fractions):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    dot = np.dot(q0, q1)
    if dot < 0:
        q1, dot = -q1, -dot
    f = np.asarray(fractions)[:, None]
    if dot > 0.9995:
        q = q0 + f * (q1 - q0)
        return q / np.linalg.norm(q, axis=1)[:, None]
    theta = np.arccos(np.clip(dot, -1, 1))
    return (np.sin((1 - f) * theta) * q0 + np.sin(f * theta) * q1) / np.sin(theta)


def rotate(points, quaternion):
    q = np.asarray(quaternion)
    uv = 2 * np.cross(q[..., :3], points)
    return points + q[..., 3:4] * uv + np.cross(q[..., :3], uv)


def deskew(points, fractions, pitches, mount_start, mount_end,
           pivot, laser_transform, reference):
    """Compose odom->mount->pivot->tilt->laser for each return.

    Poses are (translation, quaternion xyzw). Output is in the scan-start
    laser TF frame, preserving octomap's physical raycasting origin.
    """
    lt, lq = laser_transform
    local = rotate(points, lq) + lt
    qpitch = np.zeros((len(points), 4))
    qpitch[:, 1] = np.sin(pitches / 2)
    qpitch[:, 3] = np.cos(pitches / 2)
    local = rotate(local, qpitch) + pivot
    t0, q0 = mount_start
    t1, q1 = mount_end
    world = rotate(local, slerp(q0, q1, fractions))
    world += np.asarray(t0) + fractions[:, None] * (np.asarray(t1) - t0)
    rt, rq = reference
    inverse = np.asarray(rq) * [-1, -1, -1, 1]
    return rotate(world - rt, inverse).astype('<f4')
