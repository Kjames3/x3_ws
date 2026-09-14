"""Payload checks for a full parked continuous sweep (no ROS dependency)."""
import numpy as np


def sweep_quality(cloud_meta, joints):
    clouds = np.asarray(cloud_meta, dtype=float)
    samples = np.asarray(joints, dtype=float)
    if len(clouds) < 40 or len(samples) < 2:
        return False, {'reason': 'insufficient clouds or joint samples'}
    times = clouds[:, 2]
    if (not np.isfinite(times).all() or not np.isfinite(samples[:, :2]).all()
            or np.any(np.diff(times) <= 0) or np.any(np.diff(samples[:, 0]) <= 0)):
        return False, {'reason': 'invalid or nonmonotonic acquisition timestamps'}
    # Require measured joint samples bracketing every retained cloud.
    if times[0] < samples[0, 0] or times[-1] > samples[-1, 0]:
        return False, {'reason': 'joint samples do not bracket clouds'}
    pitch = np.rad2deg(np.interp(times, samples[:, 0], samples[:, 1]))
    relevant = samples[(samples[:, 0] >= times[0]) & (samples[:, 0] <= times[-1]), 0]
    gap = float(np.max(np.diff(times)))
    joint_gap = float(np.max(np.diff(relevant))) if len(relevant) > 1 else float('inf')
    bins = np.histogram(pitch, bins=[-90, -30, -15, 0, 15, 30, 90])[0]
    ok = bool(pitch.min() <= -35 and pitch.max() >= 35
              and np.all(bins >= 3) and gap <= .5 and joint_gap <= .3)
    return ok, dict(reason='ok' if ok else 'incomplete sweep or acquisition gaps',
                    clouds=len(clouds), tilt_min_deg=float(pitch.min()),
                    tilt_max_deg=float(pitch.max()), max_cloud_gap_s=gap,
                    max_joint_gap_s=joint_gap, angle_bin_counts=bins.tolist())
