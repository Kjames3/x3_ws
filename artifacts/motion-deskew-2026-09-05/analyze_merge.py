"""Score chassis-motion deskew by whether consecutive clouds MERGE consistently.

`analyze.py`'s wall-straightness metric failed and is superseded. Two reasons,
both worth recording:

  * It is dominated by scene content, not motion. The PARKED bin scored the
    WORST residual (75 mm, against 23 mm while turning) purely because the
    robot happened to be parked facing clutter. Cross-bin comparison was
    therefore meaningless.
  * A 3-5 cm motion shear is invisible inside a 23-75 mm baseline set by
    furniture and corners, so even the controlled deskew-vs-rigid ratio came
    out 1.00 when the deskew was demonstrably moving points by up to 1.15 m.

This metric instead asks the question localization actually cares about: when
two clouds taken 137 ms apart are placed in a common frame by odometry, do
their surfaces coincide? An internally sheared cloud cannot agree with its
neighbour no matter how the pair is positioned, so the median nearest-neighbour
distance between consecutive clouds is a direct read on internal distortion --
the same measure that showed the pose graph collapsing a smeared map
(0.0660 -> 0.0234 m).

Deskewed and rigid clouds are scored identically, on the same cloud pairs, with
the same odometry placement, so only the compensation differs.
"""

import json
import os

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'motion_capture.npz')
# base_footprint -> laser_link, from the URDF chain: lidar_mount 0.0557 +
# tilt joint -0.0125 in x, 0.076 + 0.062 + 0.153 + 0.022 in z, and
# laser_joint's yaw = pi.
LASER_XYZ = np.array([0.0432, 0.0, 0.313])
LASER_YAW = np.pi
SAMPLE = 4000
MAX_NN = 0.30


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def to_odom(points, pose):
    """laser frame at scan start -> odom, given base pose (x, y, yaw)."""
    p = points @ rotz(LASER_YAW).T + LASER_XYZ
    return p @ rotz(pose[2]).T + np.array([pose[0], pose[1], 0.0])


def pose_at(odom, t):
    i = int(np.argmin(np.abs(odom[:, 0] - t)))
    if abs(odom[i, 0] - t) > 0.20:
        return None
    return odom[i, 1:4]


def merge_error(a, b, rng):
    """Median NN distance from a sample of cloud a to cloud b, both in odom."""
    if len(a) < 200 or len(b) < 200:
        return np.nan
    s = a[rng.choice(len(a), min(SAMPLE, len(a)), replace=False)]
    d, _ = cKDTree(b).query(s, k=1, distance_upper_bound=MAX_NN)
    d = d[np.isfinite(d)]
    if len(d) < 100:
        return np.nan
    return float(np.median(d))


def main():
    z = np.load(SRC)
    dk, rg, cid, meta, odom = (z['deskew'], z['rigid'], z['cloud_id'],
                               z['meta'], z['odom'])
    rng = np.random.default_rng(0)
    speed, yaw = meta[:, 3], np.abs(meta[:, 4])
    idx_of = {int(m[0]): k for k, m in enumerate(meta)}

    rows = []
    order = meta[:, 0].astype(int)
    for a, b in zip(order[:-1], order[1:]):
        ka, kb = idx_of[a], idx_of[b]
        ta, tb = meta[ka, 1], meta[kb, 1]
        if not (0.05 < tb - ta < 0.40):
            continue
        pa, pb = pose_at(odom, ta), pose_at(odom, tb)
        if pa is None or pb is None:
            continue
        ma, mb = cid == a, cid == b
        e_d = merge_error(to_odom(dk[ma], pa), to_odom(dk[mb], pb), rng)
        e_r = merge_error(to_odom(rg[ma], pa), to_odom(rg[mb], pb), rng)
        if not (np.isfinite(e_d) and np.isfinite(e_r)):
            continue
        rows.append([0.5 * (speed[ka] + speed[kb]),
                     0.5 * (yaw[ka] + yaw[kb]), e_d, e_r])
    r = np.array(rows)
    print('consecutive-cloud merge error, %d pairs\n' % len(r))
    print('%-22s %6s %12s %12s %8s'
          % ('bin', 'pairs', 'deskew mm', 'rigid mm', 'ratio'))

    out = []
    for name, lo, hi, ylo, yhi in [('parked', 0, .02, 0, .05),
                                   ('slow', .02, .15, 0, 10),
                                   ('medium', .15, .30, 0, 10),
                                   ('fast', .30, 9, 0, 10),
                                   ('turning yaw>0.3', 0, 9, .30, 10),
                                   ('turning yaw>0.6', 0, 9, .60, 10)]:
        m = (r[:, 0] >= lo) & (r[:, 0] < hi) & (r[:, 1] >= ylo) & (r[:, 1] < yhi)
        if m.sum() < 5:
            continue
        a, b = 1000 * np.median(r[m, 2]), 1000 * np.median(r[m, 3])
        out.append({'bin': name, 'pairs': int(m.sum()),
                    'deskew_mm': round(a, 2), 'rigid_mm': round(b, 2),
                    'ratio': round(b / max(a, 1e-6), 3)})
        print('%-22s %6d %12.2f %12.2f %8.2f' % (name, m.sum(), a, b, b / max(a, 1e-6)))

    with open(os.path.join(HERE, 'merge_analysis.json'), 'w') as f:
        json.dump({'bins': out, 'pairs': int(len(r))}, f, indent=1)

    parked = next((o for o in out if o['bin'] == 'parked'), None)
    print()
    if parked:
        print('control: parked deskew %.2f mm vs rigid %.2f mm (must agree -- '
              'no motion to correct)' % (parked['deskew_mm'], parked['rigid_mm']))
    for o in out:
        if o['bin'] in ('fast', 'turning yaw>0.6') and parked:
            gd = o['deskew_mm'] / max(parked['deskew_mm'], 1e-6)
            gr = o['rigid_mm'] / max(parked['rigid_mm'], 1e-6)
            print('%-22s deskew %.2fx parked, rigid %.2fx parked -> %s'
                  % (o['bin'], gd, gr,
                     'compensation WORKING' if gd < 1.5 <= gr else
                     'both degrade: compensation INSUFFICIENT' if gd >= 1.5
                     else 'no motion penalty detected'))


if __name__ == '__main__':
    main()
