"""Score chassis-motion deskew: wall straightness vs speed, deskewed vs rigid.

A level scan is a horizontal slice, so walls are straight LINES in it.
Uncompensated chassis motion shears those lines, and the shear grows with
speed. Per cloud we fit a line to each bearing sector and take the residual.

Sector choice matters. A sector spanning a corner contains two walls and fits
badly no matter what, so the headline metric is the 25th-percentile sector --
the cleanest walls in view -- with the median reported alongside. Both are
computed identically for the deskewed and rigid clouds, which describe the SAME
returns, so the comparison is not confounded by which points were kept.

Read it this way: if the rigid residual climbs with speed while the deskewed
residual stays flat, compensation works. If both climb together, it does not.
The parked bin is the control -- at zero speed the two must agree, and if they
do not, something other than motion is being measured.
"""

import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, 'motion_capture.npz')
N_SECTORS = 12
MIN_SECTOR_PTS = 40
MAX_RANGE = 6.0


def sector_residuals(points):
    """RMS distance to a fitted line, per bearing sector, in metres."""
    d = np.hypot(points[:, 0], points[:, 1])
    keep = (d > 0.2) & (d < MAX_RANGE)
    p = points[keep][:, :2]
    if len(p) < MIN_SECTOR_PTS:
        return np.array([])
    b = np.arctan2(p[:, 1], p[:, 0])
    idx = np.floor((b + np.pi) / (2 * np.pi) * N_SECTORS).astype(int) % N_SECTORS
    out = []
    for s in range(N_SECTORS):
        q = p[idx == s]
        if len(q) < MIN_SECTOR_PTS:
            continue
        c = q - q.mean(0)
        # Total-least-squares line: residual is the smaller singular value.
        _, sv, _ = np.linalg.svd(c, full_matrices=False)
        out.append(sv[-1] / np.sqrt(len(q)))
    return np.array(out)


def score(points_by_cloud):
    p25, med = [], []
    for pts in points_by_cloud:
        r = sector_residuals(pts)
        if len(r) < 3:
            continue
        p25.append(np.quantile(r, 0.25))
        med.append(np.median(r))
    return np.array(p25), np.array(med)


def main():
    d = np.load(SRC)
    dk, rg, cid, meta = d['deskew'], d['rigid'], d['cloud_id'], d['meta']
    speed, yaw = meta[:, 3], np.abs(meta[:, 4])

    bins = [('parked', 0.0, 0.02, 0.0, 0.05),
            ('slow',   0.02, 0.15, 0.0, 10.0),
            ('medium', 0.15, 0.30, 0.0, 10.0),
            ('fast',   0.30, 9.0,  0.0, 10.0),
            ('turning (yaw>0.3)', 0.0, 9.0, 0.30, 10.0)]

    print('%-20s %6s %14s %14s %9s'
          % ('bin', 'clouds', 'deskew p25 mm', 'rigid p25 mm', 'ratio'))
    result = []
    for name, lo, hi, ylo, yhi in bins:
        sel = np.flatnonzero((speed >= lo) & (speed < hi)
                             & (yaw >= ylo) & (yaw < yhi))
        if len(sel) < 5:
            print('%-20s %6d  (too few)' % (name, len(sel)))
            continue
        ids = meta[sel, 0].astype(int)
        dsets = [dk[cid == i] for i in ids]
        rsets = [rg[cid == i] for i in ids]
        dp25, dmed = score(dsets)
        rp25, rmed = score(rsets)
        if not len(dp25) or not len(rp25):
            continue
        a, b = 1000 * np.median(dp25), 1000 * np.median(rp25)
        result.append({'bin': name, 'clouds': int(len(sel)),
                       'deskew_p25_mm': round(a, 2), 'rigid_p25_mm': round(b, 2),
                       'deskew_med_mm': round(1000 * np.median(dmed), 2),
                       'rigid_med_mm': round(1000 * np.median(rmed), 2),
                       'ratio': round(b / max(a, 1e-6), 2)})
        print('%-20s %6d %14.2f %14.2f %9.2f'
              % (name, len(sel), a, b, b / max(a, 1e-6)))

    with open(os.path.join(HERE, 'analysis.json'), 'w') as f:
        json.dump({'bins': result, 'n_clouds': int(len(meta))}, f, indent=1)

    parked = [r for r in result if r['bin'] == 'parked']
    fast = [r for r in result if r['bin'] in ('fast', 'turning (yaw>0.3)')]
    print()
    if not parked:
        print('NO PARKED BASELINE -- the control bin is missing, so a flat '
              'result cannot be distinguished from a broken measurement.')
        return
    p = parked[0]
    print('control: parked deskew %.2f mm vs rigid %.2f mm (should agree; '
          'they describe identical returns with no motion to correct)'
          % (p['deskew_p25_mm'], p['rigid_p25_mm']))
    for f_ in fast:
        grew_d = f_['deskew_p25_mm'] / max(p['deskew_p25_mm'], 1e-6)
        grew_r = f_['rigid_p25_mm'] / max(p['rigid_p25_mm'], 1e-6)
        print('%-20s deskew grew %.2fx vs parked, rigid grew %.2fx  -> %s'
              % (f_['bin'], grew_d, grew_r,
                 'compensation WORKING' if grew_d < 1.5 and grew_r > 1.5
                 else 'inconclusive' if grew_r < 1.5
                 else 'compensation NOT working'))


if __name__ == '__main__':
    main()
