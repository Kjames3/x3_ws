"""Offline item-5 study: virtual /scan coverage, age and gaps.

Replays the 30 s parked 45 deg/s continuous sweep captured on 2026-09-05
(artifacts/continuous-2026-09-05/continuous_45_points.npz, deskewed clouds in
base_footprint) through virtual_scan.synthesize.

The npz stores concatenated points with no per-cloud boundary or stamp, so the
acquisition tilt of every return is RECONSTRUCTED geometrically: the tilt axis
is +Y, so a beam at bearing psi leaves along
(cos psi cos th, sin psi, -cos psi sin th) and th = atan2(-vz, vx) for every
psi except the two on-axis bearings.  Validated against the servo record: the
recovered p1/p99 tilt is -44.1/+44.4 deg versus the joint topic's -44.1/+44.4.

A tilt window then maps to a time window at the measured 44.97 deg/s, and the
ping-pong period is 2 * 90 / 45 = 4 s.  Each tilt is visited twice per cycle
(once climbing, once descending), which is how a per-bearing refresh gap is
computed.  Caveat: the two passes are merged, so COVERAGE is marginally
optimistic (a bearing lit on only one pass counts as lit); the GAP figures are
not affected, since they model both visits explicitly.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src',
                                'yahboomcar_bringup'))
from yahboomcar_bringup.virtual_scan import (  # noqa: E402
    DEFAULT_Z_MAX, DEFAULT_Z_MIN, synthesize)

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'continuous-2026-09-05', 'continuous_45_points.npz')
PIVOT = np.array([0.0432, 0.0, 0.291])   # tilt joint origin in base_footprint
LASER_OFFSET = 0.022                     # tilt_link -> laser_link, along tilted z
SPEED_DEG_S = 44.97                      # measured central sweep speed
SWEEP_DEG = 45.0
PERIOD_S = 4 * SWEEP_DEG / SPEED_DEG_S   # full ping-pong cycle
N_BINS = 360
RANGE_MAX = 6.0                          # the deployed OctoMap/cloud cap


def recover_tilt(points, iters=3):
    theta = np.zeros(len(points))
    for _ in range(iters):
        laser = PIVOT + np.stack([LASER_OFFSET * np.sin(theta),
                                  np.zeros(len(points)),
                                  LASER_OFFSET * np.cos(theta)], axis=1)
        v = points - laser
        theta = np.arctan2(-v[:, 2], v[:, 0])
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        # Rear beams have cos(psi) < 0, which flips both components.
        theta = np.where(np.abs(theta) > np.pi / 2,
                         theta - np.sign(theta) * np.pi, theta)
    return theta


def bearings(points):
    b = np.arctan2(points[:, 1], points[:, 0])
    return np.floor((b + np.pi) / (2 * np.pi) * N_BINS).astype(int) % N_BINS


def sector(idx, half_width_deg):
    """Boolean mask over bin indices for a forward sector of given half width."""
    deg = (idx * 360.0 / N_BINS) - 180.0     # bin 180 == +x == forward
    return np.abs(deg) <= half_width_deg


def coverage_vs_window(points, theta_deg, z_min, z_max):
    """Coverage as a function of how much sweep travel is accumulated."""
    out = []
    for win_deg in (6.4, 12.8, 22.5, 45.0, 90.0):
        # 6.4 deg == one 0.1424 s scan at 44.97 deg/s: a single cloud.
        best = None
        for start in np.arange(-45.0, 45.0 - win_deg + 1e-9, 5.0):
            m = (theta_deg >= start) & (theta_deg < start + win_deg)
            r, _ = synthesize(points[m], N_BINS, z_min, z_max,
                              range_max=RANGE_MAX)
            hit = np.isfinite(r)
            idx = np.arange(N_BINS)
            cov = float(hit.mean())
            fwd = float(hit[sector(idx, 30)].mean())
            if best is None or cov < best[0]:
                best = (cov, fwd, float(start))
        out.append({'window_deg': win_deg,
                    'window_s': round(win_deg / SPEED_DEG_S, 3),
                    'worst_coverage_all': round(best[0], 3),
                    'worst_coverage_forward_60deg': round(best[1], 3),
                    'worst_window_start_deg': best[2]})
    return out


def refresh_gaps(points, theta_deg, z_min, z_max):
    """Longest interval each bearing goes unmeasured over one ping-pong cycle."""
    d = np.hypot(points[:, 0], points[:, 1])
    keep = ((points[:, 2] >= z_min) & (points[:, 2] <= z_max)
            & (d >= 0.15) & (d <= RANGE_MAX))
    idx, th = bearings(points)[keep], theta_deg[keep]
    # 1-deg tilt cells: finer than a single scan's 6.4 deg of travel, so a lit
    # cell means the bearing really was illuminated during that travel.
    cell = np.clip(((th + 45.0)).astype(int), 0, 89)
    lit = np.zeros((N_BINS, 90), dtype=bool)
    lit[idx, cell] = True

    gaps = np.full(N_BINS, np.inf)
    for b in range(N_BINS):
        cells = np.flatnonzero(lit[b])
        if cells.size == 0:
            continue
        # Each lit tilt cell is visited twice per cycle: climbing at
        # t = th/speed and descending at t = period - th/speed.
        t_up = (cells + 0.5) / SPEED_DEG_S
        times = np.sort(np.concatenate([t_up, PERIOD_S - t_up]))
        wrapped = np.diff(np.concatenate([times, [times[0] + PERIOD_S]]))
        gaps[b] = wrapped.max()
    return gaps


def amplitude_study(points, theta_deg, z_min, z_max, speed_deg_s=SPEED_DEG_S):
    """Forward refresh gap and 3D vertical reach against sweep amplitude.

    The forward sector is only inside the height band near a level crossing,
    which happens twice per ping-pong cycle, so the gap tracks 2*amplitude/
    speed.  Amplitude is therefore the lever for obstacle freshness -- and it
    is simultaneously the lever the 3D map needs pushed the other way, which
    is the whole tension this table exists to price.
    """
    d = np.hypot(points[:, 0], points[:, 1])
    in_range = (d >= 0.15) & (d <= RANGE_MAX)
    z = points[:, 2]
    fwd_bins = sector(np.arange(N_BINS), 30)
    rows = []
    for ampl in (5, 10, 15, 20, 30, 45):
        pick = np.abs(theta_deg) <= ampl
        period = 4 * ampl / speed_deg_s
        keep = pick & in_range & (z >= z_min) & (z <= z_max)
        idx, t = bearings(points)[keep], theta_deg[keep]
        n_cell = max(1, int(2 * ampl))
        cell = np.clip(((t + ampl) / (2 * ampl) * n_cell).astype(int), 0, n_cell - 1)
        lit = np.zeros((N_BINS, n_cell), dtype=bool)
        lit[idx, cell] = True
        gaps = np.full(N_BINS, np.inf)
        for b in range(N_BINS):
            cells = np.flatnonzero(lit[b])
            if cells.size == 0:
                continue
            t_up = (cells + 0.5) / n_cell * (2 * ampl) / speed_deg_s
            times = np.sort(np.concatenate([t_up, period - t_up]))
            gaps[b] = np.diff(np.concatenate([times, [times[0] + period]])).max()
        seen = np.isfinite(gaps)
        zz = z[pick & in_range]
        rows.append({
            'amplitude_deg': ampl, 'period_s': round(period, 3),
            'forward_gap_p50_s': round(float(np.median(gaps[fwd_bins & seen])), 3),
            'forward_gap_max_s': round(float(gaps[fwd_bins & seen].max()), 3),
            'all_gap_p95_s': round(float(np.quantile(gaps[seen], 0.95)), 3),
            # Safety-relevant returns: below the laser plane, above floor noise.
            'low_obstacle_pct': round(100 * float(((zz > 0.02) & (zz < 0.12)).mean()), 1),
            # Overhang / ceiling: the reason the sweep is +-45 in the first place.
            'ceiling_pct': round(100 * float((zz > 1.5).mean()), 1),
            'z_p95_m': round(float(np.quantile(zz, 0.95)), 2),
        })
    return rows


def main():
    points = np.load(SRC)['points']
    theta = np.degrees(recover_tilt(points))
    inside = np.abs(theta) <= 45.0
    points, theta = points[inside], theta[inside]

    bands = {'band (0.12-0.40 m)': (DEFAULT_Z_MIN, DEFAULT_Z_MAX),
             'thin slice (0.263-0.363 m)': (0.263, 0.363)}
    result = {'source': os.path.relpath(SRC, HERE), 'n_points': int(len(points)),
              'period_s': round(PERIOD_S, 3), 'range_max_m': RANGE_MAX,
              'bands': {}}

    for name, (lo, hi) in bands.items():
        gaps = refresh_gaps(points, theta, lo, hi)
        seen = np.isfinite(gaps)
        idx = np.arange(N_BINS)
        fwd = sector(idx, 30)
        entry = {
            'z_min': lo, 'z_max': hi,
            'bearings_ever_seen_pct': round(100 * seen.mean(), 1),
            'coverage_vs_window': coverage_vs_window(points, theta, lo, hi),
            'refresh_gap_s': {
                'all_p50': round(float(np.median(gaps[seen])), 3),
                'all_p95': round(float(np.quantile(gaps[seen], 0.95)), 3),
                'all_max': round(float(gaps[seen].max()), 3),
                'forward_60deg_p50': round(float(np.median(gaps[fwd & seen])), 3),
                'forward_60deg_max': round(float(gaps[fwd & seen].max()), 3),
                'never_seen_bearings': int((~seen).sum()),
            },
        }
        result['bands'][name] = entry

    result['amplitude_study'] = amplitude_study(
        points, theta, DEFAULT_Z_MIN, DEFAULT_Z_MAX)

    with open(os.path.join(HERE, 'replay.json'), 'w') as f:
        json.dump(result, f, indent=1)

    for name, e in result['bands'].items():
        print('\n== %s ==' % name)
        print('  bearings ever seen: %.1f%% (%d never)'
              % (e['bearings_ever_seen_pct'],
                 e['refresh_gap_s']['never_seen_bearings']))
        print('  coverage by accumulation window (worst-case placement):')
        for c in e['coverage_vs_window']:
            print('    %5.1f deg (%.3f s): all %.0f%%  forward+-30 %.0f%%'
                  % (c['window_deg'], c['window_s'],
                     100 * c['worst_coverage_all'],
                     100 * c['worst_coverage_forward_60deg']))
        g = e['refresh_gap_s']
        print('  refresh gap s: all p50 %.2f p95 %.2f max %.2f | forward p50 %.2f max %.2f'
              % (g['all_p50'], g['all_p95'], g['all_max'],
                 g['forward_60deg_p50'], g['forward_60deg_max']))

    print('\n== sweep amplitude trade (band %.2f-%.2f m, %.0f deg/s) ==' %
          (DEFAULT_Z_MIN, DEFAULT_Z_MAX, SPEED_DEG_S))
    print('  %5s %8s %12s %12s %11s %9s' % ('ampl', 'period', 'fwd gap p50',
                                            'fwd gap max', 'low obst', 'ceiling'))
    for r in result['amplitude_study']:
        print('  %4d° %7.2fs %11.2fs %11.2fs %10.1f%% %8.1f%%'
              % (r['amplitude_deg'], r['period_s'], r['forward_gap_p50_s'],
                 r['forward_gap_max_s'], r['low_obstacle_pct'], r['ceiling_pct']))


if __name__ == '__main__':
    main()
