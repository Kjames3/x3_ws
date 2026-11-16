#!/usr/bin/env python3
"""Measure each sensor's time offset against the gyro by cross-correlating yaw rate.

Spin the robot in place, back and forth, and every sensor sees the same yaw rate
signal. Recover a yaw rate (or something proportional to it) from each topic,
cross-correlate it with the reference gyro, and the lag of the peak is that
topic's stamp offset.

    python3 src/tools/calibration/time_offset_xcorr.py BAG \
        [--ref /imu/data_raw] [--topics /livox/lidar /odom_raw /cam/image_raw ...] \
        [--out artifacts/time-sync-YYYY-MM-DD] [--plot]

What is extracted per message type
----------------------------------
  sensor_msgs/Imu                     angular_velocity.<axis>  (topic:axis, default z)
  nav_msgs/Odometry                   d(yaw)/dt from the pose  (--odom-twist: twist.angular.z)
  geometry_msgs/Twist[Stamped]        angular.z  (plain Twist has no header: bag receive time)
  sensor_msgs/LaserScan               bearing shift of the range profile between scans
  livox_ros_driver2/CustomMsg,
  sensor_msgs/PointCloud2             same, from an elevation band of the cloud (--elev-band)
  sensor_msgs/Image, CompressedImage  horizontal phase-correlation shift between frames

Anything that only needs to be *proportional* to yaw rate is enough: the
correlation is normalized, and the sign of the peak is reported, not assumed.

Reading the result
------------------
`offset_ms` > 0 means the topic's stamps are LATE: an event the gyro stamps at t
is stamped at t + offset in that topic. Correct with `stamp - offset`. The
reference's own offset is the zero point. If you want absolute latency, the
reference must be hardware-stamped (Livox IMU under PTP, or the ICM with its
driver stamping at the data-ready interrupt).

Cloud topics are timed at header stamp + mean per-point offset (the middle of
the sweep), so the reported offset already applies to the header stamp.

Recording protocol (what makes the peak sharp)
----------------------------------------------
  * 30-60 s of in-place spins that REVERSE direction every 1-3 s, with hard
    starts and stops. Constant-rate spinning has no features to correlate.
  * A feature-rich room (furniture, door frames), not an empty hall.
  * Nothing moving in view; keep yaw rate under ~1.5 rad/s so frames stay sharp
    and lidar scans overlap.
  * Record every topic you want to check, plus the reference gyro.

Fused topics (/odom from the EKF, FAST-LIO /Odometry) already contain the gyro,
so their offset is not independent; they are flagged in the report.
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_HERE = str(_Path(__file__).resolve().parent)
if _HERE not in _sys.path:
    _sys.path.insert(0, _HERE)

import argparse
import json
import math
import os

import numpy as np

import bag_signals as bs

IMU = 'sensor_msgs/msg/Imu'
ODOM = 'nav_msgs/msg/Odometry'
TWIST = 'geometry_msgs/msg/Twist'
TWIST_STAMPED = 'geometry_msgs/msg/TwistStamped'
SCAN = 'sensor_msgs/msg/LaserScan'
CUSTOM = 'livox_ros_driver2/msg/CustomMsg'
CLOUD = 'sensor_msgs/msg/PointCloud2'
IMAGE = 'sensor_msgs/msg/Image'
CIMAGE = 'sensor_msgs/msg/CompressedImage'

FUSED_HINTS = ('/odom', '/odometry/filtered', '/Odometry', '/fastlio', '/aft_mapped')


# ======================================================================= signal extraction

def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def bearing_profile(xyz, bins=2880, elev_band=(-0.05, 0.17), rmin=0.3, rmax=40.0):
    """Log-range vs bearing (median range per bin) for points inside an elevation band (rad).

    Median, not min: the Mid-360's scan pattern is fixed in the sensor frame, and
    a min over unevenly dense bins imprints that pattern on the profile, which
    pulls the shift estimate toward zero (yaw rate read ~25% low on a real bag;
    median reads ~12% low, cause of the remainder not established). Only the
    timing matters here: the correlation is scale-invariant.
    Empty bins are 0 after mean removal, so they don't add to the correlation.
    """
    r_xy = np.hypot(xyz[:, 0], xyz[:, 1])
    elev = np.arctan2(xyz[:, 2], r_xy)
    r = np.linalg.norm(xyz, axis=1)
    keep = (elev >= elev_band[0]) & (elev <= elev_band[1]) & (r > rmin) & (r < rmax)
    if keep.sum() < 50:
        return None
    b = np.arctan2(xyz[keep, 1], xyz[keep, 0])
    idx = ((b + np.pi) / (2 * np.pi) * bins).astype(int) % bins
    rk = r[keep]
    order = np.lexsort((rk, idx))
    idx, rk = idx[order], rk[order]
    u, first, count = np.unique(idx, return_index=True, return_counts=True)
    out = np.zeros(bins)
    out[u] = np.log(rk[first + count // 2])
    out[u] -= out[u].mean()
    return out


def scan_profile(ranges, angle_min, angle_inc, bins=2880, rmin=0.3, rmax=40.0):
    r = np.asarray(ranges, dtype=np.float64)
    ang = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    xyz = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), np.zeros(ok.sum())], 1)
    return bearing_profile(xyz, bins=bins, elev_band=(-0.1, 0.1), rmin=rmin, rmax=rmax)


def profile_shift(prev, cur, max_bins):
    """Circular shift k (sub-bin) such that cur ~= roll(prev, k). None if no clear peak."""
    if prev is None or cur is None:
        return None
    c = np.real(np.fft.ifft(np.fft.fft(cur) * np.conj(np.fft.fft(prev))))
    n = len(c)
    cand = np.r_[np.arange(0, max_bins + 1), np.arange(n - max_bins, n)]
    k = cand[np.argmax(c[cand])]
    y0, y1, y2 = c[(k - 1) % n], c[k], c[(k + 1) % n]
    if y1 <= 0:
        return None
    den = y0 - 2 * y1 + y2
    frac = 0.5 * (y0 - y2) / den if den < 0 else 0.0
    k = k if k <= n // 2 else k - n
    return k + frac


def image_shift(prev, cur):
    """Horizontal shift (px) of cur relative to prev by phase correlation."""
    import cv2
    win = cv2.createHanningWindow(prev.shape[::-1], cv2.CV_32F)
    (dx, _dy), resp = cv2.phaseCorrelate(prev, cur, win)
    return dx, resp


def _gray_small(img, width=320):
    import cv2
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    scale = width / img.shape[1]
    if scale < 1:
        img = cv2.resize(img, (width, int(round(img.shape[0] * scale))), interpolation=cv2.INTER_AREA)
    return img.astype(np.float32)


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def extract(path, topic, typ, axis='z', odom_twist=False, bins=2880, elev_band=(-0.05, 0.17),
            max_deg_per_frame=45.0):
    """-> dict(t=..., x=..., kind=..., latency=median(receive - header) or None)."""
    t, x, lat = [], [], []
    prev, prev_t = None, None
    max_bins = int(round(max_deg_per_frame / 360.0 * bins))
    ax = 'xyz'.index(axis)
    for _topic, _typ, stamp, recv, data in bs.iter_messages(path, [topic]):
        if stamp is not None:
            lat.append(recv - stamp)
        ts = stamp if stamp is not None else recv
        if typ == IMU:
            m = bs.deserialize(data, typ)
            w = m.angular_velocity
            t.append(ts)
            x.append((w.x, w.y, w.z)[ax])
        elif typ == ODOM:
            m = bs.deserialize(data, typ)
            t.append(ts)
            x.append(m.twist.twist.angular.z if odom_twist else _yaw(m.pose.pose.orientation))
        elif typ in (TWIST, TWIST_STAMPED):
            m = bs.deserialize(data, typ)
            t.append(ts)
            x.append((m.twist if typ == TWIST_STAMPED else m).angular.z)
        elif typ in (SCAN, CUSTOM, CLOUD):
            if typ == SCAN:
                m = bs.deserialize(data, typ)
                prof = scan_profile(m.ranges, m.angle_min, m.angle_increment, bins=bins)
                tc = ts + 0.5 * m.scan_time if m.scan_time > 0 else ts
            else:
                xyz, dt = bs.cloud_xyz(data, typ)
                prof = bearing_profile(xyz, bins=bins, elev_band=elev_band)
                tc = ts + (float(dt.mean()) if dt is not None and len(dt) else 0.0)
            k = profile_shift(prev, prof, max_bins)
            if k is not None and prev_t is not None and tc > prev_t:
                t.append(0.5 * (tc + prev_t))
                # the scene turns opposite to the robot
                x.append(-k * (2 * np.pi / bins) / (tc - prev_t))
            prev, prev_t = prof, tc
        elif typ in (IMAGE, CIMAGE):
            g = _gray_small(bs.decode_image(data, typ))
            if prev is not None and prev.shape == g.shape and ts > prev_t:
                dx, resp = image_shift(prev, g)
                if resp > 0.05:
                    t.append(0.5 * (ts + prev_t))
                    x.append(dx / (ts - prev_t))
            prev, prev_t = g, ts
        else:
            raise ValueError(f'{topic}: unsupported type {typ}')
    t, x = np.asarray(t, float), np.asarray(x, float)
    order = np.argsort(t, kind='stable')
    t, x = t[order], x[order]
    kind = typ.split('/')[-1]
    if typ == ODOM and not odom_twist and len(t) > 2:
        yaw = np.unwrap(x)
        dt = np.diff(t)
        good = dt > 1e-4
        t = 0.5 * (t[1:] + t[:-1])[good]
        x = (np.diff(yaw)[good] / dt[good])
        kind = 'Odometry(dyaw/dt)'
    return {'t': t, 'x': x, 'kind': kind,
            'latency': float(np.median(lat)) if lat else None,
            'stamped': bool(lat)}


# ======================================================================= correlation

def _box(x, n):
    """Centred moving average. n is forced odd: an even kernel in 'same' mode shifts by half a sample."""
    n = int(n) | 1
    if n <= 1:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode='same')


def _valid_mask(t_src, t_query, max_gap):
    """True where t_query falls inside a gap-free stretch of t_src."""
    i = np.clip(np.searchsorted(t_src, t_query), 1, len(t_src) - 1)
    gap = t_src[i] - t_src[i - 1]
    inside = (t_query >= t_src[0]) & (t_query <= t_src[-1])
    return inside & (gap <= max_gap)


def xcorr_offset(t_ref, x_ref, t_sig, x_sig, max_lag=0.5, rate=200.0, lag_step=0.001,
                 t_range=None):
    """Offset d (s) with sig(t + d) ~ ref(t), i.e. sig stamps are late by d.

    Both signals are resampled onto a uniform grid, box-filtered to the coarser
    sample interval (symmetric, so it adds no lag), and Pearson-correlated at
    every lag. Returns dict(offset, peak, sign, lags, corr, n).
    """
    dt_ref = float(np.median(np.diff(t_ref)))
    dt_sig = float(np.median(np.diff(t_sig)))
    coarse = max(dt_ref, dt_sig)
    lo = max(t_ref[0], t_sig[0] + max_lag)
    hi = min(t_ref[-1], t_sig[-1] - max_lag)
    if t_range is not None:
        lo, hi = max(lo, t_range[0]), min(hi, t_range[1])
    if hi - lo < 4 * max_lag + 2:
        return None
    grid = np.arange(lo, hi, 1.0 / rate)
    ref_ok = _valid_mask(t_ref, grid, 5 * dt_ref)
    ref = _box(np.interp(grid, t_ref, x_ref), max(1, int(round(coarse * rate))))

    # sig on a fine grid once, then shift by integer samples of the fine grid
    fine_rate = 1.0 / lag_step
    sg = np.arange(lo - max_lag - coarse, hi + max_lag + coarse, 1.0 / fine_rate)
    sig_fine = np.interp(sg, t_sig, x_sig)
    sig_ok_fine = _valid_mask(t_sig, sg, 5 * dt_sig)
    sig_fine = _box(sig_fine, max(1, int(round(coarse * fine_rate))))
    lags = np.arange(-max_lag, max_lag + lag_step / 2, lag_step)
    corr = np.full(len(lags), np.nan)
    base = np.round((grid - sg[0]) * fine_rate).astype(int)
    for j, L in enumerate(lags):
        idx = base + int(round(L * fine_rate))
        ok = ref_ok & sig_ok_fine[idx]
        if ok.sum() < rate * 2:
            continue
        a, b = ref[ok], sig_fine[idx[ok]]
        a = a - a.mean()
        b = b - b.mean()
        den = math.sqrt(float(a @ a) * float(b @ b))
        corr[j] = float(a @ b) / den if den > 0 else np.nan
    if not np.isfinite(corr).any():
        return None
    j = int(np.nanargmax(np.abs(corr)))
    sign = 1.0 if corr[j] > 0 else -1.0
    off = lags[j]
    if 0 < j < len(lags) - 1 and np.isfinite(corr[j - 1]) and np.isfinite(corr[j + 1]):
        y0, y1, y2 = sign * corr[j - 1], sign * corr[j], sign * corr[j + 1]
        den = y0 - 2 * y1 + y2
        if den < 0:
            off += 0.5 * (y0 - y2) / den * lag_step
    edge = j == 0 or j == len(lags) - 1
    return {'offset': float(off), 'peak': float(abs(corr[j])), 'sign': sign, 'lags': lags,
            'corr': corr, 'n': int(ref_ok.sum()), 'at_edge': bool(edge)}


def segment_offsets(t_ref, x_ref, t_sig, x_sig, n_seg=4, **kw):
    lo, hi = max(t_ref[0], t_sig[0]), min(t_ref[-1], t_sig[-1])
    edges = np.linspace(lo, hi, n_seg + 1)
    out = []
    for a, b in zip(edges[:-1], edges[1:]):
        r = xcorr_offset(t_ref, x_ref, t_sig, x_sig, t_range=(a - kw.get('max_lag', 0.5),
                                                               b + kw.get('max_lag', 0.5)), **kw)
        if r is not None and r['peak'] > 0.5 and not r['at_edge']:
            out.append(r['offset'])
    return out


# ======================================================================= main

DEFAULT_SKIP_SUBSTR = ('depth', 'person', 'people', 'obstacle', 'tof', 'candidates', 'cmd_vel')


def default_topics(types, ref):
    pick = []
    for name, typ in sorted(types.items()):
        if name == ref or any(s in name.lower() for s in DEFAULT_SKIP_SUBSTR):
            continue
        if typ in (IMU, ODOM, SCAN, CUSTOM, IMAGE, CIMAGE) or name == '/vel_raw':
            pick.append(name)
    return pick


def excitation(t, x, on=0.3):
    """How usable the reference is: rms rate and direction reversals (hysteresis at +/-on rad/s)."""
    xs = _box(x, max(1, int(0.1 / max(float(np.median(np.diff(t))), 1e-3))))
    state, rev = 0, 0
    for v in xs:
        s = 1 if v > on else -1 if v < -on else 0
        if s and state and s != state:
            rev += 1
        state = s or state
    return {'rms_rad_s': float(np.sqrt(np.mean(x ** 2))), 'reversals': rev,
            'duration_s': float(t[-1] - t[0])}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--ref', default='/imu/data_raw', help='reference gyro topic[:axis] (default z)')
    ap.add_argument('--topics', nargs='*', help='topics to check, each topic[:axis] for Imu (default: auto)')
    ap.add_argument('--max-lag', type=float, default=0.5, help='search window +/- s (default 0.5)')
    ap.add_argument('--odom-twist', action='store_true', help='use Odometry twist instead of d(yaw)/dt')
    ap.add_argument('--elev-band', type=float, nargs=2, default=(-3.0, 10.0), metavar=('LO', 'HI'),
                    help='elevation band (deg, sensor frame) used for cloud bearing profiles')
    ap.add_argument('--segments', type=int, default=4, help='sub-windows for the consistency check')
    ap.add_argument('--out', help='write report.json (and plots with --plot) here')
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args(argv)

    types = bs.topic_types(args.bag)
    ref_name, _, ref_axis = args.ref.partition(':')
    if ref_name not in types:
        imus = [n for n, t in types.items() if t == IMU]
        ap.error(f'reference {ref_name} not in bag; Imu topics present: {imus}')
    specs = args.topics if args.topics else default_topics(types, ref_name)
    if not specs:
        ap.error('no usable topics found; pass --topics')

    print(f'reference: {ref_name} ({types[ref_name]}) axis {ref_axis or "z"}')
    ref = extract(args.bag, ref_name, types[ref_name], axis=ref_axis or 'z')
    if len(ref['t']) < 100:
        raise SystemExit(f'reference has only {len(ref["t"])} samples')
    exc = excitation(ref['t'], ref['x'])
    print(f'  {len(ref["t"])} samples over {exc["duration_s"]:.1f} s, rms {exc["rms_rad_s"]:.2f} rad/s, '
          f'{exc["reversals"]} direction reversals')
    if exc['reversals'] < 6 or exc['rms_rad_s'] < 0.2:
        print('  WARNING: weak excitation; record back-and-forth spins (>=10 reversals, ~0.5-1.5 rad/s)')

    report = {'bag': os.path.abspath(args.bag), 'reference': ref_name, 'reference_axis': ref_axis or 'z',
              'excitation': exc, 'reference_receive_latency_ms':
              None if ref['latency'] is None else round(ref['latency'] * 1e3, 1), 'topics': {}}
    results = {}
    elev = tuple(math.radians(v) for v in args.elev_band)
    print(f'\n{"topic":34s} {"kind":20s} {"offset_ms":>9s} {"spread":>7s} {"peak":>5s} '
          f'{"sign":>4s} {"recv-hdr_ms":>11s}  note')
    for spec in specs:
        name, _, axis = spec.partition(':')
        if name not in types:
            print(f'{name:34s} not in bag')
            continue
        typ = types[name]
        try:
            sig = extract(args.bag, name, typ, axis=axis or 'z', odom_twist=args.odom_twist,
                          elev_band=elev)
        except Exception as e:  # noqa: BLE001 - report and carry on with the other topics
            print(f'{name:34s} failed: {e}')
            continue
        if len(sig['t']) < 20:
            print(f'{name:34s} {sig["kind"]:20s} too few samples ({len(sig["t"])})')
            continue
        r = xcorr_offset(ref['t'], ref['x'], sig['t'], sig['x'], max_lag=args.max_lag)
        if r is None:
            print(f'{name:34s} {sig["kind"]:20s} no overlap with the reference')
            continue
        segs = segment_offsets(ref['t'], ref['x'], sig['t'], sig['x'], n_seg=args.segments,
                               max_lag=args.max_lag)
        spread = float(np.std(segs)) if len(segs) >= 2 else float('nan')
        notes = []
        if not sig['stamped']:
            notes.append('no header: bag receive time')
        if any(name == h or name.startswith(h + '/') for h in FUSED_HINTS) and typ == ODOM:
            notes.append('fused with gyro: not independent')
        if r['at_edge']:
            notes.append('peak at window edge: raise --max-lag')
        if r['peak'] > 0.995:
            notes.append('near-perfect match: probably built from the reference gyro')
        if r['peak'] < 0.7:
            notes.append('weak peak')
        if np.isfinite(spread) and spread > 0.005:
            notes.append('segments disagree')
        lat = sig['latency']
        print(f'{name:34s} {sig["kind"]:20s} {r["offset"] * 1e3:9.1f} '
              f'{spread * 1e3 if np.isfinite(spread) else float("nan"):7.1f} {r["peak"]:5.2f} '
              f'{int(r["sign"]):+4d} {"" if lat is None else f"{lat * 1e3:.1f}":>11s}  {"; ".join(notes)}')
        report['topics'][name] = {
            'type': typ, 'kind': sig['kind'], 'axis': axis or None, 'samples': int(len(sig['t'])),
            'offset_ms': round(r['offset'] * 1e3, 2),
            'segment_offsets_ms': [round(s * 1e3, 2) for s in segs],
            'segment_std_ms': None if not np.isfinite(spread) else round(spread * 1e3, 2),
            'peak_corr': round(r['peak'], 3), 'sign': int(r['sign']),
            'receive_minus_header_ms': None if lat is None else round(lat * 1e3, 1),
            'notes': notes}
        results[name] = (sig, r)

    print('\noffset > 0: the topic is stamped LATE relative to the reference; correct with stamp - offset.')
    print('spread: std of the offset over sub-windows (<2 ms is good). recv-hdr: bag receive minus header stamp.')

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, 'report.json'), 'w') as fh:
            json.dump(report, fh, indent=2)
        print(f'wrote {os.path.join(args.out, "report.json")}')
        if args.plot and results:
            plot(results, ref, args.out)
    return report


def busiest_window(t, x, span=15.0):
    """Start time of the span-second window with the most reference energy."""
    if t[-1] - t[0] <= span:
        return t[0]
    starts = np.arange(t[0], t[-1] - span, 1.0)
    c = np.r_[0, np.cumsum(x ** 2)]
    e = [c[np.searchsorted(t, a + span)] - c[np.searchsorted(t, a)] for a in starts]
    return starts[int(np.argmax(e))]


def plot(results, ref, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    n = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(14, 2.6 * n), squeeze=False)
    t0 = busiest_window(ref['t'], ref['x'])
    span = (t0, t0 + 15.0)
    m = (ref['t'] >= span[0]) & (ref['t'] <= span[1])
    for row, (name, (sig, r)) in enumerate(results.items()):
        ax = axes[row, 0]
        ax.plot(r['lags'] * 1e3, r['corr'], lw=1)
        ax.axvline(r['offset'] * 1e3, color='r', ls='--')
        ax.set_title(f'{name}: offset {r["offset"] * 1e3:.1f} ms, peak {r["peak"]:.2f}', fontsize=9)
        ax.set_xlabel('lag (ms)')
        ax.grid(alpha=0.3)
        ax = axes[row, 1]
        ts = sig['t'] - r['offset']
        # least-squares gain onto the reference (rad/s), so both curves share units
        ok = (ts > ref['t'][0]) & (ts < ref['t'][-1])
        rr = np.interp(ts[ok], ref['t'], ref['x'])
        gain = float(rr @ sig['x'][ok]) / max(float(sig['x'][ok] @ sig['x'][ok]), 1e-12)
        m2 = (ts >= span[0]) & (ts <= span[1])
        ax.plot(ref['t'][m] - t0, ref['x'][m], lw=0.6, alpha=0.7, label='reference')
        ax.plot(ts[m2] - t0, gain * sig['x'][m2], lw=1.0, label=f'{name} x {gain:.3g}, shifted')
        ax.set_xlabel('s (busiest 15 s of the reference)')
        ax.set_ylabel('rad/s')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(alpha=0.3)
    fig.tight_layout()
    p = os.path.join(out, 'xcorr.png')
    fig.savefig(p, dpi=110)
    print(f'wrote {p}')


if __name__ == '__main__':
    main()
