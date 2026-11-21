#!/usr/bin/env python3
"""Offline A/B of how C3 reads a person's depth from its box, against the Livox range.

    python3 src/tools/perception/c3_depth_roi_eval.py RUN [RUN ...] [--out report.json]

RUN is a person-lidar recording directory holding c3/c3.ndjson, c3/c3_depth.bin and
lidar_persons.ndjson (scripts/record_person_lidar.sh). Every post-NMS person box of every
packet is re-measured on that packet's recorded depth frame under four variants:

  old           window on the unclipped box (the pre-fix behaviour, clip_box=False)
  clip          window on the box clipped to the image
  hist60        clip + nearest histogram mode over the top 60 % of the box
  hist100       clip + nearest histogram mode over the whole box (shows the desk/floor trap)

Reference: the Livox person candidate on the camera's bearing (+-7 deg), person-sized
(span <= 1.0 m, n >= 20 points), nearest to the camera, from the lidar frame closest in
time. The lidar and C3 clocks differ, so the offset is fitted from tracks both logs hold.
It is a range reference, not ground truth: no candidate on the bearing means no row, and
a stack of candidates on one bearing (person + furniture) gives the nearest person-sized one.

Boxes that were measured over a segmentation silhouette (mask) are skipped: the mask is
not logged. The depth is stored every 2nd pixel and is repeated back to the full grid, so the
'old' column is checked against the logged live z (sanity gate).
"""
import argparse
import bisect
import json
import math
import pathlib
import sys

import numpy as np

SRC = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC))
import c3_live  # noqa: E402
import c3_person_tracker as pt  # noqa: E402

MOUNT_X = 0.3069            # oakd_driver.OAK_MOUNT_X
BEARING_GATE_RAD = 0.12
REF_RANGE_M = (0.4, 6.0)
VARIANTS = {
    'old': dict(clip_box=False, depth_mode='window', rows=None),
    'clip': dict(clip_box=True, depth_mode='window', rows=None),
    'hist60': dict(clip_box=True, depth_mode='nearest_mode', rows=0.6),
    'hist100': dict(clip_box=True, depth_mode='nearest_mode', rows=1.0),
}


def load_run(run):
    run = pathlib.Path(run)
    pkts = []
    for line in (run / 'c3' / 'c3.ndjson').open():
        r = json.loads(line)
        if r.get('k') == 'pkt':
            pkts.append(r)
    lid = [json.loads(line) for line in (run / 'lidar_persons.ndjson').open()]
    return pkts, lid


def clock_offset(pkts, lid):
    """epoch - monotonic, from measured C3 tracks present in both logs (same id and place)."""
    by_id = {}
    for p in pkts:
        for t in p.get('tracks', []):
            if t.get('measured'):
                by_id.setdefault(t['id'], []).append((p['recv'], t['x'], t['y']))
    offs = []
    for fr in lid:
        for c in fr.get('camera') or []:
            if c.get('measured') is False or c.get('obs_age_s') is None:
                continue
            for recv, x, y in by_id.get(c['id'], ()):
                if abs(x - c['odom_x']) < 0.02 and abs(y - c['odom_y']) < 0.02:
                    offs.append(c['state_stamp_s'] - c['obs_age_s'] - recv)
                    break
    if len(offs) < 5:
        return None
    return float(np.median(offs))


def reference_range(pose, intr, box_u, cands, fields):
    """Range along the optical axis of the lidar candidate on the box's bearing, or None."""
    fx, cx = intr[0], intr[2]
    ang_cam = math.atan2(-(box_u - cx), fx)             # left positive
    c, s = math.cos(pose['theta']), math.sin(pose['theta'])
    ix, iy, ilo, ihi, iylo, iyhi, izlo, izhi, inn = (fields.index(k) for k in
        ('x', 'y', 'x_lo', 'x_hi', 'y_lo', 'y_hi', 'z_lo', 'z_hi', 'n'))
    best = None
    for cd in cands:
        if cd[inn] < 20 or max(cd[ihi] - cd[ilo], cd[iyhi] - cd[iylo]) > 1.0:
            continue
        dx, dy = cd[ix] - pose['x'], cd[iy] - pose['y']
        fwd, left = dx * c + dy * s, -dx * s + dy * c
        fwd -= MOUNT_X
        if not REF_RANGE_M[0] <= fwd <= REF_RANGE_M[1]:
            continue
        if abs(math.atan2(left, fwd + MOUNT_X) - ang_cam) > BEARING_GATE_RAD:
            continue
        if best is None or fwd < best:
            best = fwd
    return best


def stats(errs):
    if not errs:
        return dict(n=0)
    e = np.array(errs)
    return dict(n=len(e), median=round(float(np.median(e)), 3),
                p25=round(float(np.percentile(e, 25)), 3), p75=round(float(np.percentile(e, 75)), 3),
                abs_median=round(float(np.median(np.abs(e))), 3),
                within_15cm=round(float(np.mean(np.abs(e) <= 0.15)), 3))


def evaluate(run):
    pkts, lid = load_run(run)
    depth = c3_live.read_depth_log(pathlib.Path(run) / 'c3' / 'c3_depth.bin')
    dts = np.array([d[0] for d in depth])
    off = clock_offset(pkts, lid)
    lstamps = [fr['stamp'] for fr in lid]
    res = {v: dict(err=[], boxes=0, meas=0, z=[], single=[]) for v in VARIANTS}
    live_diff, skipped_mask, n_boxes = [], 0, 0
    for p in pkts:
        if p.get('depth_t') is None:
            continue
        j = int(np.argmin(np.abs(dts - p['depth_t']))) if len(dts) else -1
        if j < 0 or abs(dts[j] - p['depth_t']) > 0.01:
            continue
        _, step, small = depth[j]
        full = np.repeat(np.repeat(small, step, 0), step, 1)
        shape = tuple(p['depth_shape'])
        if full.shape != shape:
            full = full[:shape[0], :shape[1]]
            if full.shape != shape:
                continue
        logged = {tuple(m['bbox']): m for m in p.get('meas', [])}
        lidar_fr = None
        if off is not None:
            k = bisect.bisect_left(lstamps, p['recv'] + off)
            k = min(range(max(0, k - 1), min(len(lid), k + 1)), key=lambda q: abs(lstamps[q] - p['recv'] - off))
            if abs(lstamps[k] - p['recv'] - off) < 0.3:
                lidar_fr = lid[k]
        for d in p.get('dets', []):
            if not d.get('bbox') or d.get('mask_px'):
                skipped_mask += bool(d.get('mask_px'))
                continue
            box = tuple(d['bbox'])
            n_boxes += 1
            lg = logged.get(box)
            if lg is not None and lg.get('mask'):
                skipped_mask += 1
                n_boxes -= 1
                continue
            ref = (reference_range(p['pose'], p['intr'], 0.5 * (box[0] + box[2]), lidar_fr['candidates'],
                                   lidar_fr['fields']) if lidar_fr else None)
            for name, v in VARIANTS.items():
                if v['rows'] is not None:
                    pt.HIST_ROW_FRACTION = v['rows']
                m = pt.person_measurement(full, box, clip_box=v['clip_box'], depth_mode=v['depth_mode'])
                r = res[name]
                r['boxes'] += 1
                if m is None:
                    r['single'].append(None) if len(p['dets']) == 1 else None
                    continue
                r['meas'] += 1
                if len(p['dets']) == 1:
                    r['single'].append(m['z'])
                if ref is not None:
                    u_term = m['z'] - ref
                    r['err'].append(u_term)
                if name == 'old' and lg is not None:
                    live_diff.append(m['z'] - lg['z'])
    pt.HIST_ROW_FRACTION = 0.6
    out = {'run': str(run), 'clock_offset': off, 'boxes': n_boxes, 'skipped_mask': skipped_mask,
           'old_vs_logged_live_z_abs_median_m': (round(float(np.median(np.abs(live_diff))), 4) if live_diff else None),
           'old_vs_logged_live_z_p95_m': (round(float(np.percentile(np.abs(live_diff), 95)), 4) if live_diff else None)}
    for name, r in res.items():
        z = [x for x in r['single']]
        jit = [abs(b - a) for a, b in zip(z, z[1:]) if a is not None and b is not None]
        out[name] = dict(yield_=round(r['meas'] / r['boxes'], 3) if r['boxes'] else None,
                         vs_lidar=stats(r['err']),
                         frame_jitter_m=round(float(np.median(jit)), 4) if jit else None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out')
    args = ap.parse_args()
    allres = []
    print(f"{'run':44s} {'boxes':>5s} {'liveΔ':>6s} | " + ' | '.join(f'{v:^26s}' for v in VARIANTS))
    print(f"{'':44s} {'':>5s} {'(mm)':>6s} | " + ' | '.join(f"{'yield  med   |e|  <15cm n':^26s}" for _ in VARIANTS))
    for run in args.runs:
        try:
            r = evaluate(run)
        except (OSError, ValueError, KeyError) as e:
            print(f'{run}: skipped ({e})')
            continue
        allres.append(r)
        cells = []
        for v in VARIANTS:
            s = r[v]['vs_lidar']
            cells.append(f"{r[v]['yield_']!s:>5.5s} " + (f"{s['median']:+5.2f} {s['abs_median']:5.2f} {s['within_15cm']:4.2f} {s['n']:4d}" if s['n'] else '   -     -     -    0'))
        ld = r['old_vs_logged_live_z_abs_median_m']
        print(f"{pathlib.Path(run).parent.name[-5:] + '/' + pathlib.Path(run).name:44s} {r['boxes']:5d} {'' if ld is None else round(ld*1000):>6} | " + ' | '.join(cells))
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(allres, indent=1))


if __name__ == '__main__':
    main()
