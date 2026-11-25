#!/usr/bin/env python3
"""Rerun the lidar person tracker on frames recorded by server_x3 and score it.

    python3 src/tools/perception/lidar_person_replay.py RUN/lidar_persons.ndjson [--hold 3.0] [--base]

By default the tracker gets what x3_server gives it live (X3_SEEDED_TRACKER=1,
X3_SIDE_PERSONS=1): the camera-seeded candidates, the side-camera rows and anchors, and
the robot position from the run's bag /odom for snapping camera people along their ray.
--base reruns it on the ordinary candidates alone (the summary before 2026-11-24, which
understated crowd tracking: crowd-r2 28 births base vs 8 live-like).

Input: the NDJSON server_x3 writes while scripts/record_person_lidar.sh runs (one line per
/livox/person_candidates frame: candidates, the C3 tracks at that moment, the live
tracker's results).  The tracker is rerun from scratch, so constants can be changed with
the flags or by editing lidar_person_tracker.py, and compared with the live run.

Scores (what the 2026-10-22 professor feedback was about):
  * camera coverage   frames with >= 1 fresh C3 person
  * combined coverage frames with a fresh C3 person OR a confirmed lidar track
  * lidar-only frames combined minus camera: what the Livox added
  * ids               distinct C3 ids vs distinct lidar ids (fewer = less churn)
  * held / ended      lidar-held seconds, ends at the hold cap vs lidar loss
Coverage is per frame over the whole run, so a long empty stretch lowers both; compare
the arms on the same run, not runs against each other.
"""
import argparse
import json
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import lidar_person_tracker as lpt  # noqa: E402

CAMERA_ROLES = ('front', 'rear', 'left', 'right')   # == server_x3.CAMERA_ROLES
SIDE_ROWS_MAX_AGE_S = 0.3                         # == server_x3.SIDE_ROWS_MAX_AGE_S


def bag_origin(ndjson):
    """t -> robot (x, y) in odom from the run's bag /odom, or None if there is no bag."""
    bag = pathlib.Path(ndjson).parent / 'bag' / 'bag_0.db3'
    if not bag.exists():
        return None
    import sqlite3
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import lidar_person_dataset as lpd
    db = sqlite3.connect(str(bag))
    tid = {n: i for i, n in db.execute('select id, name from topics')}
    if '/odom' not in tid:
        return None
    poses = lpd.Poses([lpd.parse_odom(d) for (d,) in db.execute(
        'select data from messages where topic_id=?', (tid['/odom'],))])

    def at(t):
        p = poses.at(t)
        return None if p is None else (float(p[0]), float(p[1]))
    return at


def _fresh_camera(camera, now):
    for c in camera or []:
        st, age = c.get('state_stamp_s'), c.get('obs_age_s')
        if c.get('measured') is False or st is None or age is None:
            continue
        if now - (st - age) <= lpt.CAMERA_MAX_AGE_S:
            yield c


def replay(lines, hold_s=lpt.HOLD_S, live=True, origin_at=None):
    tr = lpt.LidarPersonTracker(hold_s=hold_s)
    seeded = None
    n = cam_frames = any_frames = 0
    cam_ids, lidar_ids, near_lidar_only = set(), set(), 0
    for rec in lines:
        names = rec.get('fields') or ['x', 'y', 'x_lo', 'x_hi', 'y_lo', 'y_hi', 'z_lo', 'z_hi', 'n']
        cands = [dict(zip(names, row)) for row in rec['candidates']]
        age = rec.get('packet_age')
        now, side, origin = rec['stamp'], None, None
        if live:
            if rec.get('seeded'):     # recorded only within 0.05 s; held like merge_seeded allows
                seeded = (rec['seeded'], now)
            if seeded:
                cands = lpt.merge_seeded(cands, seeded[0], names, seeded[1], now)
            sf, ss = rec.get('side_fields'), rec.get('side_stamp')
            if rec.get('side') and sf and ss is not None and abs(ss - now) < SIDE_ROWS_MAX_AGE_S:
                cands = lpt.merge_seeded(cands, rec['side'], sf, ss, now)
                f = {k: i for i, k in enumerate(sf)}
                side = [{'id': f"{CAMERA_ROLES[int(r[f['cam']])]}:{int(r[f['tid']])}",
                         'x': float(r[f['ax']]), 'y': float(r[f['ay']]),
                         'last_seen': ss - float(r[f['age']])} for r in rec['side']]
            if origin_at is not None:
                origin = origin_at(now)
        res = tr.update(now, cands, rec['camera'], math.inf if age is None else age,
                        origin=origin, side=side)
        n += 1
        fresh = list(_fresh_camera(rec['camera'], rec['stamp']))
        cam_ids.update(str(c.get('id')) for c in fresh)
        confirmed = [r for r in res if r['confirmed']]
        lidar_ids.update(r['id'] for r in confirmed)
        cam_frames += bool(fresh)
        any_frames += bool(fresh or confirmed)
        if not fresh and confirmed:
            near_lidar_only += 1
    return {'frames': n, 'camera_frames': cam_frames, 'combined_frames': any_frames,
            'lidar_only_frames': near_lidar_only, 'camera_ids': len(cam_ids),
            'lidar_ids': len(lidar_ids), 'stats': tr.stats}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ndjson')
    ap.add_argument('--hold', type=float, default=lpt.HOLD_S,
                    help='short lidar-only hold (young tracks), s')
    ap.add_argument('--base', action='store_true',
                    help='ordinary candidates only: no seeded/side rows, no ray snapping')
    a = ap.parse_args()
    with open(a.ndjson) as f:
        lines = [json.loads(l) for l in f if l.strip()]
    if not lines:
        sys.exit('no frames in ' + a.ndjson)
    origin_at = None if a.base else bag_origin(a.ndjson)
    r = replay(lines, a.hold, live=not a.base, origin_at=origin_at)
    dur = lines[-1]['stamp'] - lines[0]['stamp']
    pct = lambda k: 100.0 * r[k] / max(r['frames'], 1)
    s = r['stats']
    mode = 'base (ordinary candidates)' if a.base else (
        'live-like (seeded + side' + (', ray snap from /odom)' if origin_at else ', no bag: no ray snap)'))
    print(f"{r['frames']} frames over {dur:.0f} s ({r['frames'] / max(dur, 1e-9):.1f} Hz), {mode}; "
          f"hold {a.hold:.0f} s young / {max(a.hold, lpt.LONG_HOLD_S):.0f} s established")
    print(f"  camera coverage    {pct('camera_frames'):5.1f}%")
    print(f"  combined coverage  {pct('combined_frames'):5.1f}%   (lidar-only {pct('lidar_only_frames'):.1f}%)")
    print(f"  ids                camera {r['camera_ids']}, lidar {r['lidar_ids']}")
    print(f"  births {s['births']}, camera ids re-linked {s['relinked']}, lidar-held {s['lidar_held_s']:.1f} s")
    print(f"  ended: hold cap {s['hold_expired']}, lidar lost {s['lost_lidar']}; resets {s['resets']}")


if __name__ == '__main__':
    main()
