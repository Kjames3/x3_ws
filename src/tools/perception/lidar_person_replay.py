#!/usr/bin/env python3
"""Rerun the lidar person tracker on frames recorded by server_x3 and score it.

    python3 src/tools/perception/lidar_person_replay.py RUN/lidar_persons.ndjson [--hold 3.0]

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


def _fresh_camera(camera, now):
    for c in camera or []:
        st, age = c.get('state_stamp_s'), c.get('obs_age_s')
        if c.get('measured') is False or st is None or age is None:
            continue
        if now - (st - age) <= lpt.CAMERA_MAX_AGE_S:
            yield c


def replay(lines, hold_s=lpt.HOLD_S):
    tr = lpt.LidarPersonTracker(hold_s=hold_s)
    n = cam_frames = any_frames = 0
    cam_ids, lidar_ids, near_lidar_only = set(), set(), 0
    for rec in lines:
        names = rec.get('fields') or ['x', 'y', 'x_lo', 'x_hi', 'y_lo', 'y_hi', 'z_lo', 'z_hi', 'n']
        cands = [dict(zip(names, row)) for row in rec['candidates']]
        age = rec.get('packet_age')
        res = tr.update(rec['stamp'], cands, rec['camera'], math.inf if age is None else age)
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
    ap.add_argument('--hold', type=float, default=lpt.HOLD_S, help='lidar-only hold cap, s')
    a = ap.parse_args()
    with open(a.ndjson) as f:
        lines = [json.loads(l) for l in f if l.strip()]
    if not lines:
        sys.exit('no frames in ' + a.ndjson)
    r = replay(lines, a.hold)
    dur = lines[-1]['stamp'] - lines[0]['stamp']
    pct = lambda k: 100.0 * r[k] / max(r['frames'], 1)
    s = r['stats']
    print(f"{r['frames']} frames over {dur:.0f} s ({r['frames'] / max(dur, 1e-9):.1f} Hz), hold cap {a.hold:.1f} s")
    print(f"  camera coverage    {pct('camera_frames'):5.1f}%")
    print(f"  combined coverage  {pct('combined_frames'):5.1f}%   (lidar-only {pct('lidar_only_frames'):.1f}%)")
    print(f"  ids                camera {r['camera_ids']}, lidar {r['lidar_ids']}")
    print(f"  births {s['births']}, camera ids re-linked {s['relinked']}, lidar-held {s['lidar_held_s']:.1f} s")
    print(f"  ended: hold cap {s['hold_expired']}, lidar lost {s['lost_lidar']}; resets {s['resets']}")


if __name__ == '__main__':
    main()
