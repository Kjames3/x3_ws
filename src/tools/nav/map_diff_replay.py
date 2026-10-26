"""
map_diff_replay.py
Phase 0 of notes/map_prior_dynamic_detection_plan.md: replay /scan from a bag
against a saved map and report how many returns fall in mapped free space.

No ROS needed at runtime beyond the message definitions (sourced Humble).

Pose: the first usable scan is site-identified and globally matched
(map_prior.SiteIdentifier); every later scan is refined locally around the
previous pose plus the /odom delta.  This is scan-to-map tracking, which is fine
for the mostly-stationary C3 captures but is not AMCL.  The script says so when
the per-scan score drops.

    source /opt/ros/humble/setup.bash
    python3 src/tools/nav/map_diff_replay.py artifacts/c3-captures/empty-r2 \
        artifacts/c3-captures/cross-2.4m-r2 [--map M.yaml] [--d-tol 0.05 0.10 0.15]
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)


import argparse
import glob
import json
import math
import os
import sqlite3
import sys

import numpy as np

sys.path.insert(0, _SRC_DIR)
from auto_initial_pose import FINE_BEAMS, ScanMatcher, subsample  # noqa: E402
from map_prior import (  # noqa: E402
    DEFAULT_SITES, LEARNED, NEW, STATIC, UNKNOWN, ChangeDetector,
    LikelihoodField, MapPrior,
    SiteIdentifier, base_to_map, load_sites, scan_to_points,
)

# laser_link in base_link at zero tilt: lidar_mount_joint x 0.0557 +
# lidar_tilt_joint x -0.0125, yaw 180 + 4.85 deg (URDF laser_joint).
LASER_X = 0.0432
LASER_YAW = 3.226241
MAX_RANGE = 6.0     # production range (continuous sweep note)


def _bag_db(path: str) -> str:
    if path.endswith(".db3"):
        return path
    hits = sorted(glob.glob(os.path.join(path, "**", "*.db3"), recursive=True))
    if not hits:
        raise FileNotFoundError(f"no .db3 under {path}")
    return hits[0]


def read_bag(path: str):
    """Yield ('scan'|'odom', t_ns, msg) in timestamp order."""
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    con = sqlite3.connect(_bag_db(path))
    topics = {name: tid for tid, name in con.execute("select id, name from topics")}
    want = {topics.get("/scan"): ("scan", LaserScan),
            topics.get("/odom"): ("odom", Odometry)}
    want.pop(None, None)
    q = (f"select topic_id, timestamp, data from messages where topic_id in "
         f"({','.join(str(k) for k in want)}) order by timestamp")
    for tid, t, data in con.execute(q):
        kind, cls = want[tid]
        yield kind, t, deserialize_message(data, cls)


def _odom_xyyaw(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))


def _compose_delta(prev_odom, odom, pose):
    """Apply the odom motion since prev_odom to a map pose."""
    if prev_odom is None:
        return pose
    dx, dy = odom[0] - prev_odom[0], odom[1] - prev_odom[1]
    c, s = math.cos(-prev_odom[2]), math.sin(-prev_odom[2])
    lx, ly = c * dx - s * dy, s * dx + c * dy
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * lx - s * ly, pose[1] + s * lx + c * ly,
            pose[2] + odom[2] - prev_odom[2])


def replay(bag: str, ident: SiteIdentifier, map_override: str | None,
           d_tols, laser_yaw: float) -> dict:
    prior = matcher = None
    pose = last_odom = odom_at_pose = None
    rows = []
    site_info = {}
    for kind, t, msg in read_bag(bag):
        if kind == "odom":
            last_odom = _odom_xyyaw(msg)
            continue
        pts = scan_to_points(np.asarray(msg.ranges), msg.angle_min,
                             msg.angle_increment, msg.range_min, MAX_RANGE,
                             LASER_X, 0.0, laser_yaw)
        if len(pts) < 50:
            continue
        if pose is None:
            if map_override:
                lf = LikelihoodField.from_yaml(map_override)
                matcher = ScanMatcher(lf)
                r = matcher.locate(pts)
                site_info = {"site": os.path.basename(map_override),
                             "pose_reason": r.reason, "score": r.score}
            else:
                sr = ident.identify(pts)
                print(f"  site: {sr}")
                if not sr.accepted:
                    continue
                matcher = ident.matchers[sr.site]
                r = sr.pose
                site_info = {"site": sr.site, "margin": sr.margin,
                             "per_site": sr.per_site, "pose_reason": r.reason}
            if not r.accepted:
                print(f"  pose ambiguous, skipping scan: {r}")
                continue
            prior = MapPrior(matcher.lf)
            dets = {d: ChangeDetector(prior) for d in d_tols}
            pose = (r.x, r.y, r.yaw)
            print(f"  initial pose {r}")
        else:
            pred = (_compose_delta(odom_at_pose, last_odom, pose)
                    if last_odom else pose)
            x, y, yaw, _ = matcher.refine(subsample(pts, FINE_BEAMS), *pred)
            pose = (x, y, yaw)
        odom_at_pose = last_odom
        score = matcher.lf.score_pose(subsample(pts, FINE_BEAMS), *pose)
        mp = base_to_map(pts, *pose)
        row = {"t": t * 1e-9, "n": len(pts), "score": score,
               "pose": pose}
        for d in d_tols:
            lab = dets[d].update(mp, d)
            row[d] = ((lab == STATIC).mean(), (lab == NEW).mean(),
                      (lab == UNKNOWN).mean(), (lab == LEARNED).mean(),
                      int((lab == NEW).sum()))
        rows.append(row)
    return {"site": site_info, "rows": rows}


def summarise(name: str, res: dict, d_tols) -> dict:
    rows = res["rows"]
    if not rows:
        print(f"{name}: no scans labelled")
        return {}
    sc = np.array([r["score"] for r in rows])
    xy = np.array([r["pose"][:2] for r in rows])
    travel = float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0
    out = {"scans": len(rows), "site": res["site"],
           "match_score_median": float(np.median(sc)),
           "match_score_p5": float(np.percentile(sc, 5)),
           "path_m": travel, "by_d_tol": {}}
    print(f"{name}: {len(rows)} scans, site={res['site'].get('site')}, "
          f"match score med {np.median(sc):.3f} p5 {np.percentile(sc, 5):.3f}, "
          f"pose path {travel:.2f} m")
    for d in d_tols:
        # skip the first 10 s while the learned layer fills in
        t0 = rows[0]["t"]
        a = np.array([r[d] for r in rows if r["t"] - t0 > 10.0])
        med, p95 = np.median(a, axis=0), np.percentile(a, 95, axis=0)
        print(f"   d_tol {d:.2f}: static {med[0]*100:5.1f}%  "
              f"learned {med[3]*100:5.1f}%  NEW med {med[1]*100:5.2f}% "
              f"p95 {p95[1]*100:5.2f}% (pts med {med[4]:.0f} p95 {p95[4]:.0f})  "
              f"unknown {med[2]*100:4.1f}%")
        out["by_d_tol"][f"{d:.2f}"] = {"static_med": med[0], "new_med": med[1],
                                       "new_p95": p95[1], "unknown_med": med[2],
                                       "learned_med": med[3],
                                       "new_pts_med": med[4], "new_pts_p95": p95[4]}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--sites", default=DEFAULT_SITES)
    ap.add_argument("--map", help="skip site recognition, use this map yaml")
    ap.add_argument("--d-tol", type=float, nargs="+", default=[0.05, 0.10, 0.15, 0.20])
    ap.add_argument("--laser-yaw-deg", type=float, default=math.degrees(LASER_YAW))
    ap.add_argument("--json", help="write the summary here")
    args = ap.parse_args()

    ident = None if args.map else SiteIdentifier(load_sites(args.sites))
    summary = {}
    for bag in args.bags:
        name = os.path.basename(os.path.normpath(bag))
        print(f"== {name}")
        res = replay(bag, ident, args.map, args.d_tol,
                     math.radians(args.laser_yaw_deg))
        summary[name] = summarise(name, res, args.d_tol)
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(summary, fh, indent=1, default=float)
    return 0


if __name__ == "__main__":
    sys.exit(main())
