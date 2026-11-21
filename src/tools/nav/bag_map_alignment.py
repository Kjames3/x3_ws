#!/usr/bin/env python3
"""Check whether a bag's map->odom actually puts /scan on a saved map's walls.

    python3 src/tools/nav/bag_map_alignment.py RUN_DIR [RUN_DIR ...] \
        [--maps RealLab1 RealLab2 RealLab3] [--samples 20]

For each run (a dir holding bag/*.db3), samples /scan messages across the bag,
chains map->odom (nearest stamp) -> odom->base_footprint (nearest stamp) ->
base_footprint->scan frame (/tf_static), and reports the fraction of returns
within D_TOL of a wall on each map (map_prior.MapPrior STATIC label). A parked
robot's AMCL never updates, so a constant map->odom is not evidence of
localization; this is.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_R = _Path(__file__).resolve()
for _p in (_R.parents[2], _R.parents[1] / 'calibration'):
    if str(_p) not in _sys.path:
        _sys.path.insert(0, str(_p))

import argparse, glob, math, sqlite3
import numpy as np
import bag_signals as bs
from map_prior import MapPrior, STATIC, NEW

MAPS_DIR = _R.parents[2] / 'yahboomcar_nav' / 'maps'


def T2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def tf_series(db, parent, child):
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    c = sqlite3.connect(db)
    tid = dict(c.execute('select name,id from topics')).get('/tf')
    st, val = [], []
    if tid is None:
        return np.zeros(0), []
    for (d,) in c.execute('select data from messages where topic_id=? order by timestamp', (tid,)):
        for t in deserialize_message(d, TFMessage).transforms:
            if t.header.frame_id == parent and t.child_frame_id == child:
                q, p = t.transform.rotation, t.transform.translation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                st.append(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
                val.append((p.x, p.y, yaw))
    return np.array(st), val


def nearest(st, val, t):
    return val[int(np.argmin(np.abs(st - t)))]


def run_poses(run):
    db = glob.glob(f'{run}/bag/*.db3')[0]
    return db, tf_series(db, 'map', 'odom'), tf_series(db, 'odom', 'base_footprint')


def scans(db, n):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    c = sqlite3.connect(db)
    tid = dict(c.execute('select name,id from topics'))['/scan']
    rows = c.execute('select data from messages where topic_id=? order by timestamp', (tid,)).fetchall()
    for i in np.linspace(0, len(rows) - 1, min(n, len(rows))).astype(int):
        yield deserialize_message(rows[i][0], LaserScan)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--maps', nargs='+', default=['RealLab1', 'RealLab2', 'RealLab3'])
    ap.add_argument('--samples', type=int, default=20)
    a = ap.parse_args()
    priors = {m: MapPrior.from_yaml(str(MAPS_DIR / f'{m}.yaml')) for m in a.maps}
    print('run'.ljust(44) + ''.join(f'{m:>14}' for m in a.maps) + '   (wall% / free% of returns)')
    for run in a.runs:
        db, (mst, mval), (ost, oval) = run_poses(run)
        if not len(mst):
            print(f'{run:44s} no map->odom')
            continue
        static = bs.static_tf_tree(db)
        pts = []
        for s in scans(db, a.samples):
            t = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
            Tb = bs.lookup(static, 'base_footprint', s.header.frame_id)
            r = np.asarray(s.ranges, float)
            ang = s.angle_min + s.angle_increment * np.arange(len(r))
            ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 12))
            p = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), np.zeros(ok.sum()), np.ones(ok.sum())])
            pb = (Tb @ p)[:2]
            T = T2(*nearest(mst, mval, t)) @ T2(*nearest(ost, oval, t))
            pts.append((T @ np.vstack([pb, np.ones(pb.shape[1])]))[:2].T)
        P = np.vstack(pts)
        cols = []
        for m, pr in priors.items():
            lab = pr.label(P)
            cols.append(f'{100 * (lab == STATIC).mean():6.1f}/{100 * (lab == NEW).mean():4.1f}')
        print(f'{run:44s}' + ''.join(f'{c:>14}' for c in cols))


if __name__ == '__main__':
    main()
