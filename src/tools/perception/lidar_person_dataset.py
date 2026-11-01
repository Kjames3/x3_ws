#!/usr/bin/env python3
"""Auto-labelled person / not-person lidar clusters from the robot's own bags.

    python3 src/tools/perception/lidar_person_dataset.py RUN_DIR [RUN_DIR ...] -o out.npz

RUN_DIR is a scripts/record_person_lidar.sh run: bag/ (with /livox/lidar and /odom)
and lidar_persons.ndjson (the C3 camera tracks at every candidate frame).

Per Livox frame: every point is placed in odom with the robot pose at its own capture
time (as livox_scan_node does), candidates are extracted with person_candidates.extract
and described with cluster_features.features.  Candidates are linked across frames into
tracklets (mutual nearest within LINK_GATE_M, a few missed frames allowed), then labelled
per tracklet, which is how a label reaches places the camera cannot see:

  POSITIVE  the tracklet was matched to a measured camera person on >= MIN_CAM_HITS
            frames, and on >= MIN_CAM_FRAC of its frames inside the camera's view.
            EVERY frame of it is positive: 'cam' where the camera matched, 'carried'
            where it did not (beyond 4 m, behind the robot, closer than ~0.6 m).
  NEGATIVE  never matched, static (stays within STATIC_M of its mean), and inside
            the camera's view, with no camera person near it, on >= MIN_VIEW_FRAMES
            frames.  The camera looked and saw nobody there.
  WEAK NEGATIVE ('persistent')  never near a camera person, not part of a positive
            tracklet, and some candidate sits within PERSIST_M of it on >= PERSIST_FRAC
            of the run's frames: fixed structure, wherever it is (behind the robot,
            beyond 4 m).  Label 0 like the others but tagged, so a classifier can be
            trained and scored with or without it.  A person standing still for the
            whole run would land here, so it is the weaker label.
  UNKNOWN   everything else (moving unmatched clusters, objects only ever seen out
            of view).  Kept in the file with label -1, never used for training.

Camera positions recorded before da7d8ae came from the raw-odom pose 1 time in 5
(~1 m / 57 deg off).  Each camera id's positions are checked against the median of their
neighbours in time and outliers are dropped before matching, so a misplaced camera
person does not label a wall.
"""
import argparse
import bisect
import json
import math
import pathlib
import sqlite3
import struct
import sys

import numpy as np

_SRC = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_SRC / 'yahboomcar_bringup'))

from yahboomcar_bringup import cluster_features as cf  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

# base_footprint -> livox_frame (URDF livox_joint 0.0432, 0, 0.2285 on base_link at
# 0.0815; no rotation).  The same constant the 2026-10-30 evaluation used.
LIVOX_XYZ = (0.0432, 0.0, 0.31)
RANGE_MAX_M = 12.0            # wider than the live 8 m, to see what lies beyond
CAM_MATCH_M = 0.5
LINK_GATE_M = 0.45
LINK_MAX_MISS = 3
MIN_CAM_HITS = 3
MIN_CAM_FRAC = 0.25
STATIC_M = 0.25
MIN_VIEW_FRAMES = 5
VIEW_HALF_DEG = 40.0          # 90% of measured C3 people are within 15 deg; edges are soft
VIEW_RANGE_M = (0.8, 3.8)     # C3 people are measured from 0.6 to 4.6 m
CAM_CLEAR_M = 0.8             # a negative needs no camera person this close
OUTLIER_M = 0.4
PERSIST_M = 0.25
PERSIST_FRAC = 0.5
LABEL_NAMES = {1: 'person', 0: 'not_person', 2: 'sitting', -1: 'unknown'}
OVERRIDE_LABELS = {'person': 1, 'not_person': 0, 'sitting': 2, 'unknown': -1}


def apply_overrides(run, xy, label, source, overrides):
    """Hand corrections: every candidate of `run` within r_m of (x, y) in odom gets the
    entry's label.  The persistent rule cannot tell a person sitting still from a chair
    (furniture-r1: a seated person 0.69 m from the robot was labelled not-person), so
    the user's word wins.  'sitting' (2) is left out of training, as SiT's seated people are."""
    for o in overrides:
        if o['run'] != run:
            continue
        m = np.hypot(xy[:, 0] - o['x'], xy[:, 1] - o['y']) < o.get('r_m', 0.5)
        label[m] = OVERRIDE_LABELS[o['label']]
        source[m] = 'override_' + o['label']
    return label, source

POINT_DT = np.dtype({'names': ['t', 'x', 'y', 'z', 'r'],
                     'formats': ['<u4', '<f4', '<f4', '<f4', 'u1'],
                     'offsets': [0, 4, 8, 12, 16], 'itemsize': 20})


def decode_points(data):
    """CustomMsg CDR bytes -> structured array (offset_time ns, x, y, z, reflectivity)."""
    n = (len(data) + 1) // 20
    for n in range(n, max(n - 200, 0), -1):
        start = len(data) - (n * 20 - 1)
        if start >= 4 and struct.unpack_from('<I', data, start - 4)[0] == n:
            return np.frombuffer(bytes(data[start:]) + b'\0', dtype=POINT_DT, count=n)
    return None


def parse_odom(d):
    """nav_msgs/Odometry CDR -> (stamp, x, y, yaw)."""
    sec, nsec = struct.unpack_from('<iI', d, 4)
    al = lambda o, a: ((o - 4 + a - 1) // a) * a + 4  # noqa: E731
    o = 12
    n = struct.unpack_from('<I', d, o)[0]
    o = al(o + 4 + n, 4)
    n = struct.unpack_from('<I', d, o)[0]
    o = al(o + 4 + n, 8)
    x, y, _z, qx, qy, qz, qw = struct.unpack_from('<7d', d, o)
    return sec + nsec * 1e-9, x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class Poses:
    def __init__(self, rows):
        h = np.asarray(sorted(rows))
        self.t, self.x, self.y = h[:, 0], h[:, 1], h[:, 2]
        self.a = np.unwrap(h[:, 3])

    def at(self, t):
        """Interpolated (x, y, yaw); None outside the recorded span (+-0.1 s)."""
        t = np.asarray(t, dtype=np.float64)
        if t.min() < self.t[0] - 0.1 or t.max() > self.t[-1] + 0.1:
            return None
        return (np.interp(t, self.t, self.x), np.interp(t, self.t, self.y),
                np.interp(t, self.t, self.a))


def clean_camera(records):
    """{camera id: [(t_seen, x, y)]} -> set of (id, t_seen) that are outliers: further
    than OUTLIER_M from the median of their +-3 neighbours in time."""
    by_id = {}
    for rec in records:
        for c in rec['camera']:
            if c.get('measured') is False or c.get('state_stamp_s') is None:
                continue
            t = c['state_stamp_s'] - c.get('obs_age_s', 0.0)
            by_id.setdefault(str(c['id']), {})[round(t, 3)] = (c['odom_x'], c['odom_y'])
    bad = set()
    for cid, obs in by_id.items():
        ts = sorted(obs)
        xy = np.array([obs[t] for t in ts])
        for i, t in enumerate(ts):
            lo, hi = max(0, i - 3), min(len(ts), i + 4)
            nb = np.delete(xy[lo:hi], i - lo, axis=0)
            if len(nb) >= 2 and np.hypot(*(xy[i] - np.median(nb, axis=0))) > OUTLIER_M:
                bad.add((cid, t))
    return bad


def camera_people(rec, now, bad):
    """Measured, fresh, non-outlier camera people at `now` (odom XY)."""
    out = []
    if rec is None or rec.get('packet_age', 0.0) >= 0.5:
        return out
    for c in rec['camera']:
        if c.get('measured') is False or c.get('state_stamp_s') is None:
            continue
        seen = c['state_stamp_s'] - c.get('obs_age_s', 0.0)
        if (str(c['id']), round(seen, 3)) in bad or now - seen > 0.65:
            continue
        k = max(0.0, now - c['state_stamp_s'])
        out.append((c['odom_x'] + c['odom_vx'] * k, c['odom_y'] + c['odom_vy'] * k))
    return out


def mutual_nearest(a, b, gate):
    """Pairs (i, j) of 2D points that are each other's nearest within gate."""
    if not len(a) or not len(b):
        return []
    d = np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1])
    out = []
    for i in range(len(a)):
        j = int(np.argmin(d[i]))
        if d[i, j] < gate and int(np.argmin(d[:, j])) == i:
            out.append((i, j))
    return out


def link(frames):
    """frames: list of (t, (N, 2) XY).  Returns per-frame arrays of tracklet ids."""
    active = {}          # id -> [x, y, vx, vy, t_last, misses]
    nxt = 0
    out = []
    for t, xy in frames:
        ids = np.full(len(xy), -1, dtype=np.int64)
        keys = list(active)
        pred = np.array([[active[k][0] + active[k][2] * (t - active[k][4]),
                          active[k][1] + active[k][3] * (t - active[k][4])] for k in keys]).reshape(-1, 2)
        for i, j in mutual_nearest(pred, xy, LINK_GATE_M):
            k = keys[i]
            s = active[k]
            dt = max(t - s[4], 1e-3)
            vx, vy = (xy[j, 0] - s[0]) / dt, (xy[j, 1] - s[1]) / dt
            s[2], s[3] = 0.5 * s[2] + 0.5 * vx, 0.5 * s[3] + 0.5 * vy
            s[0], s[1], s[4], s[5] = xy[j, 0], xy[j, 1], t, 0
            ids[j] = k
        for k in keys:
            if active[k][4] != t:
                active[k][5] += 1
                if active[k][5] > LINK_MAX_MISS:
                    del active[k]
        for j in np.flatnonzero(ids < 0):
            active[nxt] = [xy[j, 0], xy[j, 1], 0.0, 0.0, t, 0]
            ids[j] = nxt
            nxt += 1
        out.append(ids)
    return out


def persistence(frame_idx, xy, n_frames):
    """Share of the run's frames with some candidate within PERSIST_M of each candidate."""
    from scipy.spatial import cKDTree
    out = np.zeros(len(xy))
    if not len(xy):
        return out
    nb = cKDTree(xy).query_ball_point(xy, PERSIST_M)
    for i, js in enumerate(nb):
        out[i] = len(set(frame_idx[js])) / max(n_frames, 1)
    return out


def label_tracklets(track, cam_hit, in_view, cam_near, xy, persist=None):
    """Per-candidate arrays (same length) -> (label, source) per candidate.
    source: 'cam', 'carried', 'static_neg', 'persistent', 'unknown'."""
    label = np.full(len(track), -1, dtype=np.int8)
    source = np.array(['unknown'] * len(track), dtype=object)
    for k in np.unique(track):
        m = track == k
        hits = int(cam_hit[m].sum())
        view = in_view[m]
        clear_view = view & ~cam_near[m]
        p = xy[m]
        static = np.hypot(*(p - p.mean(axis=0)).T).max() < STATIC_M
        if hits >= MIN_CAM_HITS and hits >= MIN_CAM_FRAC * max(1, int(view.sum())):
            label[m] = 1
            source[m] = np.where(cam_hit[m], 'cam', 'carried')
        elif hits == 0 and static and clear_view.sum() >= MIN_VIEW_FRAMES:
            label[m] = 0
            source[m] = 'static_neg'
    if persist is not None:
        pos_xy = xy[label == 1]
        for i in np.flatnonzero((label == -1) & (persist >= PERSIST_FRAC)):
            if cam_near[track == track[i]].any():
                continue
            if len(pos_xy) and np.hypot(*(pos_xy - xy[i]).T).min() < PERSIST_M:
                continue
            label[i], source[i] = 0, 'persistent'
    return label, source


def process_run(run_dir, overrides=()):
    run_dir = pathlib.Path(run_dir)
    recs = [json.loads(line) for line in open(run_dir / 'lidar_persons.ndjson')]
    rec_t = [r['stamp'] for r in recs]
    bad = clean_camera(recs)
    db = sqlite3.connect(str(run_dir / 'bag' / 'bag_0.db3'))
    tid = {n: i for i, n in db.execute('select id, name from topics')}
    poses = Poses([parse_odom(d) for (d,) in db.execute(
        'select data from messages where topic_id=?', (tid['/odom'],))])
    lx, ly, lz = LIVOX_XYZ
    frames, rows = [], []
    for (data,) in db.execute('select data from messages where topic_id=? order by timestamp',
                              (tid['/livox/lidar'],)):
        p = decode_points(data)
        if p is None or not len(p):
            continue
        sec, nsec = struct.unpack_from('<iI', data, 4)
        start = sec + nsec * 1e-9
        end = start + int(p['t'].max()) * 1e-9
        ok = (np.abs(p['x']) + np.abs(p['y']) + np.abs(p['z'])) > 0
        p = p[ok]
        sx, sy, z = p['x'] + lx, p['y'] + ly, p['z'] + lz
        keep = (z > 0.1) & (z < 2.3)
        p, sx, sy, z = p[keep], sx[keep], sy[keep], z[keep]
        frame = poses.at([end])
        pts = poses.at(start + p['t'].astype(np.float64) * 1e-9)
        if frame is None or pts is None:
            continue
        px, py, pa = pts
        c, s = np.cos(pa), np.sin(pa)
        wx, wy = c * sx - s * sy + px, s * sx + c * sy + py
        fx, fy, fa = (float(v[0]) for v in frame)
        ox, oy = fx + math.cos(fa) * lx, fy + math.sin(fa) * lx
        cands = pc.extract(wx, wy, z, origin=(ox, oy), range_max=RANGE_MAX_M)
        i = bisect.bisect_left(rec_t, end)
        near = [k for k in (i - 1, i) if 0 <= k < len(recs) and abs(rec_t[k] - end) < 0.06]
        rec = recs[min(near, key=lambda k: abs(rec_t[k] - end))] if near else None
        cam = np.array(camera_people(rec, end, bad)).reshape(-1, 2)
        xy = cands[:, :2].astype(np.float64)
        hits = {j for _, j in mutual_nearest(cam, xy, CAM_MATCH_M)}
        cf_, cs_ = math.cos(fa), math.sin(fa)
        for j, cand in enumerate(cands):
            m = cf.crop(wx, wy, z, cand)
            feat = cf.features(wx[m], wy[m], z[m], origin=(ox, oy))
            refl = p['r'][m].astype(np.float32)
            dx, dy = cand[0] - fx, cand[1] - fy
            fwd, left = cf_ * dx + cs_ * dy, -cs_ * dx + cf_ * dy
            bearing = math.degrees(math.atan2(left, fwd))
            view = (rec is not None and abs(bearing) < VIEW_HALF_DEG
                    and VIEW_RANGE_M[0] < math.hypot(fwd, left) < VIEW_RANGE_M[1])
            near_cam = bool(len(cam)) and np.hypot(cam[:, 0] - cand[0], cam[:, 1] - cand[1]).min() < CAM_CLEAR_M
            rows.append(dict(frame=len(frames), feat=feat, refl=(refl.mean() if len(refl) else 0.0, refl.std() if len(refl) else 0.0),
                             t=end, x=float(cand[0]), y=float(cand[1]), bearing=bearing,
                             cam_hit=j in hits, in_view=view, cam_near=near_cam))
        frames.append((end, xy))
    ids = link(frames)
    track = np.concatenate(ids) if ids else np.zeros(0, dtype=np.int64)
    if not rows:
        return None
    xy = np.array([[r['x'], r['y']] for r in rows])
    persist = persistence(np.array([r['frame'] for r in rows]), xy, len(frames))
    label, source = label_tracklets(track, np.array([r['cam_hit'] for r in rows]),
                                    np.array([r['in_view'] for r in rows]),
                                    np.array([r['cam_near'] for r in rows]), xy, persist)
    label, source = apply_overrides(run_dir.name, xy, label, source, overrides)
    return dict(
        X=np.stack([r['feat'] for r in rows]),
        refl=np.array([r['refl'] for r in rows], dtype=np.float32),
        y=label, source=source.astype(str), track=track,
        t=np.array([r['t'] for r in rows]), xy=xy.astype(np.float32),
        bearing=np.array([r['bearing'] for r in rows], dtype=np.float32),
        in_view=np.array([r['in_view'] for r in rows]),
        run=np.array([run_dir.name] * len(rows)),
        n_frames=len(frames), n_bad_camera=len(bad))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--overrides', help='JSON list of {run, x, y, r_m, label} hand corrections (odom)')
    a = ap.parse_args()
    overrides = json.load(open(a.overrides)) if a.overrides else []
    parts = []
    for run in a.runs:
        r = process_run(run, overrides)
        if r is None:
            print(f'{run}: no frames')
            continue
        y, src, rng = r['y'], r['source'], r['X'][:, 0]
        print(f"{pathlib.Path(run).name:14s} frames {r['n_frames']:4d} cands {len(y):6d} "
              f"| pos {np.sum(y == 1):5d} (cam {np.sum(src == 'cam'):4d}, carried {np.sum(src == 'carried'):4d},"
              f" >4m {np.sum((y == 1) & (rng > 4)):4d}) | neg {np.sum(src == 'static_neg'):4d} + persistent {np.sum(src == 'persistent'):5d} | sitting {np.sum(y == 2):4d} | unk {np.sum(y == -1):5d}"
              f" | camera outliers dropped {r['n_bad_camera']}")
        parts.append(r)
    keys = ['X', 'refl', 'y', 'source', 'track', 't', 'xy', 'bearing', 'in_view', 'run']
    out = {k: np.concatenate([p[k] for p in parts]) for k in keys}
    # tracklet ids are per run; make them global
    off, tr = 0, []
    for p in parts:
        tr.append(p['track'] + off)
        off += int(p['track'].max()) + 1 if len(p['track']) else 0
    out['track'] = np.concatenate(tr)
    out['feature_names'] = np.array(cf.NAMES)
    np.savez_compressed(a.out, **out)
    print(f'wrote {a.out}: {len(out["y"])} candidates')


if __name__ == '__main__':
    main()
