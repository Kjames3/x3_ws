#!/usr/bin/env python3
"""Labelled person / not-person clusters from SiT clips, through the robot's own extractor.

    python3 src/tools/perception/sit_person_dataset.py ~/datasets/sit/<Clip> [...] -o sit.npz

Each clip directory holds velo/concat/data/<i>.pcd, label_3d/<i>.txt and
ego_trajectory/<i>.txt (unzip only those from the SiT zip; the camera images are 95% of it).

Per frame: the merged cloud (two 16-ring lidars, ego frame) is lifted to heights above
the floor with the clip's floor level (median bottom of the labelled pedestrian boxes
within 10 m, ~0.95 m below the lidar), run through person_candidates.extract, the same
as on the robot, and described by cluster_features.features with SiT's sensor geometry.
Labels come from the 3D boxes (world frame -> ego with inv(T_world_ego), checked by
points-in-box counts: 9/11, 15/18, 11/15, 8/13 boxes hold > 10 points that way against
1/11, 6/18, 6/15, 5/13 without):

  1  person     candidate centre inside a 'Pedestrian' box footprint (+MATCH_PAD_M)
  2  sitting    the same with 'Pedestrain_sitting' (sic): kept apart, the robot's own
                labels cannot tell a seated person from furniture
  0  not person further than NEG_CLEAR_M from every box, within the range the clip's
                boxes reach (so an unlabelled distant person is not called furniture);
                vehicles and cyclists count as not person
 -1  unknown    the rest

It also reports how many labelled pedestrians (with >= 10 points, within range) got a
candidate at all: the extractor's own recall, which bounds any classifier on top.
"""
import argparse
import glob
import multiprocessing as mp
import os
import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import sit_pcd  # noqa: E402
from yahboomcar_bringup import cluster_features as cf  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

RANGE_MAX_M = 12.0
MATCH_PAD_M = 0.15
NEG_CLEAR_M = 1.0
VLP16_TAN = 0.268          # +-15 deg


def _boxes_ego(labels, T_world_ego):
    inv = np.linalg.inv(T_world_ego)
    rot = np.arctan2(inv[1, 0], inv[0, 0])
    out = []
    for cls, _id, h, w, l, x, y, z, yaw in labels:
        b = inv @ np.array([x, y, z, 1.0])
        out.append((cls, h, w, l, b[0], b[1], b[2], yaw + rot))
    return out


def _floor(clip):
    bot = []
    for fi in range(0, 200, 10):
        try:
            boxes = _boxes_ego(sit_pcd.read_labels(f'{clip}/label_3d/{fi}.txt'),
                               sit_pcd.read_pose(f'{clip}/ego_trajectory/{fi}.txt'))
        except FileNotFoundError:
            continue
        bot += [bz - h / 2 for c, h, w, l, bx, by, bz, yaw in boxes
                if c == 'Pedestrian' and np.hypot(bx, by) < 10]
    return float(np.median(bot)) if bot else -0.95


def _in_box(px, py, box, pad):
    c, h, w, l, bx, by, bz, yaw = box
    dx, dy = px - bx, py - by
    cy, sy = np.cos(-yaw), np.sin(-yaw)
    lx, ly = cy * dx - sy * dy, sy * dx + cy * dy
    return abs(lx) < max(l, 0.4) / 2 + pad and abs(ly) < max(w, 0.4) / 2 + pad


def process_frame(args):
    clip, fi, floor, reach = args
    try:
        p = sit_pcd.read_pcd(f'{clip}/velo/concat/data/{fi}.pcd')
        boxes = _boxes_ego(sit_pcd.read_labels(f'{clip}/label_3d/{fi}.txt'),
                           sit_pcd.read_pose(f'{clip}/ego_trajectory/{fi}.txt'))
    except FileNotFoundError:
        return None
    x = p['x'].astype(np.float64)
    y = p['y'].astype(np.float64)
    z = p['z'].astype(np.float64) - floor
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (np.abs(x) + np.abs(y) > 0.05)
    x, y, z = x[ok], y[ok], z[ok]
    sensor = {'sensor_z': -floor, 'fov_top_tan': VLP16_TAN, 'fov_bottom_tan': VLP16_TAN}
    cands = pc.extract(x, y, z, range_max=RANGE_MAX_M)
    feats, labels, src = [], [], []
    for cand in cands:
        m = cf.crop(x, y, z, cand)
        feats.append(cf.features(x[m], y[m], z[m], sensor=sensor))
        hit = [b for b in boxes if _in_box(cand[0], cand[1], b, MATCH_PAD_M)]
        near = min((np.hypot(cand[0] - b[4], cand[1] - b[5]) for b in boxes), default=99.0)
        if any(b[0] == 'Pedestrian' for b in hit):
            labels.append(1), src.append('box')
        elif any(b[0] == 'Pedestrain_sitting' for b in hit):
            labels.append(2), src.append('sitting')
        elif hit:
            labels.append(0), src.append('vehicle')
        elif near > NEG_CLEAR_M and np.hypot(cand[0], cand[1]) < reach:
            labels.append(0), src.append('clear')
        else:
            labels.append(-1), src.append('unknown')
    # extractor recall: labelled standing pedestrians with >= 10 points within range
    found = total = 0
    for b in boxes:
        if b[0] != 'Pedestrian' or np.hypot(b[4], b[5]) > min(reach, RANGE_MAX_M - 0.5):
            continue
        dz = np.abs(z - (b[6] - floor))
        near_pts = (np.abs(x - b[4]) < 0.6) & (np.abs(y - b[5]) < 0.6) & (dz < b[1] / 2)
        if near_pts.sum() < 10:
            continue
        total += 1
        found += any(_in_box(c[0], c[1], b, MATCH_PAD_M) for c in cands)
    n = len(cands)
    return dict(X=np.array(feats, dtype=np.float32).reshape(n, len(cf.NAMES)),
                y=np.array(labels, dtype=np.int8), source=np.array(src, dtype=str),
                xy=cands[:, :2].astype(np.float32).reshape(n, 2),
                frame=np.full(n, fi), found=found, total=total)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('clips', nargs='+')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('-j', '--jobs', type=int, default=os.cpu_count())
    a = ap.parse_args()
    parts = []
    with mp.Pool(a.jobs) as pool:
        for clip in a.clips:
            clip = os.path.expanduser(clip.rstrip('/'))
            name = os.path.basename(clip)
            floor = _floor(clip)
            reach = []
            for fi in range(0, 200, 10):
                try:
                    reach += [np.hypot(b[4], b[5]) for b in _boxes_ego(
                        sit_pcd.read_labels(f'{clip}/label_3d/{fi}.txt'),
                        sit_pcd.read_pose(f'{clip}/ego_trajectory/{fi}.txt'))]
                except FileNotFoundError:
                    pass
            reach = float(np.percentile(reach, 95)) if reach else 0.0
            n_frames = len(glob.glob(f'{clip}/velo/concat/data/*.pcd'))
            res = [r for r in pool.map(process_frame, [(clip, fi, floor, reach) for fi in range(n_frames)]) if r]
            found, total = sum(r['found'] for r in res), sum(r['total'] for r in res)
            part = {k: np.concatenate([r[k] for r in res]) for k in ('X', 'y', 'source', 'xy', 'frame')}
            part['run'] = np.array([name] * len(part['y']))
            y, rng = part['y'], part['X'][:, 0]
            print(f"{name:13s} floor {floor:+.2f} reach {reach:4.1f} m | cands {len(y):6d} | person {np.sum(y == 1):5d}"
                  f" (>4m {np.sum((y == 1) & (rng > 4)):5d}) sitting {np.sum(y == 2):5d} | neg {np.sum(y == 0):6d}"
                  f" | extractor recall {found}/{total} = {100 * found / max(total, 1):.0f}%")
            parts.append(part)
    out = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
    out['feature_names'] = np.array(cf.NAMES)
    np.savez_compressed(a.out, **out)
    print(f'wrote {a.out}: {len(out["y"])} candidates')


if __name__ == '__main__':
    main()
