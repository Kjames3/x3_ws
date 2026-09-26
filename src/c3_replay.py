#!/usr/bin/env python3
"""C3 three-arm tracker replay on a C1 shared dataset (offline, no ROS graph).

  python3 src/c3_replay.py detect DATASET           # freeze person boxes once
  python3 src/c3_replay.py run DATASET --output OUT # arms A/B/C + static metrics

A: deployed v3 (VelocityEstimator._step, unchanged), driven on a 10 Hz tick
   that takes the newest depth frame, as the live loop does.
B: Kalman tracker on the same v3 depth-blob centroids, every paired frame.
C: Kalman tracker on robust torso depth inside cached person boxes.

Every arm sees identical depth frames. Requires sourced ROS2 Humble for bag
deserialization only. Outputs are diagnostic; nothing reaches the CBF.
"""
import argparse
import csv
import json
import math
import sqlite3
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from c1_dataset import load_pair, rgb_on_depth_grid, file_sha256, stamp_ns  # noqa: E402
import c3_person_tracker as pt  # noqa: E402

RANGE_BINS = [(0.5, 1.8), (1.8, 3.0), (3.0, 4.0), (4.0, math.inf)]
DETECTOR = 'models/yolo26n.pt'
DETECTOR_CONF = 0.35
TICK_NS = 100_000_000          # velocity_estimator.INFER_HZ = 10
MAX_DEPTH_AGE_NS = 500_000_000  # velocity_estimator.MAX_DEPTH_FRAME_AGE_S


def load_odom(directory):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    out = []
    for db in sorted((directory / 'bag').glob('*.db3')):
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
            ids = [i for i, n in conn.execute('select id,name from topics') if n == '/odom']
            for (payload,) in conn.execute(
                    'select data from messages where topic_id=? order by timestamp,id', (ids[0],)):
                m = deserialize_message(payload, Odometry)
                q = m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                out.append((stamp_ns(m.header.stamp), m.pose.pose.position.x,
                            m.pose.pose.position.y, yaw))
    out.sort()
    return out


def odom_at(odom, t_ns, max_gap_ns=100_000_000):
    """Interpolated (x, y, yaw) bracketing t_ns within max_gap_ns, else None."""
    from bisect import bisect_left
    stamps = [o[0] for o in odom]
    i = bisect_left(stamps, t_ns)
    if i == 0 or i == len(odom):
        return None
    a, b = odom[i - 1], odom[i]
    if t_ns - a[0] > max_gap_ns or b[0] - t_ns > max_gap_ns:
        return None
    f = (t_ns - a[0]) / max(1, b[0] - a[0])
    dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw


def make_v3(directory):
    """Deployed estimator with the capture's snapshotted ground-plane config."""
    import torch
    import velocity_estimator as ve
    torch.set_num_threads(1)
    snap = directory / 'snapshot' / 'config' / 'camera_ground_plane.json'
    if snap.exists():
        ve.GROUND_PLANE_CONFIG_PATH = str(snap)
    est = ve.VelocityEstimator(None, None, model_path=str(ROOT / 'src' / 'velocity_mlp_v3.torchscript'))
    if est._model is None:
        raise RuntimeError('v3 model failed to load')
    return est


def depth_metres(arrays, row):
    """Recorded raw depth in metres with the capture's per-device correction.

    Captures from before the OAK-D Pro W record no correction (Lite: none
    fitted) and stay uncorrected, matching what the live estimator saw.
    """
    depth = arrays['depth'].astype(np.float32) / 1000.0
    c = row['metadata']['calibration'].get('depth_inv_offset_per_m') or 0.0
    if c:
        depth /= 1.0 + c * depth
    return depth


def intrinsics(row):
    k = row['depth']['camera_info']['k']
    return (k[0], k[4], k[2], k[5], row['depth']['width'], row['depth']['height'])


def detect(directory, model_rel=DETECTOR, conf=DETECTOR_CONF, name='c3-detections.json'):
    """Run a frozen detector once and cache its person boxes.

    Arm C reads c3-detections.json (DETECTOR). c3_score.py builds its
    independent reference, c3-reference.json, with a different, larger model.
    """
    from ultralytics import YOLO
    rows = json.loads((directory / 'pair-index.json').read_text())
    model = YOLO(str(ROOT / model_rel))
    frames = []
    for i in range(len(rows)):
        arrays, row = load_pair(directory, i)
        rgb, support = rgb_on_depth_grid(arrays, row)
        res = model.predict(rgb, conf=conf, classes=[0], verbose=False)[0]
        boxes = [dict(xyxy=[float(v) for v in b.xyxy[0]], conf=float(b.conf[0]))
                 for b in res.boxes]
        frames.append(dict(index=i, depth_stamp_ns=row['depth']['stamp_ns'], boxes=boxes))
    cache = dict(schema='x3.c3.detections.v1', detector=model_rel,
                 detector_sha256=file_sha256(ROOT / model_rel), conf=conf,
                 image='rgb_on_depth_grid (nominal calibration, registration unqualified)',
                 frames=frames)
    (directory / name).write_text(json.dumps(cache) + '\n')
    print(f'{sum(len(f["boxes"]) for f in frames)} person boxes in {len(frames)} frames')


def forward_depth(xy, ofc):
    _, rot = pt.camera_point_to_odom([0, 0, 0], ofc)
    fwd = rot[:2, 2] / (np.linalg.norm(rot[:2, 2]) or 1)
    return float(np.dot(np.asarray(xy) - np.asarray(ofc['translation_xyz'][:2]), fwd))


def run(directory, output, tracker_config=None, tracker_config_c=None):
    rows = json.loads((directory / 'pair-index.json').read_text())
    odom = load_odom(directory)
    det_path = directory / 'c3-detections.json'
    detections = json.loads(det_path.read_text()) if det_path.exists() else None
    v3 = make_v3(directory)
    trk_b = pt.KalmanTracker(tracker_config)
    trk_c = pt.KalmanTracker(tracker_config_c if tracker_config_c is not None else tracker_config)
    floor = trk_b.config['meas_sigma_floor_m']
    floor_c = trk_c.config['meas_sigma_floor_m']
    out = {'A': [], 'B': [], 'C': []}
    frames = []
    for i in range(len(rows)):
        arrays, row = load_pair(directory, i)
        depth_m = depth_metres(arrays, row)
        t_ns = row['depth']['stamp_ns']
        intr = intrinsics(row)
        # Arm A's blob extraction is the shared B measurement; run it once.
        cents = v3._extract_depth_centroids(None, depth_m, intr)
        frames.append((t_ns, depth_m, intr, cents, row))
        ofc = row.get('odom_from_camera')
        if ofc is None:
            trk_b.reset(); trk_c.reset()
            continue
        meas_b = []
        for x, y, z in cents:
            xy, rot = pt.camera_point_to_odom([x, y, z], ofc)
            meas_b.append((xy, pt.rotate_cov_to_odom(pt.measurement_cov_camera(z, floor), rot)))
        for o in trk_b.update(t_ns / 1e9, meas_b):
            out['B'].append(dict(o, t_ns=t_ns, z=forward_depth((o['x'], o['y']), ofc)))
        if detections is not None:
            meas_c = []
            fx, fy, cx, cy = intr[:4]
            for b in detections['frames'][i]['boxes']:
                m = pt.person_measurement(depth_m, b['xyxy'])
                if m is None:
                    continue
                p = [(m['u'] - cx) * m['z'] / fx, (m['v'] - cy) * m['z'] / fy, m['z']]
                xy, rot = pt.camera_point_to_odom(p, ofc)
                cov = pt.measurement_cov_camera(m['z'], floor_c, m['spread_m'])
                meas_c.append((xy, pt.rotate_cov_to_odom(cov, rot)))
            for o in trk_c.update(t_ns / 1e9, meas_c):
                out['C'].append(dict(o, t_ns=t_ns, z=forward_depth((o['x'], o['y']), ofc)))

    # Arm A: 10 Hz ticks over the frame timeline, newest frame at or before
    # each tick, exactly like the live loop's latest-frame polling.
    j = -1
    for tick in range(frames[0][0], frames[-1][0] + 1, TICK_NS):
        while j + 1 < len(frames) and frames[j + 1][0] <= tick:
            j += 1
        t_ns, _, _, cents, _ = frames[j]
        pose = odom_at(odom, tick)
        if tick - t_ns > MAX_DEPTH_AGE_NS or pose is None:
            v3._tracker.reset()
            continue
        estimates, _ = v3._step(cents, *pose)
        px, py, th = pose
        for e in estimates:
            # v3 reports camera-local (x right, z forward); place it in odom the
            # way v3 itself does (robot frame = (z, -x)) so all arms share a frame.
            rx, ry = e['z'], -e['x']
            out['A'].append(dict(e, x=px + rx * math.cos(th) - ry * math.sin(th),
                                 y=py + rx * math.sin(th) + ry * math.cos(th),
                                 t_ns=tick, frame_t_ns=t_ns, confirmed=True, measured=True))

    output.mkdir(parents=True, exist_ok=True)
    for arm, recs in out.items():
        if arm == 'C' and detections is None:
            continue
        with (output / f'arm{arm}_tracks.csv').open('w', newline='') as f:
            keys = ['t_ns', 'id', 'x', 'y', 'z', 'vx', 'vy', 'speed', 'confirmed', 'measured', 'obs_age_s']
            w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            w.writerows(recs)
    duration_s = (frames[-1][0] - frames[0][0]) / 1e9
    summary = dict(
        schema='x3.c3.replay.v1', dataset=str(directory),
        dataset_bag_sha256=json.loads((directory / 'audit.json').read_text())['bag_sha256'],
        git_head=subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'],
                                capture_output=True, text=True).stdout.strip(),
        v3_model_sha256=file_sha256(ROOT / 'src' / 'velocity_mlp_v3.torchscript'),
        tracker_config=dict(pt.DEFAULT_CONFIG, **(tracker_config or {})),
        tracker_config_c=dict(trk_c.config),
        detections=None if detections is None else {k: detections[k] for k in ('detector', 'detector_sha256', 'conf')},
        frames=len(frames), duration_s=duration_s,
        static=static_metrics(out, duration_s, detections is not None),
        note='Static metrics treat every output as false motion: valid only for '
             'scenes with no moving object. Person recall/onset are not measured here.')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def static_metrics(out, duration_s, have_c):
    """Speed distribution of confirmed outputs per range bin, per arm."""
    result = {}
    for arm, recs in out.items():
        if arm == 'C' and not have_c:
            continue
        recs = [r for r in recs if r['confirmed']]
        arm_res = {'confirmed_track_ids': len({r['id'] for r in recs}),
                   'confirmed_tracks_per_min': len({r['id'] for r in recs}) / (duration_s / 60)}
        for lo, hi in RANGE_BINS:
            s = np.array([r['speed'] for r in recs if lo <= r['z'] < hi])
            key = f'{lo:g}-{hi:g}m'
            arm_res[key] = dict(n=int(s.size)) if not s.size else dict(
                n=int(s.size), p50=float(np.percentile(s, 50)), p95=float(np.percentile(s, 95)),
                max=float(s.max()), frac_gt_0_15=float(np.mean(s > 0.15)),
                frac_gt_0_30=float(np.mean(s > 0.30)))
        result[arm] = arm_res
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='mode', required=True)
    d = sub.add_parser('detect'); d.add_argument('dataset', type=Path)
    r = sub.add_parser('run'); r.add_argument('dataset', type=Path)
    r.add_argument('--output', type=Path, required=True)
    r.add_argument('--config', type=json.loads, default=None,
                   help='JSON overrides of c3_person_tracker.DEFAULT_CONFIG')
    args = parser.parse_args()
    if args.mode == 'detect':
        detect(args.dataset)
    else:
        print(json.dumps(run(args.dataset, args.output, args.config)['static'], indent=2))


if __name__ == '__main__':
    main()
