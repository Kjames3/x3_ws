#!/usr/bin/env python3
"""Check a lidar->camera extrinsic by projecting Livox INTENSITY onto the camera image.

    python3 src/tools/calibration/lidar_intensity_check.py ~/calib/2026-11-20 \
        [--extrinsics config/camera_lidar_extrinsics_2026-11-20.json] \
        [--cameras front rear right] [--sweep] --out artifacts/intensity-check-2026-11-20

Input is the station-bag layout written by calib_capture.py: <dir>/<station>_<camera>/
holding one rosbag2 with /image and /points (PointCloud2 x, y, z, intensity; the
cloud is already integrated over the station's hold time).

Why intensity: the depth-edge check in lidar_camera_overlay.py only sees geometry
(door frames, table legs). Intensity also changes at texture edges on flat
surfaces: tape, posters, the checkerboard squares, floor-to-rug boundaries. Those
should land on image edges when the extrinsic is right.

Outputs per station/camera:
  <out>/<station>_<camera>.png    image | intensity-coloured points (side by side)
  <out>/summary.json              intensity-edge score per view and per camera
Score: pixels where the (log, range-normalised) intensity image has a strong
gradient, median distance (px) to the nearest Canny edge of the camera image.
Lower is better; compare cameras/stations on the same scene, not absolute values.
KNOWN WEAKNESS (2026-11-20 apartment run): on plain walls the score saturates at
the 15 px clip (front and right cameras), because Livox intensity speckle makes
gradients the image has no edge for, so the front sweep's -1 deg was noise. Judge
by eye from the PNGs (tags, posters, tape), not from the number, until fixed.
--sweep perturbs roll/pitch/yaw by up to +-1.5 deg and reports where the mean
score is minimal; a minimum within ~0.3 deg of zero means "no evidence of error".
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path

for _p in (_Path(__file__).resolve().parent, _Path(__file__).resolve().parents[2]):
    if str(_p) not in _sys.path:
        _sys.path.insert(0, str(_p))

import argparse
import glob
import json
import os

import numpy as np

import bag_signals as bs
from lidar_camera_overlay import Camera, image_edge_distance

REPO = _Path(__file__).resolve().parents[3]


def quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_xyz(r, p, y):
    """Small rotation (deg) about camera optical x, y, z."""
    r, p, y = np.radians([r, p, y])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_cameras(ext_path):
    d = json.load(open(ext_path))
    cams = {}
    for name in ('front', 'rear', 'right'):
        if name not in d:
            continue
        x, y, z, qx, qy, qz, qw = d[name]['T_lidar_camera']
        T = np.eye(4)
        T[:3, :3] = quat_to_R(qx, qy, qz, qw)
        T[:3, 3] = [x, y, z]
        if name in ('front', 'rear'):
            k = d['oak_rgb_intrinsics_1920x1440'][name]
            cam = Camera(k['K'], k['D'], k.get('model', 'rational_polynomial'), k['w'], k['h'])
        else:
            f = json.load(open(REPO / 'config/arducam_uc844_intrinsics.json'))['fisheye']
            cam = Camera(f['K'], f['D'], 'fisheye', f['width'], f['height'])
        cams[name] = {'T_cam_lidar': np.linalg.inv(T), 'cam': cam, 'status': d[name].get('status', '')}
    return cams


def load_station(bag_dir):
    from sensor_msgs_py import point_cloud2
    img, pts = None, None
    for topic, typ, _s, _r, data in bs.iter_messages(bag_dir, ['/image', '/points']):
        if topic == '/image' and img is None:
            img = bs.decode_image(data, typ)
        elif topic == '/points' and pts is None:
            msg = bs.deserialize(data, typ)
            names = [f.name for f in msg.fields]
            if 'intensity' not in names:
                raise SystemExit(f'{bag_dir}: cloud has no intensity field ({names})')
            a = point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
            pts = np.stack([a['x'], a['y'], a['z'], a['intensity']], 1).astype(np.float64)
    if img is None or pts is None:
        raise SystemExit(f'{bag_dir}: missing /image or /points')
    keep = np.isfinite(pts).all(1) & (np.abs(pts[:, :3]).sum(1) > 1e-6)
    return img, pts[keep]


def intensity_image(cam, T, pts):
    """Sparse projected intensity -> dense-ish image (log, range-normalised), validity mask, uv, values."""
    import cv2
    pc = pts[:, :3] @ T[:3, :3].T + T[:3, 3]
    uv, depth, idx = cam.project(pc)
    h, w = cam.height, cam.width
    # range normalisation: Livox reflectivity is already roughly range-compensated, but the
    # near field is still hot; a mild r^0.5 term flattens walls without killing tape/posters.
    val = np.log1p(pts[idx, 3]) + 0.5 * np.log(np.clip(depth, 0.3, 20))
    img = np.zeros((h, w), np.float32)
    zbuf = np.full((h, w), np.inf, np.float32)
    ui, vi = uv[:, 0].astype(int), uv[:, 1].astype(int)
    order = np.argsort(-depth)  # far first, near overwrites
    img[vi[order], ui[order]] = val[order]
    zbuf[vi[order], ui[order]] = depth[order]
    mask = np.isfinite(zbuf).astype(np.uint8)
    # fill gaps between points with a normalised blur (keeps edges where data is dense)
    k = max(3, int(round(np.sqrt(h * w / max(len(uv), 1)) * 1.5)) | 1)
    num = cv2.GaussianBlur(img * mask, (k, k), 0)
    den = cv2.GaussianBlur(mask.astype(np.float32), (k, k), 0)
    dense = np.where(den > 0.05, num / np.maximum(den, 1e-6), 0)
    return dense, den > 0.05, uv, val, depth


def intensity_score(dense, valid, dist_img, q=0.92, clip=15.0):
    import cv2
    gx = cv2.Sobel(dense, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(dense, cv2.CV_32F, 0, 1, ksize=3)
    g = np.hypot(gx, gy)
    inner = cv2.erode(valid.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
    g = np.where(inner, g, 0)
    if inner.sum() < 500:
        return float('nan'), 0
    thr = np.quantile(g[inner], q)
    m = g > thr
    if m.sum() < 50:
        return float('nan'), int(m.sum())
    return float(np.median(np.minimum(dist_img[m], clip))), int(m.sum())


def render(img, uv, val, depth):
    import cv2
    base = img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    over = (base * 0.35).astype(np.uint8)
    if len(val):
        lo, hi = np.percentile(val, [2, 98])
        t = np.clip((val - lo) / max(hi - lo, 1e-6), 0, 1)
        col = cv2.applyColorMap((255 * t).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_INFERNO)[:, 0]
        order = np.argsort(-depth)
        over[uv[order, 1].astype(int), uv[order, 0].astype(int)] = col[order]
    return np.hstack([base, over])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('calib_dir')
    ap.add_argument('--extrinsics', default=str(REPO / 'config/camera_lidar_extrinsics_2026-11-20.json'))
    ap.add_argument('--cameras', nargs='+', default=['front', 'rear', 'right'])
    ap.add_argument('--sweep', action='store_true')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    import cv2
    os.makedirs(a.out, exist_ok=True)
    cams = load_cameras(a.extrinsics)
    summary = {'calib_dir': os.path.abspath(a.calib_dir), 'extrinsics': a.extrinsics, 'cameras': {}}
    for name in a.cameras:
        c = cams[name]
        views = []
        for bag in sorted(glob.glob(os.path.join(a.calib_dir, f'*_{name}'))):
            if not glob.glob(os.path.join(bag, '*.db3')):
                continue
            img, pts = load_station(bag)
            if (img.shape[1], img.shape[0]) != (c['cam'].width, c['cam'].height):
                print(f'  skip {bag}: image {img.shape[1]}x{img.shape[0]} != intrinsics')
                continue
            dist = image_edge_distance(img)
            dense, valid, uv, val, depth = intensity_image(c['cam'], c['T_cam_lidar'], pts)
            s, n = intensity_score(dense, valid, dist)
            st = os.path.basename(bag)
            cv2.imwrite(os.path.join(a.out, f'{st}.png'), render(img, uv, val, depth))
            views.append({'station': st, 'img': img, 'pts': pts, 'dist': dist,
                          'score_px': s, 'edge_px': n, 'points_in_view': int(len(uv))})
            print(f'{st}: {len(uv)} pts in view, intensity-edge score {s:.2f} px over {n} px')
        if not views:
            continue
        scores = [v['score_px'] for v in views if np.isfinite(v['score_px'])]
        entry = {'status': c['status'], 'mean_score_px': float(np.mean(scores)) if scores else None,
                 'views': [{k: v[k] for k in ('station', 'score_px', 'edge_px', 'points_in_view')} for v in views]}
        if a.sweep and scores:
            best = {}
            for ax in range(3):
                curve = []
                for d in np.arange(-1.5, 1.51, 0.25):
                    delta = [0.0, 0.0, 0.0]
                    delta[ax] = d
                    T = c['T_cam_lidar'].copy()
                    R = rot_xyz(*delta)
                    T[:3, :3] = R @ T[:3, :3]
                    T[:3, 3] = R @ T[:3, 3]
                    ss = [intensity_score(*intensity_image(c['cam'], T, v['pts'])[:2], v['dist'])[0] for v in views]
                    ss = [x for x in ss if np.isfinite(x)]
                    curve.append((float(d), float(np.mean(ss)) if ss else float('nan')))
                b = min(curve, key=lambda t: t[1] if np.isfinite(t[1]) else 1e9)
                best[('rot_x_deg', 'rot_y_deg', 'rot_z_deg')[ax]] = {'best_delta_deg': b[0], 'curve': curve}
            entry['sweep'] = best
            print(f'{name} sweep best deltas (optical-frame axes): ' +
                  ', '.join(f'{k} {v["best_delta_deg"]:+.2f}' for k, v in best.items()))
        summary['cameras'][name] = entry
        print(f'== {name} ({c["status"]}): mean intensity-edge score {entry["mean_score_px"]}')
    with open(os.path.join(a.out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    print(f'wrote {a.out}/summary.json')


if __name__ == '__main__':
    main()
