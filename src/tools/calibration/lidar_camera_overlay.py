#!/usr/bin/env python3
"""Project lidar points onto camera images to check (and nudge) a lidar->camera extrinsic.

    python3 src/tools/calibration/lidar_camera_overlay.py BAG \
        --image /gs_cam/image_raw --lidar /livox/lidar \
        (--camera-info /gs_cam/camera_info | --intrinsics ost.yaml) \
        (--extrinsic X Y Z ROLL PITCH YAW [--body-frame] | --tf [--camera-frame F]) \
        [--accumulate 1.0] [--xcorr-report artifacts/time-sync/report.json] \
        [--max-images 8] [--sweep] [--interactive] --out artifacts/overlay-YYYY-MM-DD

Record the bag with the robot STILL for a few seconds at a time (the Mid-360's
non-repetitive pattern needs ~0.5-1 s to fill in), in a room with sharp depth
edges: door frames, table legs, shelf edges, a box on the floor. Points are
accumulated over --accumulate seconds around each image, with no motion
compensation, so a moving robot shows up as doubled edges.

The extrinsic
-------------
--extrinsic gives the camera's pose IN THE LIDAR FRAME, the way a URDF joint
with parent = lidar frame and child = camera frame would: metres, then
roll/pitch/yaw in DEGREES (R = Rz(yaw) Ry(pitch) Rx(roll)). By default the child
is the camera OPTICAL frame (z forward, x right, y down). With --body-frame the
child is a REP-103 camera body frame (x forward, y left, z up) and the standard
optical rotation is appended for you. Use the frame the cloud is published in
(its header.frame_id); if the Livox driver config applies an extrinsic, that is
already baked into the points.

--tf instead reads /tf_static (and the first /tf) from the bag and chains from
the cloud's frame_id to the image's frame_id (or --camera-frame). Add
--body-frame if that frame is a body frame (x forward), not an optical one.

What it reports
---------------
An edge-alignment score per image: lidar points that sit on the near side of a
depth discontinuity are depth-edge points; the score is their median distance
(px) to the nearest Canny edge in the image. Lower is better; a few px is good,
and the absolute number depends on the scene, so compare runs on one bag.

--sweep perturbs each of x, y, z, roll, pitch, yaw on its own and reports the
value that minimizes the score: a sanity check that the calibration sits in a
minimum, and a hint of which axis is off. It is coordinate descent on a noisy
score, not a calibration; use direct_visual_lidar_calibration (or a target)
for the real thing and this to check it.

RESOLUTION FLOOR (measured on noise-free synthetic data, where the true
extrinsic is known): the sweep's optimum sits up to ~0.3 deg off in pitch/yaw,
~0.6 deg in roll, and ~3 cm in translation, and scores slightly BETTER than the
truth. The depth-edge band has finite width and the occlusion filter thins its
far side. So: a delta under those numbers means "no evidence of error", never
"apply this correction". Deltas well above them (a 1 deg yaw error moves the
score from ~1.4 to ~6 px) are real. Translation barely moves edges (1 cm is
~2 px at 3 m, ~0 px along the view axis): take it from CAD + the calibrator.

Images are picked every --every seconds with no motion check, so record
stop-and-hold segments and set --start/--every to land in them, or the
moving frames (doubled edges) will pollute the mean score and the sweep.

--interactive opens a window on the first image: q/a roll, w/s pitch, e/d yaw
(0.1 deg), r/f x, t/g y, y/h z (5 mm), [ / ] step size, p prints the
extrinsic, Esc quits and prints it.
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

R_BODY_OPTICAL = bs.rpy_to_mat(-math.pi / 2, 0.0, -math.pi / 2)  # REP-103 body -> optical
PARAM_NAMES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


# ======================================================================= camera model

class Camera:
    def __init__(self, K, D, model, width, height):
        self.K = np.asarray(K, float).reshape(3, 3)
        self.D = np.asarray(D, float).ravel()
        self.model = model
        self.width, self.height = int(width), int(height)
        self.fisheye = model in ('equidistant', 'fisheye', 'kannala_brandt')
        self._bounds = self._normalized_bounds()

    def _normalized_bounds(self):
        """Max |x/z|, |y/z| that land in the image (with margin), so wild distortion can't wrap points in."""
        import cv2
        w, h = self.width, self.height
        edge = np.array([[u, v] for u in np.linspace(0, w - 1, 9) for v in (0, h - 1)] +
                        [[u, v] for u in (0, w - 1) for v in np.linspace(0, h - 1, 9)], np.float64)
        if self.fisheye:
            n = cv2.fisheye.undistortPoints(edge[None], self.K, self.D[:4])[0]
        elif self.D.size and np.any(self.D):
            n = cv2.undistortPoints(edge[None], self.K, self.D)[0]
        else:
            n = (edge - self.K[:2, 2]) / np.diag(self.K)[:2]
        return np.abs(n).max(0) * 1.15

    @classmethod
    def from_camera_info(cls, msg):
        return cls(msg.k, msg.d, msg.distortion_model or 'plumb_bob', msg.width, msg.height)

    @classmethod
    def from_file(cls, path):
        """ROS camera_calibration ost.yaml, or JSON {fx, fy, cx, cy, width, height, dist, model}."""
        with open(path) as fh:
            text = fh.read()
        if path.endswith('.json'):
            j = json.loads(text)
            K = [[j['fx'], 0, j['cx']], [0, j['fy'], j['cy']], [0, 0, 1]]
            return cls(K, j.get('dist', []), j.get('model', 'plumb_bob'), j['width'], j['height'])
        import yaml
        y = yaml.safe_load(text)
        return cls(y['camera_matrix']['data'], y['distortion_coefficients']['data'],
                   y.get('distortion_model', 'plumb_bob'), y['image_width'], y['image_height'])

    def project(self, pc):
        """Camera-frame points (N,3) -> pixels (M,2), depths (M,), index into pc (M,)."""
        import cv2
        z = pc[:, 2]
        ok = z > 0.05
        nx, ny = pc[:, 0] / np.where(ok, z, 1), pc[:, 1] / np.where(ok, z, 1)
        ok &= (np.abs(nx) < self._bounds[0]) & (np.abs(ny) < self._bounds[1])
        idx = np.nonzero(ok)[0]
        if not len(idx):
            return np.zeros((0, 2)), np.zeros(0), idx
        p = pc[idx].reshape(-1, 1, 3)
        zero = np.zeros(3)
        if self.fisheye:
            uv = cv2.fisheye.projectPoints(p, zero, zero, self.K, self.D[:4])[0].reshape(-1, 2)
        else:
            uv = cv2.projectPoints(p, zero, zero, self.K, self.D if self.D.size else None)[0].reshape(-1, 2)
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < self.width) & (uv[:, 1] >= 0) & (uv[:, 1] < self.height)
        return uv[inside], z[idx][inside], idx[inside]


# ======================================================================= extrinsic

def params_to_T(params, body_frame=False):
    """[x,y,z,roll,pitch,yaw(deg)] = camera pose in lidar frame -> T_cam_from_lidar (4x4)."""
    x, y, z, r, p, yw = params
    R = bs.rpy_to_mat(math.radians(r), math.radians(p), math.radians(yw))
    if body_frame:
        R = R @ R_BODY_OPTICAL
    T_lidar_cam = bs.make_T(R, [x, y, z])
    return np.linalg.inv(T_lidar_cam)


def T_to_params(T_cam_from_lidar, body_frame=False):
    T_lidar_cam = np.linalg.inv(T_cam_from_lidar)
    R = T_lidar_cam[:3, :3]
    if body_frame:
        R = R @ R_BODY_OPTICAL.T
    r, p, y = bs.mat_to_rpy(R)
    return [*map(float, T_lidar_cam[:3, 3]), math.degrees(r), math.degrees(p), math.degrees(y)]


# ======================================================================= scoring and drawing

def image_edge_distance(img):
    """Distance (px) from every pixel to the nearest Canny edge of the image."""
    import cv2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    med = float(np.median(gray))
    edges = cv2.Canny(gray, int(max(0, 0.66 * med)), int(min(255, 1.33 * med + 20)))
    return cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 5)


def _pixel_index(uv, shape):
    h, w = shape[:2]
    ui = np.clip(uv[:, 0].astype(int), 0, w - 1)
    vi = np.clip(uv[:, 1].astype(int), 0, h - 1)
    return vi * w + ui


def visible_points(uv, depth, shape, win=5, margin_abs=0.15, margin_rel=0.05):
    """Mask of points not hidden behind a nearer point in a small pixel window (z-buffer).

    The camera sits away from the lidar, so points the lidar saw behind an
    object still project onto that object in the image (parallax bleed-through).
    """
    import cv2
    h, w = shape[:2]
    flat = _pixel_index(uv, shape)
    near = np.full(h * w, 1e6, np.float32)
    np.minimum.at(near, flat, depth.astype(np.float32))
    near = cv2.erode(near.reshape(h, w), np.ones((win, win), np.uint8)).ravel()
    return depth <= near[flat] + np.maximum(margin_abs, margin_rel * depth)


def depth_edge_points(uv, depth, shape, win=None, jump_abs=0.25, jump_rel=0.08):
    """Mask of (visible) projected points on EITHER side of a depth discontinuity.

    Near-side points have a much farther visible point within the window; far-side
    points have a much nearer one. Taking both makes the edge band symmetric about
    the true boundary. Near side alone puts the whole band inside the object,
    which biased the sweep's optimum by ~half a band width. The window defaults to
    the visibility window + 4 px (both scale with the mean point spacing): any
    smaller and the far points just behind an edge are all hidden by the z-buffer.
    """
    import cv2
    h, w = shape[:2]
    spacing = math.sqrt(h * w / max(len(uv), 1))
    vis_win = int(min(9, max(3, round(1.5 * spacing)))) | 1
    vis = visible_points(uv, depth, shape, win=vis_win)
    if win is None:
        win = vis_win + 4   # must reach past the strip the visibility test hides behind an edge
    flat = _pixel_index(uv, shape)
    k = np.ones((win, win), np.uint8)
    far = np.zeros(h * w, np.float32)
    np.maximum.at(far, flat[vis], depth[vis].astype(np.float32))
    far = cv2.dilate(far.reshape(h, w), k).ravel()                 # max visible depth nearby
    near = np.full(h * w, 1e6, np.float32)
    np.minimum.at(near, flat[vis], depth[vis].astype(np.float32))
    near = cv2.erode(near.reshape(h, w), k).ravel()                # min visible depth nearby
    thr_near = np.maximum(jump_abs, jump_rel * depth)
    near_side = (far[flat] - depth) > thr_near
    nd = near[flat]
    far_side = (depth - nd) > np.maximum(jump_abs, jump_rel * nd)
    return vis & (near_side | far_side)


def _bilinear(img, uv):
    h, w = img.shape[:2]
    u = np.clip(uv[:, 0], 0, w - 1.001)
    v = np.clip(uv[:, 1], 0, h - 1.001)
    u0, v0 = u.astype(int), v.astype(int)
    fu, fv = u - u0, v - v0
    return ((1 - fu) * (1 - fv) * img[v0, u0] + fu * (1 - fv) * img[v0, u0 + 1] +
            (1 - fu) * fv * img[v0 + 1, u0] + fu * fv * img[v0 + 1, u0 + 1])


def edge_score(uv, depth, dist_img, clip=10.0):
    """score_px: mean distance (px, clipped) from depth-edge points to image edges.

    Mean of a bilinearly sampled, clipped distance is smooth in the extrinsic, so
    the sweep can follow it; median_px is the easier-to-read summary.
    """
    if len(uv) == 0:
        return {'score_px': float('nan'), 'median_px': float('nan'), 'edge_points': 0}
    m = depth_edge_points(uv, depth, dist_img.shape)
    if m.sum() < 20:
        return {'score_px': float('nan'), 'median_px': float('nan'), 'edge_points': int(m.sum())}
    d = np.minimum(_bilinear(dist_img, uv[m]), clip)
    return {'score_px': float(d.mean()), 'median_px': float(np.median(d)), 'edge_points': int(m.sum())}


def render(img, uv, depth, edge_mask=None, dmin=0.3, dmax=8.0, radius=1):
    import cv2
    base = img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    out = (base * 0.55).astype(np.uint8)
    if len(uv) == 0:
        return out
    spacing = math.sqrt(out.shape[0] * out.shape[1] / max(len(uv), 1))
    vis = visible_points(uv, depth, out.shape, win=int(min(9, max(3, round(1.5 * spacing)))) | 1)
    if edge_mask is not None:
        edge_mask = edge_mask & vis
    uv, depth = uv[vis], depth[vis]
    edge_mask = edge_mask[vis] if edge_mask is not None else None
    t = (np.log(np.clip(depth, dmin, dmax)) - math.log(dmin)) / (math.log(dmax) - math.log(dmin))
    colors = cv2.applyColorMap((255 * (1 - t)).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
    order = np.argsort(-depth)  # far first, near drawn on top
    ui, vi = uv[order, 0].astype(int), uv[order, 1].astype(int)
    c = colors[order]
    h, w = out.shape[:2]
    for dy in range(-radius + 1, radius + 1):
        for dx in range(-radius + 1, radius + 1):
            out[np.clip(vi + dy, 0, h - 1), np.clip(ui + dx, 0, w - 1)] = c
    if edge_mask is not None and edge_mask.any():
        out[uv[edge_mask, 1].astype(int), uv[edge_mask, 0].astype(int)] = (255, 255, 255)
    return out


# ======================================================================= bag pass

def load_report_offsets(path, image_topic, lidar_topic):
    with open(path) as fh:
        rep = json.load(fh)
    t = rep.get('topics', {})
    get = lambda n: (t.get(n) or {}).get('offset_ms', 0.0) or 0.0  # noqa: E731
    return get(image_topic) * 1e-3, get(lidar_topic) * 1e-3


def collect(bag, image_topic, lidar_topic, info_topic, accumulate, cam_off, lidar_off,
            max_images, every_s, start_s):
    types = bs.topic_types(bag)
    for tp in (image_topic, lidar_topic):
        if tp not in types:
            raise SystemExit(f'{tp} not in bag. Topics: {sorted(types)}')
    # pass 1: stamps only
    img_t, lid_t = [], []
    for topic, typ, stamp, _recv, data in bs.iter_messages(bag, [image_topic, lidar_topic]):
        if topic == image_topic:
            img_t.append(stamp - cam_off)
        else:
            lid_t.append(stamp - lidar_off)
    img_t, lid_t = np.array(img_t), np.array(lid_t)
    if not len(img_t) or not len(lid_t):
        raise SystemExit('no images or no lidar frames in the bag')
    lid_period = float(np.median(np.diff(lid_t))) if len(lid_t) > 1 else 0.1
    lid_c = lid_t + 0.5 * lid_period   # sweep middle
    t0 = img_t[0] + start_s
    chosen, last = [], -1e18
    for i, t in enumerate(img_t):
        if t < t0 or t - last < every_s:
            continue
        near = np.abs(lid_c - t) <= max(accumulate / 2, 0.6 * lid_period)
        if near.any():
            chosen.append(i)
            last = t
        if len(chosen) >= max_images:
            break
    if not chosen:
        raise SystemExit('no image has lidar frames within the accumulation window (check --xcorr-report offsets)')
    want_img = set(chosen)
    windows = [(img_t[i] - max(accumulate / 2, 0.6 * lid_period),
                img_t[i] + max(accumulate / 2, 0.6 * lid_period)) for i in chosen]
    # pass 2: decode only what's needed
    topics = [image_topic, lidar_topic] + ([info_topic] if info_topic else [])
    images, clouds, info, cloud_frame, image_frame = {}, [], None, None, None
    ii = li = 0
    for topic, typ, stamp, _recv, data in bs.iter_messages(bag, topics):
        if topic == info_topic:
            if info is None:
                info = bs.deserialize(data, typ)
            continue
        if topic == image_topic:
            if ii in want_img:
                images[ii] = (img_t[ii], bs.decode_image(data, typ))
                image_frame = image_frame or bs.frame_id(data)
            ii += 1
        else:
            tc = lid_c[li]
            li += 1
            if any(a <= tc <= b for a, b in windows):
                xyz, _ = bs.cloud_xyz(data, typ)
                clouds.append((tc, xyz))
                cloud_frame = cloud_frame or bs.frame_id(data)
    frames = []
    for i in chosen:
        t, img = images[i]
        a, b = windows[chosen.index(i)]
        pts = [xyz for tc, xyz in clouds if a <= tc <= b]
        frames.append({'t': float(t), 'image': img, 'points': np.vstack(pts) if pts else np.zeros((0, 3)),
                       'lidar_frames': len(pts)})
    return frames, info, cloud_frame, image_frame


# ======================================================================= main

def evaluate(frame, cam, T, dist_img=None):
    pc = frame['points'] @ T[:3, :3].T + T[:3, 3]
    uv, depth, _ = cam.project(pc)
    if dist_img is None:
        dist_img = image_edge_distance(frame['image'])
    return uv, depth, edge_score(uv, depth, dist_img)


def mean_score(frames, cam, T, dists):
    s = [evaluate(f, cam, T, d)[2]['score_px'] for f, d in zip(frames, dists)]
    s = [v for v in s if np.isfinite(v)]
    return float(np.mean(s)) if s else float('nan')


def sweep(frames, cam, params, body_frame, dists, rounds=3):
    steps = {'x': np.arange(-0.04, 0.0401, 0.005), 'y': np.arange(-0.04, 0.0401, 0.005),
             'z': np.arange(-0.04, 0.0401, 0.005), 'roll': np.arange(-1.5, 1.501, 0.1),
             'pitch': np.arange(-1.5, 1.501, 0.1), 'yaw': np.arange(-1.5, 1.501, 0.1)}
    best = list(params)
    curves = {}
    for _ in range(rounds):
        for k, name in sorted(enumerate(PARAM_NAMES), key=lambda kn: -kn[0]):  # rotations first
            vals = []
            for d in steps[name]:
                p = list(best)
                p[k] += d
                vals.append(mean_score(frames, cam, params_to_T(p, body_frame), dists))
            vals = np.array(vals)
            if np.isfinite(vals).any():
                j = int(np.nanargmin(vals))
                best[k] += steps[name][j]
                curves[name] = (steps[name], vals)
    return best, curves


def fmt_params(p):
    return (f'xyz = {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} m   rpy = {p[3]:.3f} {p[4]:.3f} {p[5]:.3f} deg  '
            f'(URDF rpy rad: {math.radians(p[3]):.5f} {math.radians(p[4]):.5f} {math.radians(p[5]):.5f})')


def interactive(frame, cam, params, body_frame):
    import cv2
    dist_img = image_edge_distance(frame['image'])
    p = list(params)
    ang, lin = 0.1, 0.005
    keys = {ord('q'): (3, 1), ord('a'): (3, -1), ord('w'): (4, 1), ord('s'): (4, -1),
            ord('e'): (5, 1), ord('d'): (5, -1), ord('r'): (0, 1), ord('f'): (0, -1),
            ord('t'): (1, 1), ord('g'): (1, -1), ord('y'): (2, 1), ord('h'): (2, -1)}
    while True:
        uv, depth, sc = evaluate(frame, cam, params_to_T(p, body_frame), dist_img)
        em = depth_edge_points(uv, depth, frame['image'].shape) if len(uv) else None
        vis = render(frame['image'], uv, depth, em)
        cv2.putText(vis, f'score {sc["score_px"]:.2f}px  step {ang:.2f}deg/{lin * 1000:.0f}mm', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, fmt_params(p)[:80], (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.imshow('lidar_camera_overlay', vis)
        k = cv2.waitKey(0) & 0xFF
        if k == 27:
            break
        if k == ord('p'):
            print(fmt_params(p))
        elif k == ord('['):
            ang, lin = ang / 2, lin / 2
        elif k == ord(']'):
            ang, lin = ang * 2, lin * 2
        elif k in keys:
            i, s = keys[k]
            p[i] += s * (ang if i >= 3 else lin)
    cv2.destroyAllWindows()
    print('final: ' + fmt_params(p))
    return p


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--image', required=True, help='Image or CompressedImage topic')
    ap.add_argument('--lidar', default='/livox/lidar', help='CustomMsg or PointCloud2 topic')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--camera-info', help='CameraInfo topic in the bag')
    src.add_argument('--intrinsics', help='ost.yaml from camera_calibration, or JSON')
    ext = ap.add_mutually_exclusive_group(required=True)
    ext.add_argument('--extrinsic', type=float, nargs=6, metavar=('X', 'Y', 'Z', 'ROLL', 'PITCH', 'YAW'),
                     help='camera pose in the lidar frame (m, deg)')
    ext.add_argument('--tf', action='store_true', help='chain the transform from /tf_static in the bag')
    ap.add_argument('--body-frame', action='store_true', help='--extrinsic is for a REP-103 body frame')
    ap.add_argument('--camera-frame', help='with --tf: override the image header frame_id')
    ap.add_argument('--accumulate', type=float, default=1.0, help='seconds of lidar per image (default 1.0)')
    ap.add_argument('--xcorr-report', help='report.json from time_offset_xcorr.py (applies both offsets)')
    ap.add_argument('--cam-offset-ms', type=float, default=0.0)
    ap.add_argument('--lidar-offset-ms', type=float, default=0.0)
    ap.add_argument('--max-images', type=int, default=8)
    ap.add_argument('--every', type=float, default=3.0, help='min seconds between chosen images')
    ap.add_argument('--start', type=float, default=0.0, help='skip this many seconds of the bag')
    ap.add_argument('--sweep', action='store_true', help='per-axis search around the extrinsic')
    ap.add_argument('--interactive', action='store_true')
    ap.add_argument('--out', required=True)
    args = ap.parse_args(argv)

    cam_off, lid_off = args.cam_offset_ms * 1e-3, args.lidar_offset_ms * 1e-3
    if args.xcorr_report:
        cam_off, lid_off = load_report_offsets(args.xcorr_report, args.image, args.lidar)
    print(f'time offsets applied: camera {cam_off * 1e3:.1f} ms, lidar {lid_off * 1e3:.1f} ms')

    frames, info, cloud_frame, image_frame = collect(
        args.bag, args.image, args.lidar, args.camera_info, args.accumulate, cam_off, lid_off,
        args.max_images, args.every, args.start)
    if args.camera_info:
        if info is None:
            raise SystemExit(f'no CameraInfo on {args.camera_info}')
        cam = Camera.from_camera_info(info)
    else:
        cam = Camera.from_file(args.intrinsics)
    h, w = frames[0]['image'].shape[:2]
    if (w, h) != (cam.width, cam.height):
        raise SystemExit(f'image is {w}x{h} but intrinsics are for {cam.width}x{cam.height}')

    body = args.body_frame
    if args.tf:
        cf = args.camera_frame or image_frame
        T = bs.lookup(bs.static_tf_tree(args.bag), cf, cloud_frame)
        if body:
            # cf is a REP-103 body frame (e.g. a URDF camera_link): rotate into optical
            T = bs.make_T(R_BODY_OPTICAL.T, [0, 0, 0]) @ T
        params = T_to_params(T, body)
        print(f'TF {cloud_frame} -> {cf}{" (body frame, optical rotation appended)" if body else ""}')
    else:
        params = list(args.extrinsic)
        T = params_to_T(params, body)
    print(f'cloud frame {cloud_frame!r}, image frame {image_frame!r}; extrinsic: {fmt_params(params)}')

    os.makedirs(args.out, exist_ok=True)
    dists = [image_edge_distance(f['image']) for f in frames]
    import cv2
    summary = {'bag': os.path.abspath(args.bag), 'image_topic': args.image, 'lidar_topic': args.lidar,
               'cloud_frame': cloud_frame, 'image_frame': image_frame, 'body_frame': body,
               'extrinsic': dict(zip(PARAM_NAMES, params)), 'offsets_ms': [cam_off * 1e3, lid_off * 1e3],
               'images': []}
    for i, (f, d) in enumerate(zip(frames, dists)):
        uv, depth, sc = evaluate(f, cam, T, d)
        em = depth_edge_points(uv, depth, f['image'].shape) if len(uv) else None
        p = os.path.join(args.out, f'overlay_{i:02d}.png')
        cv2.imwrite(p, render(f['image'], uv, depth, em))
        row = {'file': os.path.basename(p), 't': f['t'], 'lidar_frames': f['lidar_frames'],
               'points': int(len(f['points'])), 'in_image': int(len(uv)), **sc}
        summary['images'].append(row)
        if len(f['points']) and len(uv) < 0.01 * len(f['points']):
            print(f'  WARNING: only {len(uv)} of {len(f["points"])} points land in the image: wrong frame '
                  f'convention (--body-frame?), wrong sign, or the camera faces away from the cloud')
        print(f'  {row["file"]}: {row["lidar_frames"]} lidar frames, {row["in_image"]} pts in view, '
              f'score {sc["score_px"]:.2f} px (median {sc["median_px"]:.2f}) over {sc["edge_points"]} edge pts')
    summary['mean_score_px'] = mean_score(frames, cam, T, dists)
    print(f'mean score {summary["mean_score_px"]:.2f} px (white = depth-edge points used for the score)')

    if args.sweep:
        best, curves = sweep(frames, cam, params, body, dists)
        best_score = mean_score(frames, cam, params_to_T(best, body), dists)
        summary['sweep'] = {'best': dict(zip(PARAM_NAMES, best)), 'best_score_px': best_score,
                            'delta': {n: b - a for n, a, b in zip(PARAM_NAMES, params, best)}}
        print(f'sweep best ({best_score:.2f} px): {fmt_params(best)}')
        print('  delta: ' + ', '.join(f'{n} {b - a:+.3f}{"m" if k < 3 else "deg"}'
                                      for k, (n, a, b) in enumerate(zip(PARAM_NAMES, params, best))))
        print('  (resolution floor ~0.3 deg pitch/yaw, ~0.6 deg roll, ~3 cm, translation along the view axis'
              ' unconstrained: smaller deltas are noise,'
              ' not corrections)')
        plot_sweep(curves, os.path.join(args.out, 'sweep.png'))
    if args.interactive:
        final = interactive(frames[0], cam, params, body)
        summary['interactive'] = dict(zip(PARAM_NAMES, final))
    with open(os.path.join(args.out, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=2)
    print(f'wrote {args.out}/summary.json')
    return summary


def plot_sweep(curves, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(12, 6))
    for ax, name in zip(axes.ravel(), PARAM_NAMES):
        if name in curves:
            x, y = curves[name]
            ax.plot(x * (1000 if name in 'xyz' else 1), y, '.-')
        ax.set_title(name)
        ax.set_xlabel('mm' if name in ('x', 'y', 'z') else 'deg')
        ax.set_ylabel('score px')
        ax.grid(alpha=0.3)
    fig.suptitle('edge score vs per-axis perturbation (last round). Minima within ~0.3 deg / 3 cm of 0 = no evidence of error')
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    print(f'wrote {path}')


if __name__ == '__main__':
    main()
