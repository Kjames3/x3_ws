"""side_person_rays.py -- left/right mono side-camera person boxes -> bearing-only rays.

The side cameras (left Innomaker OV9281, right Arducam OV9281; fisheye, mono) have no
depth.  A person box gives a direction from the camera centre; livox_scan_node walks
that ray through the Livox cloud (person_candidates.ray_candidates) and the body it
finds is both a tracker candidate and the camera anchor (X3_SIDE_PERSONS, server_x3).

Geometry:
  * intrinsics: equidistant fisheye K, D fitted at 1280x800 (config/*_intrinsics.json,
    "fisheye"); K is scaled to the frame actually captured (left runs binned 640x400),
    D is unchanged.  The left fit is on 180-deg-rotated (upright) frames, which is what
    server_x3 captures, so frames are used as they are.
  * extrinsics: camera OPTICAL frame in base_footprint, from /tf_static (the URDF frames
    left_cam_optical_frame / arducam_optical_frame, which tests/test_x3_v2_urdf.py pins to
    config/camera_lidar_extrinsics_2026-11-20.json), or that JSON composed with
    base_footprint <- livox_frame when only the Livox chain is on /tf_static.
  * a ray row is RAY_IN_FIELDS of person_candidates: camera centre (odom XY, height above
    the floor), horizontal unit bearing, a search window along it and tan(elevation) of
    the box top (NaN when the box touches the image top).

Range window from the box (coarse, used only to narrow the search):
  * feet: when the box bottom is inside the image and below the horizon, the floor
    contact is at s = oz / -tan(bottom); the window is 0.6 s .. 1.6 s + 0.3 (camera pitch
    error and partly hidden feet).  The cameras are ~0.2 m up, so 1 deg of pitch is
    ~25% of range at 3 m.
  * height: a person is at most ~2 m tall, so s <= 2.1 / (tan(top) - tan(bottom)) + 0.3.
    This holds even when the legs are hidden (the box is then shorter, the bound looser).
  * no lower bound from the height: a partly hidden person makes the box short.

Pure numpy + cv2, no ROS, so tests and the offline tools can import it.
"""
import json
import math
import pathlib

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
EXTRINSICS = ROOT / 'config' / 'camera_lidar_extrinsics_2026-11-20.json'
CAMERAS = {
    'left': {'intrinsics': ROOT / 'config' / 'left_u20cam_intrinsics.json',
             'frame': 'left_cam_optical_frame'},
    'right': {'intrinsics': ROOT / 'config' / 'arducam_uc844_intrinsics.json',
              'frame': 'arducam_optical_frame'},
}
RANGE_M = (0.4, 6.0)          # == person_candidates.RAY_RANGE_M
EDGE_PX = 3                   # a box edge this close to the image edge is cut off
FOOT_MIN_TAN = 0.02           # bottom ray at least this far below the horizon to use the feet
FOOT_WINDOW = (0.6, 1.6, 0.3)  # lo factor, hi factor, hi pad (m)
MAX_PERSON_H_M = 2.1
HEIGHT_PAD_M = 0.3
MIN_BOX_H_PX = 16             # at the captured resolution


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_camera(role, extrinsics_path=EXTRINSICS):
    """{'K', 'D', 'wh' (calibration size), 'frame', 'R_lc', 't_lc'} for 'left' / 'right'.
    R_lc, t_lc: the optical frame in livox_frame (p_lidar = R p_cam + t)."""
    spec = CAMERAS[role]
    fe = json.loads(pathlib.Path(spec['intrinsics']).read_text())['fisheye']
    v = json.loads(pathlib.Path(extrinsics_path).read_text())[role]['T_lidar_camera']
    return {'K': np.array(fe['K'], dtype=np.float64), 'D': np.array(fe['D'], dtype=np.float64).reshape(4),
            'wh': (int(fe['width']), int(fe['height'])), 'frame': spec['frame'],
            'R_lc': quat_to_R(*v[3:7]), 't_lc': np.array(v[:3], dtype=np.float64)}


def scaled_K(K, calib_wh, frame_wh):
    """K for a frame of another size (binning / resize of the same full field of view)."""
    sx, sy = frame_wh[0] / calib_wh[0], frame_wh[1] / calib_wh[1]
    K = np.array(K, dtype=np.float64).copy()
    K[0, 0] *= sx
    K[0, 2] = (K[0, 2] + 0.5) * sx - 0.5
    K[1, 1] *= sy
    K[1, 2] = (K[1, 2] + 0.5) * sy - 0.5
    return K


def pixel_dirs(K, D, uv):
    """(N, 2) pixels -> (N, 3) unit directions in the optical frame (x right, y down, z fwd)."""
    import cv2
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 1, 2)
    xy = cv2.fisheye.undistortPoints(uv, np.asarray(K, np.float64), np.asarray(D, np.float64)).reshape(-1, 2)
    d = np.c_[xy, np.ones(len(xy))]
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def camera_in_base(cam, static_chain):
    """(R, t) of the camera optical frame in base_footprint, or None.

    static_chain(frame) -> (R, t) of `frame` in base_footprint from /tf_static, or None.
    Prefers the URDF optical frame; falls back to the JSON extrinsic on livox_frame."""
    ext = static_chain(cam['frame'])
    if ext is not None:
        return ext
    lv = static_chain('livox_frame')
    if lv is None:
        return None
    R_bl, t_bl = lv
    return R_bl @ cam['R_lc'], R_bl @ cam['t_lc'] + t_bl


def range_window(oz, top_tan, bot_tan):
    """(s_lo, s_hi, kind) along the horizontal bearing; kind 'feet', 'height' or 'none'."""
    lo, hi = RANGE_M
    kind = 'none'
    if math.isfinite(bot_tan) and bot_tan < -FOOT_MIN_TAN and oz > 0:
        s = oz / -bot_tan
        f_lo, f_hi, pad = FOOT_WINDOW
        lo, hi = max(lo, f_lo * s), min(hi, f_hi * s + pad)
        kind = 'feet'
    if math.isfinite(top_tan) and math.isfinite(bot_tan) and top_tan - bot_tan > 1e-3:
        h = MAX_PERSON_H_M / (top_tan - bot_tan) + HEIGHT_PAD_M
        if h < hi:
            hi = h
            kind = 'feet+height' if kind == 'feet' else 'height'
    if hi <= lo:              # inconsistent box (e.g. a cut-off one): search the whole ray
        return RANGE_M[0], RANGE_M[1], 'none'
    return lo, hi, kind


def detection_ray(box, frame_wh, K, D, R_bc, t_bc, pose):
    """One person box -> a ray (dict with RAY_IN_FIELDS keys plus diagnostics), or None.

    box: (x1, y1, x2, y2) pixels of the captured frame; K already scaled to frame_wh.
    R_bc, t_bc: optical frame in base_footprint.  pose: robot {'x', 'y', 'theta'} in odom
    at the capture time."""
    x1, y1, x2, y2 = (float(v) for v in box[:4])
    w, h = frame_wh
    if y2 - y1 < MIN_BOX_H_PX or x2 <= x1:
        return None
    uc = 0.5 * (x1 + x2)
    d = pixel_dirs(K, D, [(uc, 0.5 * (y1 + y2)), (uc, y1), (uc, y2)])
    c, s = math.cos(pose['theta']), math.sin(pose['theta'])
    R_ob = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    do = (R_ob @ R_bc @ d.T).T                      # odom axes (z up)
    o = R_ob @ np.asarray(t_bc, dtype=np.float64) + [pose['x'], pose['y'], 0.0]
    hz = np.hypot(do[:, 0], do[:, 1])
    if hz[0] < 1e-6:
        return None
    tans = do[:, 2] / np.maximum(hz, 1e-9)
    top_tan = float(tans[1]) if y1 > EDGE_PX else float('nan')
    bot_tan = float(tans[2]) if y2 < h - 1 - EDGE_PX else float('nan')
    lo, hi, kind = range_window(float(o[2]), top_tan, bot_tan)
    return {'ox': float(o[0]), 'oy': float(o[1]), 'oz': float(o[2]),
            'ux': float(do[0, 0] / hz[0]), 'uy': float(do[0, 1] / hz[0]),
            's_lo': float(lo), 's_hi': float(hi), 'top_tan': top_tan,
            'bot_tan': bot_tan, 'prior': kind}


# == person_candidates.RAY_IN_FIELDS (tests/test_side_person_rays.py checks), duplicated so
# the server does not need the colcon-built package on its path for this.
RAY_IN_FIELDS = ('ox', 'oy', 'oz', 'ux', 'uy', 's_lo', 's_hi', 'top_tan')
# /side/<role>/person_rays (PointCloud2, frame odom, stamped at the image capture time):
# one row per ready person id, float32.
RAY_MSG_FIELDS = RAY_IN_FIELDS + ('tid',)


def ray_row(ray):
    return [ray[k] for k in RAY_IN_FIELDS]


def _iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


class SideTrackIds:
    """Per-camera ids for person boxes across frames: greedy IoU matching.

    An id is reported (ready) once it has been seen in min_hits frames (gaps up to
    max_miss_s allowed), so a one-frame false detection never reaches the Livox.  A track unmatched for
    max_miss_s is dropped; a box with no match starts a new id."""

    def __init__(self, iou_min=0.2, max_miss_s=0.6, min_hits=2):
        self.iou_min, self.max_miss_s, self.min_hits = iou_min, max_miss_s, min_hits
        self.next_id = 1
        self.tracks = {}          # id -> {'box', 't', 'hits'}

    def update(self, boxes, t):
        """boxes: list of (x1, y1, x2, y2).  Returns [(id, hits, ready)] aligned with boxes."""
        self.tracks = {k: v for k, v in self.tracks.items() if t - v['t'] <= self.max_miss_s}
        pairs = sorted(((_iou(b, tr['box']), i, k) for i, b in enumerate(boxes)
                        for k, tr in self.tracks.items()), reverse=True)
        out = [None] * len(boxes)
        used = set()
        for iou, i, k in pairs:
            if iou < self.iou_min:
                break
            if out[i] is not None or k in used:
                continue
            tr = self.tracks[k]
            tr['box'], tr['t'], tr['hits'] = tuple(boxes[i]), t, tr['hits'] + 1
            out[i] = k
            used.add(k)
        for i, b in enumerate(boxes):
            if out[i] is None:
                k = self.next_id
                self.next_id += 1
                self.tracks[k] = {'box': tuple(b), 't': t, 'hits': 1}
                out[i] = k
        return [(k, self.tracks[k]['hits'], self.tracks[k]['hits'] >= self.min_hits) for k in out]
