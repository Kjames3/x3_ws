"""Low floor obstacles (power strips, thick cords, low bars) from the front OAK depth.

The Livox low band (/livox/obstacles) starts at 0.06 m and the lidar's lowest ray
only reaches a 5 cm object from ~2 m out, so a power strip is invisible to Nav2.
Front-OAK stereo depth sees it: sigma ~0.0025*Z^2 is 6 mm at 1.5 m.

Pipeline, ~5 Hz on a daemon thread (pure numpy, ~a few ms):
  depth (CAM_A-aligned, metres) -> optical-frame points (stride 4, 0.3..range_max)
  -> odom via TF -> height above floor from base_footprint
  -> voxels with z_min < h < z_max, outside the chassis box
  -> odom-frame memory -> /oak/obstacles (PointCloud2, odom), mark-only costmap source.

Memory: once the robot is close, the strip leaves the camera's view, and /scan
raytracing would clear its costmap cells.  A remembered voxel is forgotten only
when it projects inside the image, in the trusted depth range, is NOT occluded
(the depth there is not much shorter), and was not seen this frame; or after
memory_s; or when it is far from the robot.

X3_OAK_FLOOR_FAST=1 (read when OakFloorObstacles is built; default off) selects a
prototype path with the same outputs and no BLAS/LAPACK calls: closed-form plane
fit from centred sums instead of lstsq, elementwise rigid transforms instead of
`@`, packed int64 voxel keys instead of np.unique(axis=0), and an array-backed
voxel memory instead of a per-point Python dict loop.  See
tests/test_oak_floor_fast.py and src/tools/perf/oak_floor_bench.py.
"""
import logging
import math
import os
import threading
import time

import numpy as np

import oakd_cloud

logger = logging.getLogger(__name__)


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FloorObstacleMemory:
    """Odom-frame voxel memory with view-based forgetting.  ROS-free, testable."""

    def __init__(self, cell=0.04, memory_s=30.0, keep_radius=4.0):
        self.cell, self.memory_s, self.keep_radius = cell, memory_s, keep_radius
        self.vox = {}        # key -> [x, y, z, last_seen]

    def update(self, pts_odom, now, robot_xy, visible_fn=None):
        seen = set()
        c = self.cell
        for x, y, z in pts_odom:
            k = (int(math.floor(x / c)), int(math.floor(y / c)), int(math.floor(z / c)))
            seen.add(k)
            self.vox[k] = [float(x), float(y), float(z), now]
        if not self.vox:
            return
        keys = [k for k in self.vox if k not in seen]
        if not keys:
            return
        arr = np.array([self.vox[k] for k in keys])
        far = np.hypot(arr[:, 0] - robot_xy[0], arr[:, 1] - robot_xy[1]) > self.keep_radius
        old = now - arr[:, 3] > self.memory_s
        vis = visible_fn(arr[:, :3]) if visible_fn is not None else np.zeros(len(keys), bool)
        for k, drop in zip(keys, far | old | vis):
            if drop:
                del self.vox[k]

    def points(self):
        if not self.vox:
            return np.zeros((0, 3), np.float32)
        return np.array([v[:3] for v in self.vox.values()], np.float32)

    def __len__(self):
        return len(self.vox)


# Fixed bit fields for odom-frame voxel keys that must stay stable across frames:
# 21 bits per axis is +-2^20 cells, i.e. +-41.9 km at 4 cm.
_KEY_BITS = 21
_KEY_OFF = 1 << (_KEY_BITS - 1)


def _pack_fixed(ik):
    """(N,3) int64 cell indices -> (N,) int64 keys, or None if any is out of range."""
    if len(ik) and (ik.min() < -_KEY_OFF or ik.max() >= _KEY_OFF):
        return None
    o = ik + _KEY_OFF
    return (o[:, 0] << (2 * _KEY_BITS)) | (o[:, 1] << _KEY_BITS) | o[:, 2]


def _pack_frame(ik):
    """(N,3) int64 -> (N,) int64 keys unique within this array (per-frame min/span)."""
    o = ik - ik.min(axis=0)
    sy = int(o[:, 1].max()) + 1
    sz = int(o[:, 2].max()) + 1
    return (o[:, 0] * sy + o[:, 1]) * sz + o[:, 2]


class FastFloorObstacleMemory:
    """Array-backed FloorObstacleMemory: same voxels and values, no per-point Python.

    Falls back to a FloorObstacleMemory if a key ever leaves the packed range.
    """

    def __init__(self, cell=0.04, memory_s=30.0, keep_radius=4.0):
        self.cell, self.memory_s, self.keep_radius = cell, memory_s, keep_radius
        self._keys = np.zeros(0, np.int64)       # packed cell keys, unique
        self._vals = np.zeros((0, 4))            # x, y, z, last_seen
        self._slow = None

    def _to_slow(self):
        slow = FloorObstacleMemory(self.cell, self.memory_s, self.keep_radius)
        c = self.cell
        for x, y, z, t in self._vals:
            k = (int(math.floor(x / c)), int(math.floor(y / c)), int(math.floor(z / c)))
            slow.vox[k] = [float(x), float(y), float(z), float(t)]
        self._slow = slow

    def update(self, pts_odom, now, robot_xy, visible_fn=None):
        if self._slow is not None:
            return self._slow.update(pts_odom, now, robot_xy, visible_fn)
        pts = np.asarray(pts_odom)
        if len(pts):
            packed = _pack_fixed(np.floor(pts / self.cell).astype(np.int64))
            if packed is None:
                self._to_slow()
                return self._slow.update(pts_odom, now, robot_xy, visible_fn)
            # Last occurrence wins, like dict assignment: unique over the reversed keys.
            n = len(packed)
            new_keys, first_rev = np.unique(packed[::-1], return_index=True)
            last = n - 1 - first_rev
            new_vals = np.empty((len(new_keys), 4))
            new_vals[:, :3] = pts[last]
            new_vals[:, 3] = now
        else:
            new_keys, new_vals = np.zeros(0, np.int64), np.zeros((0, 4))
        if len(self._keys) == 0:
            self._keys, self._vals = new_keys, new_vals
            return
        unseen = ~np.isin(self._keys, new_keys, assume_unique=True)
        old_k, old_v = self._keys[unseen], self._vals[unseen]
        if len(old_k):
            far = np.hypot(old_v[:, 0] - robot_xy[0], old_v[:, 1] - robot_xy[1]) > self.keep_radius
            old = now - old_v[:, 3] > self.memory_s
            vis = visible_fn(old_v[:, :3]) if visible_fn is not None \
                else np.zeros(len(old_k), bool)
            keep = ~(far | old | vis)
            old_k, old_v = old_k[keep], old_v[keep]
        self._keys = np.concatenate((old_k, new_keys))
        self._vals = np.concatenate((old_v, new_vals))

    def points(self):
        if self._slow is not None:
            return self._slow.points()
        return self._vals[:, :3].astype(np.float32)

    def __len__(self):
        return len(self._slow) if self._slow is not None else len(self._keys)


def min_returns_per_voxel(cand, cell, fast=False, n_min=2):
    """Rows of `cand` whose cell holds >= n_min rows (order preserved)."""
    if not len(cand):
        return cand
    keys = np.floor(cand / cell).astype(np.int64)
    if fast:
        _, inv, cnt = np.unique(_pack_frame(keys), return_inverse=True, return_counts=True)
    else:
        _, inv, cnt = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return cand[cnt[inv.ravel()] >= n_min]


def rigid(p, M, t=None, fast=False):
    """p @ M.T (+ t).  fast: elementwise, no BLAS (not bit-identical to dgemm)."""
    if not fast:
        out = p @ M.T
        if t is not None:
            out += t
        return out
    q = np.asarray(p).T.astype(np.float64)     # (3, N) contiguous rows
    out = np.empty((q.shape[1], 3))
    for j in range(3):
        r = q[0] * M[j, 0]
        r += q[1] * M[j, 1]
        r += q[2] * M[j, 2]
        if t is not None:
            r += t[j]
        out[:, j] = r
    return out


def _fit_lstsq(x, y, z):
    A = np.column_stack((x, y, np.ones(len(x))))
    coef = np.linalg.lstsq(A, z, rcond=None)[0]
    for _ in range(2):
        r = z - A @ coef
        keep = np.abs(r) < max(0.015, 2.5 * np.median(np.abs(r)))
        if keep.sum() < 200:
            return None
        coef = np.linalg.lstsq(A[keep], z[keep], rcond=None)[0]
    return coef


def _lsq_plane(x, y, z):
    """Least-squares z = a*x + b*y + c from centred sums; no BLAS/LAPACK."""
    mx, my, mz = x.mean(), y.mean(), z.mean()
    dx, dy, dz = x - mx, y - my, z - mz
    sxx, syy, sxy = (dx * dx).sum(), (dy * dy).sum(), (dx * dy).sum()
    sxz, syz = (dx * dz).sum(), (dy * dz).sum()
    det = sxx * syy - sxy * sxy
    # Degenerate (x or y spread < ~1 um, or x, y collinear): lstsq's min-norm answer.
    tiny = 1e-12 * len(x)
    if not (sxx > tiny and syy > tiny and det > 1e-9 * sxx * syy):
        A = np.column_stack((x, y, np.ones(len(x))))
        return np.linalg.lstsq(A, z, rcond=None)[0]
    a = (sxz * syy - syz * sxy) / det
    b = (syz * sxx - sxz * sxy) / det
    return np.array([a, b, mz - a * mx - b * my])


def _fit_fast(x, y, z):
    coef = _lsq_plane(x, y, z)
    for _ in range(2):
        a, b, c = coef
        ar = np.abs(z - (a * x + b * y + c))
        keep = ar < max(0.015, 2.5 * np.median(ar))
        if keep.sum() < 200:
            return None
        coef = _lsq_plane(x[keep], y[keep], z[keep])
    return coef


def fit_floor_plane(p_base, fast=False):
    """(a, b, c) of z = a*x + b*y + c fitted to the floor in base_footprint coords, or None.

    Least squares on points within 0.12 m of the URDF floor, refitted twice on the
    lower residuals.  None if fewer than 300 candidates, fewer than 200 survive a
    trim, or the plane is implausible (> ~8 deg or > 0.10 m from the URDF floor).
    """
    m = np.abs(p_base[:, 2]) < 0.12
    if m.sum() < 300:
        return None
    P = p_base[m]
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    if fast:
        x, y, z = (np.ascontiguousarray(v, dtype=np.float64) for v in (x, y, z))
    coef = _fit_fast(x, y, z) if fast else _fit_lstsq(x, y, z)
    if coef is None:
        return None
    a, b, c = coef
    if math.hypot(a, b) > 0.14 or abs(c) > 0.10:
        return None
    return float(a), float(b), float(c)


class OakFloorObstacles:
    def __init__(self, oak, bridge, camera_frame='oak_rgb_camera_optical_frame',
                 z_min=0.03, z_max=0.45, range_max=1.6, hz=5.0,
                 self_x=0.37, self_y=0.34, fast=None):
        self.oak, self.bridge, self.frame = oak, bridge, camera_frame
        self.z_min, self.z_max, self.range_max = z_min, z_max, range_max
        self.period = 1.0 / hz
        self.self_x, self.self_y = self_x, self_y
        if fast is None:
            fast = os.environ.get('X3_OAK_FLOOR_FAST', '0') == '1'
        self.fast = bool(fast)
        self.mem = FastFloorObstacleMemory() if self.fast else FloorObstacleMemory()
        if self.fast:
            logger.info('oak floor obstacles: X3_OAK_FLOOR_FAST path')
        self.stats = {}
        self._stop = threading.Event()
        node = bridge._node
        from sensor_msgs.msg import PointCloud2, PointField
        self._PC2, self._PF = PointCloud2, PointField
        self.pub = node.create_publisher(PointCloud2, '/oak/obstacles', 5)
        self._last_log = 0.0
        self._floor_h = []

    def start(self):
        threading.Thread(target=self._run, name='oak_floor_obstacles', daemon=True).start()

    def stop(self):
        self._stop.set()

    def _tf(self, target, source):
        from rclpy.time import Time
        tf = self.bridge._explore_tf.lookup_transform(target, source, Time())
        t = tf.transform.translation
        return quat_to_mat(tf.transform.rotation), np.array([t.x, t.y, t.z])

    def _run(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self.step()
            except Exception as e:
                if time.monotonic() - self._last_log > 10:
                    logger.warning('oak floor obstacles: %s', e)
                    self._last_log = time.monotonic()
            self._stop.wait(max(0.0, self.period - (time.monotonic() - t0)))

    def step(self):
        if self.oak.get_depth_frame_age() > 0.5:
            return
        depth = self.oak.get_raw_depth_frame()
        intr = oakd_cloud.scale_intrinsics(self.oak.get_depth_intrinsics(), depth.shape) \
            if depth is not None else None
        if intr is None:
            return
        fx, fy, cx, cy = intr
        h_img, w_img = depth.shape[:2]
        pts = oakd_cloud.depth_to_points(depth, fx, fy, cx, cy, stride=4,
                                         z_min=0.30, z_max=self.range_max - 0.25, max_points=12000)
        # Mark only where visible() below can also clear (it trusts up to range_max - 0.1).
        R_oc, t_oc = self._tf('odom', self.frame)          # camera -> odom
        R_ob, t_ob = self._tf('odom', 'base_footprint')    # base -> odom
        p_odom = rigid(pts, R_oc, t_oc, self.fast)
        p_base = rigid(p_odom - t_ob, R_ob.T, None, self.fast)  # odom -> base
        h = self.floor_relative_height(p_base)
        if h is None:
            return
        self._floor_h.append(h[np.abs(h) < 0.06])
        if time.monotonic() - getattr(self, '_dbg_t', 0) > 10:
            self._dbg_t = time.monotonic()
            try:   # one frame for offline threshold checks
                np.savez('/tmp/oak_floor_debug.npz', p_base=p_base, h=h)
            except OSError:
                pass
        # Lab carpet vs fitted plane (2026-11-22): max 5 mm out to 1.35 m past the
        # bumper; an orange extension cord 0.5 m out read 13-31 mm.  At 12 mm the cord
        # was lethal and stalled Auto-Explore; the user is fine driving over cords and
        # wants power strips avoided, so 3 cm (+1 cm per metre of depth beyond 1 m).
        zmin = self.z_min + 0.01 * np.maximum(pts[:, 2] - 1.0, 0.0)
        keep = ((h > zmin) & (h < self.z_max)
                & ~((np.abs(p_base[:, 0]) < self.self_x) & (np.abs(p_base[:, 1]) < self.self_y)))
        # Need 2+ returns per 4 cm cell, to reject single-pixel stereo speckle.
        cand = min_returns_per_voxel(p_odom[keep], self.mem.cell, self.fast)
        R_co, t_co = R_oc.T, -R_oc.T @ t_oc                  # odom -> camera

        def visible(xyz):
            pc = rigid(xyz, R_co, t_co, self.fast)
            z = pc[:, 2]
            ok = (z > 0.35) & (z < self.range_max - 0.1)
            u = np.where(ok, fx * pc[:, 0] / np.maximum(z, 1e-3) + cx, -1)
            v = np.where(ok, fy * pc[:, 1] / np.maximum(z, 1e-3) + cy, -1)
            ok &= (u >= 8) & (u < w_img - 8) & (v >= 8) & (v < h_img - 8)
            out = np.zeros(len(xyz), bool)
            idx = np.nonzero(ok)[0]
            if len(idx):
                d = depth[v[idx].astype(int), u[idx].astype(int)]
                # Visible = the camera's ray reaches at least as far as the voxel.
                out[idx] = np.isfinite(d) & (d > z[idx] - 0.05)
            return out

        self.mem.update(cand, time.monotonic(), (t_ob[0], t_ob[1]), visible)
        self.publish(self.mem.points())
        now = time.monotonic()
        if now - self._last_log > 30:
            self._last_log = now
            fh = np.concatenate(self._floor_h) if self._floor_h else np.zeros(0)
            self._floor_h = []
            if len(fh):
                p = np.percentile(fh, [1, 50, 99])
                logger.info('oak floor obstacles: %d voxels; floor h p1/p50/p99 %.3f/%.3f/%.3f m; '
                            'plane a,b,c %s', len(self.mem), *p,
                            tuple(round(v, 3) for v in self.stats.get('floor', ())))

    def floor_relative_height(self, p_base):
        """Height above a floor plane fitted in this frame (base_footprint coords).

        The URDF height put the carpet at +5.7 cm (2026-11-22, B1F4 depth is
        uncorrected and reads long), so trust the frame's own floor instead:
        least-squares z = a*x + b*y + c on points within 0.12 m of the URDF floor,
        twice, trimming to the lower residuals.  None if the floor is not seen
        (fewer than 300 candidate points) or the fit is implausible.
        """
        h0 = p_base[:, 2]
        coef = fit_floor_plane(p_base, getattr(self, 'fast', False))
        if coef is None:
            return None
        a, b, c = coef
        self.stats['floor'] = (float(a), float(b), float(c))
        return (h0 - (a * p_base[:, 0] + b * p_base[:, 1] + c)) / math.sqrt(1 + a * a + b * b)

    def publish(self, points):
        msg = self._PC2()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.bridge._node.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(points)
        msg.fields = [self._PF(name=n, offset=i * 4, datatype=self._PF.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.is_bigendian, msg.is_dense = False, True
        msg.point_step, msg.row_step = 12, 12 * len(points)
        msg.data = np.ascontiguousarray(points, dtype='<f4').tobytes()
        self.pub.publish(msg)
