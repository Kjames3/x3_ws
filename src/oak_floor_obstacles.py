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
"""
import logging
import math
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


class OakFloorObstacles:
    def __init__(self, oak, bridge, camera_frame='oak_rgb_camera_optical_frame',
                 z_min=0.03, z_max=0.45, range_max=1.6, hz=5.0,
                 self_x=0.37, self_y=0.34):
        self.oak, self.bridge, self.frame = oak, bridge, camera_frame
        self.z_min, self.z_max, self.range_max = z_min, z_max, range_max
        self.period = 1.0 / hz
        self.self_x, self.self_y = self_x, self_y
        self.mem = FloorObstacleMemory()
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
        p_odom = pts @ R_oc.T + t_oc
        p_base = (p_odom - t_ob) @ R_ob                      # odom -> base
        h = self.floor_relative_height(p_base)
        if h is None:
            return
        self._floor_h.append(h[np.abs(h) < 0.06])
        keep = ((h > self.z_min) & (h < self.z_max)
                & ~((np.abs(p_base[:, 0]) < self.self_x) & (np.abs(p_base[:, 1]) < self.self_y)))
        # Need 2+ returns per 4 cm cell, to reject single-pixel stereo speckle.
        cand = p_odom[keep]
        if len(cand):
            keys = np.floor(cand / self.mem.cell).astype(np.int64)
            _, inv, cnt = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
            cand = cand[cnt[inv.ravel()] >= 2]
        R_co, t_co = R_oc.T, -R_oc.T @ t_oc                  # odom -> camera

        def visible(xyz):
            pc = xyz @ R_co.T + t_co
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
                            'plane a,b,c %s', len(self.mem.vox), *p,
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
        m = np.abs(h0) < 0.12
        if m.sum() < 300:
            return None
        P = p_base[m]
        A = np.column_stack((P[:, 0], P[:, 1], np.ones(len(P))))
        z = P[:, 2]
        coef = np.linalg.lstsq(A, z, rcond=None)[0]
        for _ in range(2):
            r = z - A @ coef
            keep = np.abs(r) < max(0.015, 2.5 * np.median(np.abs(r)))
            if keep.sum() < 200:
                return None
            coef = np.linalg.lstsq(A[keep], z[keep], rcond=None)[0]
        a, b, c = coef
        # Plausible: within ~8 deg and 0.10 m of the URDF floor.
        if math.hypot(a, b) > 0.14 or abs(c) > 0.10:
            return None
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
