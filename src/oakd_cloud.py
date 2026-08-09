"""
oakd_cloud.py — Back-project OAK-D Lite depth into a PointCloud2 for Nav2.

Why this exists
---------------
The 4ROS lidar scans a single horizontal plane at z = 0.191 m (see
`yahboomcar_X3.urdf.xacro`, `laser_joint`, rpy = "0 0 pi"). Anything that does
not intersect that plane is invisible to the costmaps: tabletops, kitchen
counters, chair seats, open drawers, sofa arms. The robot sees table *legs* and
plans straight into the tabletop.

The OAK-D Lite does see them. This module turns its metric depth map into a
`sensor_msgs/PointCloud2` that a Nav2 voxel layer can mark obstacles from.

Design notes
------------
* The projection math (`depth_to_points`) is deliberately ROS-free so it can be
  unit-tested on a laptop with no rclpy and no device.
* Points come out in the **optical frame** convention (x right, y down,
  z forward), tagged `oak_rgb_camera_optical_frame`. That frame already exists
  in the URDF, so TF places the cloud in base_link without any new static
  publisher. Height filtering is left to the costmap layer's
  `min_obstacle_height`/`max_obstacle_height`, which apply *after* that TF —
  doing it here would mean duplicating the extrinsics.
* Stereo depth error grows as roughly 0.006*z^2 m for this baseline (~2 cm at
  2 m, ~10 cm at 4 m), so the default range gate stops at 4 m. Beyond that the
  points are noise that would smear obstacles across the costmap.
* The per-pixel ray directions depend only on (shape, stride, intrinsics), so
  they are computed once and cached. On the Orin Nano the OAK driver and the
  velocity estimator are already the top two CPU consumers; this path is meant
  to stay in the noise.
"""

import logging
import threading

import numpy as np

logger = logging.getLogger(__name__)

# Defaults tuned for indoor navigation, not for reconstruction quality.
DEFAULT_STRIDE = 4        # every 4th pixel in each axis -> 1/16 the points
DEFAULT_Z_MIN = 0.30      # below the OAK-D Lite's reliable stereo range
DEFAULT_Z_MAX = 4.00      # past this, 0.006*z^2 noise dominates
DEFAULT_MAX_POINTS = 8000  # hard cap on message size / costmap raytrace cost

_grid_cache = {}
_grid_lock = threading.Lock()


def _ray_grid(h, w, stride, fx, fy, cx, cy):
    """Cached per-pixel unit rays: kx[u] = (u-cx)/fx, ky[v] = (v-cy)/fy.

    Kept as two 1-D arrays rather than a full (H,W) map — broadcasting them
    against the sparse valid-pixel indices is both faster and far smaller than
    materialising the dense grid.
    """
    key = (h, w, stride, round(fx, 4), round(fy, 4), round(cx, 4), round(cy, 4))
    with _grid_lock:
        hit = _grid_cache.get(key)
        if hit is not None:
            return hit
        us = np.arange(0, w, stride, dtype=np.float32)
        vs = np.arange(0, h, stride, dtype=np.float32)
        kx = (us - np.float32(cx)) / np.float32(fx)
        ky = (vs - np.float32(cy)) / np.float32(fy)
        # Bound the cache: shape/stride/intrinsics are effectively fixed at
        # runtime, so more than a couple of entries means something upstream is
        # churning and we would otherwise leak.
        if len(_grid_cache) > 8:
            _grid_cache.clear()
        _grid_cache[key] = (kx, ky)
        return kx, ky


def depth_to_points(depth_m, fx, fy, cx, cy, stride=DEFAULT_STRIDE,
                    z_min=DEFAULT_Z_MIN, z_max=DEFAULT_Z_MAX,
                    max_points=DEFAULT_MAX_POINTS):
    """Back-project a float32 metric depth map to an (N,3) float32 cloud.

    `depth_m` is metres, 0 / NaN / inf meaning "no return" (the convention
    `OakDCamera.get_raw_depth_frame()` already uses). `cx`/`cy`/`fx`/`fy` are in
    full-resolution pixel units — the stride is applied here, not by the caller.

    Returns points in the optical frame (x right, y down, z forward).
    """
    if depth_m is None or depth_m.size == 0:
        return np.zeros((0, 3), dtype=np.float32)

    h, w = depth_m.shape[:2]
    d = depth_m[::stride, ::stride]

    # np.isfinite also rejects the NaN/inf that a failed stereo match leaves.
    valid = np.isfinite(d) & (d >= z_min) & (d <= z_max)
    n_valid = int(np.count_nonzero(valid))
    if n_valid == 0:
        return np.zeros((0, 3), dtype=np.float32)

    vs, us = np.nonzero(valid)
    z = d[vs, us].astype(np.float32, copy=False)

    # Cap before projecting, so the arithmetic scales with the cap and not with
    # how much of the frame happened to be in range.
    if max_points and n_valid > max_points:
        take = np.linspace(0, n_valid - 1, max_points).astype(np.intp)
        vs, us, z = vs[take], us[take], z[take]

    kx, ky = _ray_grid(h, w, stride, fx, fy, cx, cy)
    pts = np.empty((z.shape[0], 3), dtype=np.float32)
    pts[:, 0] = kx[us] * z
    pts[:, 1] = ky[vs] * z
    pts[:, 2] = z
    return pts


def make_cloud_msg(points, stamp, frame_id):
    """Wrap an (N,3) float32 array in a sensor_msgs/PointCloud2.

    Imported lazily so `depth_to_points` stays usable without a ROS install.
    """
    import array

    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    # Same rosidl trap as Image.data: only array.array('B') takes the fast path
    # in the generated setter; bytes/ndarray fall through to per-element
    # validation, which at these point counts shows up in the publish loop.
    msg.data = array.array(
        "B", np.ascontiguousarray(points, dtype=np.float32).tobytes())
    msg.is_dense = True     # invalid pixels were dropped, not encoded as NaN
    return msg
