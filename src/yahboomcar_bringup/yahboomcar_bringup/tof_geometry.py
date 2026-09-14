#!/usr/bin/env python3
"""Pure geometry for the VL53L5CX multizone ToF array.

No hardware, no ROS, no I2C -- everything here is testable off the robot, which
is the whole point: the parts of a ToF integration that are actually easy to get
wrong (zone ordering, status filtering, the ray table) are the parts that need
no sensor to exercise.

Two conventions are fixed here and must not drift:

* **Points come out in the SENSOR frame** (ROS convention: x forward out of the
  cover glass, y left, z up).  The mount pitch and height live in the static
  TF published by ``tof_node``, NOT in this math.  One source of truth for
  extrinsics, and it stays re-measurable at runtime -- see the camera_pitch_deg
  lesson: a mount angle that is baked into code is a mount angle nobody
  re-measures after the bracket moves.

* **Zone ordering is NOT assumed.**  The ULD hands back a flat 64-element array
  whose corner-zero corresponds to a physical corner that depends on how the
  part is oriented in its package and on the board.  ``transpose``/``flip_h``/
  ``flip_v`` exist so this is a config change, not a code change, and the
  default is identity so nothing is silently "corrected".  VERIFY IT ON
  HARDWARE by putting a hand in one corner of the FoV and watching which corner
  of the printed grid goes near -- do not infer it from a datasheet drawing.
"""
from __future__ import annotations

import math
from functools import lru_cache

import numpy as np

# Square FoV of the VL53L5CX.  ST quotes 63 deg diagonal, which is a 45x45 deg
# square.  Zone centres are spaced at equal ANGLE (fov/n), not equal tangent --
# this is the ST convention.  The difference is under 1 deg at the edge zones
# at this FoV, but it is a real bias if you later back-project against a pinhole
# camera model, so keep the two straight.
FOV_DEG = 45.0

# Target status codes from the ULD.  5 = range valid.  9 = range valid with a
# large return pulse (typically a merged target) -- still a real surface, and
# dropping it throws away most close-range floor returns.
#
# Everything else is noise dressed up as a distance: 4 = target phase out of
# bounds, 8 = signal below the wrap-around threshold, 10 = valid but no target
# at the PREVIOUS range (i.e. it just appeared, unstable), 12 = blurred by
# motion, 255 = no update.  An unfiltered frame is mostly these, and feeding
# them to a costmap fabricates obstacles.
VALID_STATUS = (5, 9)

# The datasheet ceiling is ~4 m in the dark against a white target.  Indoors,
# off a matte floor at a grazing angle, useful returns stop well short of that.
# The default is deliberately NOT the datasheet number.
DEFAULT_MAX_RANGE_M = 2.5
DEFAULT_MIN_RANGE_M = 0.02


@lru_cache(maxsize=8)
def ray_table(resolution: int = 8, fov_deg: float = FOV_DEG) -> np.ndarray:
    """Unit direction of every zone's optical axis, in the sensor frame.

    Returns ``(resolution**2, 3)`` float64, row-major over (row, col) with
    row 0 = TOP of the field of view and col 0 = LEFT (+y, robot-left).

    NOTE the sensor does NOT report distance along these rays -- it reports
    DEPTH along the boresight (sensor +x), like a depth camera.  Use
    ``frame_to_points`` rather than ``distance * ray``; see its docstring for
    the wall test that settled this.
    """
    if resolution not in (4, 8):
        raise ValueError(f"resolution must be 4 or 8, got {resolution}")
    n = resolution
    step = math.radians(fov_deg) / n
    # Zone centres, symmetric about the axis: for n=8 this is +-3.5, +-2.5 ...
    offsets = (n / 2.0 - 0.5 - np.arange(n)) * step
    el = offsets[:, None]                      # row 0 -> +elevation (up)
    az = offsets[None, :]                      # col 0 -> +azimuth  (left)
    rays = np.empty((n, n, 3), dtype=np.float64)
    rays[..., 0] = np.cos(el) * np.cos(az)     # x forward
    rays[..., 1] = np.cos(el) * np.sin(az)     # y left
    rays[..., 2] = np.broadcast_to(np.sin(el), (n, n))   # z up
    return rays.reshape(n * n, 3)


def reorder(flat: np.ndarray, resolution: int = 8, transpose: bool = False,
            flip_h: bool = False, flip_v: bool = False) -> np.ndarray:
    """Map the ULD's flat zone array onto (row=top-down, col=left-right).

    Applied in the order transpose -> flip_v -> flip_h, matching what the
    bench tool prints, so a setting that looks right in ``tof_array.py
    --stream`` is the setting to put in the node's params.
    """
    n = resolution
    grid = np.asarray(flat).reshape(n, n)
    if transpose:
        grid = grid.T
    if flip_v:
        grid = grid[::-1, :]
    if flip_h:
        grid = grid[:, ::-1]
    return np.ascontiguousarray(grid).reshape(n * n)


def valid_mask(distance_mm, target_status, resolution: int = 8,
               valid_status=VALID_STATUS,
               min_range_m: float = DEFAULT_MIN_RANGE_M,
               max_range_m: float = DEFAULT_MAX_RANGE_M,
               sigma_mm=None, max_sigma_mm: float | None = None,
               **reorder_kw) -> np.ndarray:
    """Boolean keep-mask over zones, already reordered."""
    n2 = resolution * resolution
    dist = reorder(np.asarray(distance_mm, dtype=np.float64).reshape(n2),
                   resolution, **reorder_kw)
    status = reorder(np.asarray(target_status).reshape(n2), resolution, **reorder_kw)

    keep = np.isin(status, np.asarray(valid_status))
    keep &= dist >= min_range_m * 1000.0
    keep &= dist <= max_range_m * 1000.0
    # A dead or saturated zone reports 0 mm with a plausible status often
    # enough to matter; 0 mm is never a real measurement through a cover glass.
    keep &= dist > 0
    if sigma_mm is not None and max_sigma_mm is not None:
        sig = reorder(np.asarray(sigma_mm, dtype=np.float64).reshape(n2),
                      resolution, **reorder_kw)
        keep &= sig <= max_sigma_mm
    return keep


def frame_to_points(distance_mm, target_status, resolution: int = 8,
                    fov_deg: float = FOV_DEG, valid_status=VALID_STATUS,
                    min_range_m: float = DEFAULT_MIN_RANGE_M,
                    max_range_m: float = DEFAULT_MAX_RANGE_M,
                    sigma_mm=None, max_sigma_mm: float | None = None,
                    transpose: bool = False, flip_h: bool = False,
                    flip_v: bool = False):
    """One ToF frame -> ``(points Nx3 float32, keep-mask)`` in the sensor frame.

    N is the number of VALID zones and is routinely 0 -- pointed at open space
    past the sensor's real range, every zone times out.  Callers must handle an
    empty cloud as "no information", never as "no obstacles": the difference is
    what drove the F7 failure, where a frozen obstacle set was treated as live.
    """
    reorder_kw = dict(transpose=transpose, flip_h=flip_h, flip_v=flip_v)
    keep = valid_mask(distance_mm, target_status, resolution, valid_status,
                      min_range_m, max_range_m, sigma_mm, max_sigma_mm,
                      **reorder_kw)
    n2 = resolution * resolution
    dist_m = reorder(np.asarray(distance_mm, dtype=np.float64).reshape(n2),
                     resolution, **reorder_kw) / 1000.0
    # distance_mm is DEPTH along the sensor boresight, not range along the zone
    # ray.  Settled 2026-09-14 against a flat door at 0.3/0.6/1.0 m: the depth
    # model fit all 64 on-target zones to 3.2 mm rms (yaw -0.05 deg), the radial
    # model to 22.4 mm rms with errors up to 78 mm.  So scale each ray to reach
    # x = depth, which stretches the outer zones by up to ~1/cos(corner angle).
    rays = ray_table(resolution, fov_deg)[keep]
    pts = rays * (dist_m[keep] / rays[:, 0])[:, None]
    return pts.astype(np.float32), keep


def floor_slant_ranges(mount_height_m: float, pitch_deg: float,
                       resolution: int = 8, fov_deg: float = FOV_DEG):
    """What each zone row READS off a flat floor, as the sensor reports it.

    That is boresight DEPTH, derived from the slant range along the ray.  ``floor_coverage`` returns the
    horizontal ground distance instead, which is the right number for "how far
    ahead does this mount see" but is ~20 deg worth of cosine SHORT of what the
    sensor actually measures -- comparing a live frame against it shows a
    fictitious error on every row.  Compare against this one.
    """
    n = resolution
    step = fov_deg / n
    rows, ranges = [], []
    for r in range(n):
        down_deg = pitch_deg - (n / 2.0 - 0.5 - r) * step
        if down_deg <= 1e-6:
            continue
        rows.append(r)
        slant = mount_height_m / math.sin(math.radians(down_deg))
        # The sensor reports depth along its boresight, not slant range.
        ranges.append(slant * math.cos(math.radians(down_deg - pitch_deg)))
    return np.asarray(rows, dtype=int), np.asarray(ranges, dtype=float)


def floor_coverage(mount_height_m: float, pitch_deg: float,
                   resolution: int = 8, fov_deg: float = FOV_DEG):
    """Where each zone ROW lands on a flat floor, given a mount.

    Returns ``(row_index, range_m)`` for the rows whose axis points below the
    horizon; rows at or above the horizon are omitted because they never
    intersect the floor.  This is the number that decides a mount angle -- a
    steep tilt spends most of its eight rows inside the first 25 cm and caps
    how far ahead the sensor can see anything at all.
    """
    n = resolution
    step = fov_deg / n
    rows, ranges = [], []
    for r in range(n):
        down_deg = pitch_deg - (n / 2.0 - 0.5 - r) * step
        if down_deg <= 1e-6:
            continue
        rows.append(r)
        ranges.append(mount_height_m / math.tan(math.radians(down_deg)))
    return np.asarray(rows, dtype=int), np.asarray(ranges, dtype=float)
