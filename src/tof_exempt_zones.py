"""Map regions where the LOWER ToF array's obstacles are ignored.

The lower array cannot tell a low raised floor (a platform a centimetre or two
above the carpet) from a 3 cm block: both shorten its zones past the
floor-deficit threshold, and the platform's lip passes the height test too.
Nav2 and the CBF then refuse to drive onto it.  Inside a polygon listed here
for the active map, lower-array points are dropped; the upper array (>= 6 cm
objects) and the lidar still guard the area.

config/tof_exempt_zones.json:
    {"RealLab3": [[[x, y], [x, y], [x, y], ...], ...]}
Keys are map stems (the .yaml name without extension); points are map-frame
metres.  Draw the polygon over the platform plus the ~0.4 m approach strip in
front of each edge you drive over, since the lower array sees ~0.3 m ahead.
"""
import json
import logging
import math
import os

import numpy as np

logger = logging.getLogger(__name__)

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'config', 'tof_exempt_zones.json')


def map_stem(name):
    if not name:
        return None
    base = os.path.basename(str(name))
    for ext in ('.yaml', '.yml', '.pgm'):
        if base.endswith(ext):
            return base[:-len(ext)]
    return base


def load_zones(path=DEFAULT_PATH):
    """{map stem: [Nx2 polygon arrays]}; {} when the file is absent or bad."""
    try:
        with open(path) as f:
            raw = json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as e:
        logger.error(f"ToF exempt zones unreadable ({path}): {e}")
        return {}
    zones = {}
    for key, polys in raw.items():
        if key.startswith('_'):
            continue                      # comment keys
        good = []
        for poly in polys:
            arr = np.asarray(poly, dtype=np.float64)
            if arr.ndim == 2 and arr.shape[1] == 2 and len(arr) >= 3:
                good.append(arr)
            else:
                logger.error(f"ToF exempt zone for {key} ignored: need >= 3 [x, y] points")
        if good:
            zones[map_stem(key)] = good
    return zones


def points_in_polygon(xy, poly):
    """Even-odd ray test; xy (N,2), poly (M,2) -> bool (N,)."""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    x, y = xy[:, 0], xy[:, 1]
    inside = np.zeros(len(xy), dtype=bool)
    px, py = poly[:, 0], poly[:, 1]
    j = len(poly) - 1
    for i in range(len(poly)):
        crosses = (py[i] > y) != (py[j] > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            xint = (px[j] - px[i]) * (y - py[i]) / (py[j] - py[i]) + px[i]
        inside ^= crosses & (x < xint)
        j = i
    return inside


def compose_map_from_odom(map_pose, odom_pose):
    """map<-odom transform (x, y, yaw) from a map pose and the odom pose at the same time."""
    mx, my, mth = map_pose
    ox, oy, oth = odom_pose
    yaw = mth - oth
    c, s = math.cos(yaw), math.sin(yaw)
    return (mx - (c * ox - s * oy), my - (s * ox + c * oy), yaw)


def base_to_map(xy_base, odom_pose, map_from_odom):
    """Base-frame xy -> map xy through the current odom pose and map<-odom."""
    xy = np.asarray(xy_base, dtype=np.float64).reshape(-1, 2)
    ox, oy, oth = odom_pose
    c, s = math.cos(oth), math.sin(oth)
    odom = np.column_stack((ox + c * xy[:, 0] - s * xy[:, 1], oy + s * xy[:, 0] + c * xy[:, 1]))
    tx, ty, yaw = map_from_odom
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((tx + c * odom[:, 0] - s * odom[:, 1], ty + s * odom[:, 0] + c * odom[:, 1]))


def exempt_mask(xy_base, odom_pose, map_from_odom, polygons):
    """True for base-frame points that fall inside any exempt polygon."""
    xy = np.asarray(xy_base, dtype=np.float64).reshape(-1, 2)
    out = np.zeros(len(xy), dtype=bool)
    if not polygons or map_from_odom is None or len(xy) == 0:
        return out
    m = base_to_map(xy, odom_pose, map_from_odom)
    for poly in polygons:
        out |= points_in_polygon(m, poly)
    return out
