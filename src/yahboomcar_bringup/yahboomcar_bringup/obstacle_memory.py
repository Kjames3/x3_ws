"""Low-band obstacle memory for /livox/obstacles (livox_scan_node).

The Livox's lowest ray is ~-7 deg, so a point z above the floor drops out of view
inside (sensor_z - z) / tan(7 deg) + 0.15 m (~2.2 m at ankle height).  A cable the
robot drives up to would vanish from the costmap there, so voxels are REMEMBERED in
odom and only forgotten when their spot is back in view and empty, when older than
memory_s, or farther than range_max + 1.

That memory also kept a passer-by's shoes and ankles for up to memory_s after they had
walked on (2026-11-26 walk-by: costmap cells 1-2 m out lingered 5-45 s where the person
paused).  A person differs from a cable by height: their legs reach well above tall_m,
and the upper part of a leg stays in view right up to the robot.  So when a tall_cell
grid cell that held points above tall_m is in view at that height and it and its
neighbours have had no such point for gone_s, the remembered low voxels within gone_r_m
of it are forgotten too.  Short things (cables, bars, a
headphone case) never had a tall part and keep the full memory.  tall_m=None restores
the old behaviour.
"""
import math

import numpy as np

TALL_M = 0.2       # a column whose points reach this high is a leg / furniture, not a cable
TALL_CELL_M = 0.15 # tall parts are tracked on this coarser grid: the Livox pattern is sparse,
                   # and a 5 cm cell on a static chair leg often goes unhit for a second
GONE_S = 1.0       # a tall cell AND its 8 neighbours in view and unhit this long = it moved away
GONE_R_M = 0.2     # forget hidden low voxels this close to it (a shoe is longer than a leg)


class ObstacleMemory:
    def __init__(self, cell, memory_s, min_elev_tan, sensor_z, range_max,
                 tall_m=TALL_M, gone_s=GONE_S, gone_r_m=GONE_R_M, tall_cell=TALL_CELL_M):
        self.cell, self.memory_s, self.tan = cell, memory_s, min_elev_tan
        self.sz, self.rmax = sensor_z, range_max
        self.tall_m, self.gone_s, self.gone_r, self.tcell = tall_m, gone_s, gone_r_m, tall_cell
        self.vox = {}       # (ix, iy, iz) -> (x, y, z, t last seen)
        self.tall = {}      # coarse (ix, iy) -> [x, y, z top, t a tall point last hit it]
        self.stats = {'gone_columns': 0, 'gone_voxels': 0}

    def _visible(self, d, z):
        return d >= (self.sz - z) / self.tan + 0.15 and d < self.rmax

    def update(self, wx, wy, wz, fx, fy, now):
        """Current frame's low-band points (odom XY, height z), robot XY (fx, fy) at
        `now`.  Returns the remembered points, (N, 3) float32."""
        cell, tc = self.cell, self.tcell
        seen = set()
        for x, y, z in zip(wx, wy, wz):
            k = (int(math.floor(x / cell)), int(math.floor(y / cell)), int(math.floor(z / cell)))
            seen.add(k)
            self.vox[k] = (x, y, z, now)
            if self.tall_m is not None and z >= self.tall_m:
                c = (int(math.floor(x / tc)), int(math.floor(y / tc)))
                t = self.tall.get(c)
                self.tall[c] = [(c[0] + 0.5) * tc, (c[1] + 0.5) * tc, max(z, t[2]) if t else z, now]
        gone = []
        if self.tall_m is not None:
            for c, (x, y, ztop, t) in list(self.tall.items()):
                d = math.hypot(x - fx, y - fy)
                if now - t > self.memory_s or d > self.rmax + 1.0:
                    del self.tall[c]
                    continue
                if now - t < self.gone_s or not self._visible(d, ztop):
                    continue
                recent = max((self.tall[n][3] for n in ((c[0] + i, c[1] + j) for i in (-1, 0, 1) for j in (-1, 0, 1))
                              if n in self.tall), default=t)
                if now - recent >= self.gone_s:
                    gone.append((x, y))
            for x, y in gone:
                self.tall.pop((int(math.floor(x / tc)), int(math.floor(y / tc))), None)
            self.stats['gone_columns'] += len(gone)
        g = np.array(gone).reshape(-1, 2)
        for k, (x, y, z, t) in list(self.vox.items()):
            if k in seen:
                continue
            d = math.hypot(x - fx, y - fy)
            if self._visible(d, z) or now - t > self.memory_s or d > self.rmax + 1.0:
                del self.vox[k]
            elif len(g) and np.min(np.hypot(g[:, 0] - x, g[:, 1] - y)) <= self.gone_r:
                del self.vox[k]
                self.stats['gone_voxels'] += 1
        return np.array([v[:3] for v in self.vox.values()], dtype='<f4').reshape(-1, 3)
