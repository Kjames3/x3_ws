#!/usr/bin/env python3
"""Replay /livox/lidar through the low-band obstacle memory, old rule vs tall-column rule.

    python3 src/tools/lidar/obstacle_memory_replay.py artifacts/person-lidar-2026-11-24/crowd-r1 ...

Per run: remembered-only cell-seconds (5 cm XY cells in the output but not in the current
frame), for the old memory (obstacle_memory.py tall_m=None) and the new one, and what
the new rule dropped.  On EMPTY runs everything dropped is furniture forgotten by mistake;
on people runs it should be legs and shoes.  Low-band extraction mirrors
livox_scan_node.publish_obstacles (z band with range slope, range cap, chassis box, one
point per cell); the Livox mount is treated as axis-aligned (as lidar_person_dataset).
"""
import math
import pathlib
import sqlite3
import struct
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent / 'perception'))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import lidar_person_dataset as lpd  # noqa: E402
from yahboomcar_bringup.obstacle_memory import ObstacleMemory  # noqa: E402

ZMIN, ZSLOPE, ZMAX, RMAX, CELL = 0.06, 0.015, 0.45, 4.0, 0.05
SELF_X, SELF_Y = 0.37, 0.34
MEM_S, TAN = 20.0, math.tan(math.radians(7.0))


def frames(run):
    run = pathlib.Path(run)
    db = sqlite3.connect(str(run / 'bag' / 'bag_0.db3'))
    tid = {n: i for i, n in db.execute('select id, name from topics')}
    poses = lpd.Poses([lpd.parse_odom(d) for (d,) in db.execute(
        'select data from messages where topic_id=?', (tid['/odom'],))])
    lx, ly, lz = lpd.livox_xyz_for(run)
    for (data,) in db.execute('select data from messages where topic_id=? order by timestamp',
                              (tid['/livox/lidar'],)):
        p = lpd.decode_points(data)
        if p is None or not len(p):
            continue
        sec, nsec = struct.unpack_from('<iI', data, 4)
        end = sec + nsec * 1e-9 + int(p['t'].max()) * 1e-9
        pose = poses.at([end])
        if pose is None:
            continue
        fx, fy, fa = (float(np.ravel(v)[0]) for v in pose)
        x, y, z = p['x'] + lx, p['y'] + ly, p['z'] + lz
        ok = (p['x'] != 0) | (p['y'] != 0)
        x, y, z = x[ok], y[ok], z[ok]
        r = np.hypot(x, y)
        keep = ((z > ZMIN + ZSLOPE * np.maximum(r - 2.0, 0.0)) & (z < ZMAX) & (r < RMAX)
                & ~((np.abs(x) < SELF_X) & (np.abs(y) < SELF_Y)))
        x, y, z = x[keep], y[keep], z[keep]
        _, first = np.unique(np.floor(np.c_[x, y, z] / CELL).astype(np.int64), axis=0, return_index=True)
        x, y, z = x[first], y[first], z[first]
        c, s = math.cos(fa), math.sin(fa)
        yield end, fx, fy, c * x - s * y + fx, s * x + c * y + fy, z, lz


def run_one(run):
    arms = None
    stats = {a: {'held': 0.0, 'frames': 0} for a in ('old', 'new')}
    dropped_cs, last = 0.0, None
    near_drop = []
    for end, fx, fy, wx, wy, wz, sz in frames(run):
        if arms is None:
            arms = {'old': ObstacleMemory(CELL, MEM_S, TAN, sz, RMAX, tall_m=None),
                    'new': ObstacleMemory(CELL, MEM_S, TAN, sz, RMAX)}
        dt = 0.1 if last is None else min(0.5, end - last)
        last = end
        cur = set(map(tuple, np.floor(np.c_[wx, wy] / CELL).astype(int).tolist()))
        outs = {}
        for a, m in arms.items():
            out = m.update(wx, wy, wz, fx, fy, end)
            cells = set(map(tuple, np.floor(out[:, :2] / CELL).astype(int).tolist()))
            outs[a] = cells
            stats[a]['held'] += len(cells - cur) * dt
            stats[a]['frames'] += 1
        gone = outs['old'] - outs['new']
        dropped_cs += len(gone) * dt
        for k in gone:
            near_drop.append(math.hypot((k[0] + .5) * CELL - fx, (k[1] + .5) * CELL - fy))
    nd = np.array(near_drop) if near_drop else np.zeros(0)
    name = '/'.join(pathlib.Path(run).resolve().parts[-2:])
    print(f'{name:45s} frames {stats["old"]["frames"]:5d}  remembered-only cell-s old {stats["old"]["held"]:8.0f} '
          f'new {stats["new"]["held"]:8.0f}  dropped {dropped_cs:7.0f} cell-s '
          f'({arms["new"].stats["gone_columns"]} tall columns gone)'
          + (f'  dropped cells at {np.median(nd):.1f} m median' if len(nd) else ''))


if __name__ == '__main__':
    for r in sys.argv[1:]:
        run_one(r)
