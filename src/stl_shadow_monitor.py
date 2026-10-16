#!/usr/bin/env python3
"""Offline shadow STL monitor for the CBF walking runs.

Monitoring only: it reads a recorded bag and reports the robustness of a few
timed specs. It never touches a command path.

Signals (from /pedestrian_states, which is in base_link, stamp 0 -- bag receipt
time is the only clock -- and /cmd_vel):
  clearance(t)  distance from the robot footprint to the nearest person
                centre, minus the person radius
  age(t)        time since the last /pedestrian_states message
  tracks        per-id positions, for loss and id-break events

Specs (robustness rho >= 0 means satisfied; margin in the signal's units):
  phi_clear   G (clearance >= D_MIN)                       rho = min(clearance) - D_MIN
  phi_fresh   G (age <= MAX_AGE)                           rho = MAX_AGE - max(age)
  phi_hold    a track last seen within NEAR_M must not vanish: G (near(id) -> F[0,HOLD_S] seen(id))
              reported as events: 'lost' (nothing near within BREAK_DIST after)
              or 'id_break' (a new id appears near the old one within BREAK_S)
  phi_react   closing > CLOSE_V inside NEAR_M -> F[0,REACT_S] (command moves away)
              rho = REACT_S - delay, per onset episode

The pedestrian velocity field is not used: its frame is not documented in the
publisher, so closing speed is differenced from positions.

Usage:  python3 src/stl_shadow_monitor.py <bag.db3 | run dir> [...] [--json]
"""
import argparse
import json
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np

FOOTPRINT = (0.16, 0.13, 0.11, 0.11)   # front, rear, left, right about the lidar (CBF outline)
LIDAR_X = 0.0435                       # lidar origin in base_link
PERSON_R = 0.25                        # CBF's person body radius
D_MIN = 0.10                           # CBF margin
MAX_AGE = 0.3
NEAR_M = 1.5
HOLD_S = 0.5
BREAK_S = 1.0
BREAK_DIST = 0.6
CLOSE_V = 0.3
REACT_S = 0.5
AWAY_V = 0.02
GRID_HZ = 20.0


def rect_distance(x, y, fp=FOOTPRINT):
    """Distance from point (x, y), lidar frame, to the footprint rectangle (0 inside)."""
    front, rear, left, right = fp
    dx = max(-rear - x, 0.0, x - front)
    dy = max(-right - y, 0.0, y - left)
    return math.hypot(dx, dy)


def clearance(p_base, radius=PERSON_R):
    """Footprint-to-body clearance of a person at p_base = (x, y) in base_link."""
    return rect_distance(p_base[0] - LIDAR_X, p_base[1]) - radius


def clearance_series(ped, grid):
    """ped: list of (t, [(id, x, y), ...]); returns (clearance, age, rng) on grid.

    Zero-order hold on the latest message. NaN clearance = no person reported.
    """
    ts = np.array([p[0] for p in ped])
    clr = np.full(len(grid), np.nan)
    age = np.full(len(grid), np.nan)
    rng = np.full(len(grid), np.nan)
    k = np.searchsorted(ts, grid, side='right') - 1
    for i, j in enumerate(k):
        if j < 0:
            continue
        age[i] = grid[i] - ts[j]
        pts = ped[j][1]
        if pts:
            clr[i] = min(clearance((x, y)) for _, x, y in pts)
            rng[i] = min(math.hypot(x, y) for _, x, y in pts)
    return clr, age, rng


def track_events(ped):
    """Loss and id-break events for ids last seen within NEAR_M."""
    last, first = {}, {}
    for t, pts in ped:
        for i, x, y in pts:
            last[i] = (t, x, y)
            first.setdefault(i, t)
    events = []
    for i, (t_last, x, y) in last.items():
        if math.hypot(x, y) > NEAR_M:
            continue
        if t_last >= ped[-1][0] - HOLD_S:
            continue   # still present at the end of the run
        kind = 'lost'
        for t, pts in ped:
            if t_last < t <= t_last + BREAK_S:
                if any(j != i and first[j] > t_last and
                       math.hypot(px - x, py - y) < BREAK_DIST for j, px, py in pts):
                    kind = 'id_break'
                    break
        events.append(dict(id=i, t=t_last, range=math.hypot(x, y), kind=kind))
    return sorted(events, key=lambda e: e['t'])


def closing_speed(ped, window=0.4):
    """Per-message nearest-person closing speed from range differences (m/s, + = closing)."""
    out = []
    hist = []
    for t, pts in ped:
        if not pts:
            hist.clear()
            out.append((t, 0.0, np.nan))
            continue
        i, x, y = min(pts, key=lambda p: math.hypot(p[1], p[2]))
        r = math.hypot(x, y)
        hist = [h for h in hist if h[1] == i and t - h[0] <= window] + [(t, i, r, x, y)]
        v = 0.0
        if len(hist) >= 2 and hist[-1][0] - hist[0][0] > 0.15:
            v = -(hist[-1][2] - hist[0][2]) / (hist[-1][0] - hist[0][0])
        out.append((t, v, r))
    return out


def reaction_delays(ped, cmd):
    """Onset = closing > CLOSE_V inside NEAR_M; response = a command component away from the person."""
    cs = closing_speed(ped)
    cmd_t = np.array([c[0] for c in cmd])
    episodes = []
    onset = None
    for (t, v, r), (_, pts) in zip(cs, ped):
        hot = pts and v > CLOSE_V and r < NEAR_M
        if hot and (onset is None or t - onset['last'] > 2.0):
            i, x, y = min(pts, key=lambda p: math.hypot(p[1], p[2]))
            onset = dict(t=t, last=t, range=r, closing=v, dir=(x / r, y / r), delay=None)
            episodes.append(onset)
        elif hot:
            onset['last'] = t
    for e in episodes:
        j = np.searchsorted(cmd_t, e['t'])
        for tt, vx, vy in cmd[j:]:
            if tt > e['t'] + 3.0:
                break
            if -(vx * e['dir'][0] + vy * e['dir'][1]) > AWAY_V:
                e['delay'] = tt - e['t']
                break
        e['rho'] = (REACT_S - e['delay']) if e['delay'] is not None else None
    return episodes


def load_bag(db):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    c = sqlite3.connect('file:%s?mode=ro' % db, uri=True)
    top = {n: (i, t) for i, n, t in c.execute('select id,name,type from topics')}
    out = {}
    for name in ('/pedestrian_states', '/cmd_vel'):
        if name not in top:
            out[name] = []
            continue
        i, ty = top[name]
        T = get_message(ty)
        rows = []
        for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
            m = deserialize_message(d, T)
            if name == '/cmd_vel':
                rows.append((ts * 1e-9, m.linear.x, m.linear.y))
            else:
                rows.append((ts * 1e-9, [(p.id, p.position.x, p.position.y) for p in m.pedestrians]))
        out[name] = rows
    return out['/pedestrian_states'], out['/cmd_vel']


def evaluate(ped, cmd):
    t0, t1 = ped[0][0], ped[-1][0]
    grid = np.arange(t0, t1, 1.0 / GRID_HZ)
    clr, age, rng = clearance_series(ped, grid)
    seen = np.isfinite(clr)
    res = dict(duration=t1 - t0, person_time=float(seen.mean() * (t1 - t0)),
               tracks=len({i for _, pts in ped for i, _, _ in pts}))
    if seen.any():
        k = int(np.nanargmin(clr))
        res['phi_clear'] = dict(rho=float(clr[k] - D_MIN), min_clearance=float(clr[k]), t=float(grid[k] - t0),
                                violating_s=float((clr[seen] < D_MIN).sum() / GRID_HZ))
    else:
        res['phi_clear'] = None
    gaps = np.diff([p[0] for p in ped])
    res['phi_fresh'] = dict(rho=float(MAX_AGE - np.nanmax(age)), max_age=float(np.nanmax(age)),
                            gaps_over=int((gaps > MAX_AGE).sum()))
    res['phi_hold'] = track_events(ped)
    eps = reaction_delays(ped, cmd)
    res['phi_react'] = [dict(t=e['t'] - t0, range=e['range'], closing=e['closing'],
                             delay=e['delay'], rho=e['rho']) for e in eps]
    return res


def report(name, r):
    print('== %s  %.0f s, person visible %.0f s, %d ids' % (name, r['duration'], r['person_time'], r['tracks']))
    c = r['phi_clear']
    if c:
        print('  phi_clear  rho %+.3f  (min clearance %.3f m at t=%.1f s, below %.2f m for %.1f s)'
              % (c['rho'], c['min_clearance'], c['t'], D_MIN, c['violating_s']))
    f = r['phi_fresh']
    print('  phi_fresh  rho %+.3f  (max age %.2f s, %d gaps > %.1f s)' % (f['rho'], f['max_age'], f['gaps_over'], MAX_AGE))
    lost = [e for e in r['phi_hold'] if e['kind'] == 'lost']
    brk = [e for e in r['phi_hold'] if e['kind'] == 'id_break']
    print('  phi_hold   %d lost, %d id_break (ids last seen inside %.1f m)' % (len(lost), len(brk), NEAR_M))
    for e in r['phi_react']:
        d = 'none within 3 s' if e['delay'] is None else '%.2f s (rho %+.2f)' % (e['delay'], e['rho'])
        print('  phi_react  onset t=%.1f s range %.2f m closing %.2f m/s -> response %s'
              % (e['t'], e['range'], e['closing'], d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='+')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    allr = {}
    for p in a.paths:
        p = Path(p)
        db = next(iter(sorted(p.glob('bag/*.db3')))) if p.is_dir() else p
        ped, cmd = load_bag(db)
        if len(ped) < 2:
            print('== %s: no pedestrian data' % p)
            continue
        allr[str(p)] = evaluate(ped, cmd)
        if not a.json:
            report(p.name, allr[str(p)])
    if a.json:
        json.dump(allr, sys.stdout, indent=1, default=float)


if __name__ == '__main__':
    main()
