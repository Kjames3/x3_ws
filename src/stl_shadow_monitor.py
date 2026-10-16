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
  phi_react   relative closing speed > PERSON_GAMMA * (clearance - D_MIN), the CBF's own
              person-row bound -> F[0,REACT_S] closing back within it
              rho = REACT_S - delay, per episode; track loss is reported separately

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
REACT_S = 0.5
PERSON_GAMMA = 1.0                     # CBF_PERSON_GAMMA
ONSET_EXCESS = 0.15                    # m/s over the bound: closing speed from 0.4 s of positions is good to ~0.1
RESTORE_TOL = 0.05
RESTORE_HOLD_S = 0.2
EPISODE_MAX_S = 3.0
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


def clearance_closing(ped, window=0.4):
    """Per-message (t, clearance, closing speed, id, (x, y)) of the nearest person.

    Closing speed is -d(clearance)/dt over ``window`` for one track id, so it is
    relative: a robot already backing away lowers it. None before two samples.
    """
    out, hist = [], []
    for t, pts in ped:
        if not pts:
            hist.clear()
            out.append((t, None, None, None, None))
            continue
        i, x, y = min(pts, key=lambda p: clearance((p[1], p[2])))
        c = clearance((x, y))
        hist = [h for h in hist if h[1] == i and t - h[0] <= window] + [(t, i, c)]
        v = None
        if len(hist) >= 2 and hist[-1][0] - hist[0][0] > 0.15:
            v = -(hist[-1][2] - hist[0][2]) / (hist[-1][0] - hist[0][0])
        out.append((t, c, v, i, (x, y)))
    return out


def binding_speed(c, gamma=PERSON_GAMMA):
    """Fastest relative approach the CBF person row allows at clearance c (m/s)."""
    return gamma * (c - D_MIN)


def reaction_episodes(ped, gamma=PERSON_GAMMA):
    """Episodes where the CBF's person constraint is violated by the current motion.

    Onset: relative closing speed > gamma * (clearance - D_MIN) + ONSET_EXCESS, i.e. the row
    n.(u - v) >= -gamma h is violated and the controller has to act. Because the
    speed is relative, a robot already retreating does not trigger it.

    Outcome of each episode:
      restored    closing falls back within the allowed speed (+RESTORE_TOL) for
                  RESTORE_HOLD_S; delay = time from onset; rho = REACT_S - delay
      track_lost  the person disappears for more than HOLD_S before that
      unrestored  neither by the end of the run (or EPISODE_MAX_S)
    """
    cc = clearance_closing(ped)
    eps, cur = [], None
    for k, (t, c, v, i, xy) in enumerate(cc):
        if cur is not None:
            ok = c is not None and v is not None and v <= binding_speed(c, gamma) + RESTORE_TOL
            if c is None:
                if t - cur['last_seen'] > HOLD_S:
                    cur.update(status='track_lost', end=t)
            elif ok:
                cur.setdefault('ok_since', t)
                if t - cur['ok_since'] >= RESTORE_HOLD_S:
                    cur.update(status='restored', end=cur['ok_since'], delay=cur['ok_since'] - cur['t'])
            else:
                cur.pop('ok_since', None)
            if c is not None:
                cur['last_seen'] = t
            if cur.get('status') is None and t - cur['t'] > EPISODE_MAX_S:
                cur.update(status='unrestored', end=t)
            if cur.get('status'):
                cur = None
        if cur is None and c is not None and v is not None and c < NEAR_M and v > binding_speed(c, gamma) + ONSET_EXCESS:
            if eps and t - eps[-1]['end'] < 1.0:
                continue   # one sustained approach, not a new onset
            cur = dict(t=t, clearance=c, closing=v, allowed=binding_speed(c, gamma), last_seen=t, status=None)
            eps.append(cur)
    for e in eps:
        if e.get('status') is None:
            e.update(status='unrestored', end=cc[-1][0])
        e['rho'] = (REACT_S - e['delay']) if e['status'] == 'restored' else None
    return eps


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
    eps = reaction_episodes(ped)
    res['phi_react'] = [dict(t=e['t'] - t0, clearance=e['clearance'], closing=e['closing'],
                             allowed=e['allowed'], status=e['status'], delay=e.get('delay'),
                             rho=e['rho']) for e in eps]
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
        d = {'restored': 'restored in %.2f s (rho %+.2f)' % (e['delay'] or 0, e['rho'] or 0),
             'track_lost': 'person track lost', 'unrestored': 'not restored within %.0f s' % EPISODE_MAX_S}[e['status']]
        print('  phi_react  t=%.1f s clearance %.2f m closing %.2f > allowed %.2f m/s -> %s'
              % (e['t'], e['clearance'], e['closing'], e['allowed'], d))
    n = len(r['phi_react'])
    if n:
        print('  phi_react  %d episodes: %d restored, %d track lost, %d unrestored'
              % (n, *(sum(e['status'] == k for e in r['phi_react']) for k in ('restored', 'track_lost', 'unrestored'))))


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
