#!/usr/bin/env python3
"""Does feeding the lidar person tracks to the CBF help?  A simulated robot, driven
through recorded person runs (scripts/record_person_lidar.sh), with and without them.

    python3 src/tools/nav/lidar_cbf_sim.py artifacts/person-lidar-2026-11-23/yard-*
    python3 src/tools/nav/lidar_cbf_sim.py RUN --check      # geometry gates only

"Simulation" here means: the recorded world (lidar returns and the people, placed in
odom with the real robot's pose) and a simulated robot (command delay + acceleration
limit, the live CBF tuning of the X3 v2) driven through it.  Gazebo cannot do this:
it has no Livox person candidates or camera rays for the tracker.

Arms (same world, same nominal, same CBF; only the people handed to it differ)
  scan        lidar returns only (/scan inside the CBF range gate), no person terms
  camera      + the front camera's people (/pedestrian_states), through PersonMemory and
              the server's mover gate (_cbf_movers) -- what the robot runs today
  cam+lidar   + the lidar person tracks (lidar_persons.ndjson rerun through the current
              tracker, to_local -> cbf_estimates -> PersonMemory), as X3_LIDAR_PERSONS_CBF=1

Nominal.  The robot was parked, so with u = 0 the CBF can only retreat.  Episodes give
it a reason to move: whenever a confirmed person track comes within PUSH_TRIGGER_M of
the real robot (episodes PUSH_GAP_S apart), the simulated robot starts at the real
robot's pose and is asked to drive at PUSH_SPEED for PUSH_S straight at where that
person was at the start (any direction: front, side or rear).  Plus a whole-run zero
nominal pass for self-motion (an empty run should not move at all).

Truth (independent of the tracker): clearance from the CBF outline to NON-static lidar
returns (odom cells occupied in < 35% of scans, orca_cbf_compare's rule), i.e. the
walker's body and legs.  Also scored: time under 0.5 m, and the reaction lead = how far
from the person the commanded speed first fell below 70% of the request.

Limits: the walker's real path is kept (they reacted to the parked robot, not to the
simulated one); the scans are the real robot's (no occlusion change); ToF is not
replayed; the robot keeps its recorded heading.  Compare arms, not absolute numbers.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

_SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(_SRC), str(_SRC / 'tools' / 'nav')]
import orca_cbf_compare as occ               # noqa: E402
import lidar_person_tracker as lpt           # noqa: E402
from person_memory import PersonMemory       # noqa: E402

# ── X3 v2 live tuning (server_x3.py defaults, 2026-11-23) ──
occ.CBF_GAMMA = 1.0
occ.CBF_FOOTPRINT = (0.32, 0.32, 0.293, 0.293)
occ.CBF_MARGIN = 0.05
occ.LIDAR_X = -0.0025
occ._FC = np.array([0.5 * (occ.CBF_FOOTPRINT[0] - occ.CBF_FOOTPRINT[1]),
                    0.5 * (occ.CBF_FOOTPRINT[2] - occ.CBF_FOOTPRINT[3])])
occ._HALF = np.array([0.5 * (occ.CBF_FOOTPRINT[0] + occ.CBF_FOOTPRINT[1]),
                      0.5 * (occ.CBF_FOOTPRINT[2] + occ.CBF_FOOTPRINT[3])])
SPEED_RANGE_M, MOVER_MIN_SPEED, MIN_PERSON_FRAMES, JUMP_HOLD = 1.8, 0.15, 5, 10   # _cbf_movers

PUSH_TRIGGER_M, PUSH_GAP_S, PUSH_S, PUSH_SPEED = 2.5, 8.0, 4.0, 0.3
TICK = 1.0 / occ.TICK_HZ
ROLES = ('front', 'rear', 'left', 'right')


def cbf_movers(estimates):
    """server_x3._cbf_movers."""
    out = []
    for e in estimates:
        if e.get('z', 0.0) > SPEED_RANGE_M or e.get('category') != 'person':
            continue
        if (e.get('age', 0) > 0 or e.get('person_frames', MIN_PERSON_FRAMES) < MIN_PERSON_FRAMES
                or e.get('jump_age', JUMP_HOLD) < JUMP_HOLD):
            continue
        if e.get('speed', 0.0) > MOVER_MIN_SPEED:
            out.append(((float(e['z']), -float(e['x'])), (float(e['vx']), float(e['vy']))))
    return out


def replay_tracks(ndjson, odom):
    """Rerun the current tracker; returns [(stamp, [track dict in odom])]."""
    tr, out = lpt.LidarPersonTracker(), []
    for line in open(ndjson):
        r = json.loads(line)
        now, names = r['stamp'], r['fields']
        cands = [dict(zip(names, map(float, c))) for c in r['candidates']]
        if r.get('seeded'):
            cands = lpt.merge_seeded(cands, r['seeded'], names, now, now)
        side = None
        if r.get('side') is not None and 'side_fields' in r:
            sf = r['side_fields']
            f = {k: i for i, k in enumerate(sf)}
            cands = lpt.merge_seeded(cands, r['side'], sf, r['side_stamp'], now)
            side = [{'id': f"{ROLES[int(x[f['cam']])]}:{int(x[f['tid']])}", 'x': x[f['ax']],
                     'y': x[f['ay']], 'last_seen': r['side_stamp'] - x[f['age']]} for x in r['side']]
        x, y, _ = occ.pose_at(odom, now)
        a = r.get('packet_age')
        res = tr.update(now, cands, r.get('camera'), math.inf if a is None else a, origin=(x, y), side=side)
        out.append((now, [dict(t) for t in res]))
    return out


class World:
    def __init__(self, run_dir):
        self.run = occ.to_world(occ.load_run(run_dir))
        self.tracks = replay_tracks(Path(run_dir) / 'lidar_persons.ndjson', self.run['odom'])
        self.track_t = np.array([t for t, _ in self.tracks])

    def lidar_tracks(self, t, pos, yaw):
        """Tracks at time t as to_local() would give them for a robot at (pos, yaw)."""
        k = int(np.searchsorted(self.track_t, t, side='right')) - 1
        if k < 0 or t - self.track_t[k] > 0.3:
            return []
        stamp, res = self.tracks[k]
        return lpt.to_local(res, {'x': pos[0], 'y': pos[1], 'theta': yaw}), stamp

    def camera_estimates(self, t, pos, yaw):
        """/pedestrian_states as velocity_estimator estimates for a robot at (pos, yaw)."""
        _, people, _ = occ.scene(self.run, t, pos, yaw)
        return [{'id': str(i), 'category': 'person', 'age': 0, 'z': float(p[0]), 'x': -float(p[1]),
                 'vx': float(v[0]), 'vy': float(v[1]), 'speed': float(math.hypot(*v)),
                 'person_frames': MIN_PERSON_FRAMES, 'jump_age': JUMP_HOLD} for i, p, v, _ in people]


def people_for(arm, world, mem, t, pos, yaw):
    if arm == 'scan':
        return []
    est = world.camera_estimates(t, pos, yaw)
    if arm == 'cam+lidar':
        lt = world.lidar_tracks(t, pos, yaw)
        if lt:
            local, stamp = lt
            est = est + lpt.cbf_estimates(local, est, stamp)
    moving = mem.update(est, t)
    return [(None, np.array(p), np.array(v), True) for p, v in moving]


def dyn_clearance(near):
    pts, is_static = near
    d = occ.rect_dist(pts[~is_static]) if len(pts) and (~is_static).any() else np.zeros(0)
    return float(d.min()) if len(d) else math.nan


def run_episode(world, arm, t0, t1, u_dir, speed):
    """Simulate [t0, t1) from the real pose; u_dir: unit body-frame request direction."""
    ctl = occ.CBFController(occ.CBF_GAMMA, occ.PERSON_GAMMA)
    mem = PersonMemory()
    x, y, _ = occ.pose_at(world.run['odom'], t0)
    pos, v = np.array([x, y]), np.zeros(2)
    queue = [np.zeros(2)] * int(round(0.1 / TICK))
    rows = []
    for t in np.arange(t0, t1, TICK):
        yaw = occ.pose_at(world.run['odom'], t)[2]
        lidar, _, near = occ.scene(world.run, t, pos, yaw)
        people = people_for(arm, world, mem, t, pos, yaw)
        u_nom = np.array([u_dir[0] * speed, u_dir[1] * speed, 0.0])
        u = ctl(u_nom, v, lidar, people)
        queue.append(np.asarray(u, float))
        ue = queue.pop(0)
        dv = ue - v
        n = math.hypot(*dv)
        if n > 1.0 * TICK:
            dv *= TICK / n
        v = v + dv
        pos = pos + occ.rot(yaw) @ v * TICK
        rows.append((t - t0, *pos, *v, *u, dyn_clearance(near), len(people), speed))
    return np.array(rows)


def episodes(world):
    """(t, body-frame unit direction at the person, real-robot range) per push."""
    out, last = [], -1e9
    odom = world.run['odom']
    for t, res in world.tracks:
        if t - last < PUSH_GAP_S or t + PUSH_S > odom[-1, 0] or t < odom[0, 0]:
            continue
        x, y, yaw = occ.pose_at(odom, t)
        best = None
        for r in res:
            if not r['confirmed']:
                continue
            d = math.hypot(r['odom_x'] - x, r['odom_y'] - y)
            if d < PUSH_TRIGGER_M and (best is None or d < best[0]):
                best = (d, r)
        if best:
            d, r = best
            b = occ.rot(yaw).T @ (r['odom_x'] - x, r['odom_y'] - y)
            out.append((t, b / np.linalg.norm(b), d))
            last = t
    return out


def score_ep(tr):
    t, u, c = tr[:, 0], tr[:, 5:7], tr[:, 7]
    req = tr[:, 9]
    ok = np.isfinite(c)
    slow = np.flatnonzero(np.hypot(*u.T) < 0.7 * req)
    lead = float(c[slow[0]]) if len(slow) and ok[slow[0]] else math.nan
    return dict(min_clear=float(np.nanmin(c)) if ok.any() else math.nan,
                under_05_s=float(TICK * np.sum(ok & (c < 0.5))),
                react_at=lead,
                travel=float(np.hypot(*(tr[-1, 1:3] - tr[0, 1:3]))))


def check(world):
    """Gate 1: lidar tracks sit on the walker's scan returns (frame/flip check)."""
    run, errs = world.run, []
    for t, res in world.tracks[::10]:
        k = int(np.searchsorted(run['scan_t'], t)) - 1
        if k < 0:
            continue
        w, st = run['wscans'][k][1], run['wscans'][k][2]
        dyn = w[~st]
        for r in res:
            if r['confirmed'] and len(dyn):
                errs.append(float(np.min(np.hypot(dyn[:, 0] - r['odom_x'], dyn[:, 1] - r['odom_y']))))
    e = np.array(errs)
    return dict(n=len(e), median=float(np.median(e)) if len(e) else math.nan,
                within_03=float(np.mean(e < 0.3)) if len(e) else math.nan)


ARMS = ('scan', 'camera', 'cam+lidar')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--check', action='store_true', help='geometry gate only')
    ap.add_argument('--json')
    a = ap.parse_args()
    import logging
    logging.getLogger('cbf_filter').setLevel(logging.ERROR)
    allres = []
    for rd in a.runs:
        w = World(rd)
        g = check(w)
        print(f'== {Path(rd).name}: track-on-walker gate: {g["n"]} samples, median {g["median"]:.2f} m, '
              f'{100 * g["within_03"]:.0f}% within 0.3 m')
        if a.check:
            continue
        res = {'run': Path(rd).name, 'gate': g, 'episodes': [], 'zero': {}}
        t0, t1 = occ.span(w.run)
        for arm in ARMS:   # zero nominal, whole run: self-motion
            tr = run_episode(w, arm, t0, t1, np.zeros(2), 0.0)
            res['zero'][arm] = float(np.sum(np.hypot(*np.diff(tr[:, 1:3], axis=0).T)))
        print('  zero nominal, path moved (m): ' + '  '.join(f'{k} {v:.2f}' for k, v in res['zero'].items()))
        for t, d, rng in episodes(w):
            bearing = math.degrees(math.atan2(d[1], d[0]))
            e = {'t': round(t - t0, 1), 'bearing': round(bearing), 'range': round(rng, 2)}
            for arm in ARMS:
                e[arm] = score_ep(run_episode(w, arm, t, t + PUSH_S, d, PUSH_SPEED))
            res['episodes'].append(e)
            print(f'  push t={e["t"]:5.1f}s at {e["bearing"]:+4d} deg, {rng:.1f} m:  ' + '   '.join(
                f'{arm} min {e[arm]["min_clear"]:.2f} <0.5 {e[arm]["under_05_s"]:.1f}s react@{e[arm]["react_at"]:.2f}'
                for arm in ARMS))
        allres.append(res)
    if allres and not a.check:
        eps = [e for r in allres for e in r['episodes']]
        print(f'== {len(eps)} pushes over {len(allres)} runs')
        for arm in ARMS:
            mc = np.array([e[arm]['min_clear'] for e in eps])
            ra = np.array([e[arm]['react_at'] for e in eps])
            print(f'  {arm:10s} min clearance median {np.nanmedian(mc):.2f} worst {np.nanmin(mc):.2f}   '
                  f'<0.5 m total {sum(e[arm]["under_05_s"] for e in eps):.1f}s   '
                  f'reaction range median {np.nanmedian(ra):.2f}   '
                  f'zero-nominal motion {sum(r["zero"][arm] for r in allres):.2f} m')
    if a.json:
        Path(a.json).write_text(json.dumps(allres, indent=1, default=float))


if __name__ == '__main__':
    main()
