#!/usr/bin/env python3
"""Offline comparison of the CBF filter against ORCA on the recorded walking runs.

Replays each bag through several controllers and scores them the same way.

What the robot was asked to do (the nominal command, u_nom) depends on the bag:
  gated bags   (2026-10-22 on: the driver listens on /cmd_vel_safe) record
               /cmd_vel_safe = what the CBF let through and /cmd_vel_nominal = what
               was asked of it (GUI joystick, Nav2, scripts, heartbeat). Every
               controller gets the request as u_nom. Bags from before
               /cmd_vel_nominal fall back to /cmd_vel, which misses GUI commands.
               Record them with scripts/record_walk.sh.
  older bags   (artifacts/cbf-2026-10-12, cbf-2026-10-16) only have /cmd_vel, which
               was then the CBF OUTPUT; those runs are a parked robot, so u_nom = 0.
--nominal overrides this: zero, cmd (the recorded request) or track (steer back
toward the real robot's recorded path 1 s ahead, at least as fast as the request:
for goal runs, so a controller that detours can rejoin; never for walk-ups at a
parked robot, where it would steer toward the walker).

Controllers
  recorded     what the robot actually did (/odom path, the executed command)
  cbf          src/cbf_filter.py with the live tuning (static gamma 2, person
               radius 0.25 / gamma 1, footprint + 0.10 margin, deadband 0, min 0.03)
               followed by the server's executed-speed re-check
  orca         ORCA (RVO2, van den Berg et al. 2011) from ~/RVO2-python: the robot
               is a disc round the footprint, people are agents, lidar returns are
               static zero-radius agents whose lines are kept hard. The robot takes
               FULL responsibility (people do not run ORCA)
  orca-recip   the same with the textbook reciprocal half share for people

Two passes
  open loop    at every executed-command instant, the command each controller
               would give in the real robot's scene. For cbf this checks the replay
               against what the robot did; for ORCA it shows when and how hard it
               reacts compared with the CBF.
  closed loop  the robot is simulated (command delay + acceleration limit) and
               driven by each controller through the recorded world: lidar returns
               and person tracks are placed in the odom frame with the real robot's
               pose, so they stay where they physically were while the simulated
               robot moves. `--check-dynamics` drives the model with the executed
               commands to show how well it reproduces /odom.

Limits (read before quoting numbers)
  - Walkers reacted to the real robot (they stopped when it backed away). The
    replay keeps their recorded paths, so a controller that retreats less than the
    real one meets a walker who would, in reality, have stopped earlier.
  - Simulated lidar is the real robot's scans: no occlusion change when the
    simulated robot is elsewhere, and only what the real robot saw.
  - /pedestrian_states lacks the tracker's category / person_frames / age, so the
    server's mover gate is approximated by speed > 0.15 m/s within 1.8 m. Every
    controller gets the same people.
  - The ToF arrays are not replayed (they see below the lidar plane; walkers' feet).
  - Not replayed either: person_memory (ghosts / id re-links), the people speed cap,
    the Nav2 people costmap. The live CBF had them in gated bags, so cbf open-loop
    agreement with /cmd_vel_safe is expected to be lower there.
  - The simulated robot keeps the real robot's heading; a request's yaw rate only
    costs wheel speed (and footprint rows in the CBF). The recorded request does not
    react to the simulated robot (Nav2 replanned for the real one): use --nominal
    track for goal runs and read "end gap" as "did it get there".
  - Clearance is measured from the CBF's own outline (about the base origin).
  - "min lidar" is any return (the walker's legs included); "min stat" only returns in
    odom cells occupied in >= 35% of the scans (furniture, walls).

Usage
  python3 src/tools/nav/orca_cbf_compare.py artifacts/cbf-2026-10-16/walk-g-r6 [...]
  python3 src/tools/nav/orca_cbf_compare.py artifacts/cbf-2026-10-1*/walk-* --plot out_dir --json f.json
  python3 src/tools/nav/orca_cbf_compare.py artifacts/walk-2026-11-*/goal-* --nominal track
Needs ROS (rclpy, x3_msgs) sourced to decode the bags.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
import json
import logging
import math
import os
import sqlite3
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(_SRC_DIR)))
from cbf_filter import HolonomicCBFFilter  # noqa: E402

# ── live tuning during the walking runs (robot drop-in, 2026-10-12 / 10-16) ──
CBF_GAMMA = 2.0
CBF_FOOTPRINT = (0.16, 0.13, 0.11, 0.11)       # front, rear, left, right about base_link
CBF_MARGIN = 0.10
PERSON_RADIUS = 0.25
PERSON_GAMMA = 1.0
CMD_DEADBAND = 0.0                             # closed-loop wheel speed control
CMD_MIN = 0.03
MAX_WHEEL = 0.5
MOVER_RANGE = 1.8                              # CBF_SPEED_RANGE_M (forward distance)
MOVER_MIN_SPEED = 0.15                         # CBF_MOVER_MIN_SPEED
LIDAR_X = 0.0435
LIDAR_YAW_OFFSET = math.radians(4.85)
SCAN_MIN, SCAN_MAX = 0.12, 1.0                 # _scan_cb range gate
PED_STALE_S = 0.5                              # a /pedestrian_states message older than this is ignored
TICK_HZ = 20.0

# ── ORCA settings ──
ORCA_TAU = 2.0           # s, time horizon for people
ORCA_TAU_OBST = 1.0      # s, time horizon for lidar returns
ORCA_SECTOR_DEG = 10.0   # lidar returns thinned to the nearest one per sector
ORCA_DIR = os.environ.get('RVO2_PYTHON_DIR', str(Path.home() / 'RVO2-python' / 'ORCA'))

_FC = np.array([0.5 * (CBF_FOOTPRINT[0] - CBF_FOOTPRINT[1]), 0.5 * (CBF_FOOTPRINT[2] - CBF_FOOTPRINT[3])])
_HALF = np.array([0.5 * (CBF_FOOTPRINT[0] + CBF_FOOTPRINT[1]), 0.5 * (CBF_FOOTPRINT[2] + CBF_FOOTPRINT[3])])
ORCA_ROBOT_RADIUS = float(np.hypot(*_HALF)) + CBF_MARGIN   # disc round the outline, plus the margin


# ─────────────────────────────── geometry ───────────────────────────────

def rect_dist(p):
    """Distance from body-frame points p (N, 2) to the CBF outline (0 inside)."""
    q = np.abs(np.asarray(p, float).reshape(-1, 2) - _FC) - _HALF
    return np.hypot(np.maximum(q[:, 0], 0.0), np.maximum(q[:, 1], 0.0))


def rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


# ─────────────────────────────── bag loading ───────────────────────────────

def _reader(db):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    c = sqlite3.connect('file:%s?mode=ro' % db, uri=True)
    top = {n: (i, t) for i, n, t in c.execute('select id,name,type from topics')}

    def rows(name):
        if name not in top:
            return
        i, ty = top[name]
        T = get_message(ty)
        for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
            yield ts * 1e-9, deserialize_message(d, T)
    return rows


def load_run(path):
    p = Path(path)
    db = next(iter(sorted(p.glob('bag/*.db3')))) if p.is_dir() else p
    rows = _reader(db)
    odom = []
    for t, m in rows('/odom'):
        q = m.pose.pose.orientation
        odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    odom = np.array(odom)
    odom[:, 3] = np.unwrap(odom[:, 3])
    def twists(topic):
        return np.array([(t, m.linear.x, m.linear.y, m.angular.z) for t, m in rows(topic)]).reshape(-1, 4)
    cmd, safe, req = twists('/cmd_vel'), twists('/cmd_vel_safe'), twists('/cmd_vel_nominal')
    gated = len(safe) > 0
    # gated: /cmd_vel_safe is what ran. The request is /cmd_vel_nominal (everything
    # that entered the CBF, GUI joystick included) or, in bags from before that topic,
    # /cmd_vel (Nav2 and scripts only). Before the gate /cmd_vel was the CBF output
    # and nobody recorded the request.
    if gated:
        executed, nominal = safe, (req if len(req) else cmd)
    else:
        executed, nominal = cmd, np.zeros((0, 4))
    ped = [(t, [(s.id, s.position.x, s.position.y, s.velocity.x, s.velocity.y) for s in m.pedestrians])
           for t, m in rows('/pedestrian_states')]
    scans = []
    for t, m in rows('/scan'):
        r = np.asarray(m.ranges, float)
        a = m.angle_min + LIDAR_YAW_OFFSET + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > SCAN_MIN) & (r < 3.0)
        # _scan_cb: laser_link is yawed by pi, lidar at x = 0.0435
        scans.append((t, np.column_stack([-r[ok] * np.cos(a[ok]) + LIDAR_X, -r[ok] * np.sin(a[ok])])))
    return dict(name=p.name, odom=odom, cmd=executed, nominal=nominal, gated=gated, ped=ped, scans=scans)


CELL_M, STATIC_FRAC = 0.10, 0.35


def _cell_keys(w):
    c = np.floor(w / CELL_M).astype(np.int64)
    return c[:, 0] * 100003 + c[:, 1]


def pose_at(odom, t):
    return (float(np.interp(t, odom[:, 0], odom[:, 1])), float(np.interp(t, odom[:, 0], odom[:, 2])),
            float(np.interp(t, odom[:, 0], odom[:, 3])))


def to_world(run):
    """Scan points and person tracks in the odom frame, with the real robot's pose."""
    odom = run['odom']
    ws, counts = [], {}
    for t, pts in run['scans']:
        x, y, th = pose_at(odom, t)
        w = pts @ rot(th).T + (x, y)
        keys = _cell_keys(w)
        ws.append((t, w, keys))
        for k in np.unique(keys):
            counts[k] = counts.get(k, 0) + 1
    # static scenery: odom-frame cells occupied in at least STATIC_FRAC of the scans
    # (stl_shadow_monitor's rule), so contact with furniture is told apart from the walker
    static = np.array(sorted(k for k, c in counts.items() if c >= STATIC_FRAC * len(ws)), dtype=np.int64)
    ws = [(t, w, np.isin(keys, static)) for t, w, keys in ws]
    wp = []
    for t, people in run['ped']:
        x, y, th = pose_at(odom, t)
        R = rot(th)
        out = []
        for i, px, py, vx, vy in people:
            # velocity: the estimator's own (ego-compensated) velocity in base axes
            out.append((i, R @ (px, py) + (x, y), R @ (vx, vy)))
        wp.append((t, out))
    run['wscans'], run['wped'] = ws, wp
    run['scan_t'] = np.array([s[0] for s in ws])
    run['ped_t'] = np.array([p[0] for p in wp])
    return run


def scene(run, t, pos, yaw):
    """What a robot at world pose (pos, yaw) senses at time t, in its body frame.

    Returns (lidar points inside the scan gate, people [(id, p, v, is_mover)], all
    returns for scoring, which of those are static scenery)."""
    Rt = rot(yaw).T
    k = int(np.searchsorted(run['scan_t'], t, side='right')) - 1
    pts, is_static = np.zeros((0, 2)), np.zeros(0, bool)
    if k >= 0:
        pts = (run['wscans'][k][1] - pos) @ Rt.T
        is_static = run['wscans'][k][2]
    r = np.hypot(pts[:, 0] - LIDAR_X, pts[:, 1])
    lidar = pts[(r > SCAN_MIN) & (r < SCAN_MAX)]
    people = []
    j = int(np.searchsorted(run['ped_t'], t, side='right')) - 1
    if j >= 0 and t - run['ped_t'][j] <= PED_STALE_S:
        for i, pw, vw in run['wped'][j][1]:
            p = Rt @ (pw - pos)
            v = Rt @ vw
            mover = p[0] <= MOVER_RANGE and math.hypot(*v) > MOVER_MIN_SPEED
            people.append((i, p, v, mover))
    return lidar, people, (pts, is_static)


# ─────────────────────────────── nominal command ───────────────────────────────

WATCHDOG_S = 0.5          # the driver stops a command older than this
TRACK_LOOKAHEAD_S = 1.0


def resolve_nominal(run, mode):
    if mode == 'auto':
        return 'cmd' if run['gated'] else 'zero'
    if mode in ('cmd', 'track') and not run['gated']:
        raise SystemExit('%s: no /cmd_vel_safe in this bag, so /cmd_vel is the CBF output, '
                         'not a request; use --nominal zero' % run['name'])
    return mode


def recorded_request(run, t):
    """(vx, vy, wz) requested at t, body frame; zero once the watchdog would have fired."""
    nom = run['nominal']
    k = int(np.searchsorted(nom[:, 0], t, side='right')) - 1 if len(nom) else -1
    if k < 0 or t - nom[k, 0] > WATCHDOG_S:
        return np.zeros(3)
    return nom[k, 1:4].copy()


def nominal_at(run, mode, t, pos, yaw):
    """u_nom (vx, vy, wz) for a robot at world pose (pos, yaw) at time t."""
    if mode == 'zero':
        return np.zeros(3)
    u = recorded_request(run, t)
    if mode == 'track':
        x, y, _ = pose_at(run['odom'], t + TRACK_LOOKAHEAD_S)
        want = rot(yaw).T @ ((np.array([x, y]) - pos) / TRACK_LOOKAHEAD_S)
        speed = math.hypot(*want)
        cap = max(math.hypot(u[0], u[1]), 0.1)      # catch up, but not much faster than asked
        if speed > cap:
            want *= cap / speed
        u[:2] = want if speed > 0.02 else 0.0
    return u


# ─────────────────────────────── controllers ───────────────────────────────

class CBFController:
    name = 'cbf'

    def __init__(self, gamma=CBF_GAMMA, person_gamma=PERSON_GAMMA, name='cbf'):
        self.f = HolonomicCBFFilter(gamma=gamma, footprint=CBF_FOOTPRINT, margin=CBF_MARGIN,
                                    max_wheel_speed=MAX_WHEEL)
        self.person_gamma, self.name = person_gamma, name

    def __call__(self, u_nom, v_now, lidar, people):
        """u_nom (vx, vy, wz); returns the translation (vx, vy)."""
        obstacles = [tuple(p) for p in lidar]
        moving = [(tuple(p), tuple(v)) for _, p, v, m in people if m]
        vel = radii = gammas = soft = None
        if moving:
            n = len(obstacles)
            vel = [(0.0, 0.0)] * n + [v for _, v in moving]
            radii = [0.0] * n + [PERSON_RADIUS] * len(moving)
            gammas = [self.f.gamma] * n + [self.person_gamma] * len(moving)
            soft = [False] * n + [True] * len(moving)
            obstacles = obstacles + [p for p, _ in moving]
        wz = float(u_nom[2]) if len(u_nom) > 2 else 0.0
        if not obstacles:
            vx, vy = self.f.filter_velocity(u_nom[0], u_nom[1], (0.0, 0.0), [], omega=wz)
            return np.array([vx, vy])
        vx, vy, wz = self.f.filter_twist(u_nom[0], u_nom[1], wz, (0.0, 0.0), obstacles, vel, radii, gammas, soft)
        return np.array(limit_for_deadband(self.f, vx, vy, obstacles, vel, radii, gammas, soft, omega=wz))


def limit_for_deadband(cbf, vx, vy, obstacles, velocities=None, radii=None, gammas=None, soft=None,
                       deadband=CMD_DEADBAND, cmd_min=CMD_MIN, omega=0.0):
    """server_x3.ROS2Bridge._limit_for_deadband."""
    s = math.hypot(vx, vy)
    if s < 1e-6 or not len(obstacles):
        return vx, vy
    dx, dy = vx / s, vy / s
    n, c, keep = cbf.constraint_rows((0.0, 0.0), obstacles, velocities, omega, radii, gammas, with_keep=True)
    if not len(c):
        return vx, vy
    n_dot_d = n[:, 0] * dx + n[:, 1] * dy
    if np.all(c <= 1e-9):
        limiting = n_dot_d < 0
        if not limiting.any():
            return vx, vy
        s_max = float(np.min(c[limiting] / n_dot_d[limiting])) - deadband
        if s_max <= 0.0 or s_max < min(s, cmd_min):
            return 0.0, 0.0
        if s_max < s:
            return dx * s_max, dy * s_max
        return vx, vy
    executed = np.linspace(deadband, s + deadband, 33)
    margin = n_dot_d[:, None] * executed[None, :] - c[:, None]
    worst = np.min(margin, axis=0)
    firm = None if soft is None else ~np.asarray(soft, dtype=bool)[keep]
    if firm is not None and firm.any() and not firm.all():
        firm_ok = np.min(margin[firm], axis=0) >= -1e-6
        if firm_ok.any():
            worst = np.where(firm_ok, worst, -np.inf)
    best = int(np.argmax(worst))
    if worst[best] <= float(np.min(-c)) + 1e-9:
        return 0.0, 0.0
    s_cmd = max(float(executed[best]) - deadband, cmd_min)
    return dx * s_cmd, dy * s_cmd


def _load_rvo2():
    """The RVO2-python port: Vector2, Line and the 2-D/3-D linear programs."""
    if ORCA_DIR not in sys.path:
        sys.path.append(ORCA_DIR)       # appended, so its flat module names shadow nothing here
    try:
        import Agent as rvo_agent
        import Line as rvo_line
        import Vector2 as rvo_vec
    except ImportError as e:
        raise SystemExit('RVO2-python not found at %s (set RVO2_PYTHON_DIR): %s' % (ORCA_DIR, e))
    return rvo_agent, rvo_line, rvo_vec


class ORCAController:
    """One ORCA agent (the robot) against people and lidar returns.

    Agent.computeNewVelocity hard-codes the reciprocal half share, so the
    velocity-obstacle lines are built here (the agent-agent case of RVO2's
    Agent.cpp, with the share as a parameter) and solved with the port's own
    linearProgram2 / linearProgram3. Lidar returns go first as "obstacle lines",
    which linearProgram3 never relaxes: firm statics, soft people, like the CBF.
    """

    def __init__(self, share=1.0, tau=ORCA_TAU, tau_obst=ORCA_TAU_OBST, dt=1.0 / TICK_HZ, name='orca'):
        self.agent, self.line, self.vec = _load_rvo2()
        self.share, self.tau, self.tau_obst, self.dt, self.name = share, tau, tau_obst, dt, name

    def _line(self, rel_p, rel_v, v_self, radius, tau, share):
        V = self.vec.Vector2
        rp, rv = V(*map(float, rel_p)), V(*map(float, rel_v))
        dist_sq = rp * rp
        rsq = radius * radius
        inv_tau = 1.0 / tau
        ln = self.line.Line()
        if dist_sq > rsq:
            w = rv - rp * inv_tau
            w_sq = w * w
            dot = w * rp
            if dot < 0.0 and dot * dot > rsq * w_sq:
                wl = math.sqrt(w_sq)
                uw = w / wl
                ln.direction = V(uw.y_, -uw.x_)
                u = uw * (radius * inv_tau - wl)
            else:
                leg = math.sqrt(dist_sq - rsq)
                if self.vec.det(rp, w) > 0.0:
                    ln.direction = V(rp.x_ * leg - rp.y_ * radius, rp.x_ * radius + rp.y_ * leg) / dist_sq
                else:
                    ln.direction = -V(rp.x_ * leg + rp.y_ * radius, -rp.x_ * radius + rp.y_ * leg) / dist_sq
                u = ln.direction * (rv * ln.direction) - rv
        else:
            inv_dt = 1.0 / self.dt
            w = rv - rp * inv_dt
            wl = math.sqrt(w * w)
            uw = w / wl if wl > 1e-9 else V(-rp.x_, -rp.y_) / max(math.sqrt(dist_sq), 1e-9)
            ln.direction = V(uw.y_, -uw.x_)
            u = uw * (radius * inv_dt - wl)
        ln.point = V(*map(float, v_self)) + u * share
        return ln

    def __call__(self, u_nom, v_now, lidar, people):
        # everything relative to the footprint centre, where the disc is
        lines = []
        if len(lidar):
            rel = lidar - _FC
            ang = np.floor(np.degrees(np.arctan2(rel[:, 1], rel[:, 0])) / ORCA_SECTOR_DEG)
            d = np.hypot(rel[:, 0], rel[:, 1])
            for a in np.unique(ang):
                k = np.flatnonzero(ang == a)
                p = rel[k[np.argmin(d[k])]]
                lines.append(self._line(p, v_now, v_now, ORCA_ROBOT_RADIUS, self.tau_obst, 1.0))
        n_obst = len(lines)
        for _, p, v, mover in people:
            vv = v if mover else np.zeros(2)
            lines.append(self._line(p - _FC, v_now - vv, v_now, ORCA_ROBOT_RADIUS + PERSON_RADIUS,
                                    self.tau, self.share))
        # a yaw rate uses up wheel speed, as in the CBF's wheel limit
        vmax = max(MAX_WHEEL - 0.215 * abs(float(u_nom[2]) if len(u_nom) > 2 else 0.0), 0.0)
        if not lines:
            u = np.array(u_nom[:2], float)
        else:
            V = self.vec.Vector2
            pref = V(float(u_nom[0]), float(u_nom[1]))
            fail, res = self.agent.linearProgram2(lines, vmax, pref, False, V())
            if fail < len(lines):
                res = self.agent.linearProgram3(lines, n_obst, fail, vmax, res)
            u = np.array([res.x_, res.y_])
        s = abs(u[0]) + abs(u[1])
        if s > vmax:                            # disc -> mecanum wheel diamond
            u *= vmax / s
        if math.hypot(*u) < CMD_MIN:
            u[:] = 0.0
        return u


# ─────────────────────────────── passes ───────────────────────────────

def span(run):
    t0 = max(run['odom'][0, 0], run['scan_t'][0], run['ped_t'][0] if len(run['ped_t']) else -np.inf)
    t1 = min(run['odom'][-1, 0], run['scan_t'][-1])
    return t0, t1


def open_loop(run, controllers, mode):
    """Each controller's command in the REAL robot's scene at every executed command.
    (track mode has no meaning here -- the robot is where it really was -- so it
    uses the recorded request.)"""
    ol_mode = 'cmd' if mode == 'track' else mode
    t0, t1 = span(run)
    cmd = run['cmd'][(run['cmd'][:, 0] >= t0) & (run['cmd'][:, 0] <= t1)]
    out = {c.name: [] for c in controllers}
    rng, noms = [], []
    for t, vx, vy, _ in cmd:
        x, y, th = pose_at(run['odom'], t)
        # the robot's velocity in body axes, from odom
        dt = 0.1
        xa, ya, _ = pose_at(run['odom'], t - dt)
        v_now = rot(th).T @ ((x - xa) / dt, (y - ya) / dt)
        lidar, people, _ = scene(run, t, np.array([x, y]), th)
        rng.append(min((rect_dist(p)[0] - PERSON_RADIUS for _, p, _, _ in people), default=np.nan))
        u_nom = nominal_at(run, ol_mode, t, np.array([x, y]), th)
        noms.append(u_nom[:2])
        for c in controllers:
            out[c.name].append(c(u_nom, v_now, lidar, people))
    res = dict(t=cmd[:, 0] - t0, recorded=cmd[:, 1:3], person_clearance=np.array(rng),
               nominal=np.array(noms).reshape(-1, 2))
    res.update({k: np.array(v).reshape(-1, 2) for k, v in out.items()})
    return res


def simulate(run, controller, delay=0.1, accel=1.0, commands=None, mode='zero'):
    """Closed loop through the recorded world. commands: replay these (t, vx, vy)
    instead of a controller (the dynamics check)."""
    t0, t1 = span(run)
    dt = 1.0 / TICK_HZ
    x, y, th = pose_at(run['odom'], t0)
    pos, v = np.array([x, y]), np.zeros(2)
    queue = [np.zeros(2)] * max(int(round(delay / dt)), 0)
    rows = []
    for t in np.arange(t0, t1, dt):
        th = pose_at(run['odom'], t)[2]           # heading as the real robot (no yaw commanded)
        lidar, people, near = scene(run, t, pos, th)
        u_nom = nominal_at(run, mode, t, pos, th)
        if commands is not None:
            k = int(np.searchsorted(commands[:, 0], t, side='right')) - 1
            u = commands[k, 1:3] if k >= 0 and t - commands[k, 0] < WATCHDOG_S else np.zeros(2)
        else:
            u = controller(u_nom, v, lidar, people)
        queue.append(np.asarray(u, float))
        u_exec = queue.pop(0)
        dv = u_exec - v
        n = math.hypot(*dv)
        if n > accel * dt:
            dv *= accel * dt / n
        v = v + dv
        pos = pos + rot(th) @ v * dt
        pc, lc, sc = _clearances(people, near)
        rows.append((t - t0, pos[0], pos[1], v[0], v[1], u[0], u[1], pc, lc, sc, u_nom[0], u_nom[1]))
    return np.array(rows)


def _clearances(people, near):
    """(person clearance, any-lidar-return clearance, static-scenery clearance)."""
    pts, is_static = near
    pc = min((rect_dist(p)[0] - PERSON_RADIUS for _, p, _, _ in people), default=np.nan)
    d = rect_dist(pts) if len(pts) else np.zeros(0)
    lc = float(d.min()) if len(d) else np.nan
    sc = float(d[is_static].min()) if is_static.any() else np.nan
    return pc, lc, sc


def recorded_track(run, mode='zero'):
    """The real robot scored like a simulated one."""
    t0, t1 = span(run)
    rows = []
    cmd = run['cmd']
    for t in np.arange(t0, t1, 1.0 / TICK_HZ):
        x, y, th = pose_at(run['odom'], t)
        xa, ya, _ = pose_at(run['odom'], t - 0.1)
        v = rot(th).T @ ((x - xa) / 0.1, (y - ya) / 0.1)
        k = int(np.searchsorted(cmd[:, 0], t, side='right')) - 1
        u = cmd[k, 1:3] if k >= 0 else np.zeros(2)
        _, people, near = scene(run, t, np.array([x, y]), th)
        pc, lc, sc = _clearances(people, near)
        u_nom = nominal_at(run, 'cmd' if mode == 'track' else mode, t, np.array([x, y]), th)
        rows.append((t - t0, x, y, v[0], v[1], u[0], u[1], pc, lc, sc, u_nom[0], u_nom[1]))
    return np.array(rows)


def score(tr, end_ref=None):
    """end_ref: where the real robot finished (world); end_gap = how far from it this one did."""
    t, pos, v, u, pc, lc, sc = tr[:, 0], tr[:, 1:3], tr[:, 3:5], tr[:, 5:7], tr[:, 7], tr[:, 8], tr[:, 9]
    u_nom = tr[:, 10:12]
    dt = np.diff(t, prepend=t[0])
    seen = np.isfinite(pc)
    speed = np.hypot(v[:, 0], v[:, 1])
    return dict(
        min_person_clearance=float(np.nanmin(pc)) if seen.any() else None,
        person_below_margin_s=float(dt[seen & (pc < CBF_MARGIN)].sum()),
        min_lidar_clearance=float(np.nanmin(lc)) if np.isfinite(lc).any() else None,
        lidar_below_5cm_s=float(dt[lc < 0.05].sum()),
        min_static_clearance=float(np.nanmin(sc)) if np.isfinite(sc).any() else None,
        static_below_5cm_s=float(dt[sc < 0.05].sum()),
        path_m=float(np.sum(np.hypot(*np.diff(pos, axis=0).T))),
        net_displacement_m=float(np.hypot(*(pos[-1] - pos[0]))),
        peak_speed=float(speed.max()),
        max_sideways=float(np.abs(v[:, 1]).max()),
        moving_s=float(dt[speed > 0.03].sum()),
        cmd_chatter=float(np.sum(np.hypot(*np.diff(u, axis=0).T))),
        # how much the controller overrode the request (time mean of |u - u_nom|)
        deviation=float(np.mean(np.hypot(*(u - u_nom).T))),
        end_gap=None if end_ref is None else float(np.hypot(*(pos[-1] - end_ref))),
    )


def onsets(t, cmd, clearance, thresh=0.05, gap=1.0):
    """Person clearance when each burst of commanded motion starts."""
    s = np.hypot(cmd[:, 0], cmd[:, 1]) > thresh
    out, last = [], -np.inf
    for k in np.flatnonzero(s):
        if t[k] - last > gap:
            out.append(float(clearance[k]))
        last = t[k]
    return out


def compare(run, controllers, delay, accel, check_dynamics, nominal='auto'):
    to_world(run)
    mode = resolve_nominal(run, nominal)
    ol = open_loop(run, controllers, mode)
    res = dict(name=run['name'], duration=float(span(run)[1] - span(run)[0]), gated=run['gated'],
               nominal=mode, open_loop={}, closed_loop={})
    rec = ol['recorded']
    for c in controllers:
        u = ol[c.name]
        d = np.hypot(*(u - rec).T)
        res['open_loop'][c.name] = dict(
            rms_vs_recorded=float(np.sqrt(np.mean(d ** 2))),
            agree_within_5cm_s=float(np.mean(d < 0.05)),
            mean_speed=float(np.mean(np.hypot(*u.T))),
            onset_clearance=onsets(ol['t'], u, ol['person_clearance']))
    res['open_loop']['recorded'] = dict(mean_speed=float(np.mean(np.hypot(*rec.T))),
                                        onset_clearance=onsets(ol['t'], rec, ol['person_clearance']))
    tracks = {'recorded': recorded_track(run, mode)}
    if check_dynamics:
        sim = simulate(run, None, delay, accel, commands=run['cmd'][:, :3], mode=mode)
        e = np.hypot(*(sim[:, 1:3] - tracks['recorded'][:len(sim), 1:3]).T)
        res['dynamics_check'] = dict(rms_pos_m=float(np.sqrt(np.mean(e ** 2))), max_pos_m=float(e.max()))
    for c in controllers:
        tracks[c.name] = simulate(run, c, delay, accel, mode=mode)
    end_ref = tracks['recorded'][-1, 1:3]
    for k, tr in tracks.items():
        res['closed_loop'][k] = score(tr, end_ref)
    return res, ol, tracks


# ─────────────────────────────── output ───────────────────────────────

COLS = [('min_person_clearance', 'min pers', '%+.2f'), ('person_below_margin_s', '<marg s', '%.1f'),
        ('min_lidar_clearance', 'min lidar', '%.2f'), ('lidar_below_5cm_s', '<5cm s', '%.1f'),
        ('min_static_clearance', 'min stat', '%.2f'), ('static_below_5cm_s', 'st<5cm s', '%.1f'),
        ('path_m', 'path m', '%.2f'), ('peak_speed', 'peak', '%.2f'), ('max_sideways', 'side', '%.2f'),
        ('cmd_chatter', 'chatter', '%.1f'), ('deviation', '|u-unom|', '%.3f'), ('end_gap', 'end gap', '%.2f')]


def _fmt(v, f):
    return ('%s' % '-') if v is None else f % v


def report(r):
    print('== %s  (%.0f s; executed %s, u_nom = %s)' % (
        r['name'], r['duration'], '/cmd_vel_safe' if r['gated'] else '/cmd_vel (pre-gate CBF output)',
        {'zero': '0', 'cmd': 'recorded /cmd_vel', 'track': 'track the recorded path'}[r['nominal']]))
    if 'dynamics_check' in r:
        d = r['dynamics_check']
        print('  dynamics model on executed commands vs /odom: rms %.3f m, max %.3f m' % (d['rms_pos_m'], d['max_pos_m']))
    print('  open loop (real scene, each executed-command instant):')
    for k, o in r['open_loop'].items():
        extra = '' if k == 'recorded' else '  vs recorded: rms %.3f m/s, within 5 cm/s %.0f%%' % (
            o['rms_vs_recorded'], 100 * o['agree_within_5cm_s'])
        on = ', '.join('%.2f' % c for c in o['onset_clearance'] if np.isfinite(c))
        print('    %-11s mean |u| %.3f m/s%s; person clearance at motion onsets: [%s]' % (k, o['mean_speed'], extra, on))
    print('  closed loop:  %s' % '  '.join('%9s' % h for _, h, _ in COLS))
    for k, s in r['closed_loop'].items():
        print('    %-11s  %s' % (k, '  '.join('%9s' % _fmt(s[c], f) for c, _, f in COLS)))


def summary(results):
    names = list(results[0]['closed_loop'])
    print('== all %d runs (closed loop; clearance = worst run, |u-unom| = mean, others summed)' % len(results))
    print('    %-11s  %s' % ('', '  '.join('%9s' % h for _, h, _ in COLS)))
    for k in names:
        vals = []
        for c, _, f in COLS:
            xs = [r['closed_loop'][k][c] for r in results if r['closed_loop'][k][c] is not None]
            if not xs:
                vals.append('-')
            elif c.startswith('min_'):
                vals.append(f % min(xs))
            elif c in ('peak_speed', 'max_sideways', 'end_gap'):
                vals.append(f % max(xs))
            elif c == 'deviation':
                vals.append(f % (sum(xs) / len(xs)))
            else:
                vals.append(f % sum(xs))
        print('    %-11s  %s' % (k, '  '.join('%9s' % v for v in vals)))


def plot(r, ol, tracks, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for k, tr in tracks.items():
        p0 = tr[0, 1:3]
        ax[0].plot(tr[:, 0], np.hypot(*(tr[:, 1:3] - p0).T), label=k)
        ax[1].plot(tr[:, 0], tr[:, 7], label=k)
    ax[1].axhline(CBF_MARGIN, color='k', lw=0.6, ls='--')
    if np.any(ol['nominal']):
        ax[2].plot(ol['t'], ol['nominal'][:, 0], label='request', color='k', lw=0.7, ls=':')
    for k in ['recorded'] + [k for k in ol if k not in ('t', 'recorded', 'person_clearance', 'nominal')]:
        ax[2].plot(ol['t'], ol[k][:, 0], label=k, lw=0.9)
    ax[0].set_ylabel('distance from start (m)')
    ax[1].set_ylabel('person clearance (m)')
    ax[2].set_ylabel('open-loop cmd vx (m/s)')
    ax[2].set_xlabel('t (s)')
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    ax[0].set_title('%s: closed loop (top two), open loop (bottom)' % r['name'])
    fig.tight_layout()
    fig.savefig(Path(out) / ('%s.png' % r['name']), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+', help='run dirs (containing bag/) or .db3 files')
    ap.add_argument('--delay', type=float, default=0.1, help='command-to-wheel delay in the sim (s)')
    ap.add_argument('--accel', type=float, default=1.0, help='acceleration limit in the sim (m/s^2)')
    ap.add_argument('--orca-tau', type=float, default=ORCA_TAU)
    ap.add_argument('--no-recip', action='store_true', help='skip the reciprocal ORCA variant')
    ap.add_argument('--nominal', choices=('auto', 'zero', 'cmd', 'track'), default='auto',
                    help='u_nom: auto = recorded /cmd_vel on gated bags, else 0 (see the module doc)')
    ap.add_argument('--cbf-gamma', type=float, default=CBF_GAMMA, help='static gamma of the cbf controller')
    ap.add_argument('--person-gamma', type=float, nargs='+', default=[PERSON_GAMMA],
                    help='person gamma; several values add one cbf controller each (cbf-g0.5, ...)')
    ap.add_argument('--check-dynamics', action='store_true', help='replay recorded /cmd_vel through the sim model')
    ap.add_argument('--plot', metavar='DIR', help='write one PNG per run here')
    ap.add_argument('--json', metavar='FILE', help='write all results here')
    a = ap.parse_args()
    logging.getLogger('cbf_filter').setLevel(logging.ERROR)   # the replay's infeasible warnings are expected
    if len(a.person_gamma) == 1:
        controllers = [CBFController(a.cbf_gamma, a.person_gamma[0])]
    else:
        controllers = [CBFController(a.cbf_gamma, g, name='cbf-g%g' % g) for g in a.person_gamma]
    controllers.append(ORCAController(1.0, a.orca_tau, name='orca'))
    if not a.no_recip:
        controllers.append(ORCAController(0.5, a.orca_tau, name='orca-recip'))
    if a.plot:
        Path(a.plot).mkdir(parents=True, exist_ok=True)
    results = []
    for p in a.runs:
        run = load_run(p)
        if len(run['scans']) < 2 or len(run['odom']) < 2:
            print('== %s: no scan or odom data, skipped' % run['name'])
            continue
        r, ol, tracks = compare(run, controllers, a.delay, a.accel, a.check_dynamics, a.nominal)
        report(r)
        if a.plot:
            plot(r, ol, tracks, a.plot)
        results.append(r)
    if len(results) > 1:
        summary(results)
    if a.json:
        Path(a.json).write_text(json.dumps(results, indent=1, default=float))


if __name__ == '__main__':
    main()
