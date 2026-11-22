#!/usr/bin/env python3
"""Synthetic people inserted into REAL Livox frames: exact labels, real clutter.

    python3 src/tools/perception/synthetic_people.py artifacts/person-lidar-2026-10-30/* \
        --own artifacts/lidar-person-classifier-2026-11-01/own.npz --scenes 1500 -o synth.npz

Only the people are simulated.  Each background is a recorded frame from the robot's
bags (real floor, furniture, walls, sensor noise).  The rays are that frame's own
Livox rays (direction + per-point capture time), so the non-repetitive scan pattern is
exactly the real one.  A body is a set of oriented ellipsoids (head, torso, pelvis,
upper/lower arms and legs, feet, sometimes a backpack) with a walking gait or a standing
pose, random height and girth.  Where a ray hits a body before its recorded range, the
recorded point is replaced by the hit (+ range noise); the body hides what is behind
it.  Some hits are dropped (dark clothing, grazing incidence) and then the ray has no
return at all.  Moving people are posed at each point's own capture time (5 time bins
per 100 ms frame), so they smear like real ones.

What it measures: the candidate extractor (person_candidates.extract) on our sensor
geometry, by range and by distance to the nearest other person.  Each person ends as

  found         a candidate within MATCH_M of the person whose nearest person is them
  merged_person a candidate box covers them and another person
  merged_other  a candidate box covers them but its centre is off (joined with clutter)
  missed        no candidate box covers them
  hidden        < MIN_VISIBLE points at 0.15-2.2 m (occluded or out of view), not scored

Each person of a group is also inserted ALONE into the same frame ('alone' outcome), so
a crowd failure (found alone, lost in the group) is told apart from one the extractor
would have anyway (clutter, range).  `score(..., extract=...)` takes another extractor,
so a crowd fix can be measured on the same scenes (same --seed).

Body and noise were checked against real camera-confirmed people in the same frames
(2026-11-02): point count and upper-body width within ~5%, legs while crossing within
0.04 m, along-ray tail shares within ~3 points; standing upper-body depth is still
~0.05 m shallower than real.

With --own, synthetic people are kept > REAL_CLEAR_M from real labelled people in the
same frame, and found synthetic people are compared feature by feature with real
camera-confirmed people (the check that the bodies look like the real thing).

Seated people and furniture (2026-11-22, step 0 of notes/plans/2026-11-21-isaac-sim-lidar-people.md):
``body`` also poses a SEATED person (chair/couch seat height, torso lean/slouch, thighs
forward, shins down, forearms on a desk or in the lap) and ``insert(..., furniture=)``
casts the same real rays at furniture made of oriented boxes (``raycast_boxes``): office
and dining chairs, desks with legs / side panels / drawer pedestal / monitor, couch with
arms, coffee table, bookcase, partition.  ``sample_furniture_scene`` places a seated or
standing person next to or behind such furniture at 0.4-6 m, and ``sample_close_group``
places standing/walking people at 0.4-1.5 m.  Furniture hits carry id -2 - j (piece j of the furniture list) in ``pid``.
The training/eval driver is synthetic_seated_experiment.py.

Limits: robot ego-motion during the frame is not applied to the synthetic person; no
mixed edge points; furniture is solid boxes (no shelf slats, chair-back slats, cables);
rays that returned nothing in the real frame (open space beyond range) cannot hit a
person (indoors almost every ray returns).
"""
import argparse
import math
import multiprocessing as mp
import pathlib
import sqlite3
import struct
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import lidar_person_dataset as lpd  # noqa: E402
from yahboomcar_bringup import cluster_features as cf  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

SENSOR_Z = 0.31
RANGE_MAX_M = 12.0
RANGE_NOISE_M = (0.008, 0.0012)  # sigma = a + b * range; fitted to real people at ~1 m (spec: <= 2 cm at 10 m)
TAIL_P, TAIL_M = 0.12, 0.05   # share of hits pushed back along the ray (mixed edge returns), mean push; fitted
DROP_P = 0.03                 # random no-return (dark clothing)
GRAZE_COS = 0.12              # no return when the ray meets the surface flatter than this
TIME_BINS = 5
MATCH_M = 0.35
DUP_M = 0.5
MIN_VISIBLE = 10
FREE_M = 0.4                  # no real points at body height this close to a placement
REAL_CLEAR_M = 1.5
MIN_GAP_M = 0.35              # centre-to-centre, people do not overlap
NN_BINS = ((0.0, 0.6), (0.6, 1.0), (1.0, 1.5), (1.5, 99.0))
RANGE_BINS = ((0, 2), (2, 4), (4, 6), (6, 8), (8, 12))
OUTCOMES = ('found', 'merged_person', 'merged_other', 'missed', 'hidden')


# ---------------------------------------------------------------- body model

def _frame_from(d):
    """Rotation whose third column is the unit vector d."""
    d = d / np.linalg.norm(d)
    a = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(a, d)
    u /= np.linalg.norm(u)
    return np.stack([u, np.cross(d, u), d], axis=1)


def _segment(p0, p1, r):
    L = np.linalg.norm(p1 - p0)
    return (p0 + p1) / 2, _frame_from(p1 - p0), np.array([r, r, L / 2 + 0.4 * r])


def body(person, t):
    """Ellipsoids (centres (K,3), rotations (K,3,3), radii (K,3)) of `person` at time t
    (seconds from the frame middle).  World frame: floor z = 0.  A person with
    person['seated'] is posed by _seated_parts instead."""
    if person.get('seated'):
        return _to_world(_seated_parts(person), person, t)
    H, s, g = person['height'], person['height'] / 1.75, person['girth']
    walk = person['speed'] > 0
    ph = person['phase'] + 2 * math.pi * t / person['period']
    parts = []

    def limb(chain, angles, r_list):
        p = chain
        for (a, L), r in zip(angles, r_list):
            q = p + L * np.array([math.sin(a), 0.0, -math.cos(a)])
            parts.append(_segment(p, q, r))
            p = q
        return p

    for side, sgn in ((1, 1.0), (-1, -1.0)):
        y = 0.09 * s * side
        hip = np.array([0.0, y, 0.53 * H])
        leg = ph if side > 0 else ph + math.pi           # each leg's own gait phase
        if walk:
            amp = 0.15 + 0.12 * person['speed']            # longer strides when faster
            a = amp * math.sin(leg)                       # hip flexion, forward +
            knee = 1.4 * amp * max(0.0, math.cos(leg))    # bends while the leg swings forward
        else:
            a, knee = person['stance'] * sgn, 0.0
        ankle = limb(hip, [(a, 0.25 * H), (a - knee, 0.23 * H)], [0.075 * s * g, 0.055 * s * g])
        parts.append((ankle + np.array([0.06 * s, 0, -0.01]), np.eye(3), np.array([0.12, 0.05, 0.04]) * s))
        sh = np.array([0.0, 0.215 * s * person['shoulder'] * side, 0.81 * H])
        arm = -0.3 * math.sin(leg) if walk else person['arm']    # opposite to the same-side leg
        fore = 0.35 if walk else person['forearm']
        limb(sh, [(arm, 0.18 * H), (arm + fore, 0.16 * H)], [0.055 * s * g, 0.045 * s * g])
    torso_lo, torso_hi = 0.55 * H, 0.83 * H
    parts.append((np.array([0, 0, (torso_lo + torso_hi) / 2]), np.eye(3),
                  np.array([0.13 * g, 0.2 * person['shoulder'] * g, (torso_hi - torso_lo) / 2 + 0.03]) * np.array([s, s, 1])))
    parts.append((np.array([0, 0, 0.52 * H]), np.eye(3), np.array([0.11 * g, 0.17 * g, 0.09]) * s))
    parts.append((np.array([0.01 * s, 0, 0.93 * H]), np.eye(3), np.array([0.095, 0.08, 0.115]) * s))
    parts.append((np.array([0, 0, 0.865 * H]), np.eye(3), np.array([0.05, 0.05, 0.06]) * s))
    if person['backpack']:
        parts.append((np.array([-0.2 * s * g, 0, 0.7 * H]), np.eye(3), np.array([0.08, 0.15, 0.2]) * s))
    return _to_world(parts, person, t)


def _rot_z(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])


def _to_world(parts, person, t):
    """Body-frame ellipsoids (x forward, y left, z up, origin on the floor under the hips)
    -> world frame at time t."""
    c, R, a = (np.array(v) for v in zip(*parts))
    Rz = _rot_z(person['yaw'])
    pos = np.array([person['x'] + person['vx'] * t, person['y'] + person['vy'] * t, 0.0])
    return c @ Rz.T + pos, Rz @ R, a


def _seated_parts(p):
    """A seated person, body frame.  Hip centre over the seat at seat_h + 0.11 s; thighs
    run forward (thigh_drop radians below horizontal), shins down to the floor (ankle
    leg_dx ahead of the knee); the torso leans `lean` radians forward (negative = back)
    and is shortened by `slouch`; forearms rest on a desk (arm_mode 'desk') or in the lap."""
    H, s, g = p['height'], p['height'] / 1.75, p['girth']
    hip_z = p['seat_h'] + 0.11 * s
    lean, sw = p['lean'], p['shoulder']
    u = np.array([math.sin(lean), 0.0, math.cos(lean)])
    lt = 0.285 * H * p['slouch']
    hip_c = np.array([0.0, 0.0, hip_z])
    sh_c = hip_c + lt * u
    parts = [(np.array([-0.03 * s, 0.0, hip_z]), np.eye(3), np.array([0.12 * g, 0.17 * g, 0.09]) * s)]
    wid = np.array([0.0, 1.0, 0.0])
    parts.append(((hip_c + sh_c) / 2, np.stack([wid, np.cross(u, wid), u], axis=1),
                  np.array([0.2 * sw * g * s, 0.13 * g * s, lt / 2 + 0.05 * s])))
    hd = np.array([math.sin(0.35 * lean), 0.0, math.cos(0.35 * lean)])
    parts.append((sh_c + 0.055 * H * hd, np.eye(3), np.array([0.05, 0.05, 0.06]) * s))
    parts.append((sh_c + 0.12 * H * hd + np.array([0.01 * s + p['head_fwd'], 0.0, 0.0]), np.eye(3),
                  np.array([0.095, 0.08, 0.115]) * s))
    for sd in (1.0, -1.0):
        sh = sh_c + np.array([0.0, 0.215 * s * sw * sd, 0.0])
        a1, b = p['arm_up'], (0.05 if p['arm_mode'] == 'desk' else 0.55)
        elbow = sh + 0.18 * H * np.array([math.sin(a1), 0.0, -math.cos(a1)])
        hand = elbow + 0.16 * H * np.array([math.cos(b), 0.0, -math.sin(b)])
        parts.append(_segment(sh, elbow, 0.055 * s * g))
        parts.append(_segment(elbow, hand, 0.045 * s * g))
        parts.append((hand, np.eye(3), np.array([0.05, 0.04, 0.035]) * s))
        hip = np.array([0.0, 0.09 * s * sd, hip_z])
        d = p['thigh_drop']
        knee = hip + 0.25 * H * np.array([math.cos(d), 0.0, -math.sin(d)]) + np.array([0.0, 0.04 * p['splay'] * sd, 0.0])
        ankle = np.array([knee[0] + p['leg_dx'], knee[1], 0.07 * s + p['foot_lift']])
        parts.append(_segment(hip, knee, 0.075 * s * g))
        parts.append(_segment(knee, ankle, 0.055 * s * g))
        parts.append((ankle + np.array([0.06 * s, 0.0, -0.01]), np.eye(3), np.array([0.12, 0.05, 0.04]) * s))
    return parts


def raycast(origin, dirs, ellipsoids):
    """Nearest hit of unit rays from origin.  Returns (range, |cos incidence|), inf/0 on miss."""
    centres, rots, radii = ellipsoids
    best = np.full(len(dirs), np.inf)
    cos = np.zeros(len(dirs))
    for c, R, a in zip(centres, rots, radii):
        o = (origin - c) @ R / a
        d = dirs @ R / a
        A = (d * d).sum(1)
        B = 2 * d @ o
        C = o @ o - 1
        disc = B * B - 4 * A * C
        hit = disc > 0
        if not hit.any():
            continue
        t = np.full(len(dirs), np.inf)
        t[hit] = (-B[hit] - np.sqrt(disc[hit])) / (2 * A[hit])
        t[t <= 0] = np.inf
        better = t < best
        if better.any():
            hp = o + t[better, None] * d[better]
            n = (hp / a) @ R.T
            n /= np.linalg.norm(n, axis=1, keepdims=True)
            cos[better] = np.abs((n * dirs[better]).sum(1))
            best[better] = t[better]
    return best, cos


def raycast_boxes(origin, dirs, boxes):
    """Nearest hit of unit rays against oriented boxes (centres (K,3), rotations (K,3,3),
    half sizes (K,3)); slab test.  Returns (range, |cos incidence|), inf/0 on miss."""
    centres, rots, halves = boxes
    best = np.full(len(dirs), np.inf)
    cos = np.zeros(len(dirs))
    for c, R, h in zip(centres, rots, halves):
        o = (origin - c) @ R
        d = dirs @ R
        d = np.where(np.abs(d) < 1e-12, 1e-12, d)
        t1, t2 = (-h - o) / d, (h - o) / d
        tn, tf = np.minimum(t1, t2), np.maximum(t1, t2)
        t_in, t_out = tn.max(1), tf.min(1)
        hit = (t_out >= t_in) & (t_in > 0) & (t_in < best)
        if hit.any():
            ax = tn.argmax(1)
            best[hit] = t_in[hit]
            cos[hit] = np.abs(d[hit, ax[hit]])
    return best, cos


def insert(xyz, tsec, people, rng, furniture=()):
    """xyz: (N,3) real points, floor frame (sensor at (0,0,SENSOR_Z)); tsec: capture time
    of each point from the frame middle.  furniture: placed pieces (dict x, y, radius,
    boxes) from place_furniture.  Returns (xyz', person index per point; -1 = real
    background, -2 - j = piece j of `furniture`)."""
    origin = np.array([0.0, 0.0, SENSOR_Z])
    rel = xyz - origin
    rng_real = np.linalg.norm(rel, axis=1)
    dirs = rel / rng_real[:, None]
    az = np.arctan2(dirs[:, 1], dirs[:, 0])
    hit_r = np.full(len(xyz), np.inf)
    hit_cos = np.zeros(len(xyz))
    hit_id = np.full(len(xyz), -1)
    edges = np.linspace(tsec.min(), tsec.max() + 1e-9, TIME_BINS + 1)
    for k, p in enumerate(people):
        r = math.hypot(p['x'], p['y'])
        half = math.pi if r < 0.9 else math.asin(min(1.0, 0.85 / r))
        dz = np.abs((az - math.atan2(p['y'], p['x']) + math.pi) % (2 * math.pi) - math.pi)
        near = dz < half
        for b in range(TIME_BINS):
            m = np.flatnonzero(near & (tsec >= edges[b]) & (tsec < edges[b + 1]))
            if not len(m):
                continue
            t, c = raycast(origin, dirs[m], body(p, 0.5 * (edges[b] + edges[b + 1])))
            better = t < hit_r[m]
            hit_r[m[better]], hit_cos[m[better]], hit_id[m[better]] = t[better], c[better], k
    for j, f in enumerate(furniture):
        r = math.hypot(f['x'], f['y'])
        half = math.pi if r < f['radius'] + 0.05 else math.asin(min(1.0, f['radius'] / r))
        dz = np.abs((az - math.atan2(f['y'], f['x']) + math.pi) % (2 * math.pi) - math.pi)
        m = np.flatnonzero(dz < half)
        if not len(m):
            continue
        t, c = raycast_boxes(origin, dirs[m], f['boxes'])
        better = t < hit_r[m]
        hit_r[m[better]], hit_cos[m[better]], hit_id[m[better]] = t[better], c[better], -2 - j
    take = hit_r < rng_real
    drop = take & ((rng.random(len(xyz)) < DROP_P) | (hit_cos < GRAZE_COS))
    noise = rng.normal(0, 1, len(xyz)) * (RANGE_NOISE_M[0] + RANGE_NOISE_M[1] * hit_r.clip(0, 50))
    noise += (rng.random(len(xyz)) < TAIL_P) * rng.exponential(TAIL_M, len(xyz))
    r_new = np.where(take, hit_r + noise, rng_real)
    out = origin + dirs * r_new[:, None]
    pid = np.where(take, hit_id, -1)
    keep = ~drop
    return out[keep], pid[keep]


# ---------------------------------------------------------------- furniture

def _box(cx, cy, cz, hx, hy, hz, yaw=0.0):
    """One oriented box: centre, rotation, half sizes (local frame)."""
    return np.array([cx, cy, cz]), _rot_z(yaw), np.array([hx, hy, hz])


def place_furniture(local_boxes, x, y, yaw, kind=''):
    """Boxes given in a frame whose origin is the floor point (x, y) and whose +x points
    along `yaw`.  `radius` bounds the whole piece around (x, y) for the ray prefilter."""
    Rz = _rot_z(yaw)
    c = np.array([b[0] for b in local_boxes]) @ Rz.T + np.array([x, y, 0.0])
    R = np.array([Rz @ b[1] for b in local_boxes])
    h = np.array([b[2] for b in local_boxes])
    radius = float(max(np.hypot(b[0][0], b[0][1]) + np.hypot(*b[2][:2]) for b in local_boxes))
    return dict(x=x, y=y, radius=radius, boxes=(c, R, h), kind=kind)


def office_chair(rng, seat_h):
    """Swivel chair: seat, backrest behind the sitter, pole, five-star base, armrests."""
    back_h = rng.uniform(0.38, 0.72)
    b = [_box(0, 0, seat_h - 0.03, 0.23, 0.23, 0.03),
         _box(-0.23, 0, seat_h + back_h / 2, 0.025, rng.uniform(0.19, 0.24), back_h / 2),
         _box(0, 0, (seat_h - 0.06) / 2 + 0.04, 0.03, 0.03, (seat_h - 0.06) / 2 - 0.04)]
    for yaw in (0.0, math.pi / 2, math.pi / 4, -math.pi / 4):
        b.append(_box(0, 0, 0.05, 0.30, 0.025, 0.025, yaw))
    if rng.random() < 0.5:
        for sd in (1, -1):
            b.append(_box(-0.02, sd * 0.26, seat_h + 0.2, 0.2, 0.02, 0.015))
            b.append(_box(-0.02, sd * 0.26, seat_h + 0.1, 0.02, 0.02, 0.1))
    return b


def dining_chair(rng, seat_h):
    """Four legs, a seat and a back whose top is ~0.9 m."""
    top = rng.uniform(0.85, 1.0)
    b = [_box(0, 0, seat_h - 0.02, 0.21, 0.21, 0.02),
         _box(-0.21, 0, (seat_h + top) / 2, 0.02, 0.19, (top - seat_h) / 2)]
    for sx in (1, -1):
        for sy in (1, -1):
            b.append(_box(sx * 0.19, sy * 0.19, (seat_h - 0.04) / 2, 0.02, 0.02, (seat_h - 0.04) / 2))
    return b


def desk(rng, x_edge, top_h):
    """Desk whose near edge is x_edge ahead of the sitter's hips: top, legs or side
    panels, optional modesty panel and drawer pedestal, monitor or laptop on top."""
    w, d = rng.uniform(1.1, 1.8), rng.uniform(0.55, 0.8)
    off = rng.uniform(-0.25, 0.25)
    cx = x_edge + d / 2
    b = [_box(cx, off, top_h - 0.02, d / 2, w / 2, 0.02)]
    if rng.random() < 0.4:
        for sd in (1, -1):
            b.append(_box(cx, off + sd * (w / 2 - 0.03), (top_h - 0.04) / 2, d / 2 - 0.03, 0.015, (top_h - 0.04) / 2))
    else:
        for sx in (1, -1):
            for sy in (1, -1):
                b.append(_box(cx + sx * (d / 2 - 0.05), off + sy * (w / 2 - 0.05), (top_h - 0.04) / 2, 0.025, 0.025, (top_h - 0.04) / 2))
    if rng.random() < 0.4:
        b.append(_box(cx + d / 2 - 0.05, off, top_h - 0.22, 0.01, w / 2 - 0.06, 0.18))
    if rng.random() < 0.4:
        sd = rng.choice([-1, 1])
        b.append(_box(cx, off + sd * (w / 2 - 0.22), (top_h - 0.04) / 2, d / 2 - 0.05, 0.2, (top_h - 0.04) / 2))
    if rng.random() < 0.7:
        mx, my = cx + d / 2 - 0.18, off + rng.uniform(-0.2, 0.2)
        b.append(_box(mx, my, top_h + 0.07, 0.08, 0.08, 0.07))
        b.append(_box(mx, my, top_h + 0.30, 0.02, rng.uniform(0.24, 0.34), rng.uniform(0.15, 0.2)))
    elif rng.random() < 0.6:
        b.append(_box(cx, off + rng.uniform(-0.2, 0.2), top_h + 0.01, 0.15, 0.2, 0.01))
    return b, w, d


def couch(rng):
    """Sofa, origin under the sitter's hips, front (+x) toward the coffee table: seat
    cushion, backrest and two arms.  Width 1.7-2.2 m, backrest top ~0.85 m."""
    w = rng.uniform(1.7, 2.2)
    return [_box(0.05, 0, 0.2, 0.30, w / 2, 0.2), _box(-0.34, 0, 0.425, 0.1, w / 2, 0.425),
            _box(0.05, w / 2 - 0.1, 0.3, 0.30, 0.1, 0.3), _box(0.05, -w / 2 + 0.1, 0.3, 0.30, 0.1, 0.3)], w


def small_table(rng, x, top_h, w=None, d=None):
    """Four-legged table with its centre at x ahead (coffee table 0.42, side table 0.55)."""
    w = w or rng.uniform(0.8, 1.2)
    d = d or rng.uniform(0.45, 0.7)
    b = [_box(x, 0, top_h - 0.02, d / 2, w / 2, 0.02)]
    for sx in (1, -1):
        for sy in (1, -1):
            b.append(_box(x + sx * (d / 2 - 0.04), sy * (w / 2 - 0.04), (top_h - 0.04) / 2, 0.02, 0.02, (top_h - 0.04) / 2))
    return b


def bookcase(rng):
    """Solid box, centred on its floor point: 0.25-0.4 deep, 0.6-1.2 wide, 1.0-2.0 tall."""
    d, w, h = rng.uniform(0.25, 0.4), rng.uniform(0.6, 1.2), rng.uniform(1.0, 2.0)
    return [_box(0, 0, h / 2, d / 2, w / 2, h / 2)], d, w


def partition(rng):
    """Office partition panel: 5 cm thick, 1.2-2 m wide, 1.3-1.7 m tall."""
    w, h = rng.uniform(1.2, 2.0), rng.uniform(1.3, 1.7)
    return [_box(0, 0, h / 2, 0.03, w / 2, h / 2)], 0.06, w


# ---------------------------------------------------------------- scenes

def sample_person(rng, x, y, yaw, speed):
    return dict(x=x, y=y, yaw=yaw, speed=speed, vx=speed * math.cos(yaw), vy=speed * math.sin(yaw),
                height=rng.uniform(1.5, 1.95), girth=rng.uniform(0.9, 1.35),
                shoulder=rng.uniform(0.95, 1.2), phase=rng.uniform(0, 2 * math.pi),
                period=rng.uniform(0.95, 1.2), stance=rng.uniform(0.05, 0.25),     # one foot ahead
                arm=rng.uniform(-0.05, 0.35), backpack=rng.random() < 0.25,
                forearm=rng.choice([0.1, 0.1, 1.2]))     # 1 in 3 standing people hold something in front


def sample_group(rng, occupied, avoid, open_bearing):
    """A group of 1-4 people at 0.8-12 m; spacing 0.35-1.8 m inside the group.
    occupied(x, y) -> True where real clutter is or no line of sight; avoid: (M,2) real
    people; open_bearing(r) -> a bearing along which the real frame sees past r (the
    lab is small, so a uniform bearing would almost never place anyone far out)."""
    n = int(rng.choice([1, 1, 2, 2, 3, 4]))
    walking = rng.random() < 0.6
    yaw_g = rng.uniform(-math.pi, math.pi)
    speed = rng.uniform(0.6, 1.6) if walking else 0.0
    for _ in range(40):
        r = rng.uniform(0.8, RANGE_MAX_M)
        b = open_bearing(r)
        if b is None:
            continue
        pts = [(r * math.cos(b), r * math.sin(b))]
        for _ in range(60):
            if len(pts) == n:
                break
            ax, ay = pts[rng.integers(len(pts))]
            d, a = rng.uniform(MIN_GAP_M, 1.8), rng.uniform(-math.pi, math.pi)
            q = (ax + d * math.cos(a), ay + d * math.sin(a))
            if all(math.hypot(q[0] - u, q[1] - v) >= MIN_GAP_M for u, v in pts):
                pts.append(q)
        ok = all(0.8 < math.hypot(*q) < RANGE_MAX_M and not occupied(*q) for q in pts)
        if ok and len(avoid):
            ok = all(np.hypot(avoid[:, 0] - q[0], avoid[:, 1] - q[1]).min() > REAL_CLEAR_M for q in pts)
        if ok:
            return [sample_person(rng, q[0], q[1], yaw_g if walking else rng.uniform(-math.pi, math.pi), speed)
                    for q in pts]
    return []


SENSOR_CLEAR_M = 0.3


def clear_of_sensor(people, pieces, margin=SENSOR_CLEAR_M):
    """No body part or furniture box within `margin` of the sensor (it would be inside the robot)."""
    o = np.array([0.0, 0.0, SENSOR_Z])
    for f in pieces:
        c, R, h = f['boxes']
        loc = np.einsum('kij,kj->ki', R.transpose(0, 2, 1), o - c)
        if np.linalg.norm(loc - np.clip(loc, -h, h), axis=1).min() < margin:
            return False
    for p in people:
        c, _, a = body(p, 0.0)
        if (np.linalg.norm(c - o, axis=1) - a.max(1)).min() < margin:
            return False
    return True


def sample_seated_person(rng, x, y, yaw, mode, seat_h):
    """A seated person at the hip point (x, y) facing yaw.  mode: 'desk' (torso upright or
    leaning in, thighs under the desk, forearms on it), 'chair' (free-standing chair) or
    'couch' (leaning back, legs out)."""
    p = sample_person(rng, x, y, yaw, 0.0)
    lean = {'desk': (-0.05, 0.35), 'chair': (-0.15, 0.30), 'couch': (-0.45, 0.05)}[mode]
    legs = {'desk': (-0.1, 0.1), 'chair': (-0.05, 0.25), 'couch': (0.0, 0.45)}[mode]
    p.update(seated=True, seat_h=seat_h, lean=rng.uniform(*lean),
             slouch=rng.uniform(0.8, 0.97) if mode == 'couch' else rng.uniform(0.85, 1.0),
             arm_mode='desk' if mode == 'desk' else str(rng.choice(['lap', 'lap', 'desk'])),
             arm_up=rng.uniform(0.1, 0.5), thigh_drop=rng.uniform(-0.1, 0.05) if mode == 'couch' else rng.uniform(0.0, 0.12),
             leg_dx=rng.uniform(*legs), foot_lift=rng.uniform(0.0, 0.08) if mode == 'couch' else 0.0,
             splay=rng.uniform(0.0, 1.5), head_fwd=rng.uniform(0.0, 0.06), backpack=False)
    return p


def _view_yaw(rng, x, y):
    """Which way a person at (x, y) faces: away from the robot (back view, the usual desk
    case), toward it, or anywhere."""
    u, a = rng.random(), math.atan2(y, x)
    if u < 0.4:
        return a + rng.uniform(-0.6, 0.6)
    if u < 0.6:
        return a + math.pi + rng.uniform(-0.6, 0.6)
    return rng.uniform(-math.pi, math.pi)


def _world(x, y, yaw, lx, ly):
    return x + lx * math.cos(yaw) - ly * math.sin(yaw), y + lx * math.sin(yaw) + ly * math.cos(yaw)


def _person_range(rng):
    """35% close (0.4-1.5 m), the rest 1.5-6 m."""
    return rng.uniform(0.4, 1.5) if rng.random() < 0.35 else rng.uniform(1.5, 6.0)


def sample_close_group(rng, occupied, avoid, open_bearing):
    """1-2 standing or walking people 0.4-1.5 m from the sensor (the existing groups start at 0.8 m)."""
    walking = rng.random() < 0.5
    speed = rng.uniform(0.6, 1.4) if walking else 0.0
    yaw_g = rng.uniform(-math.pi, math.pi)
    for _ in range(40):
        r = rng.uniform(0.4, 1.5)
        b = open_bearing(r)
        if b is None:
            continue
        pts = [(r * math.cos(b), r * math.sin(b))]
        if rng.random() < 0.4:
            a, d = rng.uniform(-math.pi, math.pi), rng.uniform(MIN_GAP_M, 1.0)
            pts.append((pts[0][0] + d * math.cos(a), pts[0][1] + d * math.sin(a)))
        if any(math.hypot(*q) < 0.4 or math.hypot(*q) > 1.8 or occupied(*q) for q in pts):
            continue
        if len(avoid) and any(np.hypot(avoid[:, 0] - q[0], avoid[:, 1] - q[1]).min() <= REAL_CLEAR_M for q in pts):
            continue
        return [sample_person(rng, q[0], q[1], yaw_g if walking else rng.uniform(-math.pi, math.pi), speed)
                for q in pts]
    return []


def sample_furniture_scene(rng, kind, occupied, avoid, open_bearing, abut=False):
    """People plus furniture at 0.4-6 m.  kind:
      seated_desk   office/dining chair at a desk (monitor, drawers), maybe a bookcase beside or behind
      seated_chair  free-standing chair, maybe a side table and a bookcase
      seated_couch  sofa (+ coffee table), the sitter off-centre, maybe a second sitter
      stand_furn    a standing/walking person beside a bookcase, partition, desk or couch, or with
                    one between the sensor and them (occluding the legs)
    abut: only the sitter's / person's own spot must be free of real clutter, the furniture may overlap
    real walls and furniture (a desk against a wall, a shelf beside a real cabinet).  Everything must
    stay >= SENSOR_CLEAR_M from the sensor.  Returns (people, furniture, meta) or None when no spot
    was found; the furniture list alone is the empty-scene counterpart used for hard negatives."""
    for _ in range(60):
        r = _person_range(rng)
        b = open_bearing(r)
        if b is None:
            continue
        x, y = r * math.cos(b), r * math.sin(b)
        yaw = _view_yaw(rng, x, y)
        pieces, people, checks, meta = [], [], [(x, y)], dict(kind=kind, range=r)
        if kind == 'seated_desk':
            seat_h = rng.uniform(0.42, 0.50)
            chair = office_chair(rng, seat_h) if rng.random() < 0.7 else dining_chair(rng, seat_h)
            top_h, x_edge = rng.uniform(0.72, 0.76), rng.uniform(0.30, 0.48)
            dsk, w, d = desk(rng, x_edge, top_h)
            pieces += [place_furniture(chair, x, y, yaw, 'chair'), place_furniture(dsk, x, y, yaw, 'desk')]
            checks += [_world(x, y, yaw, x_edge + d / 2, 0.0), _world(x, y, yaw, x_edge + d, w / 2), _world(x, y, yaw, x_edge + d, -w / 2)]
            if rng.random() < 0.5:
                bc, bd, bw = bookcase(rng)
                slot = rng.choice(['left', 'right', 'behind', 'front'])
                gap = rng.uniform(0.0, 0.4)
                lx, ly = {'left': (x_edge + d / 2, w / 2 + bd / 2 + gap), 'right': (x_edge + d / 2, -w / 2 - bd / 2 - gap),
                          'behind': (-0.3 - bd / 2 - gap, 0.0), 'front': (x_edge + d + bd / 2 + gap, 0.0)}[str(slot)]
                yb = yaw + (math.pi / 2 if slot in ('left', 'right') else 0.0)     # long side parallel to the desk side
                pieces.append(place_furniture(bc, *_world(x, y, yaw, lx, ly), yb, 'bookcase'))
                checks.append(_world(x, y, yaw, lx, ly))
                meta['shelf'] = str(slot)
            people.append(sample_seated_person(rng, x, y, yaw, 'desk', seat_h))
        elif kind == 'seated_chair':
            seat_h = rng.uniform(0.42, 0.50)
            chair = office_chair(rng, seat_h) if rng.random() < 0.4 else dining_chair(rng, seat_h)
            pieces.append(place_furniture(chair, x, y, yaw, 'chair'))
            if rng.random() < 0.4:
                pieces.append(place_furniture(small_table(rng, rng.uniform(0.6, 0.9), rng.uniform(0.45, 0.7)), x, y, yaw, 'table'))
                checks.append(_world(x, y, yaw, 0.75, 0.0))
            if rng.random() < 0.3:
                bc, bd, bw = bookcase(rng)
                lx, ly = rng.choice([(-0.3 - bd / 2 - rng.uniform(0.05, 0.6), 0.0), (0.0, bw / 2 + 0.35 + rng.uniform(0, 0.4))])
                pieces.append(place_furniture(bc, *_world(x, y, yaw, lx, ly), yaw, 'bookcase'))
                checks.append(_world(x, y, yaw, lx, ly))
            people.append(sample_seated_person(rng, x, y, yaw, 'chair', seat_h))
        elif kind == 'seated_couch':
            cb, w = couch(rng)
            ly = rng.uniform(-0.5, 0.5) * (w - 0.8)
            pieces.append(place_furniture(cb, x, y, yaw, 'couch'))      # origin = cushion centre line
            checks += [_world(x, y, yaw, 0.0, w / 2 - 0.2), _world(x, y, yaw, 0.0, -w / 2 + 0.2)]
            if rng.random() < 0.6:
                pieces.append(place_furniture(small_table(rng, rng.uniform(0.75, 1.0), 0.42, d=rng.uniform(0.4, 0.6)), x, y, yaw, 'table'))
                checks.append(_world(x, y, yaw, 0.9, 0.0))
            hx, hy = _world(x, y, yaw, -0.08, ly)
            people.append(sample_seated_person(rng, hx, hy, yaw, 'couch', 0.4))
            if rng.random() < 0.25 and w > 1.9:
                ly2 = ly + (0.75 if ly < 0 else -0.75)
                x2, y2 = _world(x, y, yaw, -0.08, ly2)
                people.append(sample_seated_person(rng, x2, y2, yaw + rng.normal(0, 0.15), 'couch', 0.4))
                checks.append((x2, y2))
        elif kind == 'stand_furn':
            walking = rng.random() < 0.5
            people.append(sample_person(rng, x, y, rng.uniform(-math.pi, math.pi), rng.uniform(0.6, 1.4) if walking else 0.0))
            ft = str(rng.choice(['bookcase', 'partition', 'desk', 'couch']))
            if ft == 'bookcase':
                loc, fd, fw = bookcase(rng)
            elif ft == 'partition':
                loc, fd, fw = partition(rng)
            elif ft == 'desk':
                loc, fw, fd = desk(rng, 0.0, rng.uniform(0.72, 0.76))
                loc = [(c - np.array([fd / 2, 0.0, 0.0]), R, h) for c, R, h in loc]       # centre it on its floor point
            else:
                loc, fw = couch(rng)
                fd = 0.9
            occl = r > 1.3 and rng.random() < 0.4
            if occl:        # between the sensor and the person, across the line of sight
                dist = r - rng.uniform(0.5, 1.1)
                fx, fy, fyaw = dist * math.cos(b), dist * math.sin(b), b + math.pi + rng.uniform(-0.4, 0.4)
            else:           # beside the person, its side face 0.15-0.8 m away
                phi = rng.uniform(-math.pi, math.pi)
                off = 0.25 + rng.uniform(0.0, 0.6) + fd / 2        # 0.25 m = the person's own half width
                fx, fy, fyaw = x + off * math.cos(phi), y + off * math.sin(phi), phi + math.pi
            pieces.append(place_furniture(loc, fx, fy, fyaw, ft))
            checks += [(fx, fy), _world(fx, fy, fyaw, 0.0, fw / 2), _world(fx, fy, fyaw, 0.0, -fw / 2)]
            meta.update(furn=ft, occluding=bool(occl), walking=walking)
        else:
            raise ValueError(kind)
        if any(occupied(*q) for q in (checks[:1] if abut else checks)):
            continue
        if not clear_of_sensor(people, pieces):
            continue
        meta['abut'] = bool(abut)
        pts = np.array([[p['x'], p['y']] for p in people])
        if len(avoid) and any(np.hypot(avoid[:, 0] - q[0], avoid[:, 1] - q[1]).min() <= REAL_CLEAR_M for q in pts):
            continue
        return people, pieces, meta
    return None


def score(out, pid, people, extract=pc.extract):
    """Run the extractor on a frame with people inserted; per person (outcome, features,
    visible points, duplicates).  Duplicates: other candidates within DUP_M of the person
    whose nearest person is them (one person cut into several pieces).  `extract` is
    swappable so a candidate extractor can be compared."""
    x, y, z = out[:, 0], out[:, 1], out[:, 2]
    cands = extract(x, y, z, origin=(0.0, 0.0), range_max=RANGE_MAX_M)
    P = np.array([[p['x'], p['y']] for p in people])
    if len(cands):
        dc = np.hypot(cands[:, None, 0] - P[None, :, 0], cands[:, None, 1] - P[None, :, 1])
        owner, dist = dc.argmin(1), dc.min(1)
    res = []
    for k in range(len(people)):
        vis = int(((pid == k) & (z > 0.15) & (z < 2.2)).sum())
        outcome, feat = 'missed', np.full(len(cf.NAMES), np.nan, dtype=np.float32)
        near_k = int(((owner == k) & (dist < DUP_M)).sum()) if len(cands) else 0
        if vis < MIN_VISIBLE:
            res.append(('hidden', feat, vis, 0))
            continue
        for c in cands:
            d = np.hypot(P[:, 0] - c[0], P[:, 1] - c[1])
            if int(np.argmin(d)) == k and d[k] < MATCH_M:
                outcome = 'found'
                m = cf.crop(x, y, z, c)
                feat = np.asarray(cf.features(x[m], y[m], z[m]), dtype=np.float32)
                break
        if outcome != 'found':
            pad = 0.1
            mine = [cv for cv in ((c[2] - pad <= P[:, 0]) & (P[:, 0] <= c[3] + pad) & (c[4] - pad <= P[:, 1])
                                  & (P[:, 1] <= c[5] + pad) for c in cands) if cv[k]]
            if any(cv.sum() > 1 for cv in mine):
                outcome = 'merged_person'
            elif mine:
                outcome = 'merged_other'
        res.append((outcome, feat, vis, max(0, near_k - (outcome == 'found'))))
    return res


def frame_context(xyz, rng):
    """(occupied(x, y), open_bearing(r)) for one real background frame: where a body (or a
    piece of furniture) can stand without sitting in real clutter or behind a wall, and a
    bearing along which the frame sees past range r."""
    body_h = (xyz[:, 2] > 0.15) & (xyz[:, 2] < 2.2)
    from scipy.spatial import cKDTree
    tree = cKDTree(xyz[body_h, :2]) if body_h.any() else None
    rel = xyz - np.array([0.0, 0.0, SENSOR_Z])
    r_xy = np.hypot(rel[:, 0], rel[:, 1])
    el = np.arctan2(rel[:, 2], r_xy)
    low = (el > 0) & (el < math.radians(25))          # rays that would cross a body
    az_low, r_low = np.arctan2(rel[low, 1], rel[low, 0]), r_xy[low]

    def occupied(x, y):
        if tree is not None and len(tree.query_ball_point((x, y), FREE_M)):
            return True
        # line of sight: most real rays toward the spot must reach past it, or the
        # spot is behind a wall (no recorded points there either, so it looks free)
        d = np.abs((az_low - math.atan2(y, x) + math.pi) % (2 * math.pi) - math.pi)
        sel = r_low[d < math.radians(2)]
        return len(sel) < 5 or np.mean(sel > math.hypot(x, y) + 0.3) < 0.6

    def open_bearing(r):
        cand = az_low[r_low > r + 0.5]
        return None if not len(cand) else float(cand[rng.integers(len(cand))]) + rng.normal(0, 0.01)

    return occupied, open_bearing


def run_scene(args, extract=pc.extract):
    xyz, tsec, real_people, seed = args
    rng = np.random.default_rng(seed)
    occupied, open_bearing = frame_context(xyz, rng)

    people = sample_group(rng, occupied, real_people, open_bearing)
    if not people:
        return []
    out, pid = insert(xyz, tsec, people, rng)
    res = score(out, pid, people, extract)
    # counterfactual: each person alone in the same frame (same body, same pose)
    alone = [res[0][0]] if len(people) == 1 else [
        score(*insert(xyz, tsec, [p], rng), [p], extract)[0][0] for p in people]
    P = np.array([[p['x'], p['y']] for p in people])
    rows = []
    for k, p in enumerate(people):
        others = np.delete(P, k, axis=0)
        nn = float(np.hypot(*(others - P[k]).T).min()) if len(others) else 99.0
        outcome, feat, vis, dups = res[k]
        rows.append(dict(range=float(math.hypot(p['x'], p['y'])), nn=nn, n_group=len(people),
                         walking=p['speed'] > 0, visible=vis, outcome=outcome, alone=alone[k], dups=dups,
                         X=feat, seed=seed))
    return rows


# ---------------------------------------------------------------- backgrounds

def load_frames(run_dir, every, own=None):
    """Every `every`-th Livox frame of a run: (xyz floor frame, t from frame middle,
    real labelled people (M,2) in the sensor frame)."""
    run_dir = pathlib.Path(run_dir)
    db = sqlite3.connect(str(run_dir / 'bag' / 'bag_0.db3'))
    tid = {n: i for i, n in db.execute('select id, name from topics')}
    real = {}
    if own is not None:
        m = (own['run'] == run_dir.name) & ((own['y'] == 1) | (own['y'] == 2))
        for t, r, b in zip(own['t'][m], own['X'][m, 0], own['bearing'][m]):
            real.setdefault(round(float(t), 4), []).append(
                (r * math.cos(math.radians(b)), r * math.sin(math.radians(b))))
    out = []
    rows = db.execute('select data from messages where topic_id=? order by timestamp', (tid['/livox/lidar'],))
    for i, (data,) in enumerate(rows):
        if i % every:
            continue
        p = lpd.decode_points(data)
        if p is None or len(p) < 1000:
            continue
        sec, nsec = struct.unpack_from('<iI', data, 4)
        end = sec + nsec * 1e-9 + int(p['t'].max()) * 1e-9
        ok = (np.abs(p['x']) + np.abs(p['y']) + np.abs(p['z'])) > 0.1
        p = p[ok]
        xyz = np.c_[p['x'], p['y'], p['z'] + SENSOR_Z].astype(np.float64)
        t = p['t'].astype(np.float64) * 1e-9
        out.append((xyz, t - 0.5 * (t.min() + t.max()),
                    np.array(real.get(round(end, 4), []), dtype=np.float64).reshape(-1, 2)))
    return out


# ---------------------------------------------------------------- report

def table(rows, title):
    print(f'\n{title}: found / scored  (merged with a person, merged with clutter, missed)')
    print(f'  {"nearest other":>14s} |' + ' |'.join(f'{f"{lo}-{hi} m":>22s}' for lo, hi in RANGE_BINS)
          + f' |{"all":>22s}')
    for lo, hi in NN_BINS:
        name = 'alone' if lo >= 1.5 else f'{lo}-{hi} m'
        cells = []
        for rlo, rhi in RANGE_BINS + ((0, 99),):
            sel = [r for r in rows if lo <= r['nn'] < hi and rlo <= r['range'] < rhi and r['outcome'] != 'hidden']
            if len(sel) < 15:
                cells.append(f'{"--":>22s}')
                continue
            cnt = {o: sum(r['outcome'] == o for r in sel) for o in OUTCOMES}
            cells.append(f'{100 * cnt["found"] / len(sel):3.0f}% /{len(sel):4d} ({cnt["merged_person"]:3d},'
                         f'{cnt["merged_other"]:3d},{cnt["missed"]:3d})')
        print(f'  {name:>14s} |' + ' |'.join(cells))


def compare_features(rows, own):
    names = list(cf.NAMES)
    keys = ['n', 'width', 'depth', 'height', 'z_hi', 'top_fill', 'z_std']
    keys = [k for k in keys if k in names]
    real = own['X'][(own['y'] == 1) & (own['source'] == 'cam')]
    syn = np.array([r['X'] for r in rows if r['outcome'] == 'found'])
    print('\nFound synthetic vs camera-confirmed real people (median; real / synthetic):')
    print(f'  {"range":>7s} {"count":>11s} |' + ' |'.join(f'{k:>13s}' for k in keys))
    for lo, hi in RANGE_BINS:
        a = real[(real[:, 0] >= lo) & (real[:, 0] < hi)]
        b = syn[(syn[:, 0] >= lo) & (syn[:, 0] < hi)] if len(syn) else syn
        if len(a) < 20 or len(b) < 20:
            continue
        cells = [f'{np.median(a[:, names.index(k)]):6.2f}/{np.median(b[:, names.index(k)]):<6.2f}' for k in keys]
        print(f'  {lo:>2d}-{hi:<2d} m {len(a):5d}/{len(b):<5d} |' + ' |'.join(cells))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--own', help='own.npz from lidar_person_dataset.py (real people to avoid + compare)')
    ap.add_argument('--scenes', type=int, default=1500)
    ap.add_argument('--every', type=int, default=10, help='use every n-th frame of each run')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('-j', '--jobs', type=int, default=None)
    ap.add_argument('-o', '--out')
    a = ap.parse_args()
    own = dict(np.load(a.own, allow_pickle=True)) if a.own else None
    frames = []
    for run in a.runs:
        fr = load_frames(run, a.every, own)
        print(f'{pathlib.Path(run).name:14s} {len(fr)} background frames')
        frames += fr
    rng = np.random.default_rng(a.seed)
    pick = rng.integers(len(frames), size=a.scenes)
    jobs = [(frames[i][0], frames[i][1], frames[i][2], a.seed * 1_000_003 + s) for s, i in enumerate(pick)]
    with mp.Pool(a.jobs) as pool:
        rows = [r for res in pool.imap_unordered(run_scene, jobs, chunksize=8) for r in res]
    print(f'\n{len(rows)} synthetic people in {a.scenes} scenes, '
          f'{sum(r["outcome"] == "hidden" for r in rows)} hidden (< {MIN_VISIBLE} points)')
    table(rows, 'Extractor recall, all synthetic people')
    table([r for r in rows if r['walking']], 'Walking only')
    table([r for r in rows if not r['walking']], 'Standing only')
    table([r for r in rows if r['alone'] == 'found'],
          'Crowd effect: only people the extractor finds when ALONE in the same frame')
    if own is not None:
        compare_features(rows, own)
    if a.out:
        np.savez_compressed(a.out, X=np.array([r['X'] for r in rows]),
                            range=np.array([r['range'] for r in rows]), nn=np.array([r['nn'] for r in rows]),
                            n_group=np.array([r['n_group'] for r in rows]),
                            walking=np.array([r['walking'] for r in rows]),
                            visible=np.array([r['visible'] for r in rows]),
                            outcome=np.array([r['outcome'] for r in rows]),
                            alone=np.array([r['alone'] for r in rows]),
                            seed=np.array([r['seed'] for r in rows]), feature_names=np.array(cf.NAMES))
        print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
