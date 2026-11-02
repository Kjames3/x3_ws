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

Limits: robot ego-motion during the frame is not applied to the synthetic person; no
seated poses yet; no mixed edge points; rays that returned nothing in the real frame
(open space beyond range) cannot hit a person (indoors almost every ray returns).
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
    (seconds from the frame middle).  World frame: floor z = 0."""
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
    c, R, a = (np.array(v) for v in zip(*parts))
    yaw = person['yaw']
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    pos = np.array([person['x'] + person['vx'] * t, person['y'] + person['vy'] * t, 0.0])
    return c @ Rz.T + pos, Rz @ R, a


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


def insert(xyz, tsec, people, rng):
    """xyz: (N,3) real points, floor frame (sensor at (0,0,SENSOR_Z)); tsec: capture time
    of each point from the frame middle.  Returns (xyz', person index per point or -1)."""
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
    take = hit_r < rng_real
    drop = take & ((rng.random(len(xyz)) < DROP_P) | (hit_cos < GRAZE_COS))
    noise = rng.normal(0, 1, len(xyz)) * (RANGE_NOISE_M[0] + RANGE_NOISE_M[1] * hit_r.clip(0, 50))
    noise += (rng.random(len(xyz)) < TAIL_P) * rng.exponential(TAIL_M, len(xyz))
    r_new = np.where(take, hit_r + noise, rng_real)
    out = origin + dirs * r_new[:, None]
    pid = np.where(take, hit_id, -1)
    keep = ~drop
    return out[keep], pid[keep]


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


def score(out, pid, people, extract=pc.extract):
    """Run the extractor on a frame with people inserted; per person (outcome, features,
    visible points).  `extract` is swappable so a candidate extractor can be compared."""
    x, y, z = out[:, 0], out[:, 1], out[:, 2]
    cands = extract(x, y, z, origin=(0.0, 0.0), range_max=RANGE_MAX_M)
    P = np.array([[p['x'], p['y']] for p in people])
    res = []
    for k in range(len(people)):
        vis = int(((pid == k) & (z > 0.15) & (z < 2.2)).sum())
        outcome, feat = 'missed', np.full(len(cf.NAMES), np.nan, dtype=np.float32)
        if vis < MIN_VISIBLE:
            res.append(('hidden', feat, vis))
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
        res.append((outcome, feat, vis))
    return res


def run_scene(args, extract=pc.extract):
    xyz, tsec, real_people, seed = args
    rng = np.random.default_rng(seed)
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
        outcome, feat, vis = res[k]
        rows.append(dict(range=float(math.hypot(p['x'], p['y'])), nn=nn, n_group=len(people),
                         walking=p['speed'] > 0, visible=vis, outcome=outcome, alone=alone[k],
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
