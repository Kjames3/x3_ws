#!/usr/bin/env python3
"""Offline checks of the 2026-11-24 crowd changes on synthetic people in REAL empty lab frames.

    python3 src/tools/perception/crowd_synth_check.py split  artifacts/person-lidar-2026-11-2*/*empty*
    python3 src/tools/perception/crowd_synth_check.py linger artifacts/person-lidar-2026-11-22/seeded-empty-r1

Bodies, rays and furniture come from synthetic_people.py (read its docstring for the
limits: bodies ~40% sparser than real people, absolute rates uncalibrated).  Use the
numbers as an A/B between arms on identical scenes, not as live recall.

split   person_candidates.extract with the head-peak split OFF vs ON, scored on the same
        inserted frame.  Scene kinds:
          tight      2-4 standing people, 0.35-1.0 m apart, 1-6 m out
          real_desk  1-2 people standing 0.25-0.45 m from REAL clutter in the frame
                     (the lab desks/benches), the case that made 7-9 m components
          synth_desk a person beside a synthetic desk / shelf / partition (stand_furn, abut)
        plus the empty frame alone: extra candidates the split creates on bare clutter.

linger  the track-seed risk: lidar tracks seeding their own body (/c3/person_seeds) could
        keep a track alive on furniture after the person walks away.  Consecutive frames of
        an empty run; a person stands at real clutter (or a synthetic desk) for STAND_S with
        a camera detection, then walks off at 1 m/s and the camera loses them at once.  The
        full live chain runs per frame: extract (peaks) -> seeds (camera + confirmed tracks)
        -> seeded_candidates -> merge_seeded -> LidarPersonTracker.  Reported per arm (track
        seeds off / on): how long a track stays within LINGER_M of the vacated spot after the
        person is > 1 m from it.  Design ceiling: LONG_HOLD_S (30 s).
"""
import argparse
import math
import multiprocessing as mp
import pathlib
import sys

import numpy as np
from scipy.spatial import cKDTree

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1]))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import synthetic_people as sp  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402
import lidar_person_tracker as lpt  # noqa: E402

KINDS = ('tight', 'real_desk', 'synth_desk')
ARMS = ('off', 'on', 'cam3')    # linger: track seeds off / on (live) / on only while camera-fresh
TRACK_SEED_DEDUP_M = 0.5          # server_x3.TRACK_SEED_DEDUP_M
STAND_S, WALK_SPEED, LINGER_M = 8.0, 1.0, 0.5
SEED_CAMERA_S = 3.0              # arm 'cam3': a track seeds only if the camera saw it this recently


def _ext(peaks):
    def f(x, y, z, origin=(0.0, 0.0), range_max=sp.RANGE_MAX_M):
        return pc.extract(x, y, z, origin=origin, peaks=peaks)
    return f


EXTRACT = {'off': _ext(False), 'on': _ext(True)}


def real_desk_spots(xyz, rng, n_try=400):
    """Floor spots 0.25-0.45 m from real body-height clutter (a person leaning on a bench),
    nothing closer, in line of sight, 1-6 m out."""
    body = xyz[(xyz[:, 2] > 0.15) & (xyz[:, 2] < 2.2)]
    if len(body) < 50:
        return []
    tree = cKDTree(body[:, :2])
    occupied, _ = sp.frame_context(xyz, rng)
    out = []
    for _ in range(n_try):
        a = body[rng.integers(len(body))]
        if not (0.6 < a[2] < 1.0):        # desk / bench height
            continue
        r = math.hypot(a[0], a[1])
        if not 1.2 < r < 6.5:
            continue
        # step toward the sensor, off the clutter
        d = rng.uniform(0.3, 0.5)
        q = (a[0] - d * a[0] / r, a[1] - d * a[1] / r)
        dmin = tree.query(q)[0]
        if not 0.25 <= dmin <= 0.45:
            continue
        if _blocked(xyz, q):
            continue
        out.append(q)
        if len(out) >= 8:
            break
    return out


def _blocked(xyz, q):
    rel = xyz - np.array([0.0, 0.0, sp.SENSOR_Z])
    r_xy = np.hypot(rel[:, 0], rel[:, 1])
    el = np.arctan2(rel[:, 2], r_xy)
    low = (el > 0) & (el < math.radians(25))
    d = np.abs((np.arctan2(rel[low, 1], rel[low, 0]) - math.atan2(q[1], q[0]) + math.pi) % (2 * math.pi) - math.pi)
    sel = r_xy[low][d < math.radians(2)]
    return len(sel) < 5 or np.mean(sel > math.hypot(*q) - 0.2) < 0.6


def tight_group(rng, occupied, open_bearing):
    n = int(rng.choice([2, 2, 3, 4]))
    for _ in range(60):
        r = rng.uniform(1.0, 6.0)
        b = open_bearing(r)
        if b is None:
            continue
        pts = [(r * math.cos(b), r * math.sin(b))]
        for _ in range(80):
            if len(pts) == n:
                break
            ax, ay = pts[rng.integers(len(pts))]
            d, a = rng.uniform(sp.MIN_GAP_M, 1.0), rng.uniform(-math.pi, math.pi)
            q = (ax + d * math.cos(a), ay + d * math.sin(a))
            if all(math.hypot(q[0] - u, q[1] - v) >= sp.MIN_GAP_M for u, v in pts):
                pts.append(q)
        if len(pts) == n and all(0.8 < math.hypot(*q) < 6.5 and not occupied(*q) for q in pts):
            return [sp.sample_person(rng, x, y, rng.uniform(-math.pi, math.pi), 0.0) for x, y in pts]
    return []


def split_scene(args):
    xyz, tsec, seed, kind = args
    rng = np.random.default_rng(seed)
    occupied, open_bearing = sp.frame_context(xyz, rng)
    furn = ()
    if kind == 'tight':
        people = tight_group(rng, occupied, open_bearing)
    elif kind == 'real_desk':
        spots = real_desk_spots(xyz, rng)
        people = []
        if spots:
            x, y = spots[rng.integers(len(spots))]
            people = [sp.sample_person(rng, x, y, rng.uniform(-math.pi, math.pi), 0.0)]
            if rng.random() < 0.5:      # a second person beside them, along the bench
                for _ in range(20):
                    a, d = rng.uniform(-math.pi, math.pi), rng.uniform(0.45, 0.9)
                    q = (x + d * math.cos(a), y + d * math.sin(a))
                    if not _blocked(xyz, q) and cKDTree(xyz[(xyz[:, 2] > 0.15) & (xyz[:, 2] < 2.2), :2]).query(q)[0] > 0.2:
                        people.append(sp.sample_person(rng, *q, rng.uniform(-math.pi, math.pi), 0.0))
                        break
    else:
        got = sp.sample_furniture_scene(rng, 'stand_furn', occupied, np.zeros((0, 2)), open_bearing, abut=True)
        people, furn = (got[0], got[1]) if got else ([], ())
        people = [p for p in people if p['speed'] == 0] or people
    if not people:
        return []
    out, pid = sp.insert(xyz, tsec, people, rng, furniture=furn)
    res = {arm: sp.score(out, pid, people, ex) for arm, ex in EXTRACT.items()}
    P = np.array([[p['x'], p['y']] for p in people])
    rows = []
    for k, p in enumerate(people):
        others = np.delete(P, k, axis=0)
        nn = float(np.hypot(*(others - P[k]).T).min()) if len(others) else 99.0
        rows.append(dict(kind=kind, range=float(math.hypot(p['x'], p['y'])), nn=nn,
                         off=res['off'][k][0], on=res['on'][k][0],
                         dups_off=res['off'][k][3], dups_on=res['on'][k][3]))
    return rows


def clutter_count(args):
    xyz, _, _, _ = args
    x, y, z = xyz.T
    return {arm: len(ex(x, y, z)) for arm, ex in EXTRACT.items()}


def run_split(a, frames):
    rng = np.random.default_rng(a.seed)
    jobs = []
    for kind in KINDS:
        for s in range(a.scenes):
            f = frames[rng.integers(len(frames))]
            jobs.append((f[0], f[1], a.seed * 1_000_003 + len(jobs), kind))
    with mp.Pool(a.jobs) as pool:
        rows = [r for res in pool.imap_unordered(split_scene, jobs, chunksize=4) for r in res]
        empty = list(pool.imap_unordered(clutter_count, [(f[0], f[1], 0, '') for f in frames[::max(1, len(frames) // 200)]]))
    print(f'{len(rows)} synthetic people; outcome counts, peak split OFF -> ON '
          '(found / merged_person / merged_other / missed; hidden not scored)')
    for kind in KINDS:
        for lbl, sel in (('all', lambda r: True), ('nn<0.6', lambda r: r['nn'] < 0.6),
                         ('nn 0.6-1', lambda r: 0.6 <= r['nn'] < 1.0), ('r<3m', lambda r: r['range'] < 3),
                         ('r>=3m', lambda r: r['range'] >= 3)):
            rs = [r for r in rows if r['kind'] == kind and sel(r) and r['off'] != 'hidden']
            if len(rs) < 10:
                continue
            cell = []
            for arm in ('off', 'on'):
                c = [sum(r[arm] == o for r in rs) for o in ('found', 'merged_person', 'merged_other', 'missed')]
                cell.append(f'{100 * c[0] / len(rs):3.0f}% ({c[1]:3d}/{c[2]:3d}/{c[3]:3d})')
            flips = sum(r['off'] == 'found' and r['on'] != 'found' for r in rs)
            gains = sum(r['off'] != 'found' and r['on'] == 'found' for r in rs)
            dups = (sum(r['dups_off'] for r in rs), sum(r['dups_on'] for r in rs))
            print(f'  {kind:10s} {lbl:9s} n={len(rs):4d}  off {cell[0]}  ->  on {cell[1]}'
                  f'   gained {gains:3d} lost {flips:3d}  split pieces {dups[0]}->{dups[1]}')
    off = np.array([e['off'] for e in empty]); on = np.array([e['on'] for e in empty])
    print(f'\nempty frames ({len(empty)}): candidates/frame off {off.mean():.2f} -> on {on.mean():.2f}'
          f'  (frames with more: {int((on > off).sum())}, fewer: {int((on < off).sum())})')


# ---------------------------------------------------------------- linger

def _cand_dicts(rows):
    return [dict(zip(pc.FIELDS, map(float, r))) for r in rows]


def linger_episode(args):
    frames, seed, where = args
    rng = np.random.default_rng(seed)
    xyz0 = frames[0][0]
    occupied, open_bearing = sp.frame_context(xyz0, rng)
    furn = ()
    if where == 'real':
        spots = real_desk_spots(xyz0, rng)
        if not spots:
            return None
        sx, sy = spots[rng.integers(len(spots))]
    else:
        got = None
        for _ in range(10):
            got = sp.sample_furniture_scene(rng, 'stand_furn', occupied, np.zeros((0, 2)), open_bearing, abut=True)
            if got and got[2].get('furn') == 'desk' and not got[2].get('occluding'):
                break
            got = None
        if not got:
            return None
        sx, sy, furn = got[0][0]['x'], got[0][0]['y'], got[1]
    # walk away tangentially / outward, whichever stays in line of sight longest
    best = None
    for _ in range(12):
        hd = rng.uniform(-math.pi, math.pi)
        ok = sum(not _blocked(xyz0, (sx + d * math.cos(hd), sy + d * math.sin(hd))) for d in np.arange(0.5, 4.0, 0.5))
        if best is None or ok > best[0]:
            best = (ok, hd)
    hd = best[1]
    person = sp.sample_person(rng, sx, sy, hd, 0.0)
    dt = 0.1
    n = len(frames)
    walk_end = STAND_S + 4.0 / WALK_SPEED
    bias = rng.uniform(0.2, 0.6)                 # C3 over-reads range (seated ~0.7 m, standing less)
    out = {}
    for arm in ARMS:
        trk = lpt.LidarPersonTracker()
        prng = np.random.default_rng(seed + 7)
        seeds_prev = None
        log = []
        n_seeded = 0
        for i in range(n):
            now = 1000.0 + i * dt
            t = i * dt
            xyz, tsec, _ = frames[i]
            if t < STAND_S:
                p = dict(person, x=sx, y=sy, speed=0.0, vx=0.0, vy=0.0)
            elif t < walk_end:
                d = (t - STAND_S) * WALK_SPEED
                p = dict(person, x=sx + d * math.cos(hd), y=sy + d * math.sin(hd), speed=WALK_SPEED,
                         vx=WALK_SPEED * math.cos(hd), vy=WALK_SPEED * math.sin(hd), yaw=hd)
            else:
                p = None                           # gone (left the room)
            ppl = [p] if p else []
            pts, _ = sp.insert(xyz, tsec, ppl, prng, furniture=furn)
            x, y, z = pts.T
            cands = pc.extract(x, y, z, origin=(0.0, 0.0), peaks=True)
            cam_sees = p is not None and t < STAND_S
            camera = []
            if cam_sees:
                r = math.hypot(p['x'], p['y'])
                rr = r + bias + prng.normal(0, 0.1)
                b = math.atan2(p['y'], p['x']) + prng.normal(0, 0.01)
                camera = [dict(id='c1', state_stamp_s=now, obs_age_s=0.0, odom_x=rr * math.cos(b),
                               odom_y=rr * math.sin(b), odom_vx=0.0, odom_vy=0.0, measured=True)]
            cd = _cand_dicts(cands)
            if seeds_prev is not None and len(seeds_prev):
                seeded = pc.seeded_candidates(x, y, z, seeds_prev, (0.0, 0.0), existing=cands)
                n0 = len(cd)
                cd = lpt.merge_seeded(cd, seeded, pc.FIELDS, now, now)
                if t >= STAND_S:
                    n_seeded += len(cd) - n0
            res = trk.update(now, cd, camera, 0.0, origin=(0.0, 0.0))
            # seeds for the next frame, as server_x3.publish_c3_seeds builds them
            anchors = lpt.LidarPersonTracker._anchors(camera, now, 0.0)
            seeds = [(a_['x'], a_['y']) for a_ in anchors]
            if arm != 'off':
                for tr in trk.tracks:
                    if arm == 'cam3' and now - tr.last_camera > SEED_CAMERA_S:
                        continue
                    if tr.hits >= lpt.CONFIRM_HITS and all(math.hypot(tr.x - u, tr.y - v) >= TRACK_SEED_DEDUP_M
                                                            for u, v in seeds):
                        seeds.append((tr.x, tr.y))
            seeds_prev = np.array(seeds).reshape(-1, 2)
            gone = p is None or math.hypot(p['x'] - sx, p['y'] - sy) > 1.0
            at_spot = [r_ for r_ in res if r_['confirmed'] and math.hypot(r_['odom_x'] - sx, r_['odom_y'] - sy) < LINGER_M]
            log.append((t, gone, len(at_spot), len([r_ for r_ in res if r_['confirmed']])))
        # first time the person is > 1 m away, then how long a confirmed track is still at the spot
        t_gone = next((t for t, g, _, _ in log if g), None)
        last_at = max((t for t, g, k, _ in log if g and k), default=None)
        had = any(k for t, g, k, _ in log if not g)
        last_conf = max((t for t, _, _, c in log if c), default=0.0)
        out[arm] = dict(linger=0.0 if last_at is None else last_at - t_gone, tracked=had,
                        end=last_conf - STAND_S, seeded=n_seeded)
    return dict(where=where, seed=seed, **{f'{k}_{arm}': v for arm, d in out.items() for k, v in d.items()})


def run_linger(a, runs):
    seqs = []
    for run in runs:
        fr = sp.load_frames(run, 1)
        L = int((STAND_S + 4.0 / WALK_SPEED + lpt.LONG_HOLD_S + 5.0) / 0.1)
        for s in range(0, len(fr) - L, max(1, (len(fr) - L) // max(1, a.episodes // len(runs) // 2))):
            seqs.append(fr[s:s + L])
        print(f'{pathlib.Path(run).name}: {len(fr)} frames')
    rng = np.random.default_rng(a.seed)
    jobs = [(seqs[rng.integers(len(seqs))], a.seed * 7919 + k, w)
            for k in range(a.episodes) for w in ('real', 'synth')]
    with mp.Pool(a.jobs) as pool:
        rows = [r for r in pool.imap_unordered(linger_episode, jobs) if r]
    print(f'\n{len(rows)} episodes: stand {STAND_S:.0f} s seen by the camera, walk off at {WALK_SPEED} m/s, camera lost')
    for w in ('real', 'synth'):
        rs = [r for r in rows if r['where'] == w]
        if not rs:
            continue
        print(f'  {w + " desk":10s} n={len(rs)}')
        for arm in ARMS:
            tr = [r for r in rs if r[f'tracked_{arm}']]
            L = np.array([r[f'linger_{arm}'] for r in tr])
            if not len(L):
                print(f'    track seeds {arm:4s}: never confirmed at the spot')
                continue
            print(f'    track seeds {arm:4s}: tracked {len(tr)}/{len(rs)}; stays on the vacated spot '
                  f'median {np.median(L):.1f} s, p90 {np.percentile(L, 90):.1f} s, max {L.max():.1f} s; '
                  f'> 3 s in {int((L > 3).sum())}, > 10 s in {int((L > 10).sum())}')
            E = np.array([r[f'end_{arm}'] for r in tr])
            print(f'      last confirmed track {np.median(E):.1f} s median / {E.max():.1f} s max after the camera lost them'
                  f' (walk-out takes {4.0 / WALK_SPEED:.0f} s); seeded rows used after leaving: '
                  f'{sum(r[f"seeded_{arm}"] for r in tr)}; a track outlives the person (gone at 4 s) by > 2 s in '
                  f'{int((E > 4.0 / WALK_SPEED + 2).sum())}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('test', choices=('split', 'linger'))
    ap.add_argument('runs', nargs='+', help='EMPTY runs (no real people) as backgrounds')
    ap.add_argument('--scenes', type=int, default=400, help='split: scenes per kind')
    ap.add_argument('--episodes', type=int, default=40, help='linger: episodes per spot type')
    ap.add_argument('--every', type=int, default=5)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('-j', '--jobs', type=int, default=None)
    a = ap.parse_args()
    if a.test == 'split':
        frames = []
        for run in a.runs:
            fr = sp.load_frames(run, a.every)
            print(f'{pathlib.Path(run).name}: {len(fr)} background frames')
            frames += fr
        run_split(a, frames)
    else:
        run_linger(a, a.runs)


if __name__ == '__main__':
    main()
