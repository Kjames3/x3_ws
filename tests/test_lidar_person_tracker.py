"""Camera-seeded lidar person tracker (src/lidar_person_tracker.py) and the Livox
cluster extractor that feeds it (yahboomcar_bringup/person_candidates.py)."""
import ast
import math
import pathlib
import struct
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'))

import lidar_person_tracker as lpt
from lidar_person_tracker import LidarPersonTracker, cbf_estimates, to_local
from yahboomcar_bringup import person_candidates as pc

DT = 0.1


# -- helpers -----------------------------------------------------------------------
def cand(x, y, r=0.2, n=60):
    return {'x': x, 'y': y, 'x_lo': x - r, 'x_hi': x + r, 'y_lo': y - r, 'y_hi': y + r,
            'z_lo': 0.15, 'z_hi': 1.7, 'n': n}


def cam(cid, x, y, now, vx=0.0, vy=0.0, age=0.0):
    return {'id': cid, 'odom_x': x, 'odom_y': y, 'odom_vx': vx, 'odom_vy': vy,
            'state_stamp_s': now, 'obs_age_s': age, 'measured': True}


def run(tr, frames):
    """frames: list of (now, candidates, camera).  Returns the last results."""
    out = None
    for now, cs, cm in frames:
        out = tr.update(now, cs, cm, packet_age=0.0)
    return out


# -- tracker -----------------------------------------------------------------------
def test_clusters_alone_never_make_a_person():
    tr = LidarPersonTracker()
    out = run(tr, [(i * DT, [cand(2.0, 0.0)], []) for i in range(30)])
    assert out == [] and tr.stats['births'] == 0


def test_walk_up_is_held_through_the_camera_blind_range_then_capped():
    """Walks at the robot (at the odom origin) at 1 m/s from 3 m; the camera loses the
    person at 1 m, the lidar keeps them: same id, 'lidar held', ends at the 3 s cap."""
    tr = LidarPersonTracker()
    frames, t = [], 0.0
    x = 3.0
    while x > 1.0:
        frames.append((t, [cand(x, 0.0)], [cam('a', x, 0.0, t, vx=-1.0)]))
        t += DT
        x -= 1.0 * DT
    out = run(tr, frames)
    assert len(out) == 1 and out[0]['mode'] == 'camera + lidar'
    tid, lost_at = out[0]['id'], t - DT
    # Person stops at 0.6 m and stands there, camera blind.
    x_stop = 0.6
    seen_held = False
    while t < lost_at + 3.5:
        x = max(x_stop, x - 1.0 * DT)
        out = tr.update(t, [cand(x, 0.0)], [], packet_age=0.0)
        if out:
            assert out[0]['id'] == tid
            seen_held |= out[0]['mode'] == 'lidar held'
            if t - lost_at > lpt.CAMERA_FRESH_S:
                assert out[0]['confidence'] < 1.0
        t += DT
    assert seen_held
    assert out == [] and tr.stats['hold_expired'] == 1
    assert tr.stats['lidar_held_s'] == pytest.approx(3.0 - lpt.CAMERA_FRESH_S, abs=0.25)


def test_new_camera_id_joins_the_lidar_track():
    tr = LidarPersonTracker()
    frames = []
    for i in range(40):
        t = i * DT
        x = 3.0 - 0.5 * t
        cid = 'a' if i < 15 else ('b' if i >= 22 else None)      # 0.7 s gap, new id
        frames.append((t, [cand(x, 0.3)], [cam(cid, x, 0.3, t, vx=-0.5)] if cid else []))
    out = run(tr, frames)
    assert len(out) == 1 and out[0]['id'] == 'L1' and out[0]['camera_id'] == 'b'
    assert out[0]['camera_ids'] == 2 and tr.stats['relinked'] == 1 and tr.stats['births'] == 1


def test_lidar_gap_ends_the_track():
    tr = LidarPersonTracker()
    run(tr, [(i * DT, [cand(2.0, 0.0)], [cam('a', 2.0, 0.0, i * DT)]) for i in range(5)])
    out = run(tr, [(0.5 + i * DT, [], [cam('a', 2.0, 0.0, 0.5 + i * DT)]) for i in range(5)])
    assert out == [] and tr.stats['lost_lidar'] == 1


def test_velocity_follows_a_walker():
    tr = LidarPersonTracker()
    frames = [(i * DT, [cand(1.0 + 0.8 * i * DT, 2.0)], [cam('a', 1.0 + 0.8 * i * DT, 2.0, i * DT, vx=0.8)])
              for i in range(10)]
    frames += [(1.0 + i * DT, [cand(1.8 + 0.8 * i * DT, 2.0)], []) for i in range(15)]
    out = run(tr, frames)
    assert out[0]['mode'] == 'lidar held' and out[0]['vel_trusted']
    assert out[0]['odom_vx'] == pytest.approx(0.8, abs=0.1)
    assert out[0]['odom_x'] == pytest.approx(1.8 + 0.8 * 1.4, abs=0.1)


def test_two_people_merged_into_one_cluster_get_no_update_and_keep_their_ids():
    tr = LidarPersonTracker()
    run(tr, [(i * DT, [cand(2.0, 0.6), cand(2.0, -0.6)],
              [cam('a', 2.0, 0.6, i * DT), cam('b', 2.0, -0.6, i * DT)]) for i in range(5)])
    ids = {r['camera_id']: r['id'] for r in tr.results}
    assert set(ids) == {'a', 'b'}
    # They step together for 0.2 s: one wide cluster between them.
    merged = {'x': 2.0, 'y': 0.0, 'x_lo': 1.8, 'x_hi': 2.2, 'y_lo': -0.45, 'y_hi': 0.45,
              'z_lo': 0.15, 'z_hi': 1.7, 'n': 120}
    run(tr, [(0.5 + i * DT, [merged], []) for i in range(2)])
    assert all(r['lidar_age_s'] > 0 for r in tr.results)    # coasting, not pulled together
    out = run(tr, [(0.7 + i * DT, [cand(2.0, 0.6), cand(2.0, -0.6)], []) for i in range(3)])
    assert {round(r['odom_y'], 1): r['id'] for r in out} == {0.6: ids['a'], -0.6: ids['b']}


def test_ambiguous_seed_is_not_born():
    tr = LidarPersonTracker()
    out = run(tr, [(i * DT, [cand(2.0, 0.15), cand(2.0, -0.15)], [cam('a', 2.0, 0.0, i * DT)])
                   for i in range(5)])
    assert out == []


def test_stale_or_old_camera_seeds_nothing():
    tr = LidarPersonTracker()
    for i in range(5):
        tr.update(i * DT, [cand(2.0, 0.0)], [cam('a', 2.0, 0.0, i * DT)], packet_age=0.8)
    assert tr.results == []
    tr = LidarPersonTracker()
    run(tr, [(i * DT, [cand(2.0, 0.0)], [cam('a', 2.0, 0.0, i * DT, age=1.0)]) for i in range(5)])
    assert tr.results == []


def test_stamp_going_back_resets():
    tr = LidarPersonTracker()
    run(tr, [(10 + i * DT, [cand(2.0, 0.0)], [cam('a', 2.0, 0.0, 10 + i * DT)]) for i in range(5)])
    assert tr.results
    tr.update(5.0, [cand(2.0, 0.0)], [], packet_age=0.0)
    assert tr.results == [] and tr.stats['resets'] == 1


def test_to_local_and_cbf_estimates():
    tr = LidarPersonTracker()
    out = run(tr, [(i * DT, [cand(2.0, 1.0)], [cam('a', 2.0, 1.0, i * DT)]) for i in range(8)])
    pose = {'x': 1.0, 'y': 1.0, 'theta': math.pi / 2}       # robot faces +y
    loc = to_local(out, pose)[0]
    assert loc['fwd'] == pytest.approx(0.0, abs=1e-3) and loc['left'] == pytest.approx(-1.0, abs=1e-3)
    assert loc['vx'] == pytest.approx(loc['odom_vy'], abs=1e-3)
    now = out[0]['stamp']
    est = cbf_estimates([loc], [], now)
    assert len(est) == 1 and est[0]['z'] == loc['fwd'] and est[0]['x'] == -loc['left']
    assert est[0]['person_frames'] >= 5 and est[0]['category'] == 'person'
    # The camera estimator already has someone there: it stays the source.
    v3 = [{'category': 'person', 'age': 0, 'z': 0.1, 'x': 0.9}]
    assert cbf_estimates([loc], v3, now) == []
    # Stale, coasting or tentative tracks never reach the CBF.
    assert cbf_estimates([loc], [], now + 1.0) == []
    assert cbf_estimates([dict(loc, lidar_age_s=0.1)], [], now) == []
    assert cbf_estimates([dict(loc, confirmed=False)], [], now) == []
    assert cbf_estimates([dict(loc, vel_trusted=False)], [], now)[0]['person_frames'] == 0


def test_cbf_estimates_pass_person_memory_gates():
    from person_memory import PersonMemory
    loc = {'id': 'L1', 'confirmed': True, 'lidar_age_s': 0.0, 'stamp': 0.0, 'fwd': 0.8,
           'left': 0.0, 'vx': -0.6, 'vy': 0.0, 'speed': 0.6, 'vel_trusted': True, 'hits': 9}
    movers = PersonMemory().update(cbf_estimates([loc], [], 0.0), 0.0)
    assert movers == [((0.8, 0.0), (-0.6, 0.0))]


# -- cluster extraction ------------------------------------------------------------
def _person(rng, x, y, n=300, r=0.2, top=1.7):
    a, h = rng.uniform(0, 2 * math.pi, n), rng.uniform(0.16, top, n)
    return np.c_[x + r * np.cos(a), y + r * np.sin(a), h]


def _scene(rng):
    wall = np.c_[np.linspace(-2, 2, 2000), np.full(2000, 3.0), rng.uniform(0.16, 2.1, 2000)]
    floor = np.c_[rng.uniform(-4, 4, 1000), rng.uniform(-4, 4, 1000), rng.uniform(-0.02, 0.05, 1000)]
    box = np.c_[rng.uniform(-1.6, -1.4, 100), rng.uniform(1.0, 1.2, 100), rng.uniform(0.16, 0.35, 100)]
    return np.vstack([wall, floor, box, _person(rng, 1.5, -0.5)])


def test_person_found_wall_floor_and_low_box_rejected():
    pts = _scene(np.random.default_rng(0))
    c = pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])
    assert c.shape == (1, len(pc.FIELDS))
    row = dict(zip(pc.FIELDS, c[0]))
    assert row['x'] == pytest.approx(1.5, abs=0.1) and row['y'] == pytest.approx(-0.5, abs=0.1)
    assert row['z_hi'] > 1.5 and row['n'] == 300


def test_person_touching_a_wall_is_not_a_candidate():
    """Documented limitation: within one grid cell of a wall they are one component."""
    rng = np.random.default_rng(1)
    wall = np.c_[np.linspace(-2, 2, 2000), np.full(2000, 3.0), rng.uniform(0.16, 2.1, 2000)]
    pts = np.vstack([wall, _person(rng, 0.0, 2.75)])
    assert len(pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])) == 0


def test_range_limit_uses_the_robot_origin():
    pts = _person(np.random.default_rng(2), 10.0, 0.0)
    assert len(pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])) == 0
    assert len(pc.extract(pts[:, 0], pts[:, 1], pts[:, 2], origin=(9.0, 0.0))) == 1


def test_fallback_labeler_matches_scipy():
    if pc._ndimage is None:
        pytest.skip('scipy not installed')
    occ = np.random.default_rng(3).random((60, 60)) > 0.6
    a = pc._label(occ)
    saved, pc._ndimage = pc._ndimage, None
    try:
        b = pc._label(occ)
    finally:
        pc._ndimage = saved
    # Same partition, possibly different label numbers.
    pairs = set(zip(a[occ].tolist(), b[occ].tolist()))
    assert len(pairs) == a.max() == b.max()


def test_node_publishes_candidates_in_odom():
    pytest.importorskip('sensor_msgs.msg')
    import time as _time
    from sensor_msgs.msg import PointCloud2, PointField
    path = pathlib.Path('src/yahboomcar_bringup/yahboomcar_bringup/livox_scan_node.py')
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'publish_person_candidates')
    ns = dict(np=np, struct=struct, time=_time, PointCloud2=PointCloud2, PointField=PointField,
              extract_candidates=pc.extract, CANDIDATE_FIELDS=pc.FIELDS)
    exec(compile(ast.Module(body=[method], type_ignores=[]), 'cands', 'exec'), ns)
    rng = np.random.default_rng(4)
    person = _person(rng, 2.0, 0.0, n=200) - [0.0432, 0.0, 0.31]     # sensor frame
    pts = np.zeros(len(person), dtype=[('t', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
    pts['x'], pts['y'], pts['z'] = person.T
    pts['t'] = np.linspace(0, 99_000_000, len(pts)).astype(np.uint32)
    sent = []
    # Robot at odom (5, 1) facing +y: the person 2 m ahead is at odom (5, 3).
    pose = lambda s: (np.full(len(s), 5.0), np.full(len(s), 1.0), np.full(len(s), math.pi / 2))
    logger = NS(info=lambda *_: None)
    obj = NS(cand_pub=NS(get_subscription_count=lambda: 1, publish=sent.append),
             cloud_R=np.eye(3), cloud_t=np.array([0.0432, 0.0, 0.31]), pose_at=pose,
             cand_ms=[], cand_n=[], cand_logged_ns=0, get_logger=lambda: logger,
             get_clock=lambda: NS(now=lambda: NS(nanoseconds=0)))
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    m = sent[0]
    assert m.header.frame_id == 'odom' and m.width == 1 and m.header.stamp.sec == 100
    row = dict(zip(pc.FIELDS, np.frombuffer(m.data, dtype='<f4')))
    assert row['x'] == pytest.approx(5.0, abs=0.1) and row['y'] == pytest.approx(3.0, abs=0.1)
    obj.pose_at = lambda s: None                        # no odom: nothing published
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    assert len(sent) == 1


def test_replay_tool_scores_a_recorded_walk_up(tmp_path):
    import json
    from lidar_person_replay import replay
    lines = []
    for i in range(50):
        t = 100 + i * DT
        x = max(0.6, 3.0 - 1.0 * i * DT)
        camera = [cam('a', x, 0.0, t, vx=-1.0)] if x > 1.0 else []
        lines.append(json.loads(json.dumps({'stamp': t, 'packet_age': 0.05, 'fields': list(pc.FIELDS),
                                            'candidates': [[cand(x, 0.0)[k] for k in pc.FIELDS]],
                                            'camera': camera, 'results': []})))
    r = replay(lines)
    assert r['frames'] == 50 and r['camera_ids'] == 1 and r['lidar_ids'] == 1
    assert r['camera_frames'] < r['combined_frames'] == 50
    assert r['lidar_only_frames'] == r['combined_frames'] - r['camera_frames']


def _bench(rng, y0, n=3000):
    """A 6 m bench line ~1 m tall at y = y0 (the 2026-10-30 lab trial)."""
    return np.c_[rng.uniform(-3, 3, n), rng.uniform(y0, y0 + 0.4, n), rng.uniform(0.16, 0.95, n)]


def test_person_in_front_of_low_bench_is_split_out():
    """Trial 2026-10-30: ~0.5 m in front of a bench, a few stray returns in the gap
    bridged the 18 cm grid and the person was rejected in 2 of 3 frames."""
    rng = np.random.default_rng(5)
    stray = np.c_[rng.uniform(-0.05, 0.05, 6), rng.uniform(0.3, 0.55, 6), rng.uniform(0.2, 0.8, 6)]
    pts = np.vstack([_bench(rng, 0.6), _person(rng, 0.0, 0.0), stray])
    c = pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])
    assert len(c) == 1
    row = dict(zip(pc.FIELDS, c[0]))
    assert abs(row['x']) < 0.1 and abs(row['y']) < 0.1 and row['z_lo'] < 0.3 and row['z_hi'] > 1.5


def test_split_still_rejects_walls_and_tall_furniture():
    rng = np.random.default_rng(6)
    wall = np.c_[np.linspace(-3, 3, 4000), np.full(4000, 2.0), rng.uniform(0.16, 2.1, 4000)]
    cabinet = np.c_[rng.uniform(1.0, 2.2, 1500), rng.uniform(-1.0, -0.5, 1500), rng.uniform(0.16, 1.9, 1500)]
    pts = np.vstack([wall, cabinet])
    assert len(pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])) == 0
    # A person pressed against the cabinet (inside the ring) stays unfound: known limit.
    pts = np.vstack([cabinet, _person(rng, 1.6, -0.2)])
    assert len(pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])) == 0
