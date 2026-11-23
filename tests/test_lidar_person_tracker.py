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
    names = ('publish_person_candidates', 'seeds_at', '_candidate_msg', 'side_rays_at')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    ns = dict(np=np, struct=struct, time=_time, PointCloud2=PointCloud2, PointField=PointField,
              extract_candidates=pc.extract, seeded_candidates=pc.seeded_candidates,
              ray_candidates=pc.ray_candidates, CANDIDATE_FIELDS=pc.FIELDS,
              SIDE_RAY_MAX_AGE_S=0.3)
    ns['SIDE_FIELDS'] = pc.FIELDS + ('ax', 'ay', 'cam', 'tid', 'age')
    exec(compile(ast.Module(body=methods, type_ignores=[]), 'cands', 'exec'), ns)
    rng = np.random.default_rng(4)
    person = _person(rng, 2.0, 0.0, n=200) - [0.0432, 0.0, 0.31]     # sensor frame
    pts = np.zeros(len(person), dtype=[('t', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
    pts['x'], pts['y'], pts['z'] = person.T
    pts['t'] = np.linspace(0, 99_000_000, len(pts)).astype(np.uint32)
    sent, seeded, side_sent = [], [], []
    # Robot at odom (5, 1) facing +y: the person 2 m ahead is at odom (5, 3).
    pose = lambda s: (np.full(len(s), 5.0), np.full(len(s), 1.0), np.full(len(s), math.pi / 2))
    logger = NS(info=lambda *_: None)
    obj = NS(cand_pub=NS(get_subscription_count=lambda: 1, publish=sent.append),
             cloud_R=np.eye(3), cloud_t=np.array([0.0432, 0.0, 0.31]), pose_at=pose,
             cand_ms=[], cand_n=[], cand_logged_ns=0, get_logger=lambda: logger,
             get_clock=lambda: NS(now=lambda: NS(nanoseconds=0)),
             seeds=None, seed_pub=NS(get_subscription_count=lambda: 1, publish=seeded.append),
             side_rays={}, side_pub=NS(get_subscription_count=lambda: 1, publish=side_sent.append),
             _candidate_msg=ns['_candidate_msg'])
    obj.seeds_at = lambda t: ns['seeds_at'](obj, t)
    obj.side_rays_at = lambda t: ns['side_rays_at'](obj, t)
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    assert not seeded                                   # no camera seeds: nothing seeded
    m = sent[0]
    assert m.header.frame_id == 'odom' and m.width == 1 and m.header.stamp.sec == 100
    row = dict(zip(pc.FIELDS, np.frombuffer(m.data, dtype='<f4')))
    assert row['x'] == pytest.approx(5.0, abs=0.1) and row['y'] == pytest.approx(3.0, abs=0.1)
    # A fresh camera seed on the person: seeded rows are published on their own topic,
    # and this one is a duplicate of the ordinary candidate, so the message is empty.
    obj.seeds = (100.099, np.array([[5.0, 3.0, 0.0, 0.0]]))
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    assert len(seeded) == 1 and seeded[0].width == 0 and len(sent) == 2
    obj.seeds = (99.0, np.array([[5.0, 3.0, 0.0, 0.0]]))    # stale seeds are ignored
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    assert len(seeded) == 1
    assert not side_sent                                # no side rays: nothing published
    # A left-camera ray from beside the robot through the person (camera index 2, id 7),
    # 0.05 s old; the body duplicates the ordinary candidate, so it anchors there.
    ray = [5.0, 1.05, 0.21, 0.0, 1.0, 0.4, 6.0, float('nan'), 7.0]
    obj.side_rays = {2: (100.049, np.array([ray]))}
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    m = side_sent[-1]
    assert [f.name for f in m.fields] == list(ns['SIDE_FIELDS']) and m.width == 1
    row = dict(zip(ns['SIDE_FIELDS'], np.frombuffer(m.data, dtype='<f4')))
    assert row['ax'] == pytest.approx(5.0, abs=0.1) and row['ay'] == pytest.approx(3.0, abs=0.1)
    assert (row['cam'], row['tid']) == (2.0, 7.0) and row['age'] == pytest.approx(0.05, abs=0.01)
    assert len(sent) == 4 and sent[-1].width == 1       # ordinary candidates unchanged
    obj.side_rays = {2: (99.0, np.array([ray]))}        # stale rays are ignored
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    assert len(side_sent) == 1
    obj.pose_at = lambda s: None                        # no odom: nothing published
    ns['publish_person_candidates'](obj, pts, struct.pack('<I i I', 256, 100, 0))
    assert len(sent) == 5


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


def _person_front(rng, x, y, n=400, r=0.2, top=1.7):
    """The half of a person a sensor at the origin actually sees."""
    pts = _person(rng, x, y, n, r, top)
    return pts[(pts[:, 0] - x) * x + (pts[:, 1] - y) * y < 0]


def _cut_by_fov(pts, origin=(0.0, 0.0)):
    """Drop what lies above the Livox's +52 deg upper FOV edge (sensor 0.31 m up)."""
    r = np.hypot(pts[:, 0] - origin[0], pts[:, 1] - origin[1])
    return pts[pts[:, 2] < 0.31 + r * 1.28]


def test_close_person_cut_by_the_fov_beside_a_bench_is_found():
    """Circle run 2026-10-30: at ~0.65 m the FOV top cuts a person at ~1.1 m, so the
    1.1 m split finds nothing; the near pass at 0.9 m does."""
    rng = np.random.default_rng(7)
    stray = np.c_[rng.uniform(0.85, 0.95, 6), rng.uniform(-0.3, 0.3, 6), rng.uniform(0.3, 0.8, 6)]
    pts = _cut_by_fov(np.vstack([_person_front(rng, 0.65, 0.0, n=1600), _bench(rng, 0.0)[:, [1, 0, 2]] + [0.95, 0, 0], stray]))
    c = pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])
    near = [r for r in c if math.hypot(r[0], r[1]) < 1.0]
    assert len(near) == 1 and abs(near[0][0] - 0.55) < 0.1 and abs(near[0][1]) < 0.1


def test_close_chair_back_beside_a_bench_is_not_a_candidate():
    """A 0.95 m chair back near the robot stops well short of the FOV top."""
    rng = np.random.default_rng(8)
    chair = np.c_[rng.uniform(0.55, 0.95, 400), rng.uniform(-0.2, 0.2, 400), rng.uniform(0.16, 0.95, 400)]
    stray = np.c_[rng.uniform(0.95, 1.05, 6), rng.uniform(-0.3, 0.3, 6), rng.uniform(0.3, 0.8, 6)]
    pts = _cut_by_fov(np.vstack([chair, _bench(rng, 0.0)[:, [1, 0, 2]] + [1.1, 0, 0], stray]))
    c = pc.extract(pts[:, 0], pts[:, 1], pts[:, 2])
    assert not [r for r in c if math.hypot(r[0], r[1]) < 1.0]


def test_merge_seeded_adds_fresh_rows_and_dedups():
    from lidar_person_tracker import merge_seeded
    names = ['x', 'y', 'x_lo', 'x_hi', 'y_lo', 'y_hi', 'z_lo', 'z_hi', 'n']
    base = [dict(zip(names, [1.0, 0.0, .8, 1.2, -.2, .2, 0, 1.7, 80]))]
    rows = [[1.2, 0.1, 1, 1.4, 0, .3, .3, 1.9, 150],     # within 0.5 m of base: dropped
            [3.0, 2.0, 2.8, 3.2, 1.8, 2.2, .4, 1.9, 150]]  # new
    out = merge_seeded(base, rows, names, 10.0, 10.05)
    assert len(out) == 2 and out[1]['x'] == 3.0 and out[0] is base[0]
    assert merge_seeded(base, rows, names, 10.0, 10.3) == base     # stale
    assert merge_seeded(base, None, names, None, 10.0) == base
    assert len(base) == 1                                          # input untouched


def test_seeded_track_ends_when_the_camera_leaves():
    """A seeded-only body (the couch) gives no track once the seeds stop."""
    from lidar_person_tracker import merge_seeded, LIDAR_GAP_S
    names = ['x', 'y', 'x_lo', 'x_hi', 'y_lo', 'y_hi', 'z_lo', 'z_hi', 'n']
    row = [2.0, 1.0, 1.8, 2.2, .8, 1.2, .4, 1.9, 150]
    tr = LidarPersonTracker()
    t = 100.0
    for k in range(10):
        cam = [{'id': 7, 'odom_x': 2.0, 'odom_y': 1.0, 'odom_vx': 0.0, 'odom_vy': 0.0,
                'state_stamp_s': t, 'obs_age_s': 0.0, 'measured': True}]
        r = tr.update(t, merge_seeded([], [row], names, t, t), cam)
        t += 0.1
    assert r and r[0]['confirmed']
    for k in range(int(LIDAR_GAP_S / 0.1) + 2):
        r = tr.update(t, merge_seeded([], None, names, None, t), [])
        t += 0.1
    assert r == []


def test_snap_to_ray_moves_anchor_to_nearest_body_on_the_ray():
    from lidar_person_tracker import snap_to_ray
    a = [{'id': '1', 'x': 3.0, 'y': 0.0}]
    c = [{'x': 2.3, 'y': 0.3}, {'x': 2.8, 'y': 0.1}, {'x': 1.0, 'y': 0.0}, {'x': 2.5, 'y': 1.0}]
    out = snap_to_ray(a, c, (0.0, 0.0))
    assert (out[0]['x'], out[0]['y']) == (2.3, 0.3) and a[0]['x'] == 3.0   # nearest in window; input kept
    assert snap_to_ray(a, [{'x': 1.0, 'y': 0.0}], (0.0, 0.0))[0]['x'] == 3.0  # too near: unchanged


def test_over_read_camera_confirms_track_with_origin():
    """Camera 0.7 m beyond the body (desk over-read): only the ray association keeps it."""
    def run(origin):
        tr = LidarPersonTracker(); t = 50.0; r = []
        cand = [{'x': 2.3, 'y': 0.0, 'x_lo': 2.1, 'x_hi': 2.5, 'y_lo': -.2, 'y_hi': .2,
                 'z_lo': .4, 'z_hi': 1.3, 'n': 120}]
        for _ in range(8):
            cam = [{'id': 4, 'odom_x': 3.0, 'odom_y': 0.0, 'odom_vx': 0.0, 'odom_vy': 0.0,
                    'state_stamp_s': t, 'obs_age_s': 0.0, 'measured': True}]
            r = tr.update(t, cand, cam, origin=origin)
            t += 0.1
        return r
    assert run(None) == []
    r = run((0.0, 0.0))
    assert r and r[0]['confirmed'] and r[0]['mode'] == 'camera + lidar'


# -- side-camera anchors (X3_SIDE_PERSONS) ---------------------------------------------
def side(sid, x, y, seen):
    return {'id': sid, 'x': x, 'y': y, 'last_seen': seen}


def test_side_anchor_births_and_confirms_without_c3():
    """A person only a side camera sees (C3 packet stale/absent) still starts a track."""
    tr = LidarPersonTracker()
    out = None
    for i in range(6):
        t = 10.0 + i * DT
        out = tr.update(t, [cand(0.0, 2.5)], [], packet_age=float('inf'),
                        side=[side('left:3', 0.0, 2.5, t - 0.05)])
    assert len(out) == 1 and out[0]['confirmed'] and out[0]['camera_id'] == 'left:3'
    assert out[0]['mode'] == 'camera + lidar'
    assert tr.stats['side_births'] == 1 and tr.stats['side_confirms'] == 5
    # and without side anchors nothing is born
    tr2 = LidarPersonTracker()
    assert run(tr2, [(10.0 + i * DT, [cand(0.0, 2.5)], []) for i in range(6)]) == []


def test_side_anchor_does_not_pull_velocity_to_zero():
    """A walker first seen by C3 (with velocity), then only by the side camera."""
    tr = LidarPersonTracker()
    out = None
    for i in range(30):
        t = i * DT
        x = 1.0 + 0.8 * t
        cm = [cam('a', x, 2.0, t, vx=0.8)] if i < 10 else []
        sd = [side('left:1', x, 2.0, t)] if i >= 10 else None
        out = tr.update(t, [cand(x, 2.0)], cm, packet_age=0.0, side=sd)
    assert len(out) == 1 and out[0]['camera_id'] == 'left:1' and out[0]['camera_ids'] == 2
    assert out[0]['odom_vx'] == pytest.approx(0.8, abs=0.15)


def test_side_anchor_on_a_c3_person_is_dropped_not_a_rival():
    """Both cameras see one person: the C3 anchor confirms, the side one is deduped."""
    tr = LidarPersonTracker()
    out = None
    for i in range(6):
        t = i * DT
        out = tr.update(t, [cand(1.5, 1.5)], [cam('a', 1.5, 1.5, t)], packet_age=0.0,
                        side=[side('left:2', 1.6, 1.45, t)])
    assert len(out) == 1 and out[0]['confirmed'] and out[0]['camera_id'] == 'a'
    assert tr.stats['side_confirms'] == 0 and tr.stats['side_births'] == 0


def test_stale_side_anchor_seeds_nothing():
    tr = LidarPersonTracker()
    for i in range(6):
        t = 5.0 + i * DT
        out = tr.update(t, [cand(0.0, -2.0)], [], packet_age=0.0,
                        side=[side('right:1', 0.0, -2.0, t - 1.0)])
    assert out == [] and tr.stats['births'] == 0


def test_side_anchor_is_not_snapped_along_the_robot_ray():
    """With origin given, a side anchor stays on its own body: a nearer candidate on the
    robot-centre line must not steal it (only C3 anchors are snapped)."""
    tr = LidarPersonTracker()
    for i in range(6):
        t = i * DT
        out = tr.update(t, [cand(0.0, 1.0), cand(0.0, 2.0)], [], packet_age=0.0,
                        origin=(0.0, 0.0), side=[side('left:1', 0.0, 2.0, t)])
    assert len(out) == 1 and out[0]['odom_y'] == pytest.approx(2.0, abs=0.05)


def test_two_side_anchors_on_one_body_still_confirm():
    """Two rays ending on the same body (duplicate boxes / people in line) give one anchor."""
    tr = LidarPersonTracker()
    for i in range(6):
        t = i * DT
        out = tr.update(t, [cand(0.0, 2.0)], [], packet_age=0.0,
                        side=[side('left:1', 0.0, 2.0, t), side('left:2', 0.02, 2.01, t)])
    assert len(out) == 1 and out[0]['confirmed'] and out[0]['camera_id'] == 'left:1'
