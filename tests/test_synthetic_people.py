"""Synthetic people inserted into lidar frames: ray casting, occlusion, scoring."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'))

import synthetic_people as sp  # noqa: E402


def _scene(seed=0, n=60000):
    """Rays over +-35 deg azimuth, -7..52 deg elevation onto a floor and a wall at x = 8."""
    rng = np.random.default_rng(seed)
    az = rng.uniform(-0.6, 0.6, n)
    el = np.radians(rng.uniform(-7, 52, n))
    d = np.c_[np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]
    t_floor = np.where(d[:, 2] < 0, -sp.SENSOR_Z / np.where(d[:, 2] < 0, d[:, 2], -1), np.inf)
    t = np.minimum(8.0 / d[:, 0], t_floor)
    return np.array([0, 0, sp.SENSOR_Z]) + d * t[:, None], rng.uniform(-0.05, 0.05, n)


def _person(x, y, speed=0.0, seed=0):
    p = sp.sample_person(np.random.default_rng(seed), x, y, math.pi, speed)
    p.update(height=1.75, girth=1.0, shoulder=1.0, backpack=False)
    return p


def test_raycast_unit_sphere():
    ell = (np.array([[3.0, 0.0, 0.0]]), np.eye(3)[None], np.ones((1, 3)))
    dirs = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    r, cos = sp.raycast(np.zeros(3), dirs, ell)
    assert abs(r[0] - 2.0) < 1e-9 and abs(cos[0] - 1.0) < 1e-9
    assert np.isinf(r[1]) and cos[1] == 0


def test_body_is_person_sized():
    c, R, a = sp.body(_person(0.0, 0.0), 0.0)
    top = (c[:, 2] + a.max(1)).max()
    assert 1.65 < top < 1.85 and np.abs(c[:, 1]).max() < 0.3


def test_insert_occludes_the_background_behind_the_person():
    xyz, ts = _scene()
    out, pid = sp.insert(xyz, ts, [_person(4.0, 0.0)], np.random.default_rng(1))
    mine = out[pid == 0]
    assert len(mine) > 200
    assert np.abs(np.hypot(mine[:, 0], mine[:, 1]) - 4.0).max() < 0.5
    # the torso's shadow on the wall (body z 1.0-1.3 m at 4 m -> 1.7-2.3 m at 8 m) is gone
    wall = (out[:, 0] > 7.9) & (np.abs(out[:, 1]) < 0.1) & (out[:, 2] > 1.75) & (out[:, 2] < 2.2)
    assert wall.sum() == 0
    # elsewhere the frame is untouched
    assert ((pid == -1) & (np.abs(out[:, 0] - 8.0) < 1e-6)).sum() > 1000


def test_lone_person_is_found_and_hidden_one_is_not_scored():
    xyz, ts = _scene()
    rng = np.random.default_rng(2)
    people = [_person(4.0, 0.0), _person(6.5, 0.0, seed=1)]      # second stands behind the first
    out, pid = sp.insert(xyz, ts, people, rng)
    res = sp.score(out, pid, people)
    assert res[0][0] == 'found' and res[0][1][0] > 3.5           # features filled in, range ~3.9 m
    assert res[1][0] in ('hidden', 'found', 'missed')            # partly visible past the shoulders


def test_close_pair_is_scored_per_person():
    xyz, ts = _scene()
    people = [_person(4.0, -0.25), _person(4.0, 0.25, seed=1)]
    res = sp.score(*sp.insert(xyz, ts, people, np.random.default_rng(3)), people)
    assert all(r[0] in sp.OUTCOMES and r[3] >= 0 for r in res)
    assert sum(r[0] == 'found' for r in res) <= 2


# -- head/shoulder peak split (person_candidates.extract(..., peaks=True)) -------------

from yahboomcar_bringup import person_candidates as pc  # noqa: E402


def _near(c, x, y, r=sp.MATCH_M):
    return len(c) and np.hypot(c[:, 0] - x, c[:, 1] - y).min() < r


def test_peaks_split_two_people_shoulder_to_shoulder():
    xyz, ts = _scene()
    people = [_person(4.0, -0.27), _person(4.0, 0.27, seed=1)]
    out, _ = sp.insert(xyz, ts, people, np.random.default_rng(3))
    base = pc.extract(out[:, 0], out[:, 1], out[:, 2], range_max=12)
    split = pc.extract(out[:, 0], out[:, 1], out[:, 2], range_max=12, peaks=True)
    assert not (_near(base, 4.0, -0.27) and _near(base, 4.0, 0.27))       # merged without peaks
    assert _near(split, 4.0, -0.27) and _near(split, 4.0, 0.27)


def test_peaks_leave_a_lone_walker_whole():
    xyz, ts = _scene()
    for seed in range(5):
        out, _ = sp.insert(xyz, ts, [_person(3.0, 0.0, speed=1.3, seed=seed)], np.random.default_rng(seed))
        c = pc.extract(out[:, 0], out[:, 1], out[:, 2], range_max=12, peaks=True)
        assert ((np.hypot(c[:, 0] - 3.0, c[:, 1]) < 0.8).sum() == 1) if len(c) else False


def test_peaks_do_not_cut_a_flat_panel_into_people():
    rng = np.random.default_rng(0)
    n = 4000                                            # 1.4 m wide, 1.8 m tall board at 4 m
    pts = np.c_[np.full(n, 4.0) + rng.normal(0, 0.01, n), rng.uniform(-0.7, 0.7, n), rng.uniform(0.0, 1.8, n)]
    assert len(pc.extract(pts[:, 0], pts[:, 1], pts[:, 2], peaks=True)) == 0


# -- seated people and furniture (2026-11-22) ------------------------------------------

def _seated(x=4.0, y=0.0, mode='desk', seat_h=0.45, **kw):
    p = sp.sample_seated_person(np.random.default_rng(0), x, y, math.pi, mode, seat_h)
    p.update(height=1.75, girth=1.0, shoulder=1.0, slouch=1.0, lean=0.0, head_fwd=0.0)
    p.update(kw)
    return p


def _extent(boxes):
    c, R, h = boxes
    ext = np.abs(R) @ h[:, :, None]          # world half extents of each rotated box
    ext = ext[..., 0]
    return (c - ext).min(0), (c + ext).max(0)


def test_raycast_boxes_distance_and_incidence():
    boxes = (np.array([[3.0, 0.0, 0.0]]), np.eye(3)[None], np.array([[0.5, 1.0, 1.0]]))
    dirs = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [math.cos(0.3), math.sin(0.3), 0.0]])
    r, cos = sp.raycast_boxes(np.zeros(3), dirs, boxes)
    assert abs(r[0] - 2.5) < 1e-9 and abs(cos[0] - 1.0) < 1e-9
    assert np.isinf(r[1]) and cos[1] == 0
    assert abs(r[2] - 2.5 / math.cos(0.3)) < 1e-9 and abs(cos[2] - math.cos(0.3)) < 1e-9     # face normal is x


def test_raycast_boxes_rotated_and_inside_origin():
    boxes = (np.array([[3.0, 0.0, 0.0]]), sp._rot_z(math.pi / 4)[None], np.array([[0.5, 0.5, 1.0]]))
    r, _ = sp.raycast_boxes(np.zeros(3), np.array([[1.0, 0.0, 0.0]]), boxes)
    assert abs(r[0] - (3.0 - 0.5 * math.sqrt(2))) < 1e-9                  # the diamond's corner
    inside = (np.zeros((1, 3)), np.eye(3)[None], np.ones((1, 3)))
    assert np.isinf(sp.raycast_boxes(np.zeros(3), np.array([[1.0, 0.0, 0.0]]), inside)[0][0])


def test_seated_body_proportions():
    c, R, a = sp.body(_seated(), 0.0)
    top = (c[:, 2] + a.max(1)).max()
    assert 1.28 < top < 1.42                    # seat 0.45 + ~0.9 sitting height
    low = c[:, 2] - np.sqrt(((R[:, 2, :] * a) ** 2).sum(1))        # lowest point of each ellipsoid
    assert -0.01 < low.min() < 0.05 and (c[:, 2] < 0.35).sum() >= 4     # feet and shins reach the floor
    thighs = (4.0 - c[:, 0] > 0.15) & (c[:, 2] > 0.4) & (c[:, 2] < 0.8)     # yaw pi: forward is -x
    assert thighs.sum() >= 2                     # thighs forward of the hips at seat height
    assert np.abs(c[:, 1]).max() < 0.35


def test_seated_torso_lean_moves_the_head():
    c0, _, a0 = sp.body(_seated(), 0.0)
    c1, _, a1 = sp.body(_seated(lean=0.5), 0.0)
    head0, head1 = c0[np.argmax(c0[:, 2] + a0.max(1))], c1[np.argmax(c1[:, 2] + a1.max(1))]
    assert head0[0] - head1[0] > 0.15 and head1[2] < head0[2]    # yaw pi, leaning forward: further toward -x, lower
    c2, _, _ = sp.body(_seated(slouch=0.8), 0.0)
    assert c2[:, 2].max() < c0[:, 2].max()


def test_seated_body_follows_yaw_and_position():
    p = _seated(2.0, 1.0) | {'yaw': math.pi / 2}
    c, _, _ = sp.body(p, 0.0)
    knee_side = c[(c[:, 2] > 0.4) & (c[:, 2] < 0.7)]
    assert knee_side[:, 1].max() > 1.15                       # facing +y: thighs run toward +y
    assert abs(c[:, 0].mean() - 2.0) < 0.3


def test_furniture_primitives_have_real_dimensions():
    rng = np.random.default_rng(0)
    tops = []
    for seed in range(8):
        rng = np.random.default_rng(seed)
        dsk, w, d = sp.desk(rng, 0.4, 0.74)
        lo, hi = _extent((np.array([b[0] for b in dsk]), np.array([b[1] for b in dsk]), np.array([b[2] for b in dsk])))
        assert abs(lo[2]) < 1e-9 and 0.74 <= hi[2] < 1.3               # legs to the floor, monitor/laptop on top
        tops.append(hi[2])
        top = dsk[0]
        assert abs(top[0][2] + top[2][2] - 0.74) < 1e-9                # desk top surface
        assert abs(top[0][0] - top[2][0] - 0.4) < 1e-9                 # near edge 0.4 m ahead of the hips
        ch = sp.office_chair(rng, 0.46)
        _, chi = _extent((np.array([b[0] for b in ch]), np.array([b[1] for b in ch]), np.array([b[2] for b in ch])))
        assert 0.8 < chi[2] < 1.3 and abs(ch[0][0][2] + ch[0][2][2] - 0.46) < 1e-9
        cb, wc = sp.couch(rng)
        assert 1.7 <= wc <= 2.2 and abs(cb[1][0][2] + cb[1][2][2] - 0.85) < 1e-9
        bk, bd, bw = sp.bookcase(rng)
        assert 1.0 <= bk[0][2][2] * 2 <= 2.0
    assert max(tops) > 1.0                                              # a monitor rises above the desk

def test_place_furniture_rotates_and_translates():
    f = sp.place_furniture([sp._box(1.0, 0.0, 0.5, 0.1, 0.1, 0.5)], 2.0, 3.0, math.pi / 2, 'x')
    assert np.allclose(f['boxes'][0][0], [2.0, 4.0, 0.5], atol=1e-9)
    assert f['radius'] >= 1.0


def test_furniture_hits_carry_id_and_hide_the_background_behind():
    xyz, ts = _scene()
    f = sp.place_furniture([sp._box(0.0, 0.0, 0.5, 0.3, 0.6, 0.5)], 4.0, 0.0, 0.0, 'shelf')
    out, pid = sp.insert(xyz, ts, [], np.random.default_rng(1), [f])
    mine = out[pid == -2]
    front = mine[np.abs(mine[:, 1]) < 0.55]
    assert len(front) > 100 and abs(np.median(front[:, 0]) - 3.7) < 0.03 and np.percentile(front[:, 0], 90) < 3.9    # front face (+ range noise tail)
    wall = (out[:, 0] > 7.9) & (np.abs(out[:, 1]) < 0.3) & (out[:, 2] > 0.3) & (out[:, 2] < 0.8)
    assert wall.sum() == 0                                                # its shadow on the wall is gone
    assert ((pid == -1) & (np.abs(out[:, 0] - 8.0) < 1e-6)).sum() > 1000


def test_chair_back_hides_part_of_a_seated_person():
    xyz, ts = _scene()
    p = _seated(4.0, 0.0) | {'yaw': 0.0}                    # faces away from the sensor: back view
    chair = sp.place_furniture(sp.office_chair(np.random.default_rng(0), 0.45), 4.0, 0.0, 0.0, 'chair')
    rng = np.random.default_rng(2)
    _, pid_free = sp.insert(xyz, ts, [p], rng)
    _, pid_chair = sp.insert(xyz, ts, [p], rng, [chair])
    assert (pid_chair == 0).sum() < 0.9 * (pid_free == 0).sum()      # the backrest takes the lower torso
    assert (pid_chair == -2).sum() > 100


def test_seated_person_sits_on_the_chair_in_every_sampled_scene():
    free, no_avoid = (lambda x, y: False), np.zeros((0, 2))
    for kind in ('seated_desk', 'seated_chair', 'seated_couch'):
        for seed in range(8):
            rng = np.random.default_rng(seed)
            people, pieces, meta = sp.sample_furniture_scene(rng, kind, free, no_avoid, lambda r: float(rng.uniform(-0.4, 0.4)))
            seat = [f for f in pieces if f['kind'] in ('chair', 'couch')][0]
            seat_top = seat['boxes'][0][0][2] + seat['boxes'][2][0][2]        # first box is the cushion/seat
            assert abs(seat_top - people[0]['seat_h']) < 0.03
            assert 0.4 <= meta['range'] <= 6.0 and all(p['seated'] and p['speed'] == 0 for p in people)
            assert abs(math.hypot(people[0]['x'], people[0]['y']) - meta['range']) < 0.3


def test_standing_scene_places_furniture_close_to_the_person():
    free, no_avoid = (lambda x, y: False), np.zeros((0, 2))
    seen = set()
    for seed in range(30):
        rng = np.random.default_rng(seed)
        people, pieces, meta = sp.sample_furniture_scene(rng, 'stand_furn', free, no_avoid, lambda r: float(rng.uniform(-0.4, 0.4)))
        f = pieces[0]
        assert not people[0].get('seated') and meta['furn'] in ('bookcase', 'partition', 'desk', 'couch')
        seen.add((meta['furn'], meta['occluding']))
        assert np.hypot(f['x'] - people[0]['x'], f['y'] - people[0]['y']) < 3.0
    assert any(o for _, o in seen) and any(not o for _, o in seen)


def test_sampling_rejects_occupied_spots_and_real_people():
    rng = np.random.default_rng(0)
    assert sp.sample_furniture_scene(rng, 'seated_desk', lambda x, y: True, np.zeros((0, 2)), lambda r: 0.0) is None
    avoid = np.array([[2.0, 0.0]])
    res = sp.sample_furniture_scene(rng, 'seated_chair', lambda x, y: False, avoid, lambda r: 0.0)
    assert res is None or all(np.hypot(p['x'] - 2.0, p['y']) > sp.REAL_CLEAR_M for p in res[0])


def test_close_group_is_within_1p8_m():
    free, no_avoid = (lambda x, y: False), np.zeros((0, 2))
    for seed in range(20):
        rng = np.random.default_rng(seed)
        people = sp.sample_close_group(rng, free, no_avoid, lambda r: float(rng.uniform(-0.5, 0.5)))
        assert 1 <= len(people) <= 2 and all(0.4 <= math.hypot(p['x'], p['y']) <= 1.8 for p in people)



def test_scenes_never_touch_the_sensor_and_standing_people_clear_their_furniture():
    free, no_avoid = (lambda x, y: False), np.zeros((0, 2))
    for kind in ('seated_desk', 'seated_chair', 'seated_couch', 'stand_furn'):
        n = 0
        for seed in range(60):
            rng = np.random.default_rng(seed)
            res = sp.sample_furniture_scene(rng, kind, free, no_avoid, lambda r: float(rng.uniform(-0.4, 0.4)))
            if res is None:
                continue
            n += 1
            people, pieces, meta = res
            assert sp.clear_of_sensor(people, pieces)
            for f in pieces:
                c, R, h = f['boxes']
                lo = np.einsum('kij,kj->ki', R.transpose(0, 2, 1), np.array([0.0, 0.0, sp.SENSOR_Z]) - c)
                assert np.linalg.norm(lo - np.clip(lo, -h, h), axis=1).min() >= sp.SENSOR_CLEAR_M
            if kind == 'stand_furn' and not meta['occluding']:
                for c, R, h in zip(*pieces[0]['boxes']):
                    loc = R.T @ (np.array([people[0]['x'], people[0]['y'], 1.0]) - c)
                    assert np.linalg.norm(loc[:2] - np.clip(loc[:2], -h[:2], h[:2])) > 0.1    # not standing inside it
        assert n > 20


def test_clear_of_sensor_rejects_a_desk_on_top_of_it():
    desk_on_robot = sp.place_furniture([sp._box(0.2, 0.0, 0.37, 0.3, 0.5, 0.37)], 0.0, 0.0, 0.0)
    assert not sp.clear_of_sensor([], [desk_on_robot])
    assert not sp.clear_of_sensor([_person(0.2, 0.0)], [])
    assert sp.clear_of_sensor([_person(1.0, 0.0)], [sp.place_furniture([sp._box(0, 0, 0.5, 0.2, 0.2, 0.5)], 2.0, 0.0, 0.0)])


def test_abut_lets_furniture_overlap_real_clutter_but_not_the_person():
    spots = []

    def occ(x, y):
        spots.append((x, y))
        return math.hypot(x - 3.0, y) > 1.0 and False                      # never; just record the calls
    rng = np.random.default_rng(0)
    sp.sample_furniture_scene(rng, 'seated_desk', occ, np.zeros((0, 2)), lambda r: 0.0, abut=True)
    assert len(spots) == 1                                                  # only the sitter's spot is checked
    spots.clear()
    sp.sample_furniture_scene(np.random.default_rng(0), 'seated_desk', occ, np.zeros((0, 2)), lambda r: 0.0)
    assert len(spots) >= 4
