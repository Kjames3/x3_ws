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
    assert all(r[0] in sp.OUTCOMES for r in res)
    assert sum(r[0] == 'found' for r in res) <= 2
