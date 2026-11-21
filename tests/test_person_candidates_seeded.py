"""person_candidates.seeded_candidates / extract(seeds=...): camera-seeded seated people."""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup import person_candidates as pc  # noqa: E402


def _desk_scene(rng, person=True):
    """Robot at the origin looking along +x.  A seated person (0.3-1.07 m, head under SPLIT_Z_M, ~0.4 m wide) at
    x = 2.4, a desk (top 0.75 m) from x = 2.6 to 3.2 and a tall partition at x = 3.3, all
    touching, so extract merges them into one component that is too wide."""
    parts = []
    # desk: top surface plus legs, 2.6-3.2 m deep, 1.6 m wide
    n = 2500
    parts.append(np.c_[rng.uniform(2.6, 3.2, n), rng.uniform(-0.8, 0.8, n), rng.uniform(0.70, 0.77, n)])
    # partition / wall behind the desk, up to 2.0 m
    n = 4000
    parts.append(np.c_[rng.uniform(3.28, 3.34, n), rng.uniform(-1.2, 1.2, n), rng.uniform(0.15, 2.0, n)])
    # chair seat under the person
    n = 300
    parts.append(np.c_[rng.uniform(2.2, 2.6, n), rng.uniform(-0.25, 0.25, n), rng.uniform(0.40, 0.50, n)])
    if person:
        n = 700
        a = rng.uniform(0, 2 * np.pi, n)
        r = 0.18 * np.sqrt(rng.uniform(0, 1, n))
        parts.append(np.c_[2.4 + 0.6 * r * np.cos(a), 0.02 + r * np.sin(a), rng.uniform(0.3, 1.07, n)])
    p = np.vstack(parts)
    return p[:, 0], p[:, 1], p[:, 2]


def _near(c, x, y, tol=0.3):
    return len(c) > 0 and np.hypot(c[:, 0] - x, c[:, 1] - y).min() < tol


def test_seated_person_at_desk_missed_by_extract():
    x, y, z = _desk_scene(np.random.default_rng(0))
    assert not _near(pc.extract(x, y, z), 2.4, 0.0)


def test_seed_finds_person_even_when_camera_range_overreads():
    # the camera's range lands on the partition (3.3 m), its bearing is right
    x, y, z = _desk_scene(np.random.default_rng(1))
    rows, idx, status = pc.seeded_candidates(x, y, z, [(3.3, 0.0)], return_points=True)
    assert status == [('ok', 0)]
    assert _near(rows, 2.4, 0.0, 0.15)
    # trimmed: no partition points (x > 3.25) and the body is person-sized
    assert x[idx[0]].max() < 3.0
    assert rows[0, 7] > 1.0 and rows[0, 7] < 1.25
    assert max(rows[0, 3] - rows[0, 2], rows[0, 5] - rows[0, 4]) <= pc.MAX_SPAN_M


def test_seed_on_bare_wall_gives_wall_slab_not_room():
    x, y, z = _desk_scene(np.random.default_rng(2), person=False)
    rows = pc.seeded_candidates(x, y, z, [(3.3, 0.0)])
    # no person: whatever is found must be a small slab, never the merged component
    for r in rows:
        assert max(r[3] - r[2], r[5] - r[4]) <= 2 * pc.SEED_CORE_M + 1e-6


def test_no_points_no_candidate():
    rows, idx, status = pc.seeded_candidates(np.zeros(0), np.zeros(0), np.zeros(0), [(2.0, 0.0)],
                                             return_points=True)
    assert len(rows) == 0 and status == [('none', -1)]


def test_dedup_against_existing():
    x, y, z = _desk_scene(np.random.default_rng(3))
    existing = np.array([[2.45, 0.05, 0, 0, 0, 0, 0, 0, 0]], dtype=np.float32)
    rows, _, status = pc.seeded_candidates(x, y, z, [(3.3, 0.0)], existing=existing, return_points=True)
    assert len(rows) == 0 and status == [('dup', 0)]


def test_two_seeds_one_body_one_candidate():
    x, y, z = _desk_scene(np.random.default_rng(4))
    rows, _, status = pc.seeded_candidates(x, y, z, [(3.3, 0.0), (2.5, 0.05)], return_points=True)
    assert len(rows) == 1 and [s for s, _ in status] == ['ok', 'ok']


def test_extract_seeds_opt_in_and_appends():
    x, y, z = _desk_scene(np.random.default_rng(5))
    base = pc.extract(x, y, z)
    assert np.array_equal(pc.extract(x, y, z, seeds=None), base)
    assert np.array_equal(pc.extract(x, y, z, seeds=np.zeros((0, 2))), base)
    seeded = pc.extract(x, y, z, seeds=[(3.3, 0.0)])
    assert len(seeded) == len(base) + 1
    assert np.array_equal(seeded[:len(base)], base)
    assert _near(seeded[len(base):], 2.4, 0.0, 0.15)
    assert seeded.shape[1] == len(pc.FIELDS)


def test_standing_person_in_the_open():
    rng = np.random.default_rng(6)
    n = 400
    a = rng.uniform(0, 2 * np.pi, n)
    r = 0.18 * np.sqrt(rng.uniform(0, 1, n))
    x, y, z = 2.0 + 0.6 * r * np.cos(a), 1.0 + r * np.sin(a), rng.uniform(0.15, 1.75, n)
    rows = pc.seeded_candidates(x, y, z, [(2.05, 1.02)])
    assert _near(rows, 2.0, 1.0, 0.1)
