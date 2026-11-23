"""person_candidates.ray_candidates: bearing-only side-camera rays searched along their whole length."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup import person_candidates as pc  # noqa: E402

CAM_Z = 0.21     # side cameras: ~7 cm below the Livox window


def _person(rng, x, y, top=1.75, n=900, r=0.2):
    a = rng.uniform(0, 2 * np.pi, n)
    rr = r * np.sqrt(rng.uniform(0, 1, n))
    return np.c_[x + 0.6 * rr * np.cos(a), y + rr * np.sin(a), rng.uniform(0.1, top, n)]


def _wall_across(rng, x, half=2.0, n=6000):
    """A wall perpendicular to +x at x, |y| < half, up to 2.0 m."""
    return np.c_[rng.uniform(x - 0.03, x + 0.03, n), rng.uniform(-half, half, n), rng.uniform(0.15, 2.0, n)]


def _box(rng, x0, x1, y0, y1, z0, z1, n):
    return np.c_[rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), rng.uniform(z0, z1, n)]


def _xyz(*parts):
    p = np.vstack(parts)
    return p[:, 0], p[:, 1], p[:, 2]


def _ray(ox=0.0, oy=0.0, bearing=0.0, head=None, at=None, lo=pc.RAY_RANGE_M[0], hi=pc.RAY_RANGE_M[1]):
    """A ray from (ox, oy, CAM_Z) at `bearing` (rad).  With head/at, top_tan is the box
    top of a head `head` m high at horizontal range `at`; else NaN (box cut by the image top)."""
    top = (head - CAM_Z) / at if head is not None else float('nan')
    return [ox, oy, CAM_Z, math.cos(bearing), math.sin(bearing), lo, hi, top]


def _hit(rows, x, y, tol=0.2):
    return len(rows) > 0 and np.hypot(rows[:, 0] - x, rows[:, 1] - y).min() < tol


def test_person_in_front_of_wall_is_found_not_the_wall():
    rng = np.random.default_rng(0)
    x, y, z = _xyz(_person(rng, 3.0, 0.0), _wall_across(rng, 4.5))
    rows, status = pc.ray_candidates(x, y, z, [_ray(head=1.75, at=3.0)], return_status=True)
    assert status == ['ok'] and len(rows) == 1
    assert _hit(rows, 3.0, 0.0)
    assert rows[0, pc.RAY_OUT_FIELDS.index('ray')] == 0
    assert (rows[0, 9], rows[0, 10]) == (rows[0, 0], rows[0, 1])      # anchor = body


def test_empty_ray_gives_nothing():
    rng = np.random.default_rng(1)
    # a person off to the side of the ray, nothing on it
    x, y, z = _xyz(_person(rng, 0.0, 3.0))
    rows, status = pc.ray_candidates(x, y, z, [_ray()], return_status=True)
    assert len(rows) == 0 and status == ['none']
    rows, status = pc.ray_candidates(np.zeros(0), np.zeros(0), np.zeros(0), [_ray()], return_status=True)
    assert len(rows) == 0 and status == ['none']


def test_two_people_on_the_ray_takes_the_nearest():
    rng = np.random.default_rng(2)
    x, y, z = _xyz(_person(rng, 2.0, 0.0), _person(rng, 4.0, 0.05))
    rows = pc.ray_candidates(x, y, z, [_ray()])
    assert len(rows) == 1 and _hit(rows, 2.0, 0.0)


def test_only_a_wall_on_the_ray_gives_nothing():
    """A detection whose ray ends on a bare wall must not turn the wall into a person."""
    rng = np.random.default_rng(3)
    x, y, z = _xyz(_wall_across(rng, 3.0))
    assert len(pc.ray_candidates(x, y, z, [_ray()])) == 0
    assert len(pc.ray_candidates(x, y, z, [_ray(head=1.75, at=3.0)])) == 0


def test_wall_along_the_ray_gives_nothing():
    rng = np.random.default_rng(4)
    x, y, z = _xyz(_box(rng, 0.8, 5.5, 0.05, 0.11, 0.15, 2.0, 6000))
    assert len(pc.ray_candidates(x, y, z, [_ray()])) == 0


def test_chair_back_in_front_is_skipped_with_the_box_top():
    """A 1.05 m chair back at 2.5 m on the ray; the person behind it at 3 m.  The box-top
    line (camera to head) passes ~1.5 m high over the chair, so the camera sees past it."""
    rng = np.random.default_rng(5)
    chair = _box(rng, 2.45, 2.55, -0.2, 0.2, 0.45, 1.05, 700)
    x, y, z = _xyz(chair, _person(rng, 3.0, 0.0))
    assert _hit(pc.ray_candidates(x, y, z, [_ray(head=1.75, at=3.0)]), 3.0, 0.0)
    # without the box top (head cut off by the image edge) the nearest object wins
    assert _hit(pc.ray_candidates(x, y, z, [_ray()]), 2.5, 0.0)


def test_furniture_and_shelf_tops_are_not_bodies():
    """Person-sized, isolated, but topping out at 0.95 m (a cabinet) or at the 2.0 m cut
    (a narrow shelf): neither is taken; a person behind the cabinet is."""
    rng = np.random.default_rng(12)
    cab = _box(rng, 1.9, 2.1, -0.2, 0.2, 0.1, 0.95, 900)
    shelf = _box(rng, 2.9, 3.1, -0.2, 0.2, 0.1, 1.999, 1500)
    x, y, z = _xyz(cab, shelf)
    assert len(pc.ray_candidates(x, y, z, [_ray()])) == 0
    x, y, z = _xyz(cab, _person(rng, 3.0, 0.0))
    assert _hit(pc.ray_candidates(x, y, z, [_ray()]), 3.0, 0.0)


def test_close_person_cut_by_the_fov_top_is_found():
    """At 0.6 m the Livox sees only up to ~1.08 m of a standing person."""
    rng = np.random.default_rng(13)
    p = _person(rng, 0.6, 0.0, n=3000)
    p = p[p[:, 2] < pc.SENSOR_Z_M + np.hypot(p[:, 0], p[:, 1]) * pc.FOV_TOP_TAN]
    x, y, z = _xyz(p)
    assert _hit(pc.ray_candidates(x, y, z, [_ray()]), 0.6, 0.0)


def test_range_window_narrows_the_search():
    rng = np.random.default_rng(6)
    x, y, z = _xyz(_person(rng, 2.0, 0.0), _person(rng, 4.0, 0.05))
    assert _hit(pc.ray_candidates(x, y, z, [_ray(lo=3.0, hi=5.0)]), 4.0, 0.05)


def test_seated_person_at_desk_on_a_side_ray():
    """The seeded test scene, searched along the whole ray from the side: the nearest
    upper body is the person, desk and partition behind are not taken."""
    from test_person_candidates_seeded import _desk_scene
    x, y, z = _desk_scene(np.random.default_rng(7))
    rows = pc.ray_candidates(x, y, z, [_ray(head=1.07, at=2.4)])
    assert len(rows) == 1 and _hit(rows, 2.4, 0.0, 0.15)
    assert rows[0, 7] < 1.25
    xe, ye, ze = _desk_scene(np.random.default_rng(8), person=False)
    assert len(pc.ray_candidates(xe, ye, ze, [_ray(head=1.07, at=2.4)])) == 0


def test_ray_from_an_offset_origin_at_an_angle():
    rng = np.random.default_rng(9)
    # robot at (1, -2) looking along -y (a right-side camera of a robot facing +x would)
    x, y, z = _xyz(_person(rng, 1.3, -5.0))
    b = math.atan2(-5.0 + 2.0, 1.3 - 1.0)
    rows = pc.ray_candidates(x, y, z, [_ray(1.0, -2.0, b, head=1.75, at=3.0)])
    assert _hit(rows, 1.3, -5.0)


def test_existing_candidate_takes_the_anchor():
    rng = np.random.default_rng(10)
    x, y, z = _xyz(_person(rng, 3.0, 0.0))
    existing = np.array([[3.1, 0.05, 0, 0, 0, 0, 0, 0, 0]], dtype=np.float32)
    rows, status = pc.ray_candidates(x, y, z, [_ray()], existing=existing, return_status=True)
    assert status == ['dup'] and len(rows) == 1
    assert np.allclose(rows[0, 9:11], [3.1, 0.05])


def test_several_rays_keep_their_index():
    rng = np.random.default_rng(11)
    x, y, z = _xyz(_person(rng, 3.0, 0.0), _person(rng, 0.0, 2.5))
    rows, status = pc.ray_candidates(x, y, z, [_ray(bearing=math.pi), _ray(), _ray(bearing=math.pi / 2)],
                                     return_status=True)
    assert status == ['none', 'ok', 'ok']
    assert list(rows[:, -1]) == [1, 2]
