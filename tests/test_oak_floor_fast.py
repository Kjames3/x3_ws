"""X3_OAK_FLOOR_FAST=1 path of oak_floor_obstacles must match the old path."""
import math

import numpy as np
import pytest

import oak_floor_obstacles as ofo
from oak_floor_bench import make_scene, make_stepper


def _floor_cloud(seed, n=10000, a=0.0, b=0.0, c=0.0, noise=0.003, n_obst=600,
                 n_out=300, x_off=0.0):
    """Base-frame cloud: tilted floor + raised blobs + gross outliers."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.4, 1.6, n) + x_off
    y = rng.uniform(-0.8, 0.8, n)
    z = a * x + b * y + c + rng.normal(0, noise, n)
    pts = [np.column_stack((x, y, z))]
    for _ in range(3):   # obstacles 3-10 cm tall
        cx, cy, hh = rng.uniform(0.5, 1.5), rng.uniform(-0.6, 0.6), rng.uniform(0.03, 0.10)
        k = n_obst // 3
        ox, oy = cx + rng.uniform(0, 0.1, k), cy + rng.uniform(0, 0.1, k)
        pts.append(np.column_stack((ox, oy, a * ox + b * oy + c + rng.uniform(0, hh, k))))
    pts.append(np.column_stack((rng.uniform(0.3, 1.6, n_out), rng.uniform(-1, 1, n_out),
                                rng.uniform(-0.3, 0.5, n_out))))
    p = np.concatenate(pts)
    return p[rng.permutation(len(p))]


def _assert_coef_close(old, new, allow_none=False):
    # rtol 1e-6 per coefficient; atol 1e-9 m(/m) for a, b that are ~0 on a level floor.
    if allow_none and old is None:
        assert new is None
        return
    assert old is not None and new is not None
    np.testing.assert_allclose(new, old, rtol=1e-6, atol=1e-9)


PLANES = [(0, 0.0, 0.0, 0.0), (1, 0.03, -0.02, 0.04), (2, -0.04, 0.03, -0.02),
          (3, 0.0, 0.04, 0.06), (4, 0.01, 0.0, -0.09)]


@pytest.mark.parametrize('seed,a,b,c', PLANES)
def test_plane_fit_matches_lstsq(seed, a, b, c):
    p = _floor_cloud(seed, a=a, b=b, c=c)
    old = ofo.fit_floor_plane(p, fast=False)
    new = ofo.fit_floor_plane(p, fast=True)
    _assert_coef_close(old, new)
    assert abs(old[0] - a) < 2e-3 and abs(old[1] - b) < 2e-3 and abs(old[2] - c) < 3e-3


@pytest.mark.parametrize('seed', range(5, 25))
def test_plane_fit_random_clouds(seed):
    rng = np.random.default_rng(seed)
    a, b = rng.uniform(-0.025, 0.025, 2)
    c = rng.uniform(-0.04, 0.04)
    p = _floor_cloud(seed, n=int(rng.integers(800, 15000)), a=a, b=b, c=c,
                     noise=rng.uniform(0.001, 0.008), x_off=rng.uniform(-0.3, 0.5))
    p = p.astype(np.float32) if seed % 2 else p      # float32 like real pts
    old = ofo.fit_floor_plane(p, False)
    assert old is not None
    _assert_coef_close(old, ofo.fit_floor_plane(p, True))


def test_plane_fit_none_branches_agree():
    rng = np.random.default_rng(7)
    few = _floor_cloud(0, n=250, n_obst=0, n_out=0)              # < 300 candidates
    x, y = rng.uniform(0.4, 0.6, 2000), rng.uniform(-0.3, 0.3, 2000)
    tilted = np.column_stack((x, y, 0.18 * (x - 0.5) + rng.normal(0, 0.002, 2000)))  # > ~8 deg
    high = _floor_cloud(2, c=0.0)
    high[:, 2] += 0.105                                            # |c| > 0.10, still < 0.12
    # >= 300 candidates but the trim keeps < 200: 160 floor points + 150 at +-0.11 m.
    f = np.column_stack((rng.uniform(0.4, 1.6, 160), rng.uniform(-1, 1, 160),
                         rng.normal(0, 0.001, 160)))
    o = np.column_stack((rng.uniform(0.4, 1.6, 150), rng.uniform(-1, 1, 150),
                         rng.choice([-1, 1], 150) * rng.uniform(0.1, 0.119, 150)))
    scatter = np.concatenate((f, o))
    for p in (few, tilted, high, scatter):
        assert ofo.fit_floor_plane(p, False) is None
        assert ofo.fit_floor_plane(p, True) is None


def test_plane_fit_degenerate_line_falls_back():
    rng = np.random.default_rng(3)
    x = rng.uniform(0.4, 1.6, 2000)
    p = np.column_stack((x, np.full_like(x, 0.1), 0.01 * x + rng.normal(0, 0.002, 2000)))
    _assert_coef_close(ofo.fit_floor_plane(p, False), ofo.fit_floor_plane(p, True),
                       allow_none=True)


@pytest.mark.parametrize('fast', [False, True])
def test_rigid_transform(fast):
    rng = np.random.default_rng(0)
    p = rng.normal(size=(5000, 3)).astype(np.float32)
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    t = rng.normal(size=3)
    ref = p.astype(np.float64) @ q.T + t
    out = ofo.rigid(p, q, t, fast)
    assert out.dtype == np.float64
    np.testing.assert_allclose(out, ref, rtol=0, atol=1e-12)


@pytest.mark.parametrize('seed', range(6))
def test_min_returns_identical(seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(1, 6000))
    off = rng.uniform(-500, 500, 3)                 # odom far from the origin, negatives
    p = off + rng.uniform(-1.5, 1.5, (n, 3)) * (1, 1, 0.1)
    p = np.concatenate((p, p[rng.choice(n, n // 3)]))   # guaranteed doubles
    old = ofo.min_returns_per_voxel(p, 0.04, fast=False)
    new = ofo.min_returns_per_voxel(p, 0.04, fast=True)
    assert np.array_equal(old, new)                 # same rows, same order


def test_min_returns_empty():
    e = np.zeros((0, 3))
    assert len(ofo.min_returns_per_voxel(e, 0.04, True)) == 0


def _point_set(points):
    return sorted(map(tuple, np.asarray(points, np.float32).tolist()))


def _visible_fn(rng_seed):
    def vis(xyz):        # deterministic in the voxel position, not call order
        h = np.floor(xyz * 97.0).astype(np.int64).sum(1)
        return (h + rng_seed) % 5 == 0
    return vis


@pytest.mark.parametrize('seed', range(4))
def test_memory_sequence_identical(seed):
    """Multi-frame: duplicates (last wins), age, radius, visibility, empty frames."""
    rng = np.random.default_rng(seed)
    old, new = ofo.FloorObstacleMemory(memory_s=3.0), ofo.FastFloorObstacleMemory(memory_s=3.0)
    robot = np.array([0.0, 0.0])
    for frame in range(40):
        now = 0.5 * frame
        robot = robot + rng.normal(0, 0.3, 2)
        if frame % 7 == 3:
            pts = np.zeros((0, 3))
        else:
            k = int(rng.integers(1, 400))
            pts = np.column_stack((robot + rng.uniform(-3, 3, (k, 2)),
                                   rng.uniform(0.03, 0.4, k)))
            pts = np.concatenate((pts, pts[: k // 4] + rng.normal(0, 0.005, (k // 4, 3))))
        vis = _visible_fn(seed) if frame % 3 else None
        old.update(pts, now, tuple(robot), vis)
        new.update(pts, now, tuple(robot), vis)
        assert len(old) == len(new)
        assert _point_set(old.points()) == _point_set(new.points())
    assert len(old) > 0


def test_memory_last_point_wins_and_values():
    pts = np.array([[0.101, 0.2, 0.05], [0.102, 0.21, 0.06], [0.5, 0.5, 0.1]])
    old, new = ofo.FloorObstacleMemory(), ofo.FastFloorObstacleMemory()
    old.update(pts, 1.0, (0, 0))
    new.update(pts, 1.0, (0, 0))
    assert _point_set(old.points()) == _point_set(new.points())
    assert [0.102, 0.21, 0.06] in new.points().astype(np.float64).round(6).tolist()


def test_memory_out_of_range_falls_back():
    new, old = ofo.FastFloorObstacleMemory(), ofo.FloorObstacleMemory()
    a = np.array([[1.0, 2.0, 0.1]])
    b = np.array([[1e6, 0.0, 0.1]])           # 25 M cells: outside the 21-bit field
    for p, t in ((a, 0.0), (b, 1.0)):
        new.update(p, t, (0, 0))
        old.update(p, t, (0, 0))
        assert _point_set(old.points()) == _point_set(new.points())
    assert new._slow is not None


SCENES = [dict(seed=0), dict(seed=1, floor=(0.0, 0.0, 0.0), robot=(0, 0, 0)),
          dict(seed=2, floor=(-0.03, 0.02, 0.06), pitch_deg=24, robot=(-12.4, 30.1, -2.5)),
          dict(seed=3, outlier_frac=0.08, hole_frac=0.2)]


@pytest.mark.parametrize('kw', SCENES)
def test_step_end_to_end_identical(kw):
    """Whole step() on rendered depth, several frames with the robot moving."""
    runs = {}
    for fast in (False, True):
        scene = make_scene(**kw)
        st = make_stepper(scene, fast)
        out = []
        x0, y0, yaw0 = kw.get('robot', (1.3, -0.4, 0.6))
        for i in range(5):
            frame = make_scene(**{**kw, 'seed': kw['seed'] * 10 + i,
                                  'robot': (x0 + 0.05 * i, y0, yaw0 + 0.02 * i)})
            st.oak.scene = frame
            st.step()
            out.append((st.stats.get('floor'), _point_set(st.published)))
        runs[fast] = out
    assert any(len(pts) for _, pts in runs[False])
    for (c_old, p_old), (c_new, p_new) in zip(runs[False], runs[True]):
        _assert_coef_close(c_old, c_new)
        assert p_old == p_new


def test_env_flag_selects_path(monkeypatch):
    """X3_OAK_FLOOR_FAST is read when OakFloorObstacles is built; default is the old path."""
    import sys
    import types
    msg = types.ModuleType('sensor_msgs.msg')
    msg.PointCloud2 = msg.PointField = object
    pkg = types.ModuleType('sensor_msgs')
    pkg.msg = msg
    monkeypatch.setitem(sys.modules, 'sensor_msgs', pkg)
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', msg)

    class _Node:
        def create_publisher(self, *a, **k):
            return None

    class _Bridge:
        _node = _Node()
    for val, fast in ((None, False), ('0', False), ('1', True)):
        if val is None:
            monkeypatch.delenv('X3_OAK_FLOOR_FAST', raising=False)
        else:
            monkeypatch.setenv('X3_OAK_FLOOR_FAST', val)
        o = ofo.OakFloorObstacles(None, _Bridge())
        assert o.fast is fast
        assert isinstance(o.mem, ofo.FastFloorObstacleMemory if fast else ofo.FloorObstacleMemory)


def test_height_uses_fitted_plane_both_paths():
    p = _floor_cloud(11, a=0.02, b=-0.01, c=0.03)
    hs = []
    for fast in (False, True):
        o = ofo.OakFloorObstacles.__new__(ofo.OakFloorObstacles)
        o.fast, o.stats = fast, {}
        hs.append(o.floor_relative_height(p))
    np.testing.assert_allclose(hs[1], hs[0], rtol=0, atol=1e-9)
    a, b, c = ofo.fit_floor_plane(p)
    np.testing.assert_allclose(
        hs[0], (p[:, 2] - (a * p[:, 0] + b * p[:, 1] + c)) / math.sqrt(1 + a * a + b * b))
