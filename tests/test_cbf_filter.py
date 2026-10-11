"""The CBF velocity filter: exact QP, moving obstacles, wheel limit, fallback."""
import itertools
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from cbf_filter import HolonomicCBFFilter, _project  # noqa: E402


def slsqp_reference(u_nom, obstacles, safe=0.3, gamma=1.0):
    """The filter as it was before: static obstacles, SLSQP, a 1.5 m/s box."""
    obs = np.asarray(obstacles, float).reshape(-1, 2)
    h = (obs ** 2).sum(1) - safe ** 2
    a = -2.0 * obs
    res = minimize(lambda u: 0.5 * np.sum((u - u_nom) ** 2), u_nom, jac=lambda u: u - u_nom,
                   method='SLSQP', bounds=[(-1.5, 1.5)] * 2,
                   constraints=[{'type': 'ineq', 'fun': lambda u: a @ u + gamma * h, 'jac': lambda u: a}])
    return res.x if res.success else None


def brute_force(u_nom, n, c):
    """Exact optimum by trying u_nom, every line and every pair of lines."""
    cands = [u_nom]
    for i in range(len(c)):
        cands.append(u_nom + (c[i] - n[i] @ u_nom) * n[i])
    for i, j in itertools.combinations(range(len(c)), 2):
        m = np.array([n[i], n[j]])
        if abs(np.linalg.det(m)) > 1e-9:
            cands.append(np.linalg.solve(m, [c[i], c[j]]))
    ok = [u for u in cands if np.all(n @ u - c >= -1e-7)]
    return min(ok, key=lambda u: np.sum((u - u_nom) ** 2)) if ok else None


def worst_margin(cbf, u, obstacles, velocities=None):
    n, c = cbf.constraint_rows((0.0, 0.0), obstacles, velocities)
    return float(np.min(n @ np.asarray(u) - c))


def test_no_obstacles_passes_the_command_through():
    assert HolonomicCBFFilter().filter_velocity(0.2, -0.1, (0.0, 0.0), []) == (0.2, -0.1)


@pytest.mark.parametrize('seed', range(40))
def test_project_matches_brute_force(seed):
    rng = np.random.default_rng(seed)
    k = int(rng.integers(1, 9))
    ang = rng.uniform(0, 2 * math.pi, k)
    n = np.c_[np.cos(ang), np.sin(ang)]
    c = rng.uniform(-0.6, 0.25, k)
    u_nom = rng.uniform(-0.5, 0.5, 2)
    want, got = brute_force(u_nom, n, c), _project(u_nom, n, c)
    if want is None:
        assert got is None
    else:
        assert got is not None and np.allclose(got, want, atol=1e-7)


@pytest.mark.parametrize('seed', range(30))
def test_static_scenes_match_the_old_slsqp_filter(seed):
    rng = np.random.default_rng(100 + seed)
    k = int(rng.integers(1, 400))
    r = rng.uniform(0.32, 1.0, k)
    th = rng.uniform(-math.pi, math.pi, k)
    obstacles = np.c_[r * np.cos(th), r * np.sin(th)]
    u_nom = rng.uniform(-0.3, 0.3, 2)
    ref = slsqp_reference(u_nom, obstacles)
    cbf = HolonomicCBFFilter(max_wheel_speed=10.0)      # old code had no such limit
    got = np.array(cbf.filter_velocity(u_nom[0], u_nom[1], (0.0, 0.0), obstacles))
    assert cbf.last_relaxation == 0.0
    assert worst_margin(cbf, got, obstacles) >= -1e-8
    if ref is not None:
        assert np.allclose(got, ref, atol=2e-4)


def test_approaching_obstacle_makes_a_parked_robot_retreat():
    cbf = HolonomicCBFFilter()
    box, vel = [(0.5, 0.0)], [(-0.3, 0.0)]
    assert cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), box) == (0.0, 0.0)   # static: fine
    vx, vy = cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), box, vel)
    assert vx < -0.1 and abs(vy) < 1e-9
    assert worst_margin(cbf, (vx, vy), box, vel) >= -1e-8
    assert cbf.last_relaxation == 0.0


def test_receding_obstacle_restricts_less_than_a_static_one():
    cbf = HolonomicCBFFilter()
    box = [(0.45, 0.0)]
    static_vx, _ = cbf.filter_velocity(0.4, 0.0, (0.0, 0.0), box)
    away_vx, _ = cbf.filter_velocity(0.4, 0.0, (0.0, 0.0), box, [(0.2, 0.0)])
    assert away_vx == pytest.approx(static_vx + 0.2, abs=1e-6)


def test_sideways_obstacle_motion_does_not_brake_a_forward_approach():
    cbf = HolonomicCBFFilter()
    box = [(0.6, 0.0)]
    assert cbf.filter_velocity(0.2, 0.0, (0.0, 0.0), box, [(0.0, 0.5)]) == \
        pytest.approx(cbf.filter_velocity(0.2, 0.0, (0.0, 0.0), box))


def test_wheel_limit_is_a_diamond_and_shrinks_with_yaw_rate():
    cbf = HolonomicCBFFilter(max_wheel_speed=0.5)
    far = [(0.9, 0.9)]
    vx, vy = cbf.filter_velocity(-0.6, -0.6, (0.0, 0.0), far)
    assert abs(vx) + abs(vy) == pytest.approx(0.5, abs=1e-7)
    vx, vy = cbf.filter_velocity(-0.6, -0.6, (0.0, 0.0), far, omega=1.0)
    assert abs(vx) + abs(vy) == pytest.approx(0.5 - 0.215, abs=1e-7)


def test_squeezed_between_two_obstacles_is_not_a_blind_stop():
    """Inside the safe distance on both sides: no command satisfies both."""
    cbf = HolonomicCBFFilter()
    walls = [(0.25, 0.0), (-0.20, 0.0)]
    vx, vy = cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), walls)
    assert cbf.last_relaxation > 0.0
    n, c = cbf.constraint_rows((0.0, 0.0), walls)
    margins = n @ np.array([vx, vy]) - c
    assert margins[0] == pytest.approx(margins[1], abs=1e-5)   # violations balanced
    assert vx > 0.0                                              # away from the nearer one
    assert margins.min() > (n @ np.zeros(2) - c).min()           # better than stopping


def test_obstacle_faster_than_the_base_gets_the_best_available_retreat():
    cbf = HolonomicCBFFilter(max_wheel_speed=0.5)
    person, vel = [(0.5, 0.0)], [(-1.4, 0.0)]
    vx, vy = cbf.filter_velocity(0.2, 0.0, (0.0, 0.0), person, vel)
    assert (vx, vy) == pytest.approx((-0.5, 0.0), abs=1e-5)
    assert cbf.last_relaxation > 0.5


@pytest.mark.parametrize('seed', range(30))
def test_fallback_never_does_worse_than_stopping(seed):
    rng = np.random.default_rng(500 + seed)
    k = int(rng.integers(2, 60))
    r = rng.uniform(0.13, 0.6, k)
    th = rng.uniform(-math.pi, math.pi, k)
    obstacles = np.c_[r * np.cos(th), r * np.sin(th)]
    vel = rng.uniform(-0.8, 0.8, (k, 2))
    cbf = HolonomicCBFFilter()
    u_nom = rng.uniform(-0.3, 0.3, 2)
    u = cbf.filter_velocity(u_nom[0], u_nom[1], (0.0, 0.0), obstacles, vel)
    assert abs(u[0]) + abs(u[1]) <= 0.5 + 1e-6
    assert worst_margin(cbf, u, obstacles, vel) >= -cbf.last_relaxation - 1e-5
    assert worst_margin(cbf, u, obstacles, vel) >= worst_margin(cbf, (0.0, 0.0), obstacles, vel) - 1e-6


@pytest.mark.parametrize('seed', range(20))
def test_fallback_reaches_the_true_least_violation(seed):
    """Against the LP over every row, with no pruning."""
    from scipy.optimize import linprog
    rng = np.random.default_rng(900 + seed)
    k = int(rng.integers(50, 800))
    r = rng.uniform(0.15, 1.0, k)
    th = rng.uniform(-math.pi, math.pi, k)
    obstacles = np.c_[r * np.cos(th), r * np.sin(th)]
    vel = rng.uniform(-0.5, 0.5, (k, 2)) * (rng.random((k, 1)) < 0.1)
    cbf = HolonomicCBFFilter()
    u = cbf.filter_velocity(0.1, 0.0, (0.0, 0.0), obstacles, vel)
    n, c = cbf.constraint_rows((0.0, 0.0), obstacles, vel)
    signs = np.array([[1, 1], [1, -1], [-1, 1], [-1, -1]], float)
    a = np.zeros((k + 4, 3))
    a[:k, :2], a[:k, 2], a[k:, :2] = -n, 1.0, signs
    ref = linprog([0, 0, -1], A_ub=a, b_ub=np.r_[-c, np.full(4, 0.5)],
                  bounds=[(None, None), (None, None), (None, 0.0)], method='highs')
    assert cbf.last_relaxation == pytest.approx(-ref.x[2], abs=1e-6)
    assert worst_margin(cbf, u, obstacles, vel) >= ref.x[2] - 1e-5


def test_obstacle_on_top_of_the_robot_is_skipped_not_fatal():
    cbf = HolonomicCBFFilter()
    vx, vy = cbf.filter_velocity(0.1, 0.0, (0.0, 0.0), [(0.0, 0.0), (0.0, 0.6)], [(0.0, 0.0), (0.0, 0.0)])
    assert math.isfinite(vx) and math.isfinite(vy)


def test_a_full_scan_solves_quickly():
    rng = np.random.default_rng(7)
    th = np.linspace(-math.pi, math.pi, 2100)
    r = 0.33 + 0.5 * rng.random(2100)
    obstacles = np.c_[r * np.cos(th), r * np.sin(th)]
    cbf = HolonomicCBFFilter()
    cbf.filter_velocity(0.3, 0.1, (0.0, 0.0), obstacles)
    t0 = time.perf_counter()
    for _ in range(20):
        cbf.filter_velocity(0.3, 0.1, (0.0, 0.0), obstacles)
    assert (time.perf_counter() - t0) / 20 < 0.01


def test_wheel_limit_applies_with_no_obstacles_too():
    cbf = HolonomicCBFFilter(max_wheel_speed=0.5)
    assert cbf.filter_velocity(0.45, 0.0, (0.0, 0.0), []) == (0.45, 0.0)
    vx, vy = cbf.filter_velocity(0.45, 0.45, (0.0, 0.0), [])
    assert (vx, vy) == pytest.approx((0.25, 0.25))
    vx, vy = cbf.filter_velocity(0.45, 0.0, (0.0, 0.0), [], omega=1.0)
    assert (vx, vy) == pytest.approx((0.285, 0.0))


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_non_finite_points_are_ignored_on_every_path(bad):
    cbf = HolonomicCBFFilter()
    clean_far, clean_tight = [(0.9, 0.0)], [(0.25, 0.0), (-0.20, 0.0)]
    for clean in (clean_far, clean_tight):               # QP path, then LP fallback
        want = cbf.filter_velocity(0.2, 0.0, (0.0, 0.0), clean)
        for junk in ((bad, 0.2), (0.2, bad), (bad, bad)):
            assert cbf.filter_velocity(0.2, 0.0, (0.0, 0.0), [junk] + clean) == pytest.approx(want)
    assert cbf.filter_velocity(0.2, 0.0, (0.0, 0.0), [(bad, bad)]) == (0.2, 0.0)


def test_non_finite_velocity_and_command_do_not_propagate():
    cbf = HolonomicCBFFilter()
    box = [(0.5, 0.0)]
    assert cbf.filter_velocity(0.1, 0.0, (0.0, 0.0), box, [(float('nan'), 0.0)]) == \
        pytest.approx(cbf.filter_velocity(0.1, 0.0, (0.0, 0.0), box))
    out = cbf.filter_velocity(float('nan'), 0.0, (0.0, 0.0), box)
    assert out == pytest.approx(cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), box))
    assert all(math.isfinite(v) for v in cbf.filter_velocity(float('inf'), 0.0, (0.0, 0.0), []))


@pytest.mark.parametrize('d', [0.29, 0.2, 0.12, 0.05, 0.01, 1e-4])
def test_demand_inside_the_safe_distance_is_bounded(d):
    cbf = HolonomicCBFFilter()
    _, c = cbf.constraint_rows((0.0, 0.0), [(d, 0.0)])
    assert 0.0 < c[0] <= cbf.gamma * cbf.safe_distance + 1e-12
    vx, vy = cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), [(d, 0.0)])
    assert -0.3 - 1e-9 <= vx < 0.0 and vy == pytest.approx(0.0)
    assert cbf.last_relaxation == 0.0


def test_a_point_at_the_centre_does_not_outvote_a_wall_behind():
    """Before the cap this retreated at full speed into the wall 0.25 m back."""
    cbf = HolonomicCBFFilter()
    vx, vy = cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), [(0.001, 0.0), (-0.25, 0.0)])
    assert -0.15 < vx < -0.10 and vy == pytest.approx(0.0, abs=1e-6)
