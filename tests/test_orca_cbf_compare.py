import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
import orca_cbf_compare as m

needs_rvo2 = pytest.mark.skipif(not (Path(m.ORCA_DIR) / 'Agent.py').exists(), reason='RVO2-python not installed')


def _walker(x, vx):
    return [(1, np.array([x, 0.0]), np.array([vx, 0.0]), True)]


def test_rect_dist_matches_the_cbf_outline():
    assert m.rect_dist([(1.0, 0.0)])[0] == pytest.approx(1.0 - 0.16)
    assert m.rect_dist([(-1.0, 0.0)])[0] == pytest.approx(1.0 - 0.13)
    assert m.rect_dist([(0.0, 0.0)])[0] == 0.0


def test_cbf_gives_way_to_a_walker_and_ignores_an_empty_scene():
    c = m.CBFController()
    assert np.allclose(c((0, 0), np.zeros(2), np.zeros((0, 2)), []), 0.0)
    u = c((0, 0), np.zeros(2), np.zeros((0, 2)), _walker(0.9, -1.0))
    assert u[0] < -0.1 and abs(u[1]) < 1e-6


@needs_rvo2
def test_orca_gives_way_and_full_share_does_more_than_reciprocal():
    full, half = m.ORCAController(1.0), m.ORCAController(0.5, name='orca-recip')
    lidar = np.zeros((0, 2))
    assert np.allclose(full((0, 0), np.zeros(2), lidar, []), 0.0)
    uf = full((0, 0), np.zeros(2), lidar, _walker(1.2, -1.0))
    uh = half((0, 0), np.zeros(2), lidar, _walker(1.2, -1.0))
    assert uf[0] < 0 and abs(uf[0]) + abs(uf[1]) <= m.MAX_WHEEL + 1e-9
    assert np.hypot(*uf) > np.hypot(*uh)


@needs_rvo2
def test_orca_keeps_lidar_lines_hard():
    # a wall 0.5 m behind: the retreat from the walker must not drive into it within tau_obst
    wall = np.column_stack([np.full(21, -0.5), np.linspace(-1, 1, 21)])
    u = m.ORCAController(1.0)((0, 0), np.zeros(2), wall, _walker(1.0, -1.0))
    assert u[0] >= -(0.5 - m.ORCA_ROBOT_RADIUS - m._FC[0]) / m.ORCA_TAU_OBST - 1e-6


def _run(gated=True, nominal=((0.0, 0.2, 0.0, 0.0),)):
    odom = np.array([(t, 0.2 * t, 0.0, 0.0) for t in np.arange(0.0, 5.0, 0.1)])
    return dict(name='synthetic', gated=gated, odom=odom,
                nominal=np.array(nominal, float).reshape(-1, 4))


def test_nominal_follows_the_recorded_request_until_the_watchdog():
    run = _run(nominal=[(1.0, 0.2, 0.0, 0.3)])
    assert np.allclose(m.nominal_at(run, 'cmd', 1.2, np.zeros(2), 0.0), (0.2, 0.0, 0.3))
    assert np.allclose(m.nominal_at(run, 'cmd', 1.0 + m.WATCHDOG_S + 0.01, np.zeros(2), 0.0), 0.0)
    assert np.allclose(m.nominal_at(run, 'zero', 1.2, np.zeros(2), 0.0), 0.0)


def test_track_steers_back_to_the_recorded_path():
    run = _run(nominal=[(t, 0.2, 0.0, 0.0) for t in np.arange(0, 5, 0.1)])
    # simulated robot 0.3 m left of the real one: steer right and forward, no faster than ~asked
    u = m.nominal_at(run, 'track', 2.0, np.array([0.4, 0.3]), 0.0)
    assert u[0] > 0 and u[1] < 0 and np.hypot(u[0], u[1]) <= 0.2 + 1e-9


def test_pre_gate_bags_cannot_use_the_recorded_command_as_a_request():
    run = _run(gated=False)
    assert m.resolve_nominal(run, 'auto') == 'zero'
    assert m.resolve_nominal(_run(), 'auto') == 'cmd'
    with pytest.raises(SystemExit):
        m.resolve_nominal(run, 'cmd')


def test_cbf_passes_a_clear_request_and_brakes_toward_a_wall():
    c = m.CBFController()
    assert np.allclose(c((0.3, 0.0, 0.0), np.zeros(2), np.zeros((0, 2)), []), (0.3, 0.0))
    wall = np.column_stack([np.full(21, 0.35), np.linspace(-0.5, 0.5, 21)])
    u = c((0.3, 0.0, 0.0), np.zeros(2), wall, [])
    assert u[0] < 0.3


@needs_rvo2
def test_orca_passes_a_clear_request_and_turning_costs_wheel_speed():
    o = m.ORCAController(1.0)
    assert np.allclose(o((0.3, 0.0, 0.0), np.zeros(2), np.zeros((0, 2)), []), (0.3, 0.0))
    u = o((0.5, 0.0, 1.0), np.zeros(2), np.zeros((0, 2)), [])
    assert u[0] == pytest.approx(0.5 - 0.215)
