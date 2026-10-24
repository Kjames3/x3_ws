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
