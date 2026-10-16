import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
import stl_shadow_monitor as m


def test_clearance_is_footprint_distance_minus_radius():
    # person centre 1.0 m ahead of the lidar: 1.0 - front 0.16 - radius
    assert m.clearance((m.LIDAR_X + 1.0, 0.0)) == pytest.approx(1.0 - 0.16 - m.PERSON_R)
    assert m.clearance((m.LIDAR_X - 1.0, 0.0)) == pytest.approx(1.0 - 0.13 - m.PERSON_R)
    assert m.clearance((m.LIDAR_X, 0.0)) == pytest.approx(-m.PERSON_R)   # inside the outline


def _walk(speed, t_end=4.0, hz=10):
    return [(t, [(0, 2.0 - speed * t, 0.0)]) for t in np.arange(0, t_end, 1 / hz)]


def test_clear_violation_is_signed():
    ped = _walk(1.0)
    r = m.evaluate(ped, [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)])
    assert r['phi_clear']['rho'] < 0          # person reaches the robot
    ped = [(t, [(0, 3.0, 0.0)]) for t in np.arange(0, 3, 0.1)]
    assert m.evaluate(ped, [(0.0, 0.0, 0.0)])['phi_clear']['rho'] > 0


def test_reaction_delay_measured_from_onset():
    ped = _walk(1.0)
    cmd = [(t, -0.2 if t >= 1.0 else 0.0, 0.0) for t in np.arange(0, 4, 0.05)]
    eps = m.reaction_delays(ped, cmd)
    assert len(eps) == 1 and eps[0]['delay'] is not None
    onset = eps[0]['t']
    assert eps[0]['delay'] == pytest.approx(1.0 - onset, abs=0.06)


def test_no_reaction_is_reported_as_none():
    eps = m.reaction_delays(_walk(1.0), [(t, 0.0, 0.0) for t in np.arange(0, 4, 0.05)])
    assert eps and eps[0]['delay'] is None and eps[0]['rho'] is None


def test_id_break_vs_loss():
    a = [(t, [(0, 1.0, 0.0)]) for t in np.arange(0, 1, 0.1)]
    brk = a + [(t, [(1, 1.05, 0.0)]) for t in np.arange(1, 2, 0.1)]
    assert [e['kind'] for e in m.track_events(brk)] == ['id_break']
    lost = a + [(t, []) for t in np.arange(1, 3, 0.1)]
    assert [e['kind'] for e in m.track_events(lost)] == ['lost']
