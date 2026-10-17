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
    ped = [(t, [(0, 2.0 - t, 0.0)]) for t in np.arange(0, 4.0, 0.1)]
    r = m.evaluate(ped, [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)])
    assert r['phi_clear']['rho'] < 0          # person reaches the robot
    ped = [(t, [(0, 3.0, 0.0)]) for t in np.arange(0, 3, 0.1)]
    assert m.evaluate(ped, [(0.0, 0.0, 0.0)])['phi_clear']['rho'] > 0


def _approach_then_retreat(v_in, t_turn, t_end=6.0, hz=10):
    """Person walks straight at the robot at v_in from 3 m; the robot (stationary
    until t_turn) then retreats at v_in, so the relative closing speed drops to 0."""
    ped, d = [], 3.0
    for t in np.arange(0, t_end, 1 / hz):
        ped.append((t, [(0, m.LIDAR_X + d, 0.0)]))
        d -= v_in / hz * (1.0 if t < t_turn else 0.0)
    return ped


def test_no_episode_while_the_person_is_far_and_slow():
    ped = [(t, [(0, 3.0 - 0.3 * t, 0.0)]) for t in np.arange(0, 4, 0.1)]
    assert m.reaction_episodes(ped) == []


def test_episode_restored_when_relative_closing_falls():
    # fast walker at 1.2 m/s: violates gamma*(clearance-D_MIN) once close enough
    ped = _approach_then_retreat(1.2, t_turn=1.3)
    eps = m.reaction_episodes(ped)
    assert eps and eps[0]['status'] == 'restored'
    assert eps[0]['closing'] > eps[0]['allowed']
    assert 0.0 < eps[0]['delay'] < 1.5


def test_never_restored_is_reported():
    ped = [(t, [(0, m.LIDAR_X + 3.0 - 1.2 * t, 0.0)]) for t in np.arange(0, 2.2, 0.1)]
    eps = m.reaction_episodes(ped)
    assert eps and eps[0]['status'] in ('unrestored', 'track_lost')


def test_track_loss_is_not_called_a_slow_reaction():
    ped = [(t, [(0, m.LIDAR_X + 3.0 - 1.2 * t, 0.0)]) for t in np.arange(0, 1.6, 0.1)]
    ped += [(t, []) for t in np.arange(1.6, 4, 0.1)]
    eps = m.reaction_episodes(ped)
    assert eps and eps[0]['status'] == 'track_lost'


def test_id_break_vs_loss():
    a = [(t, [(0, 1.0, 0.0)]) for t in np.arange(0, 1, 0.1)]
    brk = a + [(t, [(1, 1.05, 0.0)]) for t in np.arange(1, 2, 0.1)]
    assert [e['kind'] for e in m.track_events(brk)] == ['id_break']
    lost = a + [(t, []) for t in np.arange(1, 3, 0.1)]
    assert [e['kind'] for e in m.track_events(lost)] == ['lost']


def _scan_run(n_scans=60, n_beams=360, wall=3.0, person_from=20, person_to=40, person_r=1.0):
    """Wall at 3 m everywhere; a narrow object appears dead ahead for some scans."""
    inc = 2 * np.pi / n_beams
    am = -np.pi
    r = np.full((n_scans, n_beams), wall)
    ang = am + np.arange(n_beams) * inc + np.pi
    ahead = np.abs(np.arctan2(np.sin(ang), np.cos(ang))) < 0.15
    r[person_from:person_to, ahead] = person_r
    return r, am, inc, np.arange(n_scans) * 0.14


def test_foreground_ignores_the_wall_and_finds_the_object():
    r, am, inc, t = _scan_run()
    fg = m.scan_foreground(r, am, inc, t, np.zeros((len(t), 3)))
    assert all(f is None for _, f in fg[:20]) and all(f is None for _, f in fg[40:])
    x, y, n = fg[30][1]
    assert x == pytest.approx(m.LIDAR_X + 1.0, abs=0.05) and abs(y) < 0.2 and n >= 3


def test_cross_check_reports_covered_and_missed_presence():
    r, am, inc, t = _scan_run()
    fg = m.scan_foreground(r, am, inc, t, np.zeros((len(t), 3)))
    # estimator sees the person for the first half of the lidar's window only
    ped = [(tt, [(0, m.LIDAR_X + 1.0, 0.0)] if tt < t[30] else []) for tt in np.arange(0, 8.4, 0.06)]
    c = m.cross_check(fg, ped)
    assert c['lidar_frames'] == 20 and 0.3 < c['coverage'] < 0.9
    assert c['missed'] and c['missed'][0]['t'] > t[30]
    assert abs(c['range_diff']['median']) < 0.1


def test_nothing_ahead_means_no_coverage_requirement():
    r, am, inc, t = _scan_run(person_from=0, person_to=0)
    c = m.cross_check(m.scan_foreground(r, am, inc, t, np.zeros((len(t), 3))), [(0.0, [])])
    assert c['lidar_frames'] == 0 and c['coverage'] is None


def test_a_moving_robot_does_not_turn_the_walls_into_foreground():
    """Walls at x=+3 and x=-3 in odom; the robot backs away 1 m. Per-beam ranges
    change a lot, but the world cells do not."""
    n_scans, n_beams = 60, 360
    inc, am = 2 * np.pi / n_beams, -np.pi
    ang = am + np.arange(n_beams) * inc + np.pi
    xs = -np.linspace(0, 1.0, n_scans)
    r = np.full((n_scans, n_beams), np.nan)
    for i, xr in enumerate(xs):
        dx, dy = np.cos(ang), np.sin(ang)
        with np.errstate(divide='ignore', invalid='ignore'):
            hit = np.where(dx > 1e-6, (3.0 - xr) / dx, np.where(dx < -1e-6, (-3.0 - xr) / dx, np.nan))
        r[i] = np.where((hit > 0) & (hit < 12), hit, np.nan)
    poses = np.column_stack([xs, np.zeros(n_scans), np.zeros(n_scans)])
    fg = m.scan_foreground(r, am, inc, np.arange(n_scans) * 0.14, poses)
    assert sum(f is not None for _, f in fg) <= 2
