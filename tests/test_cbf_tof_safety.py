"""Regression tests for the low-speed safety path: CBF -> deadband re-check,
and the lower ToF array's floor-deficit / persistence logic.

These pin behaviour that was found by driving into things (2026-09-22): the
robot hit a box the CBF was tracking because the base adds ~0.14 m/s to any
nonzero command, and drove over a 3 cm block the height test could not see.

server_x3.py cannot be imported offline (it parses argv and pulls in ROS and
YOLO at import), so the functions under test are lifted out of its source by
name and run against stubs.  They are the real code, not copies.
"""
import ast
import math
import os
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src'))
from cbf_filter import HolonomicCBFFilter  # noqa: E402

SERVER = REPO / 'src' / 'server_x3.py'
CONSTANTS = ('TOF_CBF_MIN_Z_M', 'TOF_FLOOR_DEFICIT_MM', 'TOF_FLOOR_TILT_MM',
             'TOF_CBF_MAX_RANGE_M', 'TOF_CBF_GRID_M', 'TOF_CBF_PERSIST_FRAMES',
             'TOF_CBF_HOLD_S', 'CMD_DEADBAND_MPS', 'CBF_SPEED_RANGE_M',
             'CBF_MOVER_MIN_SPEED', 'CMD_MIN_MPS',
             'CBF_MOVER_MIN_PERSON_FRAMES', 'CBF_MOVER_JUMP_HOLD_FRAMES')
FUNCTIONS = ('_tof_floor_deficit', '_cbf_movers')
METHODS = ('_limit_for_deadband', 'set_tof_obstacles')


def _lift():
    """Exec the named constants, functions and methods out of server_x3.py."""
    tree = ast.parse(SERVER.read_text())
    picked, found = [], set()
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and getattr(node.targets[0], 'id', None) in CONSTANTS:
            picked.append(node)
            found.add(node.targets[0].id)
        elif isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            picked.append(node)
            found.add(node.name)
        elif isinstance(node, ast.ClassDef):
            for sub in node.body:
                if isinstance(sub, ast.FunctionDef) and sub.name in METHODS:
                    picked.append(sub)
                    found.add(sub.name)
    missing = set(CONSTANTS + FUNCTIONS + METHODS) - found
    assert not missing, f'no longer in server_x3.py (renamed?): {sorted(missing)}'
    ns = {'np': np, 'math': math, 'time': time, 'os': os,
          # Geometry is covered by test_tof_array; here a point's low edge is
          # the point itself, so heights are whatever the test passes in.
          'tof_zone_low_edge_points': lambda p: np.asarray(p, dtype=np.float64),
          '_tof_floor_depth': None}
    exec(compile(ast.Module(body=picked, type_ignores=[]), str(SERVER), 'exec'), ns)
    return ns


@pytest.fixture()
def srv():
    return _lift()


@pytest.fixture()
def bridge(srv):
    """Just enough of ROS2Bridge for the lifted methods, with the server's CBF."""
    b = SimpleNamespace(
        cbf=HolonomicCBFFilter(safe_distance=0.30, gamma=1.0),
        _lock=threading.Lock(), _tof_streak={}, _tof_cells={}, _tof_obstacles={},
        # Sensor frame == base frame: tests place points directly in base xyz.
        _tof_extrinsic=lambda name: (np.eye(3), np.zeros(3)))
    b.limit = lambda vx, vy, obs, vel=None: srv['_limit_for_deadband'](b, vx, vy, obs, vel)
    b.set_tof = lambda *a, **k: srv['set_tof_obstacles'](b, *a, **k)
    return b


def _constraint(cbf, u, obstacle):
    """a.u + gamma*h for a robot at the origin; >= 0 means safe."""
    ox, oy = obstacle
    h = ox * ox + oy * oy - cbf.safe_distance ** 2
    return -2.0 * (ox * u[0] + oy * u[1]) + cbf.gamma * h


# ----------------------------------------------------------------- CBF filter
def test_cbf_passes_command_through_with_no_obstacles():
    cbf = HolonomicCBFFilter(0.30, 1.0)
    assert cbf.filter_velocity(0.2, -0.1, (0.0, 0.0), []) == (0.2, -0.1)


def test_cbf_slows_an_approach_until_the_constraint_holds():
    cbf = HolonomicCBFFilter(0.30, 1.0)
    box = (0.40, 0.0)
    vx, vy = cbf.filter_velocity(0.30, 0.0, (0.0, 0.0), [box])
    assert vx < 0.30
    assert _constraint(cbf, (vx, vy), box) >= -1e-6


def test_cbf_leaves_a_retreat_alone():
    cbf = HolonomicCBFFilter(0.30, 1.0)
    vx, vy = cbf.filter_velocity(-0.20, 0.0, (0.0, 0.0), [(0.40, 0.0)])
    assert (vx, vy) == pytest.approx((-0.20, 0.0), abs=1e-6)


def test_cbf_ignores_a_distant_obstacle():
    cbf = HolonomicCBFFilter(0.30, 1.0)
    vx, vy = cbf.filter_velocity(0.20, 0.0, (0.0, 0.0), [(3.0, 0.0)])
    assert (vx, vy) == pytest.approx((0.20, 0.0), abs=1e-6)


# ------------------------------------------------------------ deadband re-check
def test_deadband_matches_the_base_driver(srv):
    # Mcnamu_driver_X3: min_pwm 28 at 200 PWM per m/s.
    assert srv['CMD_DEADBAND_MPS'] == pytest.approx(0.14)


def test_creep_toward_a_tracked_box_is_stopped(bridge, srv):
    # The 2026-09-22 failure: a 0.02 m/s command satisfies the CBF on paper,
    # but the base executes 0.16 m/s.  At 0.33 m the executed speed breaches
    # the constraint, so the only safe command is zero.
    box = (0.33, 0.0)
    assert _constraint(bridge.cbf, (0.02, 0.0), box) > 0          # passes the CBF
    assert _constraint(bridge.cbf, (0.02 + srv['CMD_DEADBAND_MPS'], 0.0), box) < 0
    assert bridge.limit(0.02, 0.0, [box]) == (0.0, 0.0)


@pytest.mark.parametrize('dist', [0.34, 0.40, 0.50, 0.70, 1.00])
@pytest.mark.parametrize('cmd', [0.02, 0.10, 0.30])
def test_executed_speed_never_breaches_the_constraint(bridge, srv, dist, cmd):
    box = (dist, 0.0)
    vx, vy = bridge.limit(cmd, 0.0, [box])
    assert vy == 0.0
    assert 0.0 <= vx <= cmd                       # never lengthened, never reversed
    if vx > 0.0:
        executed = (vx + srv['CMD_DEADBAND_MPS'], 0.0)
        assert _constraint(bridge.cbf, executed, box) >= -1e-9


def test_deadband_limit_keeps_direction(bridge):
    vx, vy = bridge.limit(0.20, 0.20, [(0.45, 0.45)])
    assert vx == pytest.approx(vy)
    assert 0.0 <= vx < 0.20


def test_deadband_limit_ignores_obstacles_behind(bridge):
    assert bridge.limit(0.20, 0.0, [(-0.35, 0.0)]) == (0.20, 0.0)


def test_deadband_limit_leaves_zero_and_empty_alone(bridge):
    assert bridge.limit(0.0, 0.0, [(0.35, 0.0)]) == (0.0, 0.0)
    assert bridge.limit(0.20, 0.0, []) == (0.20, 0.0)


def _track(category, speed=0.6, z=1.0, x=0.2, vx=-0.6, vy=0.0):
    return {'category': category, 'speed': speed, 'z': z, 'x': x, 'vx': vx, 'vy': vy}


def test_only_people_carry_a_velocity_into_the_cbf(srv):
    movers = srv['_cbf_movers']([_track('person'), _track('dynamic'), _track('static'), _track(None)])
    assert movers == [((1.0, -0.2), (-0.6, 0.0))]


def test_a_stale_new_or_jumped_person_track_is_not_a_mover(srv):
    f = srv['_cbf_movers']
    ok = dict(_track('person'), age=0, person_frames=5, jump_age=10)
    assert len(f([ok])) == 1
    assert f([dict(ok, age=1)]) == []
    assert f([dict(ok, person_frames=4)]) == []
    assert f([dict(ok, jump_age=9)]) == []


def test_slow_or_distant_people_are_not_movers(srv):
    f = srv['_cbf_movers']
    assert f([_track('person', speed=0.15)]) == []
    assert f([_track('person', z=1.81)]) == []
    assert len(f([_track('person', speed=0.16, z=1.8)])) == 1
    assert f([]) == []


def _worst(cbf, u, obstacles, velocities=None):
    n, c = cbf.constraint_rows((0.0, 0.0), obstacles, velocities)
    return float(np.min(n @ np.asarray(u, dtype=np.float64) - c))


def test_retreat_from_a_closing_obstacle_is_not_zeroed(bridge, srv):
    # Parked, a mover 0.5 m ahead closing at 0.3 m/s.  The CBF asks for a
    # retreat; the executed-speed check must let it through, not stop.
    obs, vel = [(0.5, 0.0)], [(-0.3, 0.0)]
    vx, vy = bridge.cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), obs, vel)
    out = bridge.limit(vx, vy, obs, vel)
    assert out[0] < 0.0 and out[1] == pytest.approx(0.0)
    executed = (out[0] - srv['CMD_DEADBAND_MPS'], 0.0)
    assert _worst(bridge.cbf, executed, obs, vel) >= -1e-9


def test_escape_from_inside_the_safe_distance_is_not_zeroed(bridge, srv):
    walls = [(0.25, 0.0), (-0.20, 0.0)]
    vx, vy = bridge.cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), walls)
    out = bridge.limit(vx, vy, walls)
    if out != (0.0, 0.0):
        s = math.hypot(*out)
        executed = (out[0] * (1 + srv['CMD_DEADBAND_MPS'] / s), out[1] * (1 + srv['CMD_DEADBAND_MPS'] / s))
        assert _worst(bridge.cbf, executed, walls) > _worst(bridge.cbf, (0.0, 0.0), walls)


def test_slowest_escape_is_a_command_the_driver_acts_on(bridge, srv):
    # 2026-10-11 squeeze test: 0.27 m behind, open ahead.  The escape wanted is
    # slower than the deadband, so the slowest executable speed is chosen; it
    # was sent as 0.001 m/s, which Mcnamu_driver_X3 ignores (|v| <= 0.001), and
    # the robot sat inside the safe distance for 33 s.
    walls = [(-0.267, 0.0), (0.57, 0.0)]
    vx, vy = bridge.cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), walls)
    out = bridge.limit(vx, vy, walls)
    assert out[0] >= srv['CMD_MIN_MPS'] > 0.001 and out[1] == pytest.approx(0.0)


def test_executed_speed_check_follows_the_rectangular_footprint(srv):
    # Jambs beside the outline: no limit forward. A box ahead: shortened to
    # gamma * spare clearance. Same rows as the filter itself.
    b = SimpleNamespace(cbf=HolonomicCBFFilter(gamma=2.0, footprint=(0.18, 0.12), margin=0.10))
    limit = lambda vx, vy, obs, w=0.0: srv['_limit_for_deadband'](b, vx, vy, obs, None, w)
    db = srv['CMD_DEADBAND_MPS']
    assert limit(0.2, 0.0, [(0.0, 0.26), (0.0, -0.26)]) == (0.2, 0.0)
    vx, vy = limit(0.5, 0.0, [(0.18 + 0.10 + 0.20, 0.0)])
    assert vx == pytest.approx(2.0 * 0.20 - db) and vy == 0.0


def test_a_command_shortened_below_the_minimum_speed_becomes_a_stop(srv):
    b = SimpleNamespace(cbf=HolonomicCBFFilter(gamma=2.0, footprint=(0.18, 0.12), margin=0.10))
    srv['CMD_DEADBAND_MPS'], srv['CMD_MIN_MPS'] = 0.0, 0.03     # closed-loop base
    f = srv['_limit_for_deadband']
    assert f(b, 0.2, 0.0, [(0.18 + 0.10 + 0.010, 0.0)]) == (0.0, 0.0)      # 0.02 m/s allowed
    vx, _ = f(b, 0.2, 0.0, [(0.18 + 0.10 + 0.030, 0.0)])                   # 0.06 m/s allowed
    assert vx == pytest.approx(0.06)
    assert f(b, 0.02, 0.0, [(1.5, 0.0)]) == (0.02, 0.0)          # the operator's own crawl is left alone


def test_escape_that_the_deadband_would_overshoot_becomes_a_stop(bridge, srv):
    # 0.29 m ahead and 0.31 m behind: the tiny retreat the CBF wants would be
    # executed 0.14 m/s faster, straight at the wall behind. Stopping is better.
    walls = [(0.29, 0.0), (-0.31, 0.0)]
    vx, vy = bridge.cbf.filter_velocity(0.0, 0.0, (0.0, 0.0), walls)
    assert bridge.limit(vx, vy, walls) == (0.0, 0.0)


@pytest.mark.parametrize('seed', range(40))
def test_executed_command_is_safe_or_no_worse_than_stopping(bridge, srv, seed):
    rng = np.random.default_rng(seed)
    k = int(rng.integers(1, 40))
    r, th = rng.uniform(0.15, 0.9, k), rng.uniform(-math.pi, math.pi, k)
    obs = [tuple(p) for p in np.c_[r * np.cos(th), r * np.sin(th)]]
    vel = [tuple(v) for v in rng.uniform(-0.6, 0.6, (k, 2))]
    cmd = rng.uniform(-0.3, 0.3, 2)
    vx, vy = bridge.cbf.filter_velocity(cmd[0], cmd[1], (0.0, 0.0), obs, vel)
    out = bridge.limit(vx, vy, obs, vel)
    s = math.hypot(*out)
    if s > 0:
        k_exec = 1 + srv['CMD_DEADBAND_MPS'] / s
        executed = (out[0] * k_exec, out[1] * k_exec)
        # satisfies every constraint, or (when stopping does not) beats stopping
        assert _worst(bridge.cbf, executed, obs, vel) >= min(0.0, _worst(bridge.cbf, (0.0, 0.0), obs, vel)) - 1e-9


def test_cbf_then_deadband_stops_short_of_a_box(bridge, srv):
    # The pipeline move() runs, stepped in time at the EXECUTED speed.
    x_box, dt = 1.0, 0.02
    for _ in range(2000):
        obstacles = [(x_box, 0.0)]
        vx, vy = bridge.cbf.filter_velocity(0.30, 0.0, (0.0, 0.0), obstacles)
        vx, vy = bridge.limit(vx, vy, obstacles)
        if vx <= 0.0:
            break
        x_box -= (vx + srv['CMD_DEADBAND_MPS']) * dt
    else:
        pytest.fail('robot never stopped')
    assert x_box >= bridge.cbf.safe_distance, f'stopped {x_box:.3f} m from the box'


# -------------------------------------------------------- floor-deficit detector
OK = np.full(64, 5)          # VL53L5CX status 5 = valid


@pytest.fixture()
def floor(srv):
    """A plausible bare-floor baseline for the lower array, far row missing."""
    base = np.repeat(np.linspace(260.0, 120.0, 8), 8)
    base[:8] = np.nan
    srv['_tof_floor_depth'] = base
    return base


def _deficit(srv, dist, status=OK):
    return srv['_tof_floor_deficit'](np.nan_to_num(dist, nan=0.0), status)


def test_bare_floor_flags_nothing(srv, floor):
    # Measured: bare floor stays within 6 mm of its baseline.
    wobble = np.tile([6.0, -6.0, 3.0, -3.0], 16)
    mask = _deficit(srv, floor + wobble)
    assert mask is not None and not mask.any()


def test_low_block_flags_exactly_its_zones(srv, floor):
    # Measured: a 3 cm block shortened its zones by 16-41 mm.
    dist = floor.copy()
    block = [34, 35, 42, 43]
    dist[block] -= [16.0, 25.0, 41.0, 20.0]
    mask = _deficit(srv, dist)
    assert sorted(np.flatnonzero(mask)) == block


def test_deficit_threshold_is_between_floor_noise_and_the_block(srv):
    assert 6.0 < srv['TOF_FLOOR_DEFICIT_MM'] < 16.0


def test_chassis_tilt_frame_is_skipped(srv, floor):
    # The whole view moving together is the chassis pitching, not an obstacle.
    assert _deficit(srv, floor - 15.0) is None


def test_invalid_and_unbaselined_zones_never_flag(srv, floor):
    dist = floor.copy()
    dist[[3, 40]] -= 60.0                 # zone 3 has no baseline (NaN row)
    status = OK.copy()
    status[40] = 255                      # zone 40 is not a valid return
    mask = _deficit(srv, dist, status)
    assert not mask[3] and not mask[40]


def test_missing_baseline_degrades_to_the_height_test(srv):
    assert srv['_tof_floor_depth'] is None
    assert srv['_tof_floor_deficit'](np.full(64, 150.0), OK) is None


# ------------------------------------------------- persistence, hold, selection
def _frame(height_m=0.10, x=0.40):
    """One point in zone 20 at (x, 0, height); zone 21 always on the floor."""
    return (np.array([[x, 0.0, height_m], [x, 0.10, 0.0]]), np.array([20, 21]))


def test_obstacle_needs_three_consecutive_frames(bridge, srv):
    assert srv['TOF_CBF_PERSIST_FRAMES'] == 3
    pts, zones = _frame()
    for _ in range(2):
        assert not bridge.set_tof('upper', pts, zones).any()
        assert len(bridge._tof_obstacles['upper'][0]) == 0
    keep = bridge.set_tof('upper', pts, zones)
    assert keep.tolist() == [True, False]          # the floor point never passes
    assert bridge._tof_obstacles['upper'][0].tolist() == [[0.40, 0.0]]


def test_single_frame_ghost_resets_the_streak(bridge):
    hit, zones = _frame()
    miss, _ = _frame(height_m=0.0)
    for pts in (hit, hit, miss, hit, hit):
        assert not bridge.set_tof('upper', pts, zones).any()
    assert bridge.set_tof('upper', hit, zones)[0]


def test_confirmed_obstacle_is_held_through_a_dropout(bridge, srv, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    hit, zones = _frame()
    miss, _ = _frame(height_m=0.0)
    for _ in range(3):
        bridge.set_tof('upper', hit, zones)
    clock[0] += 0.07                                   # one 15 Hz frame later
    bridge.set_tof('upper', miss, zones)
    assert len(bridge._tof_obstacles['upper'][0]) == 1, 'brake released on a dropout'
    clock[0] += srv['TOF_CBF_HOLD_S'] + 0.01
    bridge.set_tof('upper', miss, zones)
    assert len(bridge._tof_obstacles['upper'][0]) == 0, 'held past TOF_CBF_HOLD_S'


def test_floor_deficit_zone_passes_despite_its_height(bridge):
    # Lower array: a point at floor height is kept when the deficit test says so.
    pts, zones = _frame(height_m=0.005)
    extra = np.array([True, False])
    for _ in range(2):
        bridge.set_tof('lower', pts, zones, extra)
    assert bridge.set_tof('lower', pts, zones, extra).tolist() == [True, False]


def test_points_beyond_max_range_are_dropped(bridge, srv):
    pts, zones = _frame(x=srv['TOF_CBF_MAX_RANGE_M'] + 0.2)
    for _ in range(3):
        keep = bridge.set_tof('upper', pts, zones)
    assert not keep.any()


def test_no_tf_yet_returns_none(bridge):
    bridge._tof_extrinsic = lambda name: None
    pts, zones = _frame()
    assert bridge.set_tof('upper', pts, zones) is None
