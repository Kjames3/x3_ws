"""obstacle_memory.ObstacleMemory: cables stay remembered, a departed leg's shoe does not."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup.obstacle_memory import ObstacleMemory  # noqa: E402

TAN = math.tan(math.radians(7.0))


def _mem(**kw):
    return ObstacleMemory(0.05, 20.0, TAN, 0.31, 4.0, **kw)


def _run(m, frames, t0=0.0, dt=0.1):
    out, t = None, t0
    for pts in frames:
        p = np.asarray(pts, dtype=float).reshape(-1, 3)
        out = m.update(p[:, 0], p[:, 1], p[:, 2], 0.0, 0.0, t)
        t += dt
    return out, t


def _cable():
    return [(1.025, -0.275 + 0.05 * i, 0.075) for i in range(12)]     # cell centres


def _leg(x=1.5):
    low = [(x + 0.025 + dx, 0.025, 0.075) for dx in (0.0, 0.1, 0.15)]   # shoe, toe forward
    up = [(x + 0.025, 0.025, 0.125 + 0.05 * i) for i in range(6)]       # 0.125-0.375 m
    return low + up


def test_cable_inside_the_blind_ring_is_remembered():
    m = _mem()
    out, _ = _run(m, [_cable()] * 3 + [[]] * 50)        # 5 s unseen, hidden (d=1 m < 2.2 m)
    assert len(out) == len(_cable())


def test_departed_leg_takes_its_shoe_with_it():
    m = _mem()
    out, _ = _run(m, [_leg()] * 5 + [[]] * 15)          # 1.5 s after the person left
    assert len(out) == 0


def test_old_rule_keeps_the_shoe():
    m = _mem(tall_m=None)
    out, _ = _run(m, [_leg()] * 5 + [[]] * 15)
    assert len(out) >= 3                                 # the hidden low voxels linger


def test_standing_person_with_a_sparse_hit_pattern_is_kept():
    m = _mem()
    frames = [_leg() if i % 3 == 0 else _leg()[:3] for i in range(30)]   # tall part hit 1 frame in 3
    out, _ = _run(m, frames + [[]] * 3)
    assert len(out) >= 3


def test_cable_next_to_a_static_desk_leg_survives():
    m = _mem()
    desk_leg = [(1.225, 0.525, 0.125 + 0.05 * i) for i in range(6)]
    frames = [_cable() + desk_leg] * 3 + [desk_leg] * 40
    out, _ = _run(m, frames)
    near = [p for p in out if abs(p[0] - 1.025) < 1e-3 and abs(p[2] - 0.075) < 1e-3]
    assert len(near) == len(_cable())
