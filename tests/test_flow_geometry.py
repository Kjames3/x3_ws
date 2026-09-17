"""PAA5100JE burst parsing and velocity math -- all without hardware.

What this guards is what fails silently: an axis swap that turns forward into
sideways, a lever-arm sign that doubles rotation leakage instead of cancelling
it, and trusting low-SQUAL counts that were measured under-counting.
"""
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup.flow_geometry import (  # noqa: E402
    DEFAULT_COUNTS_PER_M, GOOD_SQUAL, counts_to_sensor_velocity, parse_burst,
    sensor_to_body_velocity, velocity_variance)


def burst(dx=0, dy=0, motion=True, squal=215, shutter=7000):
    return struct.pack('<BBhhBBBBBB', 0x80 if motion else 0, 0, dx, dy, squal,
                       0, 0, 0, shutter >> 8, shutter & 0xFF)


def test_parse_round_trip():
    b = parse_burst(burst(dx=-231, dy=5405, squal=216, shutter=6164))
    assert (b.motion, b.dx, b.dy, b.squal, b.shutter) == (True, -231, 5405, 216, 6164)
    assert not b.rejected


def test_deltas_ignored_without_motion_bit():
    b = parse_burst(burst(dx=40, dy=-12, motion=False))
    assert (b.dx, b.dy) == (0, 0)


def test_reject_rule_needs_low_squal_and_pegged_shutter():
    assert parse_burst(burst(squal=0x10, shutter=0x1F00)).rejected
    assert not parse_burst(burst(squal=0x10, shutter=0x1500)).rejected
    # The tape case: low SQUAL, short shutter, still counting -> not rejected,
    # which is exactly why covariance must come from SQUAL.
    assert not parse_burst(burst(squal=78, shutter=1583)).rejected


def test_parse_rejects_wrong_length():
    with pytest.raises(ValueError):
        parse_burst(b'\x00' * 11)


def test_measured_pushes_map_to_robot_axes():
    # 20 cm forward read dy ~ +5200; 20 cm left read dx ~ +5100 (2026-09-16).
    vx, vy = counts_to_sensor_velocity(0, 5200, 1.0)
    assert vx == pytest.approx(0.2, rel=0.01) and vy == 0.0
    vx, vy = counts_to_sensor_velocity(5200, 0, 1.0)
    assert vy == pytest.approx(0.2, rel=0.01) and vx == 0.0


def test_scale_and_dt():
    vx, _ = counts_to_sensor_velocity(0, int(DEFAULT_COUNTS_PER_M), 0.5)
    assert vx == pytest.approx(2.0)
    with pytest.raises(ValueError):
        counts_to_sensor_velocity(1, 1, 0.0)


def test_pure_rotation_cancels_behind_centre():
    # Sensor 13.4 mm behind centre, spinning CCW at 1 rad/s in place: the
    # sensor point sweeps to the robot's right at vy = wz * x = -0.0134 m/s.
    # Body velocity must come out zero.
    mx, wz = -0.0134, 1.0
    vx_s, vy_s = 0.0, wz * mx
    assert sensor_to_body_velocity(vx_s, vy_s, wz, mx, 0.0) == pytest.approx((0.0, 0.0))


def test_lateral_offset_rotation_cancels():
    my, wz = 0.02, -0.5
    assert sensor_to_body_velocity(-wz * my, 0.0, wz, 0.0, my) == pytest.approx((0.0, 0.0))


def test_translation_passes_through_rotation_correction():
    assert sensor_to_body_velocity(0.3, -0.1, 0.0, -0.0134, 0.0) == (0.3, -0.1)


def test_variance_grows_with_speed_and_low_squal():
    still = velocity_variance(0.0, 215)
    moving = velocity_variance(0.5, 215)
    tape = velocity_variance(0.5, 78)
    assert 0 < still < moving < tape
    assert velocity_variance(0.5, GOOD_SQUAL) == pytest.approx(moving)
    assert velocity_variance(0.5, 0) == pytest.approx(moving * 100.0)
