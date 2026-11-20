"""Heading hold (Mcnamu_driver_X3) against a simulated drifting body."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup.heading_hold import HeadingHold  # noqa: E402

DT = 0.05


def feed(hh, t, wz):
    hh.integrate_rate(t, wz)


def test_zero_command_is_exactly_zero_with_large_error():
    hh = HeadingHold()
    t = 0.0
    feed(hh, t, 0.0)
    hh.correct(t, 0.2, 0.0, 0.0)
    for _ in range(20):
        t += DT
        feed(hh, t, 0.5)                 # builds a large yaw error
        hh.correct(t, 0.2, 0.0, 0.0)
    assert abs(hh.yaw) > 0.2
    t += DT
    feed(hh, t, 0.5)
    assert hh.correct(t, 0.0, 0.0, 0.0) == 0.0


def test_turn_passes_through_and_resets_reference():
    hh = HeadingHold()
    feed(hh, 0.0, 0.0)
    hh.correct(0.0, 0.2, 0.0, 0.0)
    assert hh._ref is not None
    feed(hh, 0.05, 0.0)
    assert hh.correct(0.05, 0.2, 0.0, 0.5) == 0.5
    assert hh._ref is None and hh._integral == 0.0


def test_constant_drift_is_held():
    drift = 0.05
    hh = HeadingHold()
    t = 0.0
    feed(hh, t, 0.0)
    start = hh.yaw
    rate = drift
    corrs = []
    for _ in range(int(10.0 / DT)):
        t += DT
        feed(hh, t, rate)
        out = hh.correct(t, 0.0, 0.2, 0.0)
        corrs.append(out)
        rate = drift + out               # body follows the commanded rate
    assert abs(hh.yaw - start) < 0.02
    assert corrs[-1] < 0.0               # opposes the +drift


def test_correction_clamps_at_max_corr():
    hh = HeadingHold(max_corr=0.3)
    feed(hh, 0.0, 0.0)
    hh.correct(0.0, 0.2, 0.0, 0.0)
    t = 0.0
    for _ in range(10):
        t += DT
        feed(hh, t, 2.0)                 # big yaw error
    out = hh.correct(t, 0.2, 0.0, 0.0)
    assert out == -0.3


def test_stale_imu_passes_through():
    hh = HeadingHold()
    feed(hh, 0.0, 0.0)
    hh.correct(0.0, 0.2, 0.0, 0.0)
    feed(hh, 0.05, 1.0)
    assert hh.correct(1.0, 0.2, 0.0, 0.0) == 0.0
    assert hh._ref is None
    assert HeadingHold().correct(0.0, 0.2, 0.0, 0.1) == 0.1   # never saw the IMU


def test_reengage_takes_new_reference():
    hh = HeadingHold()
    t = 0.0
    feed(hh, t, 0.0)
    hh.correct(t, 0.2, 0.0, 0.0)
    for _ in range(10):
        t += DT
        feed(hh, t, 0.3)                 # yaws away while engaged
        hh.correct(t, 0.2, 0.0, 0.0)
    t += DT
    feed(hh, t, 0.3)
    assert hh.correct(t, 0.0, 0.0, 0.0) == 0.0      # stop
    for _ in range(10):
        t += DT
        feed(hh, t, 0.3)                 # yaw changes while stopped
        hh.correct(t, 0.0, 0.0, 0.0)
    t += DT
    feed(hh, t, 0.0)
    assert hh.correct(t, 0.2, 0.0, 0.0) == 0.0      # no jump from the old error
