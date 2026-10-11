"""Gyro glitch rejection and zero-rate bias tracking (icm42688_node)."""
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup.gyro_bias import BiasTracker, GlitchFilter  # noqa: E402

G = (0.0, 0.0, 9.81)


def feed(tracker, n, bias, gyro_sd=0.001, accel_sd=0.02, rate=(0.0, 0.0, 0.0), rng=None):
    rng = rng or random.Random(0)
    for _ in range(n):
        g = [b + r + rng.gauss(0, gyro_sd) for b, r in zip(bias, rate)]
        a = [v + rng.gauss(0, accel_sd) for v in G]
        tracker.update(g, a)


def test_glitch_rejects_a_lone_full_scale_sample():
    f = GlitchFilter()
    assert f.accept((0.0, 0.0, 0.003))
    assert not f.accept((0.0, 0.0, 34.87))
    assert f.accept((0.0, 0.0, 0.002))
    assert f.rejected == 1


def test_glitch_first_sample_and_real_motion_pass():
    f = GlitchFilter()
    assert f.accept((0.0, 0.0, 3.0))            # nothing to compare with yet
    for k in range(50):                          # 2 rad/s swing at 5 Hz, 200 Hz
        assert f.accept((0.0, 0.0, 3.0 + 2.0 * math.sin(2 * math.pi * 5 * k / 200)))
    assert f.rejected == 0


def test_glitch_resyncs_instead_of_locking_out():
    f = GlitchFilter()
    f.accept((0.0, 0.0, 0.0))
    seen = [f.accept((0.0, 0.0, 9.0)) for _ in range(5)]
    assert seen == [False, False, False, True, True]


def test_stale_bias_is_replaced_after_two_still_windows():
    t = BiasTracker(bias=(0.0078, -0.0025, 0.0010))
    true = (0.0080, -0.0024, 0.0036)
    feed(t, 199, true)
    assert t.updates == 0
    feed(t, 201, true)
    assert t.updates == 1
    assert all(abs(b - v) < 2e-4 for b, v in zip(t.bias, true))


def test_driving_vibration_never_moves_the_bias():
    t = BiasTracker(bias=(0.0, 0.0, 0.001))
    feed(t, 4000, (0.0, 0.0, 0.004), gyro_sd=0.046, accel_sd=0.4)
    assert t.updates == 0 and t.bias == [0.0, 0.0, 0.001]


def test_quiet_gyro_with_shaking_accel_is_not_still():
    t = BiasTracker()
    feed(t, 2000, (0.0, 0.0, 0.004), accel_sd=0.4)
    assert t.updates == 0


def test_steady_rotation_is_not_taken_for_bias():
    t = BiasTracker(bias=(0.0, 0.0, 0.001))
    feed(t, 2000, (0.0, 0.0, 0.001), rate=(0.0, 0.0, 0.05))
    assert t.updates == 0


def test_one_quiet_window_between_motion_is_ignored():
    t = BiasTracker()
    rng = random.Random(1)
    feed(t, 200, (0.0, 0.0, 0.003), gyro_sd=0.05, rng=rng)
    feed(t, 200, (0.0, 0.0, 0.003), rng=rng)
    feed(t, 200, (0.0, 0.0, 0.003), gyro_sd=0.05, rng=rng)
    assert t.updates == 0


def test_later_windows_follow_drift_gradually():
    t = BiasTracker()
    feed(t, 400, (0.0, 0.0, 0.002))
    first = t.bias[2]
    feed(t, 200, (0.0, 0.0, 0.004))
    assert abs(first - 0.002) < 2e-4
    assert 0.0022 < t.bias[2] < 0.0028          # one 0.2 step toward 0.004
    feed(t, 4000, (0.0, 0.0, 0.004))
    assert abs(t.bias[2] - 0.004) < 2e-4


def test_reset_window_discards_the_partial_window():
    t = BiasTracker()
    feed(t, 399, (0.0, 0.0, 0.003))
    t.reset_window()
    feed(t, 399, (0.0, 0.0, 0.003))
    assert t.updates == 0
    feed(t, 1, (0.0, 0.0, 0.003))
    assert t.updates == 1
