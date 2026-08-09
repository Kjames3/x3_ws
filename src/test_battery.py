"""Offline checks for the battery SoC estimator (no robot / ROS required).

Run with:  python3 src/test_battery.py
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from battery import BatteryEstimator, voltage_to_soc


def approx(a, b, tol=1e-6):
    """The pack->cell scaling makes exact endpoint equality float-brittle."""
    return abs(a - b) <= tol


def _soak(est, volts, amps, seconds, t0=0.0, step=1.0):
    """Feed a constant condition for `seconds`, returning (last_t, samples)."""
    t = t0
    out = []
    for _ in range(int(seconds / step)):
        t += step
        out.append(est.update(volts, amps, t))
    return t, out


def test_curve_endpoints():
    assert approx(voltage_to_soc(12.6), 100.0)
    assert approx(voltage_to_soc(13.5), 100.0)
    assert approx(voltage_to_soc(9.6), 0.0)
    assert approx(voltage_to_soc(8.0), 0.0)


def test_curve_is_monotonic():
    prev = -1.0
    v = 9.0
    while v <= 13.0:
        soc = voltage_to_soc(v)
        assert soc >= prev, f"curve dips at {v} V"
        prev = soc
        v += 0.01


def test_calibration_constant_shifts_whole_curve():
    """PACK_FULL_V must reposition the curve coherently, not just its top.

    This is the knob to turn once the pack's true resting full voltage is known,
    so it needs to keep 'full reads 100%, empty reads 0%' true at any setting.
    """
    import battery

    original = battery.PACK_FULL_V
    try:
        for full in (12.6, 13.0):
            battery.PACK_FULL_V = full
            assert approx(battery.voltage_to_soc(full), 100.0)
            empty = full * (3.20 / 4.20)
            assert approx(battery.voltage_to_soc(empty), 0.0)
            mid = battery.voltage_to_soc(full * (3.80 / 4.20))
            assert 40.0 < mid < 50.0, f"midpoint off at PACK_FULL_V={full}: {mid}"
    finally:
        battery.PACK_FULL_V = original


def test_reported_13v_is_not_silently_pinned():
    """The live robot reads 13.0 V; that must clamp to 100%, never overflow."""
    assert approx(voltage_to_soc(13.0), 100.0)
    assert approx(voltage_to_soc(20.0), 100.0)


def test_dead_pack_reads_zero():
    """The old linear map called 9.4 V ~29%; the pack is actually flat there."""
    est = BatteryEstimator()
    _soak(est, 9.4, 0.5, 120)
    assert est.percent == 0.0


def test_code_transition_is_smoothed():
    """Alternating 0.1 V codes must not swing the gauge by ~5 points.

    At rest the robot's ADC does not dither (measured 2026-08-09: one code held
    across 4561 samples), so this is not about recovering sub-LSB resolution.
    It models a reading sitting on a code boundary or fluctuating under load,
    where the unfiltered gauge would visibly flip back and forth.
    """
    est = BatteryEstimator()
    t = 0.0
    out = []
    for i in range(120):
        t += 1.0
        out.append(est.update(11.6 if i % 2 else 11.5, 0.0, t))
    swing = max(out[-20:]) - min(out[-20:])
    raw_swing = voltage_to_soc(11.6) - voltage_to_soc(11.5)
    assert raw_swing > 3.0, "test premise: raw quantisation step is large"
    assert swing < 0.5, f"filtered swing still {swing:.2f} points"


def test_never_rises_on_load_transient():
    """Starting the motors must not spike SoC via the sag-compensation term."""
    est = BatteryEstimator()
    t, _ = _soak(est, 11.8, 0.5, 120)
    prev = est.percent
    for _ in range(120):
        t += 1.0
        p = est.update(11.2, 5.5, t)          # sag of 0.6 V under 5.5 A
        assert p <= prev + 1e-9, f"SoC rose {prev:.1f} -> {p:.1f}"
        prev = p


def test_sag_compensation_reduces_driving_dip():
    """Driving should not look like a sudden loss of a third of the pack."""
    est = BatteryEstimator()
    t, _ = _soak(est, 11.8, 0.5, 200)
    idle = est.percent
    _soak(est, 11.2, 5.5, 200, t0=t)
    driving = est.percent
    uncompensated = voltage_to_soc(11.2)
    assert idle - driving < 10.0, f"still dips {idle - driving:.1f} points"
    assert driving > uncompensated + 15.0


def test_monotonic_no_bounce_on_rest():
    """Voltage recovery after stopping must not push the gauge back up."""
    est = BatteryEstimator()
    t, _ = _soak(est, 11.2, 5.5, 200)
    driving = est.percent
    _soak(est, 11.75, 0.5, 200, t0=t)         # pack relaxes at rest
    assert est.percent <= driving + 1e-9


def test_charge_is_eventually_detected():
    """A sustained large rise (charger / pack swap) re-homes the estimate."""
    est = BatteryEstimator()
    t, _ = _soak(est, 10.6, 0.5, 200)
    low = est.percent
    assert low < 25.0
    _soak(est, 12.6, 0.0, 600, t0=t)
    assert est.percent > 90.0, f"charger not picked up (still {est.percent:.1f}%)"


def test_ignores_empty_packets():
    est = BatteryEstimator()
    assert est.update(0.0, 0.0, 1.0) == 100.0  # pre-first-sample default
    assert not est.ready
    est.update(11.4, 0.0, 2.0)
    assert est.ready


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print(f"PASS  {name}")
        except AssertionError as exc:
            failures += 1
            print(f"FAIL  {name}: {exc}")
    print(f"\n{failures} failure(s)")
    sys.exit(1 if failures else 0)
