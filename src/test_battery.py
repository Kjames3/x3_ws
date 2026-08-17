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
    import battery

    full = battery.PACK_FULL_V
    assert approx(voltage_to_soc(full), 100.0)
    assert approx(voltage_to_soc(full + 1.0), 100.0)
    assert approx(voltage_to_soc(full * 3.20 / 4.20), 0.0)
    assert approx(voltage_to_soc(full * 0.6), 0.0)


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


def test_sag_compensation_matches_measured_hardware():
    """Replay the real load profile measured on the robot 2026-08-09.

    Spinning in place moved the reported voltage 13.0 -> 12.9 at an estimated
    6.5 A; stopping returned it to 13.0.  The compensation should very nearly
    cancel that dip, since R_INT_OHMS was fitted to exactly this point.
    """
    est = BatteryEstimator()
    t, _ = _soak(est, 13.0, 0.5, 200)
    idle = est.percent
    _soak(est, 12.9, 6.5, 200, t0=t)
    driving = est.percent
    assert idle - driving < 2.0, f"driving dip is {idle - driving:.1f} points"


def test_sag_compensation_cannot_invent_charge():
    """A wildly wrong current estimate must not inflate SoC without bound."""
    est = BatteryEstimator()
    t, _ = _soak(est, 11.0, 0.0, 200)
    honest = est.percent
    est2 = BatteryEstimator()
    _soak(est2, 11.0, 500.0, 200)          # absurd current
    inflated = est2.percent
    headroom = voltage_to_soc(11.0 + 0.30) - voltage_to_soc(11.0)
    assert inflated - honest <= headroom + 1e-6, "sag clamp not holding"


def test_monotonic_no_bounce_on_rest():
    """Voltage recovery after stopping must not push the gauge back up.

    Uses the recovery actually measured on the robot (12.9 -> 13.0 when the
    motors stop), not a large synthetic swing -- a swing that big really would
    mean a charger, and the estimator is right to re-home on it.
    """
    est = BatteryEstimator()
    t, _ = _soak(est, 12.9, 6.5, 200)
    driving = est.percent
    _soak(est, 13.0, 0.5, 200, t0=t)          # pack relaxes at rest
    assert est.percent <= driving + 1e-9


def test_charge_is_eventually_detected():
    """A sustained large rise (charger / pack swap) re-homes the estimate."""
    est = BatteryEstimator()
    t, _ = _soak(est, 10.6, 0.5, 200)
    low = est.percent
    assert low < 25.0
    _soak(est, 13.0, 0.0, 600, t0=t)
    assert est.percent > 95.0, f"charger not picked up (still {est.percent:.1f}%)"


def test_ignores_empty_packets():
    est = BatteryEstimator()
    assert est.update(0.0, 0.0, 1.0) == 100.0  # pre-first-sample default
    assert not est.ready
    est.update(11.4, 0.0, 2.0)
    assert est.ready


def _ina():
    import battery
    return BatteryEstimator(pack_full_v=battery.PACK_FULL_V_INA226,
                            pack_empty_v=battery.PACK_EMPTY_V_INA226)


def test_ina226_endpoints_match_the_observed_pack():
    """Both ends are pinned to observation: 13.15 V full, 11.75 V cutoff."""
    import battery

    full = voltage_to_soc(battery.PACK_FULL_V_INA226,
                          battery.PACK_FULL_V_INA226, battery.PACK_EMPTY_V_INA226)
    empty = voltage_to_soc(battery.PACK_EMPTY_V_INA226,
                           battery.PACK_FULL_V_INA226, battery.PACK_EMPTY_V_INA226)
    assert approx(full, 100.0), full
    assert approx(empty, 0.0), empty


def test_gauge_reaches_zero_before_the_robot_cuts_out():
    """The whole point of the two-point map.

    Scaling by full voltage alone puts 0% near 10.0 V, so the gauge would still
    read a third of a pack at the moment the robot shuts off at ~11.75 V.
    """
    import battery

    two_point = voltage_to_soc(11.8, battery.PACK_FULL_V_INA226,
                               battery.PACK_EMPTY_V_INA226)
    gain_only = voltage_to_soc(11.8, battery.PACK_FULL_V_INA226)
    assert two_point < 5.0, f"gauge still reads {two_point:.1f}% at cutoff"
    assert gain_only > 25.0, (
        f"gain-only scaling should be badly wrong here, got {gain_only:.1f}%")


def test_ina226_curve_is_monotonic_across_the_real_range():
    import battery

    last = -1.0
    v = battery.PACK_EMPTY_V_INA226
    while v <= battery.PACK_FULL_V_INA226 + 1e-9:
        soc = voltage_to_soc(v, battery.PACK_FULL_V_INA226,
                             battery.PACK_EMPTY_V_INA226)
        assert soc >= last - 1e-9, f"non-monotonic at {v:.3f} V"
        last = soc
        v += 0.02


def test_ina226_calibration_is_not_the_rosmaster_curve():
    """The two calibrations must not be interchangeable.

    12.82 V is a genuine off-charger reading on this pack.  Through the
    Rosmaster curve (stretched to 13.0 to cancel that ADC's gain error) it looks
    essentially full; on the real two-point curve it is clearly partway down.
    """
    _, out_ina = _soak(_ina(), 12.82, 0.0, 120)
    _, out_ros = _soak(BatteryEstimator(), 12.82, 0.0, 120)
    assert out_ina[-1] < 80.0, out_ina[-1]      # real curve: ~70%
    assert out_ros[-1] > 90.0, out_ros[-1]      # Rosmaster curve: ~94%
    assert out_ros[-1] - out_ina[-1] > 15.0, (out_ros[-1], out_ina[-1])


def test_ina226_real_current_compensates_sag():
    """A real measured current must lift the estimate versus assuming zero draw."""
    import battery

    _, with_i = _soak(_ina(), 12.0, 6.0, 120)
    _, without_i = _soak(_ina(), 12.0, 0.0, 120)

    assert with_i[-1] > without_i[-1], (with_i[-1], without_i[-1])


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
