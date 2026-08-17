"""Offline checks for the battery SoC estimator (no robot / ROS required).

Run with:  python3 src/test_battery.py

The centrepiece is :func:`test_replay_real_discharge`, which replays the actual
5.87 h full-cycle trace in ``fixtures/`` -- the pack was run from full charge to
death to produce it, so it is not a capture anyone will casually repeat.  Any
change to the pack model should be judged against that replay first.
"""

import csv
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import battery
from battery import (BatteryEstimator, voltage_to_soc, ocv_confidence,
                     PACK_CAPACITY_AH, PACK_FULL_V, PACK_EMPTY_V)
from battery_log import SoCState

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "fixtures", "battery_full_cycle_20260815.csv")


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _soak(est, volts, amps, seconds, t0=0.0, step=1.0):
    """Feed a constant condition for `seconds`, returning (last_t, samples)."""
    t = t0
    out = []
    for _ in range(int(seconds / step)):
        t += step
        out.append(est.update(volts, amps, t))
    return t, out


def _load_trace():
    with open(FIXTURE) as fh:
        rows = list(csv.DictReader(fh))
    return [(float(r["epoch_s"]), float(r["voltage_v"]), float(r["current_a"]))
            for r in rows]


def _true_soc(trace):
    """Coulomb-counted ground truth for the fixture, 100% -> 0%."""
    q = [0.0]
    for k in range(1, len(trace)):
        dt = trace[k][0] - trace[k - 1][0]
        q.append(q[-1] + 0.5 * (trace[k][2] + trace[k - 1][2]) * dt / 3600.0)
    total = q[-1]
    return [100.0 * (total - x) / total for x in q], total


# --- curve shape -----------------------------------------------------------
def test_curve_endpoints():
    assert approx(voltage_to_soc(PACK_EMPTY_V), 0.0), voltage_to_soc(PACK_EMPTY_V)
    assert approx(voltage_to_soc(PACK_FULL_V), 100.0), voltage_to_soc(PACK_FULL_V)
    assert voltage_to_soc(9.0) == 0.0
    assert voltage_to_soc(14.0) == 100.0


def test_curve_is_monotonic():
    prev = -1.0
    v = 10.0
    while v <= 13.5:
        s = voltage_to_soc(v)
        assert s >= prev, f"curve dips at {v:.2f} V: {s} < {prev}"
        prev = s
        v += 0.01


def test_curve_matches_the_measured_trace():
    """The table must reproduce the coulomb-counted truth it was fitted to."""
    trace = _load_trace()
    truth, _ = _true_soc(trace)
    worst = 0.0
    for (t, v, i), s_true in zip(trace, truth):
        v_oc = v + i * battery.R_INT_OHMS
        worst = max(worst, abs(voltage_to_soc(v_oc) - s_true))
    # The plateau is genuinely ambiguous, so a static lookup cannot be tight
    # there; this only pins that the table is not grossly mis-shaped.  The
    # residual is itself the argument for coulomb counting -- a voltage-only
    # gauge on this pack is wrong by this much even using a perfect table.
    assert worst < 25.0, f"table deviates {worst:.1f} points from measured truth"
    assert worst > 5.0, "unexpectedly good: re-check the fit before relaxing this"


def test_pack_is_lifepo4_shaped_not_nmc():
    """Guard the finding that overturned the old model.

    An NMC curve spreads its charge fairly evenly over voltage.  This pack puts
    70% of its charge into 300 mV.  If someone "fixes" the table back to a
    textbook Li-ion shape, this fails.
    """
    span_top = PACK_FULL_V - 12.86      # 30% -> 100%
    span_bottom = 12.86 - PACK_EMPTY_V  # 0%  -> 30%
    assert span_top < 0.35, f"top 70% should be flat, spans {span_top:.3f} V"
    assert span_bottom > 2.0, f"bottom 30% should be steep, spans {span_bottom:.3f} V"


def test_plateau_is_below_sensor_resolution():
    """Documents *why* coulomb counting is required rather than optional."""
    v80, v100 = 13.12, 13.16
    assert voltage_to_soc(v100) - voltage_to_soc(v80) == 20.0
    assert (v100 - v80) < 0.05, "the top 20% really is ~40 mV wide"
    # The pack's own idle sag is larger than that entire band.
    assert 1.4 * battery.R_INT_OHMS > (v100 - v80), \
        "idle sag should exceed the whole top-of-pack voltage span"


# --- OCV trust weighting ---------------------------------------------------
def test_ocv_is_distrusted_on_the_plateau_and_trusted_at_the_bottom():
    top = ocv_confidence(13.14)
    plateau = ocv_confidence(13.0)
    knee = ocv_confidence(12.45)
    bottom = ocv_confidence(11.5)
    assert top < 0.15, f"top band confidence should collapse, got {top:.3f}"
    assert plateau < 0.15, f"plateau confidence should collapse, got {plateau:.3f}"
    assert bottom > 0.8, f"steep region should be trusted, got {bottom:.3f}"
    assert bottom > knee > plateau


def test_a_collapsed_pack_is_believed_but_an_over_reading_is_not():
    """Asymmetric on purpose: understating charge is the safe direction.

    Below the curve the pack is unambiguously flat and the reading must be
    trusted so the gauge reaches 0%.  Above the curve the same trust would let
    sag compensation invent a full pack out of a heavy motor draw.
    """
    assert ocv_confidence(9.0) > 0.9, "a dead pack should be believed"
    assert ocv_confidence(14.0) < 0.15, "an over-reading should not be believed"


# --- coulomb counting ------------------------------------------------------
def test_counts_charge_out_of_the_pack():
    """Pure integration, with the OCV correction disabled."""
    saved = battery.OCV_VOLTAGE_UNCERTAINTY_V
    battery.OCV_VOLTAGE_UNCERTAINTY_V = 1e9      # drives every confidence to ~0
    try:
        est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH)
        _soak(est, 12.98, 2.0, 3600, step=10.0)
        used = PACK_CAPACITY_AH - est.charge_ah
    finally:
        battery.OCV_VOLTAGE_UNCERTAINTY_V = saved
    assert 1.95 < used < 2.05, f"expected ~2 Ah drawn, got {used:.3f}"


def test_negative_current_recharges_without_a_heuristic():
    # A pack actually on a charger sits above the curve, not on the plateau.
    est = BatteryEstimator(initial_charge_ah=4.0)
    _soak(est, 13.60, -2.0, 3600, step=10.0)
    assert est.charge_ah > 5.5, f"charging should add charge, got {est.charge_ah:.3f}"


def test_percent_never_leaves_bounds():
    est = BatteryEstimator(initial_charge_ah=0.05)
    _soak(est, 12.9, 5.0, 3600, step=10.0)
    assert est.percent == 0.0, f"drained pack read {est.percent}"
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH)
    _soak(est, 13.80, -5.0, 3600, step=10.0)
    assert est.percent == 100.0, f"charging pack read {est.percent}"


def test_plateau_does_not_yank_a_good_estimate():
    """A correct counter must not be dragged off by an ambiguous reading.

    Sitting at 12.99 V the table says ~60%, but a 30 mV error there is worth
    10 SoC points, so a counter that says 40% must survive the hour largely
    intact rather than being pulled to the curve.
    """
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH * 0.40)
    _soak(est, 12.99, 0.05, 3600, step=10.0)   # ~no draw, so only OCV can move it
    assert est.percent < 46.0, f"plateau pulled the estimate to {est.percent:.1f}%"


def test_steep_region_does_correct_a_wrong_estimate():
    """Below the knee the curve must be allowed to fix accumulated drift."""
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH * 0.50)
    _soak(est, 11.83, 0.05, 3600, step=10.0)   # 11.83 V is ~3% on the table
    assert est.percent < 10.0, f"steep region failed to correct, got {est.percent:.1f}%"


def test_sag_compensation_matches_measured_resistance():
    est = BatteryEstimator()
    est.update(12.90, 0.0, 0.0)
    v_no_load = est.voltage_filtered
    est2 = BatteryEstimator()
    _soak(est2, 12.90, 6.0, 600, step=1.0)
    # 6 A * 0.0558 = 0.335 V, clamped to MAX_SAG_COMP_V.
    assert approx(v_no_load, 12.90, 1e-6)
    assert battery.MAX_SAG_COMP_V == 0.30


def test_sag_compensation_cannot_invent_charge():
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH * 0.5)
    _soak(est, 12.5, 500.0, 600, step=1.0)     # absurd current
    assert est.percent <= 50.0, "huge sag correction inflated the estimate"


def test_ignores_empty_packets():
    est = BatteryEstimator(initial_charge_ah=4.0)
    before = est.percent
    est.update(0.0, 1.0, 1.0)
    est.update(None, 1.0, 2.0)
    assert est.percent == before


def test_ocv_only_mode_tracks_voltage_and_ignores_current():
    """The no-current-sensor fallback must not run the integrator."""
    est = BatteryEstimator(capacity_ah=None)
    est.update(12.60, 0.0, 0.0)
    assert est.ready
    assert abs(est.percent - voltage_to_soc(12.60)) < 0.5, est.percent
    # A fabricated current must not be able to drain a counter that isn't there.
    _soak(est, 12.60, 3.0, 3600, step=10.0)
    assert est.percent > 5.0, f"OCV-only mode drifted to {est.percent:.1f}%"


def test_missing_current_sensor_does_not_freeze_the_gauge():
    """With capacity set but current stuck at 0, the counter would never move.

    Guards the failure this mode exists to prevent: if the INA226 fails to
    initialise, current reads a constant 0 and a coulomb counter reports the
    same percentage forever while the pack actually drains.
    """
    frozen = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH * 0.8)
    _soak(frozen, 12.99, 0.0, 3600, step=10.0)
    stuck = frozen.percent

    ok = BatteryEstimator(capacity_ah=None)
    _soak(ok, 12.99, 0.0, 600, step=10.0)
    _soak(ok, 12.43, 0.0, 600, t0=600.0, step=10.0)
    assert ok.percent < stuck - 20.0, \
        "OCV-only mode should follow a falling pack when the counter cannot"


def test_minutes_remaining_tracks_draw():
    est = BatteryEstimator(initial_charge_ah=4.0)
    _soak(est, 12.9, 2.0, 300, step=1.0)
    mins = est.minutes_remaining
    assert mins is not None and 60.0 < mins < 130.0, mins


# --- the real trace --------------------------------------------------------
def test_replay_real_discharge():
    """Replay the full cycle and require the gauge to track measured truth.

    Seeded at 100% as it would be off the charger, then given nothing but the
    voltage and current the robot actually saw.
    """
    trace = _load_trace()
    truth, total_ah = _true_soc(trace)
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH)
    errs = []
    for (t, v, i), s_true in zip(trace, truth):
        est.update(v, i, t)
        errs.append(est.percent - s_true)

    worst = max(abs(e) for e in errs)
    rms = (sum(e * e for e in errs) / len(errs)) ** 0.5
    # Measured 0.18 / 0.11 at the time of writing.  Tight on purpose: this is
    # the regression net for any future change to the pack model.
    assert worst < 1.0, f"worst replay error {worst:.2f} points"
    assert rms < 0.5, f"replay RMS error {rms:.2f} points"


def test_replay_beats_the_old_voltage_only_gauge():
    """The specific failure that motivated this: 48.9% shown at a true 18%."""
    trace = _load_trace()
    truth, _ = _true_soc(trace)
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH)
    worst_at_low = 0.0
    for (t, v, i), s_true in zip(trace, truth):
        est.update(v, i, t)
        if s_true <= 25.0:
            worst_at_low = max(worst_at_low, est.percent - s_true)
    # Never overstate the remaining charge by much when it is nearly gone.
    assert worst_at_low < 8.0, f"overstated a low pack by {worst_at_low:.1f} points"


def test_replay_ends_empty_when_the_pack_dies():
    """The gauge must be at the floor at the instant the pack collapsed.

    Note it reads ~0%, not "0% with minutes to spare": no artificial reserve is
    built in, because inventing one would hide real capacity and change what
    the number means.  Advance warning is :attr:`minutes_remaining`'s job.
    """
    trace = _load_trace()
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH)
    for t, v, i in trace:
        est.update(v, i, t)
    assert est.percent < 1.0, f"gauge still read {est.percent:.2f}% on a dead pack"


def test_recovers_from_a_wrong_starting_estimate():
    """The property the replay alone cannot show.

    The replay is seeded with the same capacity that was fitted from it, so it
    is partly self-fulfilling.  This starts the counter 30 points too high --
    a stale restore, or a pack that was not as full as assumed -- and requires
    the OCV curve to have hauled it back by the time the pack is genuinely low,
    which is the only place being wrong actually hurts.
    """
    trace = _load_trace()
    truth, _ = _true_soc(trace)
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH)
    est.update(trace[0][1], trace[0][2], trace[0][0])
    est.charge_ah = min(PACK_CAPACITY_AH, est.charge_ah + PACK_CAPACITY_AH * 0.30)

    worst_low = 0.0
    for (t, v, i), s_true in zip(trace[1:], truth[1:]):
        est.update(v, i, t)
        if s_true <= 20.0:
            worst_low = max(worst_low, abs(est.percent - s_true))
    assert worst_low < 10.0, \
        f"a 30-point seeding error survived into the endgame ({worst_low:.1f} points)"
    assert est.percent < 2.0, f"finished a dead pack at {est.percent:.2f}%"


def test_measured_capacity_matches_the_trace():
    trace = _load_trace()
    _, total_ah = _true_soc(trace)
    assert abs(PACK_CAPACITY_AH - total_ah) < 0.05, \
        f"PACK_CAPACITY_AH={PACK_CAPACITY_AH} vs measured {total_ah:.3f}"


# --- persistence -----------------------------------------------------------
def test_state_round_trips(tmpdir="/tmp/x3_soc_state_test"):
    os.makedirs(tmpdir, exist_ok=True)
    path = os.path.join(tmpdir, "soc.json")
    if os.path.exists(path):
        os.remove(path)
    st = SoCState(path, min_write_interval_s=0.0)
    assert st.load(12.9, now=1000.0) is None, "no file should mean no state"
    assert st.save(5.5, 12.9, now=1000.0)
    assert approx(st.load(12.9, now=1060.0), 5.5, 1e-3)


def test_state_expires():
    path = "/tmp/x3_soc_state_test/soc_old.json"
    st = SoCState(path, min_write_interval_s=0.0)
    st.save(5.5, 12.9, now=1000.0)
    assert st.load(12.9, now=1000.0 + 13 * 3600) is None, "stale state was trusted"


def test_state_detects_a_charge_while_down():
    path = "/tmp/x3_soc_state_test/soc_chg.json"
    st = SoCState(path, min_write_interval_s=0.0)
    st.save(1.0, 12.4, now=1000.0)
    assert st.load(13.2, now=1600.0) is None, "pack was charged; state should be dropped"
    assert approx(st.load(12.45, now=1600.0), 1.0, 1e-3), "small drift should be kept"


def test_restart_mid_plateau_keeps_the_estimate():
    """The whole point of persisting: a restart at 40% must not jump to 60%."""
    path = "/tmp/x3_soc_state_test/soc_restart.json"
    if os.path.exists(path):
        os.remove(path)
    st = SoCState(path, min_write_interval_s=0.0)
    est = BatteryEstimator(initial_charge_ah=PACK_CAPACITY_AH * 0.40)
    _soak(est, 12.99, 1.4, 600, step=10.0)
    st.save(est.charge_ah, 12.99, now=2000.0)

    restored = st.load(12.99, now=2100.0)
    est2 = BatteryEstimator(initial_charge_ah=restored)
    est2.update(12.99, 1.4, 0.0)
    assert abs(est2.percent - est.percent) < 1.0, \
        f"restart moved the gauge {est.percent:.1f}% -> {est2.percent:.1f}%"

    cold = BatteryEstimator()          # no persisted state: seeds from OCV
    cold.update(12.99, 1.4, 0.0)
    assert cold.percent - est.percent > 10.0, \
        "fixture assumption broken: a cold seed should be badly off here"


if __name__ == "__main__":
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  ok   {name}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL {name}: {e}")
        except Exception as e:
            failed += 1
            print(f"  ERR  {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
