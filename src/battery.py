"""
Battery state-of-charge estimation for the X3's pack.

Rewritten 2026-08-16 against a **real full-cycle discharge**: 5.87 h, 2054
samples of measured voltage *and* measured current from the INA226, taken from
13.266 V at full charge down to 9.66 V where the pack collapsed and the robot
died (``logs/battery/battery_20260815-132549.csv``).  Everything below is fitted
to that trace.  Two things it overturned:

**1. The pack is 4S LiFePO4, not 3S Li-ion.**  The old module assumed an NMC
per-cell curve and a 12.6 V nameplate.  The trace rules that out and the LFP
signature is unmistakable: 13.27 V full, a nominal 12.8 V, a dead-flat plateau
between 13.0 and 12.85 V, a knee at ~12.4 V, then a cliff straight through
10.0 V (= 2.50 V/cell, the LFP floor).  A 3S Li-ion pack cannot sit at 13.27 V.

This also settles an old open question: "the Rosmaster ADC reads ~3% high" was
inferred purely from a rested 13.0 V being impossible for a 12.6 V pack.  For
*this* pack 13.0 V rested is unremarkable, so that inference is withdrawn --
the ADC looks roughly honest and the readings just needed the right curve.

**2. Voltage alone cannot gauge this pack, so it no longer tries.**  That is
not a tuning problem, it is the chemistry.  Measured from the trace:

    85% -> 98% SoC spans      5 mV     (below the sensor's own 1.25 mV LSB)
    30% -> 100% SoC spans   298 mV
     0% ->  30% SoC spans  2418 mV

Sag at a 1.4 A idle draw is already ~78 mV, so in the top two thirds of the
pack the load noise is an order of magnitude larger than the entire signal.
The old voltage-only gauge read **48.9% when the pack truly had 18.0% left** --
about 64 minutes of runtime presented as three hours, which is how a robot ends
up stranded mid-task.

The fix is the one the INA226 made possible: integrate the measured current
(coulomb counting) and use the OCV curve only where it actually carries
information.  :meth:`BatteryEstimator.update` blends the two, weighting the OCV
correction by how much SoC a plausible voltage error would move -- near zero on
the plateau, dominant below the knee where accuracy matters most.

Coulomb counting drifts without an anchor and cannot survive a restart on its
own, so the caller should persist and restore ``charge_ah``; see
``battery_log.SoCState``.
"""

from __future__ import annotations

import math
import time

# --- Pack model ------------------------------------------------------------
# 4 cells of LiFePO4.  Kept for documentation and sanity checks only: the OCV
# table below is expressed in *pack* volts because it was measured at the pack,
# so nothing needs to divide by this any more.
CELLS = 4

# Usable capacity, amp-hours.  Measured by trapezoidal integration of the
# INA226 current over the full cycle: 8.067 Ah delivered between "just off the
# charger" and "robot dead".  This is delivered capacity at the robot's ~1.4 A
# idle draw, which is the operating point that matters; a much heavier load
# would yield somewhat less.  It sets the SoC scale, so recalibrate it from a
# fresh full-cycle CSV if the pack ages or is replaced.
PACK_CAPACITY_AH = 8.07

# Endpoints of the measured curve.  Both are observations, not nameplate.
PACK_FULL_V = 13.16
PACK_EMPTY_V = 10.44

# The INA226 measures the pack directly and is the source of the table, so for
# it the endpoints are simply the table's own.  Names kept for the call site in
# server_x3.py; passing them is now a no-op re-anchor rather than a correction.
PACK_FULL_V_INA226 = PACK_FULL_V
PACK_EMPTY_V_INA226 = PACK_EMPTY_V

# Open-circuit *pack* voltage -> SoC percent, ascending.
#
# Built by binning the full-cycle trace on coulomb-counted SoC and taking the
# median sag-compensated voltage in each bin (see the fit in the commit that
# introduced this).  Chemistry-independent by construction: it maps what this
# sensor reads to how much charge was actually left, so it stays valid even if
# the voltage reading carries a gain error.
#
# The top of the table is deliberately coarse.  Between 80% and 100% the real
# data spans 40 mV with non-monotonic noise inside it; pretending to resolve
# 85 from 95 there would be fabrication.  The coulomb counter carries that
# region, and the OCV weighting below correctly reports near-zero confidence.
_OCV_TABLE_PACK = (
    (10.44,   0.0),
    (10.98,   1.0),
    (11.49,   2.0),
    (11.83,   3.0),
    (12.25,   5.0),
    (12.43,   7.0),
    (12.52,  10.0),
    (12.60,  15.0),
    (12.73,  20.0),
    (12.81,  25.0),
    (12.86,  30.0),
    (12.92,  40.0),
    (12.96,  50.0),
    (12.99,  60.0),
    (13.06,  70.0),
    (13.12,  80.0),
    (13.16, 100.0),
)

# Internal resistance of the pack plus wiring, used to undo load sag:
# V_oc ~= V_terminal + I * R_INT.
#
# Re-measured 2026-08-16 by regressing short-timescale voltage deviations
# against current deviations across the full cycle (n=1999, R^2=0.49):
# 0.0558 ohm.  The previous 0.015 was fitted against the *fabricated* current
# estimate (`0.5 + avg_pwr*6.0`) and is withdrawn along with it -- notably, the
# old observation "13.0 -> 12.9 V while spinning" gives ~0.05 ohm once the real
# current draw (~2 A above idle, not the imagined 6.5 A) is used, so the two
# measurements agree.
#
# Caveat worth respecting: the regression only saw +/-0.075 A of natural load
# variation, so extrapolating to a 6 A drive current is not supported by this
# data.  That is exactly why the sag term is clamped and why the OCV correction
# is down-weighted under load rather than trusted.
R_INT_OHMS = 0.0558

# Sag correction ceiling.  At the measured resistance this is reached around
# 5.4 A; beyond that the estimate simply reads slightly pessimistic, which is
# the safe direction.
MAX_SAG_COMP_V = 0.30

# EMA time constant for the voltage and current filters, seconds.
TAU_S = 12.0

# --- OCV trust model -------------------------------------------------------
# How wrong the open-circuit voltage estimate plausibly is, in volts.  This is
# not sensor noise (the INA226's LSB is 1.25 mV) but residual sag error: the
# uncertainty in R_INT times a realistic current, plus pack relaxation.
OCV_VOLTAGE_UNCERTAINTY_V = 0.03

# Standing uncertainty of the coulomb counter itself, in SoC points.  The two
# estimates are combined by relative variance (the scalar Kalman gain), so this
# is the yardstick the OCV estimate has to beat.  One point is about what the
# INA226's ~1% current error accumulates over several hours at idle draw.
#
# Variance weighting rather than a linear roll-off matters here: it squares the
# penalty for a flat curve, which turns the chemistry's knee into a sharp
# handover instead of a gentle blend.  Across the pack it yields roughly
#   11.5 V -> 0.99    12.45 V -> 0.53    12.9 V -> 0.04    13.14 V -> 0.004
# i.e. the curve governs the endgame and the counter is left alone above it.
OCV_TOLERANCE_PCT = 1.0

# Time constant for the OCV correction at full confidence, seconds.  Slow
# enough that a transient cannot yank the gauge, fast enough that the endgame
# is governed by the curve rather than by accumulated integration error.
TAU_CORRECT_S = 600.0


def voltage_to_soc(volts: float, pack_full_v: float | None = None,
                   pack_empty_v: float | None = None) -> float:
    """Interpolate an open-circuit pack voltage onto the measured SoC curve.

    ``pack_full_v`` / ``pack_empty_v`` optionally re-anchor the voltage axis
    affinely onto the table's own endpoints, for a voltage source that reads
    the same pack through a different gain and offset.  Both must be supplied
    for the re-anchor to happen; the default is to use the table directly.
    """
    v_lo_t, v_hi_t = _OCV_TABLE_PACK[0][0], _OCV_TABLE_PACK[-1][0]
    if pack_full_v is not None and pack_empty_v is not None:
        span = pack_full_v - pack_empty_v
        if span <= 0:
            return 0.0
        volts = v_lo_t + (volts - pack_empty_v) * (v_hi_t - v_lo_t) / span

    if volts <= v_lo_t:
        return 0.0
    if volts >= v_hi_t:
        return 100.0
    for (v_lo, s_lo), (v_hi, s_hi) in zip(_OCV_TABLE_PACK, _OCV_TABLE_PACK[1:]):
        if volts <= v_hi:
            return s_lo + (volts - v_lo) / (v_hi - v_lo) * (s_hi - s_lo)
    return 100.0


def ocv_sensitivity(volts: float) -> float:
    """Local slope of the curve, in SoC percent per volt.

    Large means the pack is on its flat plateau and voltage says little; small
    means a millivolt is meaningful.  Used to weight the OCV correction.
    """
    v_lo_t, v_hi_t = _OCV_TABLE_PACK[0][0], _OCV_TABLE_PACK[-1][0]
    if volts <= v_lo_t:
        # A pack this far down is unambiguously empty; report the steep bottom
        # segment's slope so it is believed.
        return (_OCV_TABLE_PACK[1][1] - _OCV_TABLE_PACK[0][1]) / \
               (_OCV_TABLE_PACK[1][0] - _OCV_TABLE_PACK[0][0])
    if volts >= v_hi_t:
        # Above the top of the curve the reading is *not* trustworthy in the
        # same way: over-reading is how sag compensation invents charge, and
        # overstating a pack is the dangerous direction.  Report the flat top
        # segment's slope, which yields near-zero confidence.
        return (_OCV_TABLE_PACK[-1][1] - _OCV_TABLE_PACK[-2][1]) / \
               (_OCV_TABLE_PACK[-1][0] - _OCV_TABLE_PACK[-2][0])
    for (v_lo, s_lo), (v_hi, s_hi) in zip(_OCV_TABLE_PACK, _OCV_TABLE_PACK[1:]):
        if volts <= v_hi:
            dv = v_hi - v_lo
            return (s_hi - s_lo) / dv if dv > 0 else float("inf")
    return float("inf")


def ocv_confidence(volts: float) -> float:
    """How far to trust an OCV reading at this voltage, in 0..1.

    Derived, not tuned.  The OCV estimate's standard deviation is the voltage
    uncertainty times the local slope; weighting it against the counter's own
    standard deviation by relative variance is the scalar Kalman gain,

        w = sigma_counter^2 / (sigma_counter^2 + sigma_ocv^2)
    """
    sens = ocv_sensitivity(volts)
    if not math.isfinite(sens) or OCV_TOLERANCE_PCT <= 0:
        return 0.0
    ratio = OCV_VOLTAGE_UNCERTAINTY_V * sens / OCV_TOLERANCE_PCT
    return 1.0 / (1.0 + ratio * ratio)


class BatteryEstimator:
    """Coulomb-counted SoC, anchored by the OCV curve where the curve is sharp.

    Call :meth:`update` with each voltage/current sample and read
    :attr:`percent` whenever telemetry is built.  Persist :attr:`charge_ah`
    across restarts and pass it back as ``initial_charge_ah``, otherwise every
    restart re-seeds from the plateau and throws the estimate away.
    """

    def __init__(self, r_int: float = R_INT_OHMS, tau_s: float = TAU_S,
                 pack_full_v: float | None = None,
                 pack_empty_v: float | None = None,
                 capacity_ah: float | None = PACK_CAPACITY_AH,
                 initial_charge_ah: float | None = None):
        """``capacity_ah=None`` disables coulomb counting (OCV lookup only).

        Pass None whenever the current reading is absent or synthesised -- with
        no current sensor the integral either never moves (freezing the gauge
        at its seed) or drifts on a fabricated number, both of which are worse
        than the plain curve.  On this pack the curve alone is poor above the
        knee, so this is a real degradation, not an equivalent path.
        """
        self.r_int = r_int
        self.tau_s = tau_s
        self.pack_full_v = pack_full_v
        self.pack_empty_v = pack_empty_v
        self.capacity_ah = capacity_ah
        self.charge_ah: float | None = initial_charge_ah
        self._soc_ocv_only: float | None = None
        self._v_ema: float | None = None
        self._i_ema: float = 0.0
        self._last_t: float | None = None

    # -- inputs -------------------------------------------------------------
    def update(self, volts: float, current_a: float = 0.0,
               now: float | None = None) -> float:
        """Feed one terminal-voltage/current sample; returns SoC percent.

        ``current_a`` is the measured pack draw: positive discharging, negative
        charging.  With no current sensor pass 0 and the estimator degrades to
        an uncompensated OCV lookup, which on this pack is poor above the knee
        -- that degradation is inherent to the chemistry, not to this code.
        """
        if not volts or volts <= 0.1:      # sensor sends 0 before the first read
            return self.percent
        now = time.monotonic() if now is None else now

        if self._v_ema is None or self._last_t is None:
            self._v_ema = volts
            self._i_ema = current_a
            dt = 0.0
        else:
            dt = max(0.0, now - self._last_t)
            alpha = 1.0 - math.exp(-dt / self.tau_s) if self.tau_s > 0 else 1.0
            self._v_ema += alpha * (volts - self._v_ema)
            # Filtered on the *same* time constant as the voltage: a raw sag
            # term would jump the instant the motors start while the voltage
            # EMA still lagged, briefly inflating the estimate.
            self._i_ema += alpha * (current_a - self._i_ema)
        self._last_t = now

        # 1. Undo load sag to approximate the open-circuit voltage.
        sag = self._i_ema * self.r_int
        sag = max(-MAX_SAG_COMP_V, min(MAX_SAG_COMP_V, sag))
        v_oc = self._v_ema + sag
        soc_ocv = voltage_to_soc(v_oc, self.pack_full_v, self.pack_empty_v)

        # 1b. No current sensor: report the curve directly and stop here.
        if self.capacity_ah is None:
            self._soc_ocv_only = soc_ocv
            return soc_ocv

        # 2. Seed the counter from the curve on the very first sample.  This is
        #    the weakest moment for the estimate; restoring a persisted
        #    charge_ah instead is strongly preferred.
        if self.charge_ah is None:
            self.charge_ah = self.capacity_ah * soc_ocv / 100.0
            return self.percent

        # 3. Integrate the measured current.  Discharge removes charge; a
        #    negative reading (charger attached) adds it back, so no separate
        #    charge-detection heuristic is needed any more.
        if dt > 0:
            self.charge_ah -= current_a * dt / 3600.0

        # 4. Pull toward the curve, weighted by how much the curve is worth
        #    here.  On the plateau the weight is ~0.02 and this is a no-op over
        #    a whole session; below the knee it approaches 1 and dominates.
        if dt > 0:
            conf = ocv_confidence(v_oc)
            if conf > 0.0 and TAU_CORRECT_S > 0:
                gain = (1.0 - math.exp(-dt / TAU_CORRECT_S)) * conf
                target = self.capacity_ah * soc_ocv / 100.0
                self.charge_ah += gain * (target - self.charge_ah)

        self.charge_ah = max(0.0, min(self.capacity_ah, self.charge_ah))
        return self.percent

    # -- outputs ------------------------------------------------------------
    @property
    def percent(self) -> float:
        if self.capacity_ah is None:
            return 100.0 if self._soc_ocv_only is None else self._soc_ocv_only
        if self.charge_ah is None or self.capacity_ah <= 0:
            return 100.0
        return max(0.0, min(100.0, 100.0 * self.charge_ah / self.capacity_ah))

    @property
    def voltage_filtered(self) -> float:
        return PACK_FULL_V if self._v_ema is None else self._v_ema

    @property
    def minutes_remaining(self) -> float | None:
        """Runtime left at the currently filtered draw, or None if unknown."""
        if self.charge_ah is None or self._i_ema <= 0.05:
            return None
        return self.charge_ah / self._i_ema * 60.0

    @property
    def ready(self) -> bool:
        if self.capacity_ah is None:
            return self._soc_ocv_only is not None
        return self.charge_ah is not None
