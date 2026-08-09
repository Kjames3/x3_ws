"""
Battery state-of-charge estimation for the X3's 3S Li-ion pack (12.6 V, ~6 Ah).

The Rosmaster board reports pack voltage as a single byte scaled by 10, so the
raw signal is quantised to 0.1 V and arrives on /voltage at 10 Hz.  Turning that
into a percentage well needs three things the old one-line linear map lacked:

  1. An OCV->SoC curve.  Li-ion discharge is flat between ~3.9 and ~3.6 V/cell
     (roughly 70% of the capacity); a straight line over the voltage span makes
     the gauge plummet at the top and stall at the bottom.
  2. A realistic empty point.  0% must land where the robot stops working
     (~3.2 V/cell = 9.6 V), not at the cell's absolute floor.
  3. Filtering + load compensation.  One 0.1 V code is ~2-5 points of SoC, so
     the gauge stepped between codes, and motor draw sags the terminal voltage
     by several tenths of a volt, making the reading dive whenever the robot
     drives and bounce back when it stops.

     Measured on the robot 2026-08-09: at rest the reported code is *perfectly*
     stable (13.0 V held across 4561 samples / 5 min, zero dither).  So the EMA
     does not recover resolution below the 0.1 V step the way oversampling a
     dithering ADC would -- there is nothing to average at rest.  What it does
     buy is a smooth ramp across a code transition instead of a 2-5 point jump,
     and rejection of the genuine fluctuation seen under motor load.

No new hardware or dependencies: the only inputs are the voltage already on
/voltage and the commanded-motor-power current estimate already computed by the
telemetry loop.
"""

from __future__ import annotations

import math
import time

# --- Pack model ------------------------------------------------------------
CELLS = 3

# Pack voltage that corresponds to a full charge (4.20 V/cell).
#
# CALIBRATE THIS.  It is the single knob that positions the whole curve, and it
# is the one number that cannot be derived from the code.  The robot reports
# 13.0 V (4.33 V/cell) when idle, which is above the Li-ion ceiling -- that is
# either charger float voltage or a gain error in the Rosmaster's ADC, and the
# two call for different values here.  To measure it: unplug the charger, let
# the pack rest ~30 min, and read the idle voltage off the GUI.
#
# 12.6 V is the nameplate value and keeps the previous behaviour; if the true
# resting full voltage is higher, raise this rather than editing the table, so
# the whole curve shifts together.
PACK_FULL_V = 12.6

# Per-cell rest-voltage -> SoC for a typical 18650 NMC cell.  Sorted ascending.
# Kept per-cell so PACK_FULL_V (and CELLS) reposition the curve without anyone
# having to re-derive thirteen pack voltages by hand.
_OCV_TABLE_CELL = (
    (3.20,   0.0),
    (3.30,   3.0),
    (3.40,   6.0),
    (3.50,  11.0),
    (3.60,  18.0),
    (3.68,  26.0),
    (3.73,  34.0),
    (3.80,  45.0),
    (3.87,  55.0),
    (3.95,  67.0),
    (4.00,  76.0),
    (4.10,  90.0),
    (4.20, 100.0),
)

# Internal resistance of the pack plus wiring and the Rosmaster's shunt path.
# Used only to undo load sag: V_oc ~= V_terminal + I * R_INT.
R_INT_OHMS = 0.12

# Sag correction is clamped so a bad current estimate cannot invent charge.
MAX_SAG_COMP_V = 0.80

# EMA time constant for the voltage filter, seconds.  Long enough to average
# away the 0.1 V quantisation step across many 10 Hz samples, short enough to
# follow a real discharge.
TAU_S = 12.0

# A jump of this many SoC points above the running estimate means the pack is on
# the charger (or was swapped), so the monotonic clamp is released.
CHARGE_DETECT_PCT = 5.0

# ...and must persist for this many consecutive updates before it is believed.
CHARGE_DETECT_SAMPLES = 30


def _pack_to_cell(volts: float) -> float:
    """Scale a reported pack voltage onto the per-cell curve.

    PACK_FULL_V is defined to sit at 4.20 V/cell, so a pack that reads high by a
    constant gain still lands on the right part of the curve.
    """
    return volts * (4.20 * CELLS / PACK_FULL_V) / CELLS


def voltage_to_soc(volts: float) -> float:
    """Interpolate an open-circuit pack voltage onto the SoC curve."""
    v = _pack_to_cell(volts)
    if v <= _OCV_TABLE_CELL[0][0]:
        return 0.0
    if v >= _OCV_TABLE_CELL[-1][0]:
        return 100.0
    for (v_lo, s_lo), (v_hi, s_hi) in zip(_OCV_TABLE_CELL, _OCV_TABLE_CELL[1:]):
        if v <= v_hi:
            frac = (v - v_lo) / (v_hi - v_lo)
            return s_lo + frac * (s_hi - s_lo)
    return 100.0


class BatteryEstimator:
    """Filtered, load-compensated SoC estimate.

    Call :meth:`update` with each voltage sample (cheap; safe at 10 Hz) and read
    :attr:`percent` / :attr:`voltage_filtered` whenever telemetry is built.
    """

    def __init__(self, r_int: float = R_INT_OHMS, tau_s: float = TAU_S):
        self.r_int = r_int
        self.tau_s = tau_s
        self._v_ema: float | None = None
        self._i_ema: float = 0.0
        self._last_t: float | None = None
        self._soc: float | None = None
        self._rise_count = 0

    # -- inputs -------------------------------------------------------------
    def update(self, volts: float, current_a: float = 0.0, now: float | None = None) -> float:
        """Feed one terminal-voltage sample.  Returns the current SoC percent.

        ``current_a`` is the estimated pack draw used for sag compensation; pass
        0 when unknown and the estimate simply reads a little pessimistic while
        the motors are running.
        """
        if not volts or volts <= 0.1:      # Rosmaster sends 0 before the first report
            return self.percent
        now = time.monotonic() if now is None else now

        # 1. EMA over the raw samples.  Averaging many quantised readings
        #    recovers resolution below the 0.1 V LSB.
        current_a = max(0.0, current_a)
        if self._v_ema is None or self._last_t is None:
            self._v_ema = volts
            self._i_ema = current_a
        else:
            dt = max(0.0, now - self._last_t)
            alpha = 1.0 - math.exp(-dt / self.tau_s) if self.tau_s > 0 else 1.0
            self._v_ema += alpha * (volts - self._v_ema)
            # The current must be filtered on the *same* time constant: a raw
            # sag term would jump the instant the motors start while the voltage
            # EMA is still lagging, briefly inflating the estimate.
            self._i_ema += alpha * (current_a - self._i_ema)
        self._last_t = now

        # 2. Undo load sag to approximate the open-circuit voltage.
        sag = min(MAX_SAG_COMP_V, self._i_ema * self.r_int)
        v_oc = self._v_ema + sag

        soc = voltage_to_soc(v_oc)

        # 3. A discharging pack's SoC only falls.  Without this the gauge
        #    bounces up every time the robot stops and the pack recovers.
        if self._soc is None:
            self._soc = soc
        elif soc < self._soc:
            self._soc = soc
            self._rise_count = 0
        elif soc > self._soc + CHARGE_DETECT_PCT:
            # Only re-home upward once the rise persists — a transient must not
            # be mistaken for a charger or a pack swap.
            self._rise_count += 1
            if self._rise_count >= CHARGE_DETECT_SAMPLES:
                self._soc = soc
                self._rise_count = 0
        else:
            self._rise_count = 0
        return self._soc

    # -- outputs ------------------------------------------------------------
    @property
    def percent(self) -> float:
        return 100.0 if self._soc is None else max(0.0, min(100.0, self._soc))

    @property
    def voltage_filtered(self) -> float:
        return 12.6 if self._v_ema is None else self._v_ema

    @property
    def ready(self) -> bool:
        return self._soc is not None
