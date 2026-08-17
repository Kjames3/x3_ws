"""Wire SoCState persistence + OCV-only fallback into server_x3.py.

Idempotent: refuses to double-apply.
"""
import os, py_compile, shutil, sys, time

S = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/x3_ws/src/server_x3.py")
s = open(S).read()
orig = s

if "SoCState" in s:
    print("already applied; nothing to do")
    sys.exit(0)

# --- A. import -------------------------------------------------------------
a_old = "from battery_log import BatteryLogger\n"
a_new = "from battery_log import BatteryLogger, SoCState\n"
assert s.count(a_old) == 1, "import site not found"
s = s.replace(a_old, a_new)

# --- B. module global ------------------------------------------------------
b_old = "batt_logger      = None    # BatteryLogger; always-on CSV discharge trace\n"
b_new = (b_old +
         "batt_state       = None    # SoCState; persists the coulomb counter across restarts\n")
assert s.count(b_old) == 1, "global site not found"
s = s.replace(b_old, b_new)

c_old = "    global batt_est, batt_logger\n"
c_new = "    global batt_est, batt_logger, batt_state\n"
assert s.count(c_old) == 1, "init-globals site not found"
s = s.replace(c_old, c_new)

# --- C. construction -------------------------------------------------------
d_old = '''    # The OCV curve has two calibrations: the INA226 measures the pack
    # directly so it uses the 12.6 V nameplate, while the Rosmaster ADC
    # reads high and needs the stretched module default (pack_full_v=None).
    batt_est = BatteryEstimator(
        pack_full_v=PACK_FULL_V_INA226 if ina226 is not None else None,
        pack_empty_v=PACK_EMPTY_V_INA226 if ina226 is not None else None,
    )
    logger.info(
        f"BatteryEstimator: OCV curve, pack_full_v="
        f"{batt_est.pack_full_v or 'Rosmaster default'}"
    )
'''
d_new = '''    # SoC is coulomb-counted from the INA226 and only anchored by the OCV
    # curve where the curve is steep -- this pack puts 70% of its charge into
    # 300 mV, so voltage alone cannot gauge it (see src/battery.py).
    #
    # Without the INA226 there is no current to integrate, so the estimator is
    # put in explicit OCV-only mode: a coulomb counter fed a constant 0 A would
    # freeze the gauge at its seed while the pack quietly drained.
    _batt_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "logs", "battery")
    _restored = None
    if ina226 is not None:
        batt_state = SoCState(os.path.join(_batt_dir, "soc_state.json"))
        _v_now = ina226.get_voltage()
        if _v_now:
            _restored = batt_state.load(_v_now)
    batt_est = BatteryEstimator(
        pack_full_v=PACK_FULL_V_INA226 if ina226 is not None else None,
        pack_empty_v=PACK_EMPTY_V_INA226 if ina226 is not None else None,
        capacity_ah=PACK_CAPACITY_AH if ina226 is not None else None,
        initial_charge_ah=_restored,
    )
    logger.info(
        "BatteryEstimator: %s%s" % (
            "coulomb counting + OCV anchor" if ina226 is not None
            else "OCV curve only (no current sensor)",
            f", restored {_restored:.3f} Ah" if _restored is not None else "",
        )
    )
'''
assert s.count(d_old) == 1, "construction site not found"
s = s.replace(d_old, d_new)

e_old = "from battery import BatteryEstimator, PACK_FULL_V_INA226, PACK_EMPTY_V_INA226\n"
e_new = ("from battery import (BatteryEstimator, PACK_FULL_V_INA226,\n"
         "                     PACK_EMPTY_V_INA226, PACK_CAPACITY_AH)\n")
assert s.count(e_old) == 1, "battery import site not found"
s = s.replace(e_old, e_new)

# --- D. persist on each sample --------------------------------------------
f_old = '''                    if batt_est is not None:
                        batt_est.update(_batt_cache_v, _batt_cache_i, now)
                        if batt_logger is not None:
                            batt_logger.log(_batt_cache_v, _batt_cache_i,
                                            batt_est.percent)
'''
f_new = '''                    if batt_est is not None:
                        batt_est.update(_batt_cache_v, _batt_cache_i, now)
                        if batt_logger is not None:
                            batt_logger.log(_batt_cache_v, _batt_cache_i,
                                            batt_est.percent)
                        if batt_state is not None and batt_est.charge_ah is not None:
                            batt_state.save(batt_est.charge_ah, _batt_cache_v)
'''
n = s.count(f_old)
assert n == 2, f"expected 2 sample sites, found {n}"
s = s.replace(f_old, f_new)

g_old = "    global _batt_cache_v, _batt_cache_i, _batt_cache_time  # P9\n    global batt_logger\n"
g_new = "    global _batt_cache_v, _batt_cache_i, _batt_cache_time  # P9\n    global batt_logger, batt_state\n"
assert s.count(g_old) == 1, "telemetry-globals site not found"
s = s.replace(g_old, g_new)

# --- E. no-INA226 percentage should still use the fitted curve -------------
h_old = '''                est_current = 0.5 + (avg_pwr * 6.0)
                est_watts   = batt_v * est_current
                batt_pct    = max(0.0, min(100.0, (batt_v - 8.1) / (12.6 - 8.1) * 100.0))
'''
h_new = '''                est_current = 0.5 + (avg_pwr * 6.0)
                est_watts   = batt_v * est_current
                # Still the measured LiFePO4 curve, just uncompensated: the old
                # linear (v-8.1)/(12.6-8.1) map called a 12.6 V pack full when
                # this one is barely half there.
                batt_pct    = (batt_est.percent
                               if batt_est is not None and batt_est.ready
                               else max(0.0, min(100.0, (batt_v - 10.44) /
                                                 (13.16 - 10.44) * 100.0)))
'''
assert s.count(h_old) == 1, "fallback percentage site not found"
s = s.replace(h_old, h_new)

# --- F. flush state on shutdown -------------------------------------------
i_old = "    if batt_logger: batt_logger.close()\n"
i_new = ('''    if batt_logger: batt_logger.close()
    if batt_state is not None and batt_est is not None and batt_est.charge_ah is not None:
        batt_state.min_write_interval_s = 0.0   # a clean shutdown always records
        batt_state.save(batt_est.charge_ah, batt_est.voltage_filtered)
''')
assert s.count(i_old) == 1, "cleanup site not found"
s = s.replace(i_old, i_new)

assert s != orig
bak = S + ".bak-" + time.strftime("%Y%m%d-%H%M%S")
shutil.copy2(S, bak)
open(S, "w").write(s)
py_compile.compile(S, doraise=True)
print("patched OK, backup at", bak)
