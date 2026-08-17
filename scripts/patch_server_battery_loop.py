"""Move battery sampling out of the client-gated telemetry loop.

The whole telemetry body sits behind `if connected_clients:`, so with no
browser open the pack was never sampled.  That was survivable for a voltage
lookup but not for a coulomb counter: integration would stop while the pack
kept draining, and the gauge would resume from a stale charge reading far too
high.  Sampling now runs in its own 1 Hz task that does not care about clients.

Idempotent.
"""
import os, py_compile, shutil, sys, time

S = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/x3_ws/src/server_x3.py")
s = open(S).read()
orig = s

if "async def battery_loop" in s:
    print("already applied; nothing to do")
    sys.exit(0)

# --- 1. the two in-telemetry sampling blocks become plain cache reads -------
old_drive = '''                if now - _batt_cache_time >= 1.0:  # Throttle to 1Hz (Idea 67)
                    if ina226 is not None:
                        _batt_cache_v = ina226.get_voltage() or drive.get_battery_voltage()
                        _batt_cache_i = ina226.get_current()
                    else:
                        _batt_cache_v = drive.get_battery_voltage()
                    _batt_cache_time = now
                    # One update per real measurement (not per loop pass), so the
                    # estimator's sample-counted charger detection stays meaningful.
                    if batt_est is not None:
                        batt_est.update(_batt_cache_v, _batt_cache_i, now)
                        if batt_logger is not None:
                            batt_logger.log(_batt_cache_v, _batt_cache_i,
                                            batt_est.percent)
                        if batt_state is not None and batt_est.charge_ah is not None:
                            batt_state.save(batt_est.charge_ah, _batt_cache_v)
                batt_v = _batt_cache_v
'''
new_drive = '''                # Sampled by battery_loop(), which runs whether or not a
                # browser is connected; this is just the latest cached value.
                batt_v = _batt_cache_v
'''
assert s.count(old_drive) == 1, "drive sampling block not found"
s = s.replace(old_drive, new_drive)

old_ros = '''                if now - _batt_cache_time >= 1.0:
                    if ina226 is not None:
                        _batt_cache_v = ina226.get_voltage() or ros_board.get_battery_voltage()
                        _batt_cache_i = ina226.get_current()
                    else:
                        _batt_cache_v = ros_board.get_battery_voltage()
                    _batt_cache_time = now
                    if batt_est is not None:
                        batt_est.update(_batt_cache_v, _batt_cache_i, now)
                        if batt_logger is not None:
                            batt_logger.log(_batt_cache_v, _batt_cache_i,
                                            batt_est.percent)
                        if batt_state is not None and batt_est.charge_ah is not None:
                            batt_state.save(batt_est.charge_ah, _batt_cache_v)
                batt_v = _batt_cache_v
'''
new_ros = '''                batt_v = _batt_cache_v
'''
assert s.count(old_ros) == 1, "ros_board sampling block not found"
s = s.replace(old_ros, new_ros)

# --- 2. the new task -------------------------------------------------------
anchor = "async def main():\n"
loop_src = '''async def battery_loop():
    """Sample the pack at 1 Hz, independently of any connected client.

    Deliberately not part of broadcast_loop(): that loop's body is gated on
    `connected_clients`, so with the GUI closed the pack went unsampled.  A
    coulomb counter cannot tolerate that -- the charge drawn while nobody was
    watching would simply never be subtracted.  It also means an unattended
    robot still produces a complete discharge trace, which is the whole point
    of the logger being always-on.
    """
    global _batt_cache_v, _batt_cache_i, _batt_cache_time
    global batt_est, batt_logger, batt_state

    while not _shutting_down:
        try:
            now = time.time()
            v = None
            if ina226 is not None:
                v = ina226.get_voltage()
                _batt_cache_i = ina226.get_current()
            if not v:
                if drive is not None:
                    v = drive.get_battery_voltage()
                elif ros_board is not None:
                    v = ros_board.get_battery_voltage()
            if v:
                _batt_cache_v = v
                _batt_cache_time = now
                if batt_est is not None:
                    batt_est.update(_batt_cache_v, _batt_cache_i, now)
                    if batt_logger is not None:
                        batt_logger.log(_batt_cache_v, _batt_cache_i, batt_est.percent)
                    if batt_state is not None and batt_est.charge_ah is not None:
                        batt_state.save(batt_est.charge_ah, _batt_cache_v)
        except Exception as e:
            logger.error(f"battery_loop: {e}")
        await asyncio.sleep(1.0)


'''
assert s.count(anchor) == 1, "main() not found"
s = s.replace(anchor, loop_src + anchor)

# --- 3. register it --------------------------------------------------------
old_gather = "await asyncio.gather(broadcast_loop(), motion_loop(), oled_loop(), map_push_loop())"
new_gather = ("await asyncio.gather(broadcast_loop(), motion_loop(), oled_loop(),\n"
              "                             map_push_loop(), battery_loop())")
assert s.count(old_gather) == 1, "gather site not found"
s = s.replace(old_gather, new_gather)

assert s != orig
bak = S + ".bak-" + time.strftime("%Y%m%d-%H%M%S")
shutil.copy2(S, bak)
open(S, "w").write(s)
py_compile.compile(S, doraise=True)
print("patched OK, backup at", bak)
