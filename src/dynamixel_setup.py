#!/usr/bin/env python3
"""One-shot commissioning for the XL430-W250-T on the OpenRB-150.

This is the script you run once, on the bench, when the servo comes out of
the box -- everything after that is dynamixel_tilt.py's job.

    python3 src/dynamixel_setup.py --identify         # what/where is it?
    python3 src/dynamixel_setup.py --set-id 1         # give it bus id 1
    python3 src/dynamixel_setup.py --zero             # go to 2048, call it 0
    python3 src/dynamixel_setup.py --commission       # all three, in order

Why 2048 is "zero": position-control mode spans 0..4095 counts over one turn,
so 2048 is dead centre and leaves +-180 deg of travel in both directions.
Homing there before bolting the lidar mount on means the tilt joint can never
run out of range on one side.

How zero is stored matters. There are two ways to make 2048 read as 0:

  1. SOFTWARE (default, what --zero does). 2048 is written to
     config/lidar_tilt_calibration_dynamixel.json as `horizontal_counts`, and
     dynamixel_tilt.py reports every position relative to it. Costs nothing,
     is version-controlled, and survives a servo swap.

  2. EEPROM (--write-homing-offset, opt-in). Writes Homing Offset = -2048 so
     the servo itself reports 0 at mechanical centre. This is NOT free: Goal
     Position is still clamped to Min/Max Position Limit (0..4095) in the
     offset frame, so the reachable arc becomes mechanical 2048..4095 -- you
     silently lose half the travel. Only do this if some downstream tool
     insists on reading a zeroed Present Position, and expect to widen the
     position limits by hand afterwards.

Nothing here is destructive except --set-id and --write-homing-offset, which
touch EEPROM, and the physical move to 2048, which is confirmed before it
runs unless --yes is passed. Keep a hand on the mount the first time.
"""

import argparse
import logging
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dynamixel_servo import (XL430, DynamixelError, BAUD, BAUD_CODES,  # noqa: E402
                             BAUD_SCAN_ORDER, COUNTS_PER_DEG, COUNTS_PER_REV,
                             DEG_PER_COUNT,
                             EXPECTED_MODEL, MAX_ID, OPERATING_MODE_POSITION,
                             PortHandler, deg_s_to_profile_velocity,
                             find_ports, model_name)
import dynamixel_tilt  # noqa: E402  (reuses its calibration-file writer)

CENTER_COUNTS = COUNTS_PER_REV // 2      # 2048
TARGET_ID = 1
MOVE_SPEED_DEG_S = 20.0                  # deliberately slow for a first move
SETTLE_TIMEOUT_S = 8.0
# A sweep waypoint is "reached" within this much. 10 counts = 0.88 deg -- loose
# enough that an unloaded servo's PID deadband does not read as a failure,
# tight enough to catch a slipping horn or a mis-scaled command.
SWEEP_TOL_COUNTS = 10
SWEEP_AMPLITUDE_DEG = 45.0
# ids worth checking when broadcast ping comes back empty. A factory servo is
# id 1; anything hand-set is usually low. --deep-scan sweeps all 253.
SHALLOW_IDS = tuple(range(0, 21))

log = logging.getLogger("dynamixel_setup")


# ---------------------------------------------------------------- discovery
def probe(port, baud, ids=SHALLOW_IDS):
    """Find every servo answering on one port at one baud rate.

    Returns [{"id":, "model":, "model_name":, "firmware":}, ...]. A port that
    cannot be opened at this baud yields [] rather than raising -- during a
    sweep that is an expected outcome, not an error.
    """
    try:
        servo = XL430(port, baud)
    except DynamixelError:
        return []
    try:
        found = servo.broadcast_ping()
        if not found:
            # Some USB bridges drop the reply burst from a broadcast; a
            # one-at-a-time sweep is slower but far more reliable.
            found = {}
            for i in ids:
                model = servo.ping(i)
                if model is not None:
                    found[i] = (model, None)
        out = []
        for i in sorted(found):
            model, fw = found[i]
            if fw is None:
                try:
                    fw = servo.read_firmware(i)
                except DynamixelError:
                    fw = None
            out.append({"id": i, "model": model,
                        "model_name": model_name(model), "firmware": fw})
        return out
    finally:
        servo.close()


def identify(ports=None, bauds=BAUD_SCAN_ORDER, ids=SHALLOW_IDS, stop_early=True):
    """Sweep ports x baud rates for X-series servos."""
    ports = ports or find_ports()
    if not ports:
        log.error("no candidate serial ports (%s). Is the OpenRB-150 plugged "
                  "in and powered? `dmesg | tail` after replugging.",
                  "/dev/openrb150, /dev/ttyACM*, /dev/ttyUSB*")
        return []

    results = []
    for port in ports:
        for baud in bauds:
            hits = probe(port, baud, ids)
            for h in hits:
                h["port"] = port
                h["baud"] = baud
                log.info("found id %d: %s (fw %s) on %s @ %d",
                         h["id"], h["model_name"], h["firmware"], port, baud)
                if h["model"] != EXPECTED_MODEL:
                    log.warning("  id %d is %s, not the XL430-W250-T (model "
                                "%d) this project expects", h["id"],
                                h["model_name"], EXPECTED_MODEL)
            results.extend(hits)
            if hits and stop_early:
                break   # a servo bus runs at one baud; no point sweeping on
    if not results:
        log.error("no servos answered on %s at any of %s.\n"
                  "  - is the servo powered from its own 12 V supply (USB "
                  "alone does not drive an XL430)?\n"
                  "  - is the OpenRB-150 still in DYNAMIXEL passthrough mode? "
                  "an uploaded Arduino sketch replaces it\n"
                  "  - try --deep-scan to sweep all 253 ids",
                  ", ".join(ports), ", ".join(str(b) for b in bauds))
    return results


# ------------------------------------------------------------------- id set
def set_id(port, baud, from_id, new_id, expect_single=True):
    """Rewrite one servo's bus id. Torque is forced off first (EEPROM rule)."""
    if not 0 <= new_id <= MAX_ID:
        raise ValueError("id must be 0..%d" % MAX_ID)

    with XL430(port, baud) as servo:
        present = probe_open(servo)
        if expect_single and len(present) > 1:
            raise DynamixelError(
                "%d servos on %s (%s) -- refusing to guess which to renumber. "
                "Unplug the others, or pass --from-id explicitly."
                % (len(present), port, ", ".join(str(i) for i in present)))
        if from_id not in present:
            raise DynamixelError("id %d did not answer on %s @ %d (saw %s)"
                                 % (from_id, port, baud,
                                    present or "nothing"))
        if new_id in present and new_id != from_id:
            raise DynamixelError(
                "id %d is already taken by another servo on this bus -- "
                "renumber that one first" % new_id)
        if from_id == new_id:
            log.info("id is already %d, nothing to do", new_id)
            return {"changed": False, "id": new_id}

        servo.set_load(from_id, False)
        servo.set_id(from_id, new_id)
        time.sleep(0.1)          # EEPROM write needs a moment to commit

        if servo.ping(new_id) is None:
            raise DynamixelError(
                "wrote id %d but it does not answer -- power-cycle the servo "
                "and re-run --identify before assuming the write failed"
                % new_id)
        if servo.ping(from_id) is not None:
            log.warning("old id %d still answers; is there a second servo on "
                        "the bus?", from_id)
        log.info("id %d -> %d (EEPROM, survives power-cycle)", from_id, new_id)
        return {"changed": True, "id": new_id, "previous_id": from_id}


def probe_open(servo, ids=SHALLOW_IDS):
    """Ids present on an already-open bus."""
    found = servo.broadcast_ping()
    if found:
        return sorted(found)
    return [i for i in ids if servo.ping(i) is not None]


# -------------------------------------------------------------------- zero
def goto_center(servo, servo_id, target=CENTER_COUNTS,
                speed_deg_s=MOVE_SPEED_DEG_S, assume_yes=False):
    """Put the servo in position mode and drive it to `target` counts."""
    # Operating Mode lives in EEPROM and is rejected while torque is on.
    if servo.is_loaded(servo_id):
        servo.set_load(servo_id, False)
    servo.set_operating_mode(servo_id, OPERATING_MODE_POSITION)

    start = servo.read_pos(servo_id)
    delta = target - start
    log.info("current position %d counts (%.1f deg); moving %+d counts "
             "(%+.1f deg) to %d at %.0f deg/s",
             start, start * DEG_PER_COUNT, delta, delta * DEG_PER_COUNT,
             target, speed_deg_s)
    if not assume_yes and not _confirm(
            "The servo is about to MOVE %+.1f deg. Clear of the mount? [y/N] "
            % (delta * DEG_PER_COUNT)):
        raise DynamixelError("move declined")

    servo.set_load(servo_id, True)
    servo.set_profile_acceleration(servo_id, 20)   # gentle ramp, ~2 s to speed
    servo.move(servo_id, target, deg_s_to_profile_velocity(speed_deg_s))
    settled = servo.wait_until_settled(servo_id, SETTLE_TIMEOUT_S)
    final = servo.read_pos(servo_id)
    err = final - target

    hw = servo.read_hardware_error(servo_id)
    if hw:
        log.error("hardware error status 0x%02x after the move (overload? "
                  "over-voltage? check the 12 V supply)", hw)
    if not settled:
        log.warning("still moving after %.0f s -- mechanical bind?",
                    SETTLE_TIMEOUT_S)
    elif abs(err) > 4:
        log.warning("settled %+d counts (%+.2f deg) off target -- the load is "
                    "back-driving the position PID", err, err * DEG_PER_COUNT)
    else:
        log.info("at %d counts (%+d from target)", final, err)
    return {"start_counts": start, "target_counts": target,
            "final_counts": final, "settled": settled, "error_counts": err,
            "hardware_error": hw}


def _move_traced(servo, servo_id, target, speed_deg_s, dwell_s=0.4,
                 timeout_s=SETTLE_TIMEOUT_S):
    """Move to `target` and sample the encoder on the way, so a waypoint that
    is reached can be told apart from one the servo never actually left for."""
    start = servo.read_pos(servo_id)
    t0 = time.monotonic()
    servo.move(servo_id, target, deg_s_to_profile_velocity(speed_deg_s))

    trace = []
    peak_load = 0
    started = False
    arrived = False
    # Arrival is judged on position error, not on the Moving flag alone: the
    # flag lags the goal write on the way out (see wait_until_settled) and a
    # slow profile may never clear Moving Threshold at all.
    while time.monotonic() - t0 < timeout_s:
        pos = servo.read_pos(servo_id)
        try:
            load = servo.read_load(servo_id)
        except DynamixelError:
            load = None
        elapsed = time.monotonic() - t0
        trace.append((round(elapsed, 3), pos, load))
        if load is not None:
            peak_load = max(peak_load, abs(load))
        moving = servo.read_moving(servo_id)
        if moving:
            started = True
        elif (abs(pos - target) <= SWEEP_TOL_COUNTS
                and (started or elapsed >= 0.5)):
            arrived = True
            break
        time.sleep(0.05)

    time.sleep(dwell_s)          # let the PID settle before judging the error
    final = servo.read_pos(servo_id)
    err = final - target
    travel = final - start
    hw = servo.read_hardware_error(servo_id)
    ok = arrived and abs(err) <= SWEEP_TOL_COUNTS and hw == 0
    log.info("  %5d counts (%+7.2f deg): reached %5d, err %+3d (%+.2f deg), "
             "travelled %+5d, peak load %d, %.1f s  %s",
             target, target * DEG_PER_COUNT, final, err, err * DEG_PER_COUNT,
             travel, peak_load, time.monotonic() - t0,
             "ok" if ok else ("TIMEOUT" if not arrived else "FAIL"))
    if hw:
        log.error("  hardware error status 0x%02x at this waypoint", hw)
    return {"target_counts": target, "final_counts": final,
            "error_counts": err, "travel_counts": travel,
            "peak_load": peak_load, "hardware_error": hw,
            "seconds": round(time.monotonic() - t0, 2), "ok": ok,
            "arrived": arrived,
            "samples": len(trace), "trace": trace}


def sweep(servo, servo_id, center, amplitude_deg=SWEEP_AMPLITUDE_DEG,
          speed_deg_s=MOVE_SPEED_DEG_S, cycles=1, dwell_s=0.4,
          assume_yes=False):
    """Functional check: centre -> +A -> centre -> -A -> centre.

    This is a *bench* test. It proves the servo takes position commands, moves
    the commanded distance in both directions, and comes back to where it
    started -- it says nothing about which direction is nose-up (that is
    tilt_direction, which still needs a human watching it).
    """
    amp_counts = int(round(amplitude_deg * COUNTS_PER_DEG))
    lo, hi = center - amp_counts, center + amp_counts
    if not (0 <= lo and hi < COUNTS_PER_REV):
        raise DynamixelError(
            "+-%.1f deg (%d counts) around %d runs outside 0..%d -- pick a "
            "smaller --sweep or re-centre first"
            % (amplitude_deg, amp_counts, center, COUNTS_PER_REV - 1))
    try:
        limits = servo.read_position_limits(servo_id)
        if not (limits[0] <= lo and hi <= limits[1]):
            raise DynamixelError(
                "sweep range %d..%d is outside the servo's position limits %s"
                % (lo, hi, list(limits)))
    except DynamixelError as e:
        if "position limits" in str(e):
            raise
        log.warning("could not read position limits: %s", e)

    if servo.is_loaded(servo_id):
        servo.set_load(servo_id, False)
    servo.set_operating_mode(servo_id, OPERATING_MODE_POSITION)

    start = servo.read_pos(servo_id)
    log.info("sweep: +-%.1f deg (%d counts) about %d -> range %d..%d, "
             "%d cycle(s) at %.0f deg/s; starting from %d",
             amplitude_deg, amp_counts, center, lo, hi, cycles, speed_deg_s,
             start)
    if not assume_yes and not _confirm(
            "The servo will SWEEP %+.1f/%.1f deg. Clear? [y/N] "
            % (amplitude_deg, -amplitude_deg)):
        raise DynamixelError("sweep declined")

    servo.set_load(servo_id, True)
    servo.set_profile_acceleration(servo_id, 20)

    legs = []
    try:
        # Start from centre so the first commanded leg is a known amplitude.
        legs.append(_move_traced(servo, servo_id, center, speed_deg_s, dwell_s))
        for c in range(cycles):
            if cycles > 1:
                log.info(" cycle %d/%d", c + 1, cycles)
            for target in (hi, center, lo, center):
                legs.append(_move_traced(servo, servo_id, target,
                                         speed_deg_s, dwell_s))
    finally:
        # Leave it holding centre, not wherever a failure stopped it.
        try:
            servo.move(servo_id, center, deg_s_to_profile_velocity(speed_deg_s))
            servo.wait_until_settled(servo_id, SETTLE_TIMEOUT_S)
        except DynamixelError:
            pass

    end = servo.read_pos(servo_id)
    worst = max((abs(l["error_counts"]) for l in legs), default=0)
    peak_load = max((l["peak_load"] for l in legs), default=0)
    # A servo whose horn is slipping still reports "reached" on every leg but
    # drifts across the sweep, so check the round trip independently.
    # ...measured from the first CENTRE arrival, not from wherever the servo
    # happened to be parked before the sweep started.
    round_trip = end - (legs[0]["final_counts"] if legs else start)
    failed = [l for l in legs if not l["ok"]]

    result = {"center_counts": center, "amplitude_deg": amplitude_deg,
              "amplitude_counts": amp_counts, "range_counts": [lo, hi],
              "cycles": cycles, "speed_deg_s": speed_deg_s,
              "start_counts": start, "end_counts": end,
              "center_reference_counts": legs[0]["final_counts"] if legs else start,
              "round_trip_counts": round_trip,
              "worst_error_counts": worst, "peak_load": peak_load,
              "temp_C": servo.read_temp(servo_id),
              "vin_V": servo.read_vin(servo_id),
              "hardware_error": servo.read_hardware_error(servo_id),
              "legs": legs, "failed_legs": len(failed)}
    result["passed"] = (not failed
                        and abs(round_trip) <= SWEEP_TOL_COUNTS
                        and result["hardware_error"] == 0)

    log.info("worst waypoint error %d counts (%.2f deg), round trip %+d, "
             "peak load %d, %.1f C, %.1f V",
             worst, worst * DEG_PER_COUNT, round_trip, peak_load,
             result["temp_C"], result["vin_V"])
    if result["passed"]:
        log.info("SWEEP PASSED")
    else:
        log.error("SWEEP FAILED: %d/%d waypoints missed, round trip %+d counts",
                  len(failed), len(legs), round_trip)
    return result


def write_homing_offset(servo, servo_id, center=CENTER_COUNTS):
    """EEPROM zero. Read the module docstring before using this."""
    if servo.is_loaded(servo_id):
        servo.set_load(servo_id, False)
    before = servo.read_homing_offset(servo_id)
    servo.set_homing_offset(servo_id, -center)
    time.sleep(0.1)
    after = servo.read_homing_offset(servo_id)
    pos = servo.read_pos(servo_id)
    if after != -center:
        raise DynamixelError("homing offset read back %d, expected %d"
                             % (after, -center))
    log.info("homing offset %d -> %d; present position now reads %d",
             before, after, pos)
    log.warning("reachable arc is now mechanical %d..%d counts only -- widen "
                "Min/Max Position Limit if you need the other half",
                center, COUNTS_PER_REV - 1)
    return {"previous": before, "offset": after, "present_position": pos}


def _confirm(prompt):
    if not sys.stdin.isatty():
        log.error("not a tty and --yes was not passed; refusing to move")
        return False
    try:
        return input(prompt).strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


# --------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--identify", action="store_true",
                    help="scan ports and baud rates for X-series servos")
    ap.add_argument("--set-id", type=int, nargs="?", const=TARGET_ID,
                    metavar="ID", help="assign a bus id (default %d)" % TARGET_ID)
    ap.add_argument("--from-id", type=int, default=None, metavar="ID",
                    help="the servo's current id (default: the only one found)")
    ap.add_argument("--zero", action="store_true",
                    help="move to %d counts and record it as zero"
                         % CENTER_COUNTS)
    ap.add_argument("--commission", action="store_true",
                    help="--identify, --set-id %d, then --zero" % TARGET_ID)
    ap.add_argument("--sweep", type=float, nargs="?", const=SWEEP_AMPLITUDE_DEG,
                    metavar="DEG",
                    help="functional check: sweep +-DEG (default %.0f) about "
                         "zero and verify every waypoint" % SWEEP_AMPLITUDE_DEG)
    ap.add_argument("--cycles", type=int, default=1, metavar="N",
                    help="repeat the sweep N times (default 1)")
    ap.add_argument("--dwell", type=float, default=0.4, metavar="SEC",
                    help="settle time at each waypoint (default 0.4)")
    ap.add_argument("--write-homing-offset", action="store_true",
                    help="also zero it in EEPROM (costs half the travel -- "
                         "read the docstring)")

    ap.add_argument("--port", default=None,
                    help="skip port discovery and use this one")
    ap.add_argument("--baud", type=int, default=None,
                    help="skip the baud sweep and use this rate")
    ap.add_argument("--center", type=int, default=CENTER_COUNTS, metavar="N",
                    help="counts to treat as zero (default %d)" % CENTER_COUNTS)
    ap.add_argument("--speed", type=float, default=MOVE_SPEED_DEG_S,
                    metavar="DEG_S", help="move speed (default %.0f deg/s)"
                                          % MOVE_SPEED_DEG_S)
    ap.add_argument("--calib-file", default=dynamixel_tilt.DEFAULT_CALIB,
                    help="where --zero writes the software zero")
    ap.add_argument("--deep-scan", action="store_true",
                    help="sweep all 253 ids instead of 0-20 (slow)")
    ap.add_argument("--yes", "-y", action="store_true",
                    help="do not prompt before physically moving the servo")
    ap.add_argument("--json", action="store_true", help="machine-readable out")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    if args.commission:
        args.identify = True
        if args.set_id is None:
            args.set_id = TARGET_ID
        args.zero = True
    if not (args.identify or args.zero or args.set_id is not None
            or args.write_homing_offset or args.sweep is not None):
        args.identify = True

    if PortHandler is None:
        log.error("dynamixel_sdk is not installed (pip3 install dynamixel-sdk)")
        return 1
    if not 0 <= args.center < COUNTS_PER_REV:
        log.error("--center must be 0..%d", COUNTS_PER_REV - 1)
        return 2

    ids = tuple(range(0, MAX_ID + 1)) if args.deep_scan else SHALLOW_IDS
    out = {}

    # --- locate the bus -------------------------------------------------
    found = identify(ports=[args.port] if args.port else None,
                     bauds=[args.baud] if args.baud else BAUD_SCAN_ORDER,
                     ids=ids)
    out["found"] = found
    if not found:
        return 1
    if len(found) > 1 and args.from_id is None and (
            args.set_id is not None or args.zero or args.sweep is not None):
        log.error("%d servos found -- pass --from-id to say which one",
                  len(found))
        return 2

    servo_id = args.from_id if args.from_id is not None else found[0]["id"]
    port = args.port or found[0]["port"]
    baud = args.baud or found[0]["baud"]

    try:
        # --- id ---------------------------------------------------------
        if args.set_id is not None:
            out["set_id"] = set_id(port, baud, servo_id, args.set_id,
                                   expect_single=args.from_id is None)
            servo_id = args.set_id

        # --- move + zero ------------------------------------------------
        if args.sweep is not None:
            # Sweep about the recorded zero when there is one -- that is the
            # centre the mount was built around, not necessarily 2048.
            center = args.center
            try:
                cal = dynamixel_tilt.load_calibration(args.calib_file)
                center = int(cal["horizontal_counts"])
                log.info("sweeping about the calibrated zero %d counts (%s)",
                         center, args.calib_file)
            except (OSError, ValueError):
                log.warning("no calibration at %s -- sweeping about --center "
                            "%d instead", args.calib_file, center)
            with XL430(port, baud) as servo:
                out["sweep"] = sweep(servo, servo_id, center, args.sweep,
                                     args.speed, args.cycles, args.dwell,
                                     args.yes)

        if args.zero or args.write_homing_offset:
            with XL430(port, baud) as servo:
                if args.zero:
                    out["move"] = goto_center(servo, servo_id, args.center,
                                              args.speed, args.yes)
                    out["zero"] = dynamixel_tilt.calibrate(
                        servo, servo_id, port, args.calib_file,
                        # Pin the zero to what we ASKED for. The servo stops a
                        # few counts short, and recording that reading would
                        # walk the zero further off on every --zero run.
                        counts=args.center,
                        note="Software zero set by dynamixel_setup.py --zero: "
                             "%d counts is position-mode centre, not a "
                             "physically levelled mount. Re-run "
                             "dynamixel_tilt.py --calibrate once the lidar is "
                             "bolted on and levelled." % args.center)
                if args.write_homing_offset:
                    out["homing_offset"] = write_homing_offset(
                        servo, servo_id, args.center)
    except (DynamixelError, ValueError) as e:
        log.error("%s", e)
        if args.json:
            out["error"] = str(e)
            print(json.dumps(out, indent=2))
        return 1

    if args.json:
        print(json.dumps(out, indent=2, default=str))
    else:
        log.info("done: servo id %d on %s @ %d, zero = %d counts",
                 servo_id, port, baud, args.center)
    if out.get("sweep") and not out["sweep"]["passed"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
