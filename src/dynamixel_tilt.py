#!/usr/bin/env python3
"""Lidar tilt-mount control for the XL430-W250-T / OpenRB-150 replacement.

Same job as lidar_tilt.py (see that file for the full rationale): the mount
has no absolute notion of "level", so it is captured once, physically level,
into config/lidar_tilt_calibration_dynamixel.json.

    python3 src/dynamixel_tilt.py --status            # read-only
    python3 src/dynamixel_tilt.py --calibrate         # store current pos as level
    python3 src/dynamixel_tilt.py --home              # drive back to level

This is a SEPARATE calibration file and a SEPARATE script from lidar_tilt.py
on purpose -- the LX-16A stays the live driver (wired into x3_lidar_home.
service) until the XL430 is physically mounted and this script has been
exercised by hand. Do not point both scripts at the same servo bus, and do
not enable a Dynamixel systemd unit until the LX-16A one is disabled.

UNVERIFIED until hardware is in hand (mirrors lidar_tilt.py's tilt_direction
flag): whether the XL430's closed-loop position control needs the LX-16A's
overshoot trim (home() below skips that -- Dynamixel position PID should not
need it, but this has not been watched on real hardware yet).
"""

import argparse
import json
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dynamixel_servo import (XL430, DynamixelError, DEG_PER_COUNT,  # noqa: E402
                              COUNTS_PER_DEG, COUNTS_PER_REV,
                              OPERATING_MODE_POSITION,
                              deg_s_to_profile_velocity)

DEFAULT_CALIB = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config",
    "lidar_tilt_calibration_dynamixel.json")
DEFAULT_ID = 1               # Robotis factory default; change if reassigned
DEFAULT_PORT = "/dev/openrb150"

# How far off level we tolerate before bothering to move it back.
# The XL430's *resolution* is 0.088 deg/count, but its measured closed-loop
# accuracy on this mount is not: it settles 7-11 counts short of any goal,
# always trailing the direction of travel, steady for at least 4 s with PWM
# still applied. Sweeping Position I Gain 0 -> 400 changes nothing, so this is
# gearbox friction/backlash, not PID droop (measured 2026-08-28). 12 counts
# = 1.05 deg accepts that; anything tighter warns on every single boot.
# If a sweep ever needs better, always approach from the same direction --
# the hysteresis is directional, so one-sided approach cancels it.
DEFAULT_TOLERANCE_COUNTS = 12
HOMING_SPEED_DEG_S = 30.0
SETTLE_TIMEOUT_S = 5.0

log = logging.getLogger("dynamixel_tilt")


def load_calibration(path=DEFAULT_CALIB):
    with open(path) as f:
        cal = json.load(f)
    if "horizontal_counts" not in cal:
        raise ValueError("%s has no horizontal_counts" % path)
    return cal


def _connect(port, wait_device=0.0):
    """Open the servo bus, optionally waiting for udev to create the node."""
    deadline = time.monotonic() + wait_device
    while True:
        if os.path.exists(port):
            try:
                return XL430(port)
            except Exception as e:
                if time.monotonic() >= deadline:
                    raise DynamixelError("cannot open %s: %s" % (port, e))
        elif time.monotonic() >= deadline:
            raise DynamixelError("%s does not exist" % port)
        time.sleep(0.5)


def read_state(servo, servo_id):
    """Everything worth logging about the servo, best-effort."""
    state = {"pos_counts": servo.read_pos(servo_id)}
    state["pos_deg"] = round(state["pos_counts"] * DEG_PER_COUNT, 2)
    for key, fn in (("temp_C", servo.read_temp), ("vin_V", servo.read_vin),
                    ("load", servo.read_load), ("loaded", servo.is_loaded),
                    ("moving", servo.read_moving),
                    ("hardware_error", servo.read_hardware_error)):
        try:
            state[key] = fn(servo_id)
        except DynamixelError:
            state[key] = None
    try:
        state["position_limit_counts"] = list(
            servo.read_position_limits(servo_id))
    except DynamixelError:
        state["position_limit_counts"] = None
    return state


def calibrate(servo, servo_id, port, path, note="", counts=None):
    """Record horizontal.

    By default this captures wherever the mount is physically sitting, which
    is the right thing when you have just levelled it by hand. Pass `counts`
    to pin the zero to a *commanded* value instead: the servo settles ~10
    counts short of any goal (see DEFAULT_TOLERANCE_COUNTS), so recording the
    measured position after a move to 2048 would bake that error into the zero
    and drift it a little further on every re-run.
    """
    samples = [servo.read_pos(servo_id) for _ in range(15)]
    measured = int(round(sum(samples) / float(len(samples))))
    counts = measured if counts is None else int(counts)
    spread = max(samples) - min(samples)
    if spread > 4:
        log.warning("position is unsteady (spread %d counts = %.2f deg) -- "
                    "is the mount still settling?", spread,
                    spread * DEG_PER_COUNT)

    state = read_state(servo, servo_id)
    if counts != measured:
        log.info("pinning zero to the commanded %d counts (servo is resting "
                 "at %d, %+d off -- backlash, not a calibration error)",
                 counts, measured, measured - counts)
    elif state.get("loaded"):
        log.warning("torque is ON: this records the servo's held target, not "
                    "necessarily where you physically set the mount")

    cal = {
        "_comment": "Servo counts for a level (horizontal) lidar scan plane. "
                    "Captured with the mount physically level; re-run "
                    "src/dynamixel_tilt.py --calibrate after any remount.",
        "horizontal_counts": counts,
        "horizontal_deg": round(counts * DEG_PER_COUNT, 2),
        "servo_id": servo_id,
        "port": port,
        "counts_per_deg": round(COUNTS_PER_DEG, 4),
        "tolerance_counts": DEFAULT_TOLERANCE_COUNTS,
        "tilt_direction": None,
        "_tilt_direction_comment":
            "+1 if increasing counts pitches the lidar nose-DOWN (the URDF's "
            "positive lidar_tilt_joint), -1 if nose-up. Unverified -- angle "
            "commands stay disabled until someone watches it move.",
        "calibrated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "calibrated_on": os.uname().nodename,
        "sample_spread_counts": spread,
        "measured_counts_at_calibration": measured,
        "servo_state_at_calibration": state,
    }
    if note:
        cal["note"] = note

    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cal, f, indent=2, sort_keys=False)
        f.write("\n")
    os.replace(tmp, path)
    log.info("horizontal = %d counts (%.2f deg), spread %d -> %s",
             counts, counts * DEG_PER_COUNT, spread, path)
    return cal


def home(servo, servo_id, cal, tolerance=None, torque=True, verify=True):
    """Drive the mount back to the calibrated horizontal position."""
    target = int(cal["horizontal_counts"])
    tol = tolerance if tolerance is not None else int(
        cal.get("tolerance_counts", DEFAULT_TOLERANCE_COUNTS))

    try:
        limits = list(servo.read_position_limits(servo_id))
    except DynamixelError:
        limits = cal.get("servo_state_at_calibration", {}).get(
            "position_limit_counts")
    if limits and not (limits[0] <= target <= limits[1]):
        raise DynamixelError(
            "calibrated horizontal %d is outside the servo's position limits "
            "%s -- refusing to move" % (target, limits))

    pos = servo.read_pos(servo_id)
    delta = target - pos
    if abs(delta) <= tol:
        log.info("already level: %d counts, %+d from horizontal (%.2f deg)",
                 pos, delta, delta * DEG_PER_COUNT)
        if torque:
            servo.set_load(servo_id, True)
            servo.move(servo_id, target,
                       deg_s_to_profile_velocity(HOMING_SPEED_DEG_S))
        return {"moved": False, "start_counts": pos, "target_counts": target}

    log.info("homing: %d -> %d counts (%+.2f deg)",
             pos, target, delta * DEG_PER_COUNT)
    servo.set_load(servo_id, True)
    servo.move(servo_id, target, deg_s_to_profile_velocity(HOMING_SPEED_DEG_S))

    result = {"moved": True, "start_counts": pos, "target_counts": target}
    if verify:
        settled = servo.wait_until_settled(servo_id, SETTLE_TIMEOUT_S)
        final = servo.read_pos(servo_id)
        result["settled"] = settled
        result["final_counts"] = final
        err = final - target
        if abs(err) <= tol:
            log.info("homed: %d counts (%+d from target)", final, err)
        else:
            log.warning("homing stopped %+d counts (%.2f deg) from horizontal "
                        "-- %s", err, err * DEG_PER_COUNT,
                        "did not settle within timeout"
                        if not settled else
                        "well past the %d-count backlash band, so this is not "
                        "the usual hysteresis: check for a mechanical bind or "
                        "torque loss" % tol)
    if not torque:
        servo.set_load(servo_id, False)
        result["torque"] = "released"
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--id", type=int, default=None,
                    help="servo bus id (default: from the calibration file)")
    ap.add_argument("--calib-file", default=os.environ.get(
        "X3_LIDAR_TILT_CALIB_DXL", DEFAULT_CALIB))
    ap.add_argument("--wait-device", type=float, default=0.0, metavar="SEC",
                    help="wait up to SEC for the serial device (boot use)")

    ap.add_argument("--status", action="store_true", help="read-only dump")
    ap.add_argument("--calibrate", action="store_true",
                    help="store the current position as horizontal")
    ap.add_argument("--home", action="store_true",
                    help="drive back to the calibrated horizontal")
    ap.add_argument("--goto-counts", type=int, metavar="N",
                    help="move to a raw servo count (0-4095), for testing")
    ap.add_argument("--release", action="store_true",
                    help="release holding torque when done")
    ap.add_argument("--tolerance", type=int, default=None, metavar="COUNTS")
    ap.add_argument("--note", default="", help="note to store with --calibrate")
    ap.add_argument("--json", action="store_true", help="machine-readable out")
    ap.add_argument("--strict", action="store_true",
                    help="exit non-zero if the servo is unreachable")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    if not (args.status or args.calibrate or args.home
            or args.goto_counts is not None or args.release):
        args.status = True

    cal = None
    if not args.calibrate:
        try:
            cal = load_calibration(args.calib_file)
        except (OSError, ValueError) as e:
            if args.home:
                log.error("no usable calibration (%s) -- run --calibrate with "
                          "the mount level first", e)
                return 2
    servo_id = args.id or (cal or {}).get("servo_id", DEFAULT_ID)

    try:
        servo = _connect(args.port, args.wait_device)
    except DynamixelError as e:
        log.error("%s", e)
        return 1 if args.strict else 0

    out = {"port": args.port, "servo_id": servo_id}
    try:
        if servo.ping(servo_id) is None:
            log.error("servo id %d did not answer on %s (powered? baud rate "
                      "57600? OpenRB-150 still in passthrough mode?)",
                      servo_id, args.port)
            return 1 if args.strict else 0

        # Position control mode must be set with torque OFF, and is a one-time
        # thing per servo -- cheap to re-assert every run.
        try:
            if servo.is_loaded(servo_id):
                servo.set_load(servo_id, False)
            servo.set_operating_mode(servo_id, OPERATING_MODE_POSITION)
        except DynamixelError as e:
            log.warning("could not confirm/set position-control mode: %s", e)

        if args.calibrate:
            out["calibration"] = calibrate(servo, servo_id, args.port,
                                           args.calib_file, args.note)
        if args.goto_counts is not None:
            servo.set_load(servo_id, True)
            servo.move(servo_id, args.goto_counts,
                      deg_s_to_profile_velocity(HOMING_SPEED_DEG_S))
            servo.wait_until_settled(servo_id, SETTLE_TIMEOUT_S)
            out["goto"] = {"target_counts": args.goto_counts,
                           "final_counts": servo.read_pos(servo_id)}
            log.info("moved to %d counts (read back %d)", args.goto_counts,
                     out["goto"]["final_counts"])
        if args.home:
            out["home"] = home(servo, servo_id, cal or out["calibration"],
                               tolerance=args.tolerance,
                               torque=not args.release)
        if args.release and not args.home:
            servo.set_load(servo_id, False)
            log.info("holding torque released")
        if args.status:
            state = read_state(servo, servo_id)
            out["state"] = state
            if cal:
                off = state["pos_counts"] - int(cal["horizontal_counts"])
                out["offset_from_horizontal_counts"] = off
                out["offset_from_horizontal_deg"] = round(
                    off * DEG_PER_COUNT, 2)
            if not args.json:
                log.info("servo %d on %s", servo_id, args.port)
                for k, v in state.items():
                    log.info("  %-22s %s", k, v)
                if cal:
                    log.info("  %-22s %d counts (%.2f deg)",
                             "horizontal", int(cal["horizontal_counts"]),
                             int(cal["horizontal_counts"]) * DEG_PER_COUNT)
                    log.info("  %-22s %+d counts (%+.2f deg)",
                             "off horizontal", out[
                                 "offset_from_horizontal_counts"],
                             out["offset_from_horizontal_deg"])
    except DynamixelError as e:
        log.error("%s", e)
        return 1 if args.strict else 0
    finally:
        servo.close()

    if args.json:
        print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
