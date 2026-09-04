#!/usr/bin/env python3
"""Lidar tilt-mount control: calibrate 'horizontal' and home to it.

The YDLidar sits on an LX-16A-driven carousel (see yahboomcar_description's
`lidar_tilt_joint`). The servo has no absolute notion of "level" -- horizontal
is wherever the mount happens to be bolted -- so it is captured once, with the
mount physically level, and stored in config/lidar_tilt_calibration.json.

    python3 src/lidar_tilt.py --status            # read the servo, no writes
    python3 src/lidar_tilt.py --calibrate         # store current pos as level
    python3 src/lidar_tilt.py --home              # drive back to level

--home runs from x3_lidar_home.service at boot, which is why it tolerates the
serial device not existing yet (--wait-device) and never exits non-zero for a
missing servo unless --strict is given.
"""

import argparse
import json
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lx16a_servo import LX16A, LX16AError, DEG_PER_COUNT, COUNTS_PER_DEG  # noqa: E402

DEFAULT_CALIB = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config",
    "lidar_tilt_calibration.json")
DEFAULT_ID = 3
DEFAULT_PORT = "/dev/lx16a"

# how far off level we tolerate before bothering to move it back.
# 3 counts = 0.72 deg; the servo's own read jitter is +/-1 count.
DEFAULT_TOLERANCE_COUNTS = 3
HOMING_SPEED_DEG_S = 30.0
# closed-loop trim after the coarse move (see home())
TRIM_DEADBAND_COUNTS = 1
MAX_TRIM_STEPS = 6
# a trim step smaller than the servo's own deadband is expected to do nothing,
# so only call it a mechanical stop once the command is this far past target
STALL_PUSH_COUNTS = 5

log = logging.getLogger("lidar_tilt")


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
                return LX16A(port)
            except Exception as e:
                if time.monotonic() >= deadline:
                    raise LX16AError("cannot open %s: %s" % (port, e))
        elif time.monotonic() >= deadline:
            raise LX16AError("%s does not exist" % port)
        time.sleep(0.5)


def read_state(servo, servo_id):
    """Everything worth logging about the servo, best-effort."""
    state = {"pos_counts": servo.read_pos(servo_id)}
    state["pos_deg"] = round(state["pos_counts"] * DEG_PER_COUNT, 2)
    for key, fn in (("temp_C", servo.read_temp), ("vin_V", servo.read_vin),
                    ("offset_counts", servo.read_offset),
                    ("loaded", servo.is_loaded)):
        try:
            state[key] = fn(servo_id)
        except LX16AError:
            state[key] = None
    try:
        state["angle_limit_counts"] = list(servo.read_angle_limits(servo_id))
    except LX16AError:
        state["angle_limit_counts"] = None
    return state


def calibrate(servo, servo_id, port, path, note=""):
    """Record wherever the mount is sitting right now as horizontal."""
    samples = [servo.read_pos(servo_id) for _ in range(15)]
    counts = int(round(sum(samples) / float(len(samples))))
    spread = max(samples) - min(samples)
    if spread > 4:
        log.warning("position is unsteady (spread %d counts = %.2f deg) -- "
                    "is the mount still settling?", spread,
                    spread * DEG_PER_COUNT)

    state = read_state(servo, servo_id)
    if state.get("loaded"):
        log.warning("torque is ON: this records the servo's held target, not "
                    "necessarily where you physically set the mount")

    cal = {
        "_comment": "Servo counts for a level (horizontal) lidar scan plane. "
                    "Captured with the mount physically level; re-run "
                    "src/lidar_tilt.py --calibrate after any remount.",
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
        limits = list(servo.read_angle_limits(servo_id))
    except LX16AError:
        limits = cal.get("servo_state_at_calibration", {}).get(
            "angle_limit_counts")
    if limits and not (limits[0] <= target <= limits[1]):
        raise LX16AError(
            "calibrated horizontal %d is outside the servo's angle limits %s "
            "-- refusing to move" % (target, limits))

    pos = servo.read_pos(servo_id)
    delta = target - pos
    if abs(delta) <= tol:
        log.info("already level: %d counts, %+d from horizontal (%.2f deg)",
                 pos, delta, delta * DEG_PER_COUNT)
        if torque:
            # Re-assert the target so the servo holds level instead of drifting;
            # commanding the position it is already at does not move it.
            servo.move(servo_id, target, 200)
        return {"moved": False, "start_counts": pos, "target_counts": target}

    move_ms = int(min(3000, max(300,
                  abs(delta) * DEG_PER_COUNT / HOMING_SPEED_DEG_S * 1000)))
    log.info("homing: %d -> %d counts (%+.2f deg) over %d ms",
             pos, target, delta * DEG_PER_COUNT, move_ms)
    servo.move(servo_id, target, move_ms)

    result = {"moved": True, "start_counts": pos, "target_counts": target,
              "move_ms": move_ms}
    if verify:
        # The LX-16A stops ~5 counts short of whatever it is commanded (its own
        # deadband plus gear backlash), and the shortfall follows the direction
        # of travel -- so homing from above and homing from below would settle
        # a degree and a half apart. Trim it out by walking the *command* past
        # the target until the encoder reads the target, which makes the rest
        # position repeatable no matter which side we came from.
        time.sleep(move_ms / 1000.0 + 0.4)
        final = servo.read_pos(servo_id)
        command = target
        for _ in range(MAX_TRIM_STEPS):
            err = final - target
            if abs(err) <= TRIM_DEADBAND_COUNTS:
                break
            command = int(max(0, min(1000, command - err)))
            if limits:
                command = max(limits[0], min(limits[1], command))
            if command == target:
                break                      # clamped; cannot push any further
            log.debug("trim: %+d off, re-commanding %d", err, command)
            servo.move(servo_id, command, 250)
            time.sleep(0.55)
            moved = servo.read_pos(servo_id)
            stalled = abs(moved - final) <= TRIM_DEADBAND_COUNTS
            final = moved
            if stalled and abs(command - target) >= STALL_PUSH_COUNTS:
                # Commanded well past the target and it still will not budge:
                # the mount is against a mechanical stop (horizontal sits only
                # ~3 deg above the bottom of travel). Back the command off --
                # leaving it pushed into a stop would hold the servo stalled
                # indefinitely, since --home leaves torque engaged.
                log.info("no motion %+d counts off with the command %+d past "
                         "target: treating %d as the end of travel",
                         final - target, command - target, final)
                servo.move(servo_id, target, 200)
                result["hit_end_of_travel"] = True
                break
        result["final_counts"] = final
        result["command_counts"] = command
        err = final - target
        if abs(err) <= tol:
            log.info("homed: %d counts (%+d from target)", final, err)
        else:
            log.warning("homing stopped %+d counts (%.2f deg) from horizontal "
                        "-- servo deadband, mechanical bind, or torque off?",
                        err, err * DEG_PER_COUNT)
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
        "X3_LIDAR_TILT_CALIB", DEFAULT_CALIB))
    ap.add_argument("--wait-device", type=float, default=0.0, metavar="SEC",
                    help="wait up to SEC for the serial device (boot use)")

    ap.add_argument("--status", action="store_true", help="read-only dump")
    ap.add_argument("--calibrate", action="store_true",
                    help="store the current position as horizontal")
    ap.add_argument("--home", action="store_true",
                    help="drive back to the calibrated horizontal")
    ap.add_argument("--goto-counts", type=int, metavar="N",
                    help="move to a raw servo count (0-1000), for testing")
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
    except LX16AError as e:
        log.error("%s", e)
        return 1 if args.strict else 0

    out = {"port": args.port, "servo_id": servo_id}
    try:
        if servo.ping(servo_id) is None:
            log.error("servo id %d did not answer on %s (powered? %s)",
                      servo_id, args.port, "check the 7.4 V rail")
            return 1 if args.strict else 0

        if args.calibrate:
            out["calibration"] = calibrate(servo, servo_id, args.port,
                                           args.calib_file, args.note)
        if args.goto_counts is not None:
            servo.move(servo_id, args.goto_counts, 1000)
            time.sleep(1.25)
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
                    log.info("  %-18s %s", k, v)
                if cal:
                    log.info("  %-18s %d counts (%.2f deg)",
                             "horizontal", int(cal["horizontal_counts"]),
                             int(cal["horizontal_counts"]) * DEG_PER_COUNT)
                    log.info("  %-18s %+d counts (%+.2f deg)",
                             "off horizontal", out[
                                 "offset_from_horizontal_counts"],
                             out["offset_from_horizontal_deg"])
    except LX16AError as e:
        log.error("%s", e)
        return 1 if args.strict else 0
    finally:
        servo.close()

    if args.json:
        print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
