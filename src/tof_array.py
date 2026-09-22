#!/usr/bin/env python3
"""Bench tool for the VL53L5CX ToF array — the first thing to run on install.

SINGLE SENSOR ON i2c-1 ONLY.  The robot's dual arrays now go through a Teensy
over USB (/dev/teensy_tof); use src/teensy_tof_serial.py to check those, with
x3_server stopped because the server holds the port.  The --coverage table
below is still valid for either mount.

No ROS, no DDS, no server restart, the same way src/dynamixel_setup.py is the
bench tool for the tilt axis.  Everything here works against --sim too, so the
tool itself is debugged before the part is on the robot.

    python3 src/tof_array.py --scan                 # is 0x29 on i2c-1 at all
    python3 src/tof_array.py --status               # one frame, decoded
    python3 src/tof_array.py --stream               # live 8x8 grid
    python3 src/tof_array.py --stream --show status # live target-status grid
    python3 src/tof_array.py --coverage 0.055 15    # where the rows hit the floor
    python3 src/tof_array.py --set-address 0x2A     # re-address (needs LPn)

ORIENTATION IS THE FIRST THING TO CHECK.  Put a hand in ONE corner of the field
of view and watch which corner of the printed grid goes near.  Top-left of the
print must be up-and-to-the-robot's-left.  If it is not, find the
--transpose/--flip-h/--flip-v combination that makes it so and put those same
three values in the node's params -- the print and the node share the ordering
code, so a setting that looks right here IS right there.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'yahboomcar_bringup'))
from yahboomcar_bringup.tof_geometry import (  # noqa: E402
    FOV_DEG, VALID_STATUS, DEFAULT_MAX_RANGE_M, floor_coverage,
    frame_to_points, reorder, valid_mask)

STATUS_TEXT = {
    0: "not updated", 4: "phase out of bounds", 5: "VALID",
    6: "wrap-around not performed", 9: "VALID (large pulse)",
    10: "no target at previous range", 12: "blurred by motion",
    255: "no measurement",
}


def i2c_scan(bus_num):
    """Probe one bus.  Deliberately independent of the ToF library so a wiring
    fault can be told apart from a driver/firmware fault."""
    try:
        import smbus2
    except ImportError:
        print("smbus2 not installed (pip install smbus2)")
        return []
    found = []
    with smbus2.SMBus(bus_num) as bus:
        for addr in range(0x08, 0x78):
            try:
                bus.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
    return found


def grid_str(values, n, fmt="{:5.0f}", highlight=None):
    out = []
    for r in range(n):
        row = []
        for c in range(n):
            i = r * n + c
            cell = fmt.format(values[i])
            if highlight is not None and not highlight[i]:
                cell = "    ." if len(cell) <= 5 else " " * (len(cell) - 1) + "."
            row.append(cell)
        out.append(" ".join(row))
    return "\n".join(out)


def open_sensor(args):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from drivers_x3 import VL53L5CXArray
    return VL53L5CXArray(i2c_bus=args.bus, i2c_addr=args.addr,
                         resolution=args.resolution,
                         ranging_freq_hz=args.ranging_freq,
                         max_range_m=args.max_range, sim_mode=args.sim,
                         transpose=args.transpose, flip_h=args.flip_h,
                         flip_v=args.flip_v)


def cmd_scan(args):
    for bus in (args.bus, 7) if args.bus != 7 else (args.bus,):
        found = i2c_scan(bus)
        names = ", ".join(f"0x{a:02X}" for a in found) or "(nothing)"
        print(f"i2c-{bus}: {names}")
        if bus == args.bus:
            if args.addr in found:
                print(f"  -> VL53L5CX present at 0x{args.addr:02X}")
            else:
                print(f"  -> 0x{args.addr:02X} NOT found. Check 3.3V on pin 1, "
                      f"SDA/SCL on pins 27/28, and that LPn is not held low.")
    return 0


def cmd_status(args):
    tof = open_sensor(args)
    if not tof.available:
        print("no sensor (use --sim to exercise this tool without one)")
        return 1
    # The FIRST frame always comes back target_status 6 ("wrap around not
    # performed"), which is not in VALID_STATUS -- reporting it would show
    # "valid zones: 0/64" on a perfectly good sensor.  Burn a few frames so
    # what gets printed is the steady state.
    frame = None
    for _ in range(4):
        for _ in range(200):
            if tof.data_ready():
                break
            time.sleep(0.01)
        frame = tof.read_frame()
    if frame is None:
        print("no frame")
        return 1
    dist, status = frame
    n = args.resolution
    keep = valid_mask(dist, status, n, max_range_m=args.max_range,
                      transpose=args.transpose, flip_h=args.flip_h,
                      flip_v=args.flip_v)
    print(f"resolution {n}x{n} @ {tof.ranging_freq_hz} Hz, FoV {FOV_DEG} deg, "
          f"max_range {args.max_range} m")
    print(f"valid zones: {int(keep.sum())}/{n * n}\n")
    print("distance (mm), '.' = rejected")
    print(grid_str(reorder(dist, n, args.transpose, args.flip_h, args.flip_v),
                   n, highlight=keep))
    print("\ntarget_status")
    print(grid_str(reorder(status, n, args.transpose, args.flip_h, args.flip_v),
                   n, fmt="{:5.0f}"))
    counts = {}
    for s in np.asarray(status).ravel():
        counts[int(s)] = counts.get(int(s), 0) + 1
    for s, c in sorted(counts.items(), key=lambda kv: -kv[1]):
        mark = "  <- kept" if s in VALID_STATUS else ""
        print(f"  {s:3d} x{c:3d}  {STATUS_TEXT.get(s, 'unknown')}{mark}")
    pts = tof.read_points()
    if pts is not None and len(pts):
        print(f"\ncloud: {len(pts)} pts, x {pts[:, 0].min():.3f}..{pts[:, 0].max():.3f} m "
              f"(sensor frame, extrinsics NOT applied)")
    tof.cleanup()
    return 0


def cmd_stream(args):
    tof = open_sensor(args)
    if not tof.available:
        print("no sensor (use --sim)")
        return 1
    n = args.resolution
    frames = 0
    t0 = time.time()
    try:
        while True:
            if not tof.data_ready():
                time.sleep(0.005)
                continue
            frame = tof.read_frame()
            if frame is None:
                continue
            dist, status = frame
            frames += 1
            keep = valid_mask(dist, status, n, max_range_m=args.max_range,
                              transpose=args.transpose, flip_h=args.flip_h,
                              flip_v=args.flip_v)
            show = status if args.show == "status" else dist
            print("\033[2J\033[H", end="")
            print(f"{args.show}  {frames} frames  {frames / (time.time() - t0):.1f} Hz  "
                  f"valid {int(keep.sum())}/{n * n}   (Ctrl-C to stop)")
            print("top-left of this grid = UP and to the robot's LEFT\n")
            print(grid_str(reorder(show, n, args.transpose, args.flip_h, args.flip_v),
                           n, highlight=None if args.show == "status" else keep))
    except KeyboardInterrupt:
        print()
    finally:
        tof.cleanup()
    return 0


def cmd_coverage(args):
    height, pitch = args.coverage
    n = args.resolution
    rows, ranges = floor_coverage(height, pitch, n)
    print(f"mount {height * 1000:.0f} mm high, pitched {pitch:.1f} deg down, "
          f"{n}x{n}, {FOV_DEG} deg FoV\n")
    print(" row   axis below horizon   floor hit")
    step = FOV_DEG / n
    for r, d in zip(rows, ranges):
        down = pitch - (n / 2.0 - 0.5 - r) * step
        print(f" {r:3d}   {down:8.2f} deg      {d:7.3f} m")
    above = n - len(rows)
    if above:
        print(f"\n {above} row(s) point at or above the horizon: they never hit the "
              f"floor and are the only rows that see far-field obstacles.")
    if len(ranges):
        print(f"\n floor coverage {ranges.min():.3f} .. {ranges.max():.3f} m")
    return 0


def cmd_set_address(args):
    print(f"Re-addressing 0x{args.addr:02X} -> 0x{args.set_address:02X}")
    print("This is VOLATILE: it is lost at power cycle and must be redone at every"
          " boot, before the second sensor's LPn is released.")
    if args.sim:
        print("(--sim: nothing done)")
        return 0
    import vl53l5cx_ctypes
    tof = vl53l5cx_ctypes.VL53L5CX(i2c_addr=args.addr, bus_id=args.bus)
    tof.set_i2c_address(args.set_address)
    print("done; re-run --scan to confirm")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=int, default=1, help="I2C bus (default 1 = pins 27/28)")
    p.add_argument("--addr", type=lambda s: int(s, 0), default=0x29)
    p.add_argument("--resolution", type=int, default=8, choices=(4, 8))
    p.add_argument("--ranging-freq", type=int, default=10, help="Hz (8x8 caps at 15)")
    p.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE_M)
    p.add_argument("--transpose", action="store_true")
    p.add_argument("--flip-h", action="store_true")
    p.add_argument("--flip-v", action="store_true")
    p.add_argument("--show", choices=("distance", "status"), default="distance")
    p.add_argument("--sim", action="store_true", help="synthetic frames, no hardware")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--scan", action="store_true")
    g.add_argument("--status", action="store_true")
    g.add_argument("--stream", action="store_true")
    g.add_argument("--coverage", nargs=2, type=float, metavar=("HEIGHT_M", "PITCH_DEG"))
    g.add_argument("--set-address", type=lambda s: int(s, 0), metavar="ADDR")
    args = p.parse_args()

    if args.scan:
        return cmd_scan(args)
    if args.status:
        return cmd_status(args)
    if args.stream:
        return cmd_stream(args)
    if args.coverage:
        return cmd_coverage(args)
    return cmd_set_address(args)


if __name__ == "__main__":
    sys.exit(main())
