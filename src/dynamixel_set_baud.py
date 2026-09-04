#!/usr/bin/env python3
"""dynamixel_set_baud — move the tilt servo off the 57600 factory default.

Why: measured on the robot 2026-08-29, a single `read_pos` at 57600 baud costs
**6.25 ms** and `read_pos`+`read_moving` costs 12.0 ms.  That is the floor
under every tilt-publisher rate, and it is why polling at 50 Hz costs 29% of a
core.  The OpenRB-150 is a passthrough bridge (like a U2D2), so the USB-serial
rate IS the bus rate and raising it raises the real thing.

    python3 dynamixel_set_baud.py --status         # find + report, no writes
    python3 dynamixel_set_baud.py --bench          # latency at the current rate
    python3 dynamixel_set_baud.py --set 1000000    # write EEPROM, verify, bench

The rate lives in the servo's EEPROM, so this is persistent across power
cycles and independent of anything in this repo.  If a change half-completes,
the servo is NOT bricked -- it is answering at the new rate while the code
still opens the old one.  `--status` sweeps every rate in the table and will
find it; then re-run `--set` with the rate you want, or update
`dynamixel_servo.BAUD`.

x3_server holds /dev/openrb150, so stop it first:
    sudo systemctl stop x3_server && ... && sudo systemctl start x3_server
"""

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dynamixel_servo import (  # noqa: E402
    XL430, DynamixelError, BAUD_CODES, BAUD_SCAN_ORDER, DEFAULT_PORT,
    find_servo, model_name, EXPECTED_MODEL,
)

CODE_FOR_BAUD = {v: k for k, v in BAUD_CODES.items()}


def _bench(dev, servo_id, n=300):
    """Round-trip latency of the two reads the tilt publisher actually does."""
    out = {}
    for label, fns in (('read_pos', (dev.read_pos,)),
                       ('read_pos+read_moving', (dev.read_pos, dev.read_moving))):
        lat = []
        t_cpu0 = time.process_time()
        t0 = time.perf_counter()
        for _ in range(n):
            a = time.perf_counter()
            for fn in fns:
                fn(servo_id)
            lat.append((time.perf_counter() - a) * 1e3)
        wall = time.perf_counter() - t0
        cpu = time.process_time() - t_cpu0
        out[label] = (statistics.mean(lat), statistics.median(lat),
                      max(lat), cpu / wall * 100.0)
    return out


def _report_bench(res):
    print('    %-22s %9s %9s %9s %11s %10s'
          % ('operation', 'mean ms', 'med ms', 'max ms', 'max rate', 'CPU %core'))
    for k, (mean, med, mx, cpu) in res.items():
        print('    %-22s %9.3f %9.3f %9.3f %9.0f Hz %10.1f'
              % (k, mean, med, mx, 1000.0 / mean, cpu))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--id', type=int, default=1)
    ap.add_argument('--status', action='store_true')
    ap.add_argument('--bench', action='store_true')
    ap.add_argument('--set', type=int, metavar='BAUD',
                    help='new bus rate, e.g. 1000000 (must be a BAUD_CODES value)')
    ap.add_argument('--iters', type=int, default=300)
    args = ap.parse_args()

    if not (args.status or args.bench or args.set):
        ap.error('nothing to do: pass --status, --bench or --set')

    if not os.path.exists(args.port):
        print('%s does not exist. Is the OpenRB-150 plugged in and '
              '64-openrb150.rules installed?' % args.port)
        return 2

    print('Sweeping %s for servo id %d ...' % (args.port, args.id))
    try:
        dev, baud = find_servo(args.id, args.port)
    except DynamixelError as e:
        print('FAILED: %s' % e)
        return 1

    try:
        model = dev.read_model(args.id)
        code = dev.read_baud_code(args.id)
        print('  found at %d baud (EEPROM code %d)' % (baud, code))
        print('  model %s (%d)%s, firmware %s'
              % (model_name(model), model,
                 '' if model == EXPECTED_MODEL else '  <-- NOT the expected XL430',
                 dev.read_firmware(args.id)))
        print('  position %d counts, torque %s, %.1f V, %d C'
              % (dev.read_pos(args.id),
                 'ON' if dev.is_loaded(args.id) else 'off',
                 dev.read_vin(args.id), dev.read_temp(args.id)))

        if args.bench or args.set:
            print('\n  latency at %d baud:' % baud)
            _report_bench(_bench(dev, args.id, args.iters))

        if not args.set:
            return 0

        if args.set not in CODE_FOR_BAUD:
            print('\n%d is not a rate this servo supports. Choose from %s'
                  % (args.set, sorted(CODE_FOR_BAUD)))
            return 2
        if args.set == baud:
            print('\nAlready at %d baud; nothing to write.' % args.set)
            return 0

        new_code = CODE_FOR_BAUD[args.set]
        print('\n  writing EEPROM baud code %d (%d baud) ...' % (new_code, args.set))
        # EEPROM writes are refused while torque is on.  Note where the mount
        # is first: this tool must not be the reason it ends up off-level,
        # because a tilted mount gates /scan off indefinitely.
        pos_before = dev.read_pos(args.id)
        if dev.is_loaded(args.id):
            dev.set_load(args.id, False)
            time.sleep(0.05)
        dev.set_baud_code(args.id, new_code)
        # The servo switches on ack, so this handle is now talking the wrong
        # rate.  Do not try to read anything through it.
        dev.close()
        dev = None
        time.sleep(0.5)

        print('  reopening ...')
        try:
            dev, baud2 = find_servo(args.id, args.port)
        except DynamixelError as e:
            print('  FAILED to find the servo after the write: %s' % e)
            print('  The servo is not bricked -- its rate is in EEPROM. Re-run '
                  '--status to locate it.')
            return 1
        code2 = dev.read_baud_code(args.id)
        pos_after = dev.read_pos(args.id)
        ok = (baud2 == args.set and code2 == new_code)
        print('  now at %d baud (code %d)  %s' % (baud2, code2, 'OK' if ok else 'MISMATCH'))
        print('  position %d -> %d counts (unchanged: %s)'
              % (pos_before, pos_after, abs(pos_after - pos_before) <= 2))
        if not ok:
            return 1

        print('\n  latency at %d baud:' % baud2)
        _report_bench(_bench(dev, args.id, args.iters))
        print('\nDone. Now set dynamixel_servo.BAUD = %d so every caller '
              'opens the right rate.' % baud2)
        return 0
    finally:
        if dev is not None:
            dev.close()


if __name__ == '__main__':
    sys.exit(main())
