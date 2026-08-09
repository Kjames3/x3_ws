#!/usr/bin/env python3
"""
Measure the pack's internal resistance so battery.py's sag compensation is
calibrated to this robot instead of a generic 0.12 ohm guess.

Procedure: log the resting voltage, spin the robot in place (rotation only --
it never translates, so it stays where you put it), log the sagged voltage,
then stop and log the recovery.  R_int = (V_rest - V_load) / I_load.

The Rosmaster reports voltage in 0.1 V steps, so run this on a pack that is not
fully charged if you can -- a full pack's voltage is stiffest and the sag may
land inside a single code.

Usage:
    python3 scripts/calibrate_battery.py [--host 127.0.0.1] [--spin-seconds 25]

Run it on the robot, or point --host at the robot's IP.  Ctrl-C is safe: the
motors are stopped in a finally block.
"""

import argparse
import asyncio
import json
import statistics
import sys

try:
    import websockets
except ImportError:
    sys.exit("websockets not installed (pip3 install websockets)")

# The server's motion watchdog cuts the motors if no command arrives within
# 500 ms, so a sustained move has to be re-sent continuously.
RESEND_HZ = 20.0


async def _collect(ws, seconds, label, drive=None):
    """Read telemetry for `seconds`, optionally re-sending a motion command."""
    volts, amps = [], []
    loop = asyncio.get_event_loop()
    t_end = loop.time() + seconds
    next_cmd = 0.0
    while loop.time() < t_end:
        if drive is not None and loop.time() >= next_cmd:
            await ws.send(json.dumps(drive))
            next_cmd = loop.time() + 1.0 / RESEND_HZ
        try:
            msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
        except asyncio.TimeoutError:
            continue
        if isinstance(msg, (bytes, bytearray)):
            continue
        try:
            data = json.loads(msg)
        except ValueError:
            continue
        pwr = data.get("power")
        if pwr:
            volts.append(pwr.get("voltage"))
            amps.append(pwr.get("current"))
    codes = sorted(set(v for v in volts if v is not None))
    print(f"  {label:9s} n={len(volts):5d}  codes={codes}  "
          f"median={statistics.median(volts) if volts else float('nan'):.2f} V  "
          f"I_est~{statistics.median(amps) if amps else float('nan'):.2f} A")
    return volts, amps


async def run(host, port, spin_seconds, omega):
    uri = f"ws://{host}:{port}"
    print(f"connecting to {uri}")
    async with websockets.connect(uri) as ws:
        stop = {"type": "set_move", "vx": 0.0, "vy": 0.0, "omega": 0.0}
        try:
            print("\n[1/3] resting baseline (30 s) -- do not touch the robot")
            v_rest, _ = await _collect(ws, 30, "rest")

            print(f"\n[2/3] spinning in place ({spin_seconds} s, omega={omega})")
            spin = {"type": "set_move", "vx": 0.0, "vy": 0.0, "omega": omega}
            # Discard the first 5 s so the measurement is of steady-state draw,
            # not the inrush of spinning the wheels up.
            await _collect(ws, 5, "spin-up", drive=spin)
            v_load, i_load = await _collect(ws, spin_seconds, "loaded", drive=spin)

            print("\n[3/3] stopping, logging recovery (40 s)")
            await ws.send(json.dumps(stop))
            await _collect(ws, 40, "recovery")
        finally:
            for _ in range(5):
                try:
                    await ws.send(json.dumps(stop))
                    await asyncio.sleep(0.05)
                except Exception:
                    break
            print("\nmotors commanded stopped")

    if not v_rest or not v_load:
        print("no telemetry captured -- is the server running?")
        return
    rest = statistics.median(v_rest)
    load = statistics.median(v_load)
    amps = statistics.median(i_load) if i_load else 0.0
    sag = rest - load
    print(f"\nresting  {rest:.2f} V")
    print(f"loaded   {load:.2f} V")
    print(f"sag      {sag:.2f} V at an estimated {amps:.2f} A")
    if sag <= 0.05:
        print("\nSag is within one 0.1 V code -- inconclusive. Retry on a "
              "partly discharged pack, or with a higher omega.")
    elif amps > 0.1:
        print(f"\n=> R_INT_OHMS ~ {sag / amps:.3f}   (battery.py currently uses 0.12)")
        print("   Note this rides on the server's *estimated* current, which is "
              "derived from commanded motor power, not measured.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--spin-seconds", type=float, default=25.0)
    ap.add_argument("--omega", type=float, default=1.0,
                    help="rotation rate; rotation only, the robot does not translate")
    args = ap.parse_args()
    try:
        asyncio.run(run(args.host, args.port, args.spin_seconds, args.omega))
    except KeyboardInterrupt:
        print("\ninterrupted -- motors stopped by the finally block")


if __name__ == "__main__":
    main()
