#!/usr/bin/env python3
"""Record ToF frames for the geometry check.  Runs on the LAPTOP.

    python3 src/tof_wall_capture.py --label floor-wood
    python3 src/tof_wall_capture.py --label wall-300 --distance 0.300
    python3 src/tof_wall_capture.py --label floor-matte

Pulls frames off x3_server's readout lane over the websocket rather than
opening the sensor.  That matters: x3_server already owns the VL53L5CX, and a
second opener would fight it for the bus.  Nothing has to be stopped or
restarted to run this, and the robot keeps driving if you want it to.

--distance is the horizontal distance from the SENSOR APERTURE to a flat
vertical target, in metres, and is recorded in the npz for the offline fit.
Leave it off for a floor-only capture.

Saves to artifacts/tof-geometry-<date>/<label>.npz with every frame kept, not
a mean: the per-frame spread is what says whether a row is noisy or biased,
and averaging first throws that away.
"""
from __future__ import annotations

import argparse
import asyncio
import datetime as _dt
import json
import os
import sys

import numpy as np


async def collect(host, port, n_frames, timeout_s):
    try:
        import websockets
    except ImportError:
        print("pip install websockets")
        return None
    url = f"ws://{host}:{port}"
    dist, status, keep, seqs = [], [], [], []
    print(f"connecting to {url} ...")
    async with websockets.connect(url, max_size=None, open_timeout=timeout_s) as ws:
        print("connected; collecting", n_frames, "distinct frames (Ctrl-C to stop early)")
        last_seq = None
        async for raw in ws:
            if isinstance(raw, bytes):
                continue                      # camera frames share this socket
            try:
                m = json.loads(raw)
            except Exception:
                continue
            if m.get("type") != "readout":
                continue
            tof = m.get("tof")
            if not tof:
                continue
            # The readout lane repeats a frame until the 10 Hz sensor produces
            # the next one; seq is what tells them apart.
            if tof["seq"] == last_seq:
                continue
            last_seq = tof["seq"]
            dist.append(tof["dist"])
            status.append(tof["status"])
            keep.append(tof["keep"])
            seqs.append(tof["seq"])
            done = len(dist)
            print(f"\r  {done}/{n_frames}  valid {tof['valid']}/64  "
                  f"{tof['min']}-{tof['max']} mm   ", end="", flush=True)
            if done >= n_frames:
                break
    print()
    if not dist:
        print("no ToF frames arrived. Is x3_server running with the ToF enabled?")
        return None
    return (np.asarray(dist, dtype=np.int32),
            np.asarray(status, dtype=np.int32),
            np.asarray(keep, dtype=bool),
            np.asarray(seqs, dtype=np.int64))


def summarise(dist, status, keep):
    n = 8
    print("\nper-row summary over", len(dist), "frames "
          "(mean of VALID zones only, +/- spread across frames):")
    print("  row    mean      sd   valid%")
    for r in range(n):
        sl = slice(r * n, (r + 1) * n)
        d = dist[:, sl].astype(float)
        k = keep[:, sl]
        if not k.any():
            print(f"   {r}        -       -     0%")
            continue
        per_frame = np.array([dd[kk].mean() if kk.any() else np.nan
                              for dd, kk in zip(d, k)])
        good = ~np.isnan(per_frame)
        print(f"   {r}   {per_frame[good].mean():6.0f}  {per_frame[good].std():6.1f}"
              f"   {100.0 * k.mean():4.0f}%")
    uniq, cnt = np.unique(status, return_counts=True)
    print("\nstatus histogram:", {int(u): int(c) for u, c in zip(uniq, cnt)})


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--label", required=True,
                   help="short name for this capture, e.g. wall-300 or floor-matte")
    p.add_argument("--distance", type=float, default=None,
                   help="metres, aperture to a flat vertical target (omit for floor-only)")
    p.add_argument("--note", default="", help="free text stored in the npz")
    p.add_argument("--frames", type=int, default=60)
    p.add_argument("--host", default="x3")
    p.add_argument("--port", type=int, default=8081)
    p.add_argument("--timeout", type=float, default=20.0)
    args = p.parse_args()

    try:
        got = asyncio.run(collect(args.host, args.port, args.frames, args.timeout))
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1
    if got is None:
        return 1
    dist, status, keep, seqs = got

    summarise(dist, status, keep)

    day = _dt.date.today().isoformat()
    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "artifacts", f"tof-geometry-{day}")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{args.label}.npz")
    np.savez_compressed(
        path, dist_mm=dist, target_status=status, keep=keep, seq=seqs,
        distance_m=np.nan if args.distance is None else args.distance,
        label=args.label, note=args.note, captured=_dt.datetime.now().isoformat(),
        # Recorded so a later fit cannot silently assume a mount that has moved.
        urdf_height_m=0.0640, urdf_pitch_deg=15.0, fov_deg=45.0, resolution=8)
    print(f"\nsaved {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
