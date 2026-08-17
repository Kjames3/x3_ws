#!/usr/bin/env python3
"""Log the velocity estimator with the robot parked -- a phantom-detection baseline.

ab_comparison_test.py drives the robot 1.5-4 m and back, which is the wrong
shape for a phantom test: you want nothing in the scene moving, including the
robot. This connects to the same WebSocket readout the A/B test uses, turns the
estimator on, and writes the same CSV schema -- but publishes no /cmd_vel at
all, so it cannot move the robot.

Every non-zero reading it records is a phantom, by construction. Score it with:

    python3 score_ab_logs.py <the csv> --static

Usage
    python3 phantom_baseline.py --seconds 300
    python3 phantom_baseline.py --seconds 300 --label v3
"""
import argparse
import asyncio
import csv
import json
import math
import os
import time
from datetime import datetime

import websockets

LOG_HZ = 10
DEFAULT_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

FIELDS = [
    "time_s", "mode", "segment", "robot_x", "robot_y", "robot_th_deg",
    "n_obstacles", "max_obs_speed", "min_obstacle_dist", "path_y",
    "vx_cmd", "vy_cmd", "vy_rep", "corrected_x", "corrected_y", "corrected_yaw_deg",
]


async def run(url, seconds, label, log_dir):
    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(log_dir, f"ab_{label}_{ts}.csv")

    latest = []
    rows = []
    start = time.monotonic()
    next_log = start

    async with websockets.connect(url, max_size=None) as ws:
        await ws.send(json.dumps({"type": "set_velocity_estimation", "enabled": True}))
        print(f"[phantom] estimator ON, logging {seconds}s -> {path}")
        print("[phantom] keep the robot parked and the scene still (stay behind it)")

        while True:
            now = time.monotonic()
            if now - start >= seconds:
                break
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=max(0.05, next_log - now))
            except asyncio.TimeoutError:
                msg = None
            except Exception as e:
                print(f"[phantom] connection lost: {e}")
                break

            if isinstance(msg, str):
                try:
                    data = json.loads(msg)
                    if data.get("type") == "readout":
                        latest = data.get("velocity_estimates", []) or []
                except Exception:
                    pass

            now = time.monotonic()
            if now >= next_log:
                next_log += 1.0 / LOG_HZ
                if next_log < now:            # fell behind, resync
                    next_log = now + 1.0 / LOG_HZ
                speeds = [e.get("speed", 0.0) for e in latest]
                dists = [math.hypot(e.get("x", 0.0), e.get("z", e.get("y", 0.0)))
                         for e in latest]
                row = {f: 0.0 for f in FIELDS}
                row.update(
                    time_s=round(now - start, 3),
                    mode=label,
                    segment="PARKED",
                    n_obstacles=len(latest),
                    max_obs_speed=round(max(speeds), 4) if speeds else 0.0,
                    min_obstacle_dist=round(min(dists), 3) if dists else 999.0,
                )
                rows.append(row)
                if len(rows) % (LOG_HZ * 30) == 0:
                    peak = max(r["max_obs_speed"] for r in rows)
                    print(f"[phantom] {int(now - start):>4}s  rows={len(rows)}  "
                          f"peak phantom so far {peak:.2f} m/s")

    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    print(f"[phantom] saved {len(rows)} rows -> {path}")
    print(f"[phantom] score it:  python3 score_ab_logs.py {path} --static")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=300, help="run length (default 300)")
    ap.add_argument("--label", default="phantom",
                    help="goes in the filename and the mode column, e.g. 'v3'")
    ap.add_argument("--url", default="ws://localhost:8081")
    ap.add_argument("--log-dir", default=DEFAULT_LOG_DIR)
    a = ap.parse_args()
    try:
        asyncio.run(run(a.url, a.seconds, a.label, a.log_dir))
    except KeyboardInterrupt:
        print("\n[phantom] interrupted -- rerun for a full-length baseline")


if __name__ == "__main__":
    main()
