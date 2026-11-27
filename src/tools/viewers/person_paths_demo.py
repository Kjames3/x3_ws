#!/usr/bin/env python3
"""Offline demo of the GUI's curved person paths (X3_PERSON_PATH + ?curvedIntent=1).

Serves src/web over HTTP and plays a fake server_x3 on ws :8081: a robot parked
yawed in a rectangular room, and synthetic Livox-tracker people (one walking
a circle, one crossing straight, one weaving), sent as lidar_persons exactly as
server_x3 sends them, run through person_paths.annotate_telemetry. No ROS, no
robot. Used for the screenshots in artifacts/surroundings-2026-11-27/curved-intent/.

  python3 src/tools/viewers/person_paths_demo.py --mode ctrv --host 127.0.0.2
  # then open http://127.0.0.2:8180/GUI.html?curvedIntent=1, tick "Reality view"

The page host must not be localhost / 127.0.0.1: main.js connects to the page's
own host only when it is something else.
"""
import argparse
import asyncio
import functools
import http.server
import json
import math
import random
import struct
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2]))           # src/

import person_paths as pp  # noqa: E402
from lidar_person_tracker import to_local  # noqa: E402

WEB = HERE.parents[2] / "web"
MODEL = HERE.parents[3] / "config" / "traj_gru_person_paths.npz"
LASER_X = -0.0025
ROBOT = {"x": 0.5, "y": -0.3, "theta": 0.6}        # odom; yawed so frame mistakes show
ROOM = (-3.0, 4.5, -3.5, 3.0)                       # odom x0, x1, y0, y1


def robot_frame(x, y):
    dx, dy = x - ROBOT["x"], y - ROBOT["y"]
    c, s = math.cos(ROBOT["theta"]), math.sin(ROBOT["theta"])
    return c * dx + s * dy, -s * dx + c * dy


def scan_points():
    """Room walls as a LIDR frame: laser_link points as (x, -y), laser yawed 180 deg."""
    x0, x1, y0, y1 = ROOM
    pts = []
    for i in range(400):
        f = i / 400
        for wx, wy in ((x0 + (x1 - x0) * f, y0), (x0 + (x1 - x0) * f, y1),
                       (x0, y0 + (y1 - y0) * f), (x1, y0 + (y1 - y0) * f)):
            bx, by = robot_frame(wx, wy)
            lx, ly = -(bx - LASER_X), -by
            pts += [lx, -ly]
    return b"LIDR" + struct.pack("<%df" % len(pts), *pts)


class Walker:
    """Scripted odom-frame walker: pose(t) -> (x, y, heading, speed), plus tracker-like noise."""

    def __init__(self, tid, pose, noise=0.02):
        self.tid, self.pose, self.noise = tid, pose, noise
        self.t0 = time.time()

    def step(self, t):
        x, y, th, sp = self.pose(t - self.t0)
        n = lambda: random.gauss(0.0, self.noise)
        return {"id": self.tid, "camera_id": None, "odom_x": round(x + n(), 4), "odom_y": round(y + n(), 4),
                "odom_vx": round(sp * math.cos(th), 4), "odom_vy": round(sp * math.sin(th), 4),
                "stamp": t, "mode": "lidar", "confirmed": True, "vel_trusted": True, "hits": 30,
                "pos_sigma_m": 0.08, "confidence": 1.0, "camera_age_s": None, "lidar_age_s": 0.0,
                "points": 120, "height_m": 1.7, "held_s": 0.0}


def circle(cx, cy, r, speed, ph=0.0):
    """Counter-clockwise circle: a sustained turn of speed / r rad/s."""
    w = speed / r
    return lambda t: (cx + r * math.cos(ph + w * t), cy + r * math.sin(ph + w * t),
                      ph + w * t + math.pi / 2, speed)


def shuttle(x0, y0, x1, y1, speed, weave=0.0, weave_hz=0.25):
    """Back and forth between two points, optionally weaving sideways (sin offset)."""
    L = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / L, (y1 - y0) / L

    def pose(t):
        s = (speed * t) % (2 * L)
        fwd = s < L
        d = s if fwd else 2 * L - s
        a = 2 * math.pi * weave_hz * t
        off, doff = weave * math.sin(a), weave * 2 * math.pi * weave_hz * math.cos(a)
        vx, vy = (ux, uy) if fwd else (-ux, -uy)
        vx, vy = speed * vx - uy * doff, speed * vy + ux * doff
        return (x0 + ux * d - uy * off, y0 + uy * d + ux * off, math.atan2(vy, vx), math.hypot(vx, vy))
    return pose


def walkers():
    return [
        # a sustained turn: circling ahead-left of the robot, 1.4 m radius at 1 m/s (0.7 rad/s)
        Walker("L1", circle(2.2, 1.2, 1.4, 1.0)),
        # crossing straight in front of the robot
        Walker("L2", shuttle(3.4, -2.8, 3.4, 2.4, 0.9)),
        # weaving behind-left: the turn keeps changing sign, so the fan shows
        Walker("L3", shuttle(-2.2, -2.6, -0.8, 2.4, 0.8, weave=0.35, weave_hz=0.3), noise=0.03),
    ]


async def serve_ws(host, port, mode):
    import websockets
    paths = pp.PersonPaths(mode, MODEL if mode == "gru" else None) if mode != "off" else None
    people = walkers()
    scan = scan_points()

    async def client(ws):
        k = 0
        try:
            while True:
                now = time.time()
                results = [w.step(now) for w in people]
                local = to_local(results, ROBOT)
                _, local, _ = pp.annotate_telemetry(paths, ROBOT, [], local, [])
                msg = {"type": "readout", "robot_pose": {"x": ROBOT["x"] * 100, "y": ROBOT["y"] * 100,
                                                        "theta": ROBOT["theta"]},
                       "lidar_persons": local,
                       "lidar_person_stats": {"births": len(local), "relinked": 0, "lidar_held_s": 0.0},
                       "velocity_estimates": [], "c3_tracks": [], "c3_stats": None}
                await ws.send(json.dumps(msg))
                if k % 2 == 0:
                    await ws.send(scan)
                k += 1
                await asyncio.sleep(0.1)            # the lidar tracker's ~10 Hz
        except websockets.ConnectionClosed:
            return

    async with websockets.serve(client, host, port):
        await asyncio.Future()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="ctrv", choices=("off", "cv", "ctrv", "gru"))
    ap.add_argument("--host", default="127.0.0.2")
    ap.add_argument("--http-port", type=int, default=8180)
    ap.add_argument("--ws-port", type=int, default=8081)
    a = ap.parse_args()
    random.seed(1)
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(WEB))
    handler.log_message = lambda *args: None
    httpd = http.server.ThreadingHTTPServer((a.host, a.http_port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print("GUI: http://%s:%d/GUI.html?curvedIntent=1  (ws :%d, mode %s)" % (a.host, a.http_port, a.ws_port, a.mode))
    asyncio.run(serve_ws(a.host, a.ws_port, a.mode))


if __name__ == "__main__":
    main()
