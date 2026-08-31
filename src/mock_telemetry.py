#!/usr/bin/env python3
"""Offline stand-in for x3_server's WebSocket, for GUI work with no robot.

Serves the web/ directory on :8080 and speaks enough of the server's protocol
on :8081 that the dashboard behaves as if it were connected:

    hello · readout (20 Hz) · readout_slow (2 Hz) · sweep_config

The payload shapes are copied from a live capture off the robot on
2026-08-31, not invented, so a panel that renders here renders there.  It is a
development harness only -- it drives nothing and reads no hardware.

    python3 src/mock_telemetry.py            # then open localhost:8080/GUI.html
    python3 src/mock_telemetry.py --scenario low-battery
    python3 src/mock_telemetry.py --scenario sweep

Scenarios exist because the states worth designing against are the ones that
are awkward to reproduce on real hardware: a nearly flat pack, a gated /scan,
a desynced tilt read, a saturated link.
"""
import argparse
import asyncio
import json
import math
import os
import random
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread

import websockets

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

SCENARIOS = ("nominal", "low-battery", "sweep", "degraded")


class Sim:
    """Rolling fake robot state."""

    def __init__(self, scenario="nominal"):
        self.scenario = scenario
        self.t0 = time.monotonic()
        # Start where the real pack was when this was written, so the battery
        # tile is exercised near its warn/bad thresholds rather than at 100%.
        self.volts = 12.42 if scenario != "low-battery" else 11.30
        self.charge_pct = 42.0 if scenario != "low-battery" else 6.0
        self.tilt_deg = 0.0
        self.sweep_dir = 1
        self.tilt_reads = 0
        self.tilt_errors = 0

    @property
    def elapsed(self):
        return time.monotonic() - self.t0

    # -- individual subsystems ------------------------------------------
    def battery(self):
        # Drain fast enough to watch the tile change state within a session.
        drain = self.elapsed / 600.0
        pct = max(0.0, self.charge_pct - drain)
        amps = 1.5 + 0.25 * math.sin(self.elapsed / 7.0)
        volts = self.volts - 0.0004 * self.elapsed
        return {
            "voltage": round(volts, 4),
            "current": round(amps, 4),
            "power": round(volts * amps, 4),
            "battery_pct": round(pct, 3),
            # 'degraded' models a dead INA226: the server falls back to the
            # duty-cycle guess and flags it, and the GUI must not present that
            # the same way it presents a measurement.
            "measured": self.scenario != "degraded",
            "minutes_left": round(pct / 100.0 * 8.07 / amps * 60.0, 1) if amps > 0.05 else None,
        }

    def tilt(self):
        self.tilt_reads += 1
        scanning = self.scenario == "sweep"
        if scanning:
            # Ping-pong between -45 and +45, matching lidar_scan_loop.
            self.tilt_deg += self.sweep_dir * 0.9
            if abs(self.tilt_deg) >= 45.0:
                self.sweep_dir *= -1
                self.tilt_deg = max(-45.0, min(45.0, self.tilt_deg))
        else:
            self.tilt_deg = -0.44 + 0.02 * math.sin(self.elapsed)

        if self.scenario == "degraded" and random.random() < 0.02:
            self.tilt_errors += 1

        # A stale sample with a plausible angle is the dangerous tilt failure,
        # so 'degraded' reproduces it: the number keeps looking fine while
        # age_s climbs.
        age = 2.4 if self.scenario == "degraded" else round(random.uniform(0.01, 0.04), 2)

        return {
            "deg": round(self.tilt_deg, 2),
            "counts": int(2032 + self.tilt_deg * 11.378),
            "moving": scanning,
            "reads": self.tilt_reads,
            "errors": self.tilt_errors,
            "age_s": age,
            "scanning": scanning,
            "scan_gated": scanning and self.tilt_reads % 60 < 25,
        }

    def system(self):
        base = 55.0 if self.scenario == "degraded" else 30.0
        cores = [round(max(0.0, min(100.0,
                 base + 12 * math.sin(self.elapsed / 3.0 + i) + random.uniform(-4, 4))), 1)
                 for i in range(6)]
        return {
            "cpu_per_core": cores,
            "cpu_total": round(sum(cores) / len(cores), 1),
            "proc_cpu": round(126.0 + random.uniform(-8, 8), 1),
            "proc_rss_mb": 999.5,
            "mem_pct": 24.7,
            "loadavg": round(2.0 + random.uniform(-0.4, 0.9), 2),
            "temp_c": round((68.0 if self.scenario == "degraded" else 49.0)
                            + random.uniform(-0.6, 0.6), 1),
        }

    def oak_imu(self):
        # Bias chosen so the IMU tile lands in 'warn'/'bad', which is where the
        # real MPU9250 sat and therefore what the tile has to communicate.
        drift = -0.0022 if self.scenario != "degraded" else -0.0090
        return {
            "accel": {"x": 0.02, "y": -0.01, "z": 9.79},
            "gyro": {"x": 0.001, "y": -0.002, "z": drift + random.gauss(0, 0.0006)},
        }

    def readout(self):
        p = self.battery()
        return {
            "type": "readout",
            "depth_image": "", "oak_left": "", "oak_right": "",
            "oak_imu": self.oak_imu(),
            "oak_detections": [],
            "robot_pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "m1_pos": 0, "m2_pos": 0, "m3_pos": 0, "m4_pos": 0,
            "m1_power": 0, "m2_power": 0, "m3_power": 0, "m4_power": 0,
            "left_power": 0, "right_power": 0,
            "detection_enabled": False,
            "is_auto_driving": False,
            "detections": [],
            "velocity_estimates": [],
            "battery": {"voltage": p["voltage"], "amps": p["current"], "watts": p["power"]},
            "power": p,
            "tilt": self.tilt(),
        }

    def readout_slow(self):
        return {
            "type": "readout_slow",
            "nav_phase": "IDLE",
            "nav": {"state": "IDLE"},
            "slam_active": False,
            "slam_3d_active": self.scenario == "sweep",
            "frontier": None,
            "active_model_name": "yolo26n",
            "active_velocity_model_name": "velocity_mlp",
            "p2p_test_running": False,
            "ab_test_running": False,
            "ab_test_mode": None,
            "servo_stats": {"voltage": "11.10 V", "state": "Free"},
            "fps_camera": 0.0,
            "fps_detection": 0.0,
            "system": self.system(),
            "fps_oak_depth": 3.0 if self.scenario == "degraded" else 14.8,
        }

    def sweep_config(self):
        return {
            "type": "sweep_config",
            "mode": "continuous" if self.scenario == "sweep" else "step",
            "speed_deg_s": 40.0, "speed_min": 2, "speed_max": 90,
            "step_deg": 0.97, "dwell_s": 0.25,
            "smear_deg": 6.24 if self.scenario == "sweep" else 0.0,
            "smear_cm_at_3m": 32.7 if self.scenario == "sweep" else 0.0,
            "settled_bypass": False,
            "scanning": self.scenario == "sweep",
            "note": None,
        }


async def handler(ws, sim):
    print(f"[mock] client connected ({ws.remote_address})")
    await ws.send(json.dumps({
        "type": "hello",
        "mode": "MOCK",
        "active_velocity_model": "velocity_mlp",
        "webrtc_camera": False,
        "build": {"rev": "mock", "dirty": True, "server_mtime": int(time.time()),
                  "host": "mock-telemetry"},
    }))
    await ws.send(json.dumps(sim.sweep_config()))

    async def rx():
        # Drain and log inbound commands so a new control can be checked to be
        # sending the message it thinks it is.
        async for raw in ws:
            try:
                print(f"[mock] <- {json.loads(raw).get('type')}")
            except Exception:
                pass

    async def tx():
        i = 0
        while True:
            await ws.send(json.dumps(sim.readout()))
            if i % 10 == 0:
                await ws.send(json.dumps(sim.readout_slow()))
            i += 1
            await asyncio.sleep(0.05)

    try:
        await asyncio.gather(rx(), tx())
    except websockets.ConnectionClosed:
        print("[mock] client disconnected")


def serve_http(port):
    handler_cls = partial(SimpleHTTPRequestHandler, directory=WEB_DIR)
    httpd = ThreadingHTTPServer(("0.0.0.0", port), handler_cls)
    Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"[mock] http  http://localhost:{port}/GUI.html  (serving {WEB_DIR})")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=SCENARIOS, default="nominal")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--ws-port", type=int, default=8081)
    args = ap.parse_args()

    sim = Sim(args.scenario)
    serve_http(args.http_port)
    print(f"[mock] ws    ws://localhost:{args.ws_port}   scenario={args.scenario}")
    async with websockets.serve(lambda w: handler(w, sim), "0.0.0.0", args.ws_port,
                                compression=None):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[mock] stopped")
