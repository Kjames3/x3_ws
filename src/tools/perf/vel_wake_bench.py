"""Publish->estimate latency of VelocityEstimator, 10 Hz poll vs X3_VEL_WAKE_ON_FRAME.

Runs on the laptop: no camera, no ROS. A fake producer drives the real
OakDCamera._process_depth / wait_for_depth at several depth rates (with jitter,
so the poll is not phase-locked), and the estimator's real _inference_loop
runs with its _step and centroid extraction stubbed out. Latency is the time
from the producer storing a frame to that frame reaching _step: scheduling
delay only, the model and extraction cost (a few ms on the Orin) come on top.

    python3 src/tools/perf/vel_wake_bench.py [--seconds 6] [--rates 5,10,15,30,60]
"""
import argparse
import json
import os
import random
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC))
import logging  # noqa: E402
logging.disable(logging.WARNING)
from oakd_driver import OakDCamera  # noqa: E402
from velocity_estimator import VelocityEstimator  # noqa: E402

MODEL = str(SRC / 'velocity_mlp_v3.torchscript')


def make_camera():
    cam = OakDCamera.__new__(OakDCamera)
    cam._lock = threading.Lock()
    cam._depth_history = deque(maxlen=20)
    cam._inv_depth_offset = 0.
    cam._depth_intr_size = (4, 4)
    cam._depth_fx = cam._depth_fy = 100.
    cam._depth_cx = cam._depth_cy = 2.
    cam._latest_depth_observation = None
    cam._depth_seq = 0
    cam._depth_cond = threading.Condition()
    return cam


def run(wake, hz, seconds, jitter=0.2, seed=0):
    os.environ.pop('X3_VEL_WAKE_ON_FRAME', None)
    if wake:
        os.environ['X3_VEL_WAKE_ON_FRAME'] = '1'
    os.environ.pop('DISPLAY', None)
    est = VelocityEstimator(None, None, model_path=MODEL)
    cam = make_camera()
    est.camera = cam
    est._extract_depth_centroids = lambda *a: [(0., 0., 2.)]
    reached = []

    def step(*args, capture_t=None, **kwargs):
        reached.append((capture_t, time.monotonic()))
        return [], {}
    est._step = step
    published = []
    stop = threading.Event()
    rng = random.Random(seed)

    def produce():
        n = 0
        depth = np.full((4, 4), 2000, np.uint16)
        while not stop.is_set():
            n += 1
            t = time.monotonic()
            cam._process_depth(depth, t, ('bench', n), t)
            published.append(t)
            stop.wait(max(0.001, (1 / hz) * (1 + rng.uniform(-jitter, jitter))))
    est._running = True
    loop = threading.Thread(target=est._inference_loop, daemon=True)
    prod = threading.Thread(target=produce, daemon=True)
    cpu0 = time.process_time()
    loop.start()
    time.sleep(0.3)            # let the loop settle before frames start
    prod.start()
    time.sleep(seconds)
    est._running = False
    stop.set()
    prod.join(1)
    loop.join(1)
    cpu = time.process_time() - cpu0
    caps = [c for c, _ in reached]
    lat = np.array([(t - c) * 1e3 for c, t in reached[2:]])  # skip startup
    return {
        'mode': 'wake' if wake else 'poll', 'depth_hz': hz,
        'frames': len(published), 'estimates': len(reached),
        'estimate_hz': round(len(reached) / seconds, 2),
        'double_processed': len(caps) - len(set(caps)),
        'lat_ms_mean': round(float(lat.mean()), 1),
        'lat_ms_p50': round(float(np.median(lat)), 1),
        'lat_ms_p90': round(float(np.percentile(lat, 90)), 1),
        'lat_ms_max': round(float(lat.max()), 1),
        'cpu_s_per_s': round(cpu / (seconds + 0.3), 3),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--rates', default='5,10,15,30,60', help='depth Hz, comma list')
    ap.add_argument('--json', help='also write the rows to this file')
    args = ap.parse_args()
    rows = []
    hdr = (f"{'depth_hz':>8} {'mode':>5} {'est_hz':>6} {'mean':>6} {'p50':>6} "
           f"{'p90':>6} {'max':>6} {'dup':>3} {'cpu':>6}   (latency ms, cpu s/s)")
    print(hdr)
    for hz in [float(r) for r in args.rates.split(',')]:
        for wake in (False, True):
            r = run(wake, hz, args.seconds)
            rows.append(r)
            print(f"{hz:8g} {r['mode']:>5} {r['estimate_hz']:6.2f} {r['lat_ms_mean']:6.1f} "
                  f"{r['lat_ms_p50']:6.1f} {r['lat_ms_p90']:6.1f} {r['lat_ms_max']:6.1f} "
                  f"{r['double_processed']:3d} {r['cpu_s_per_s']:6.3f}", flush=True)
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=1))


if __name__ == '__main__':
    main()
