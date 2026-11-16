#!/usr/bin/env python3
"""Smoke-test OakDCamera on whatever depthai is installed (written for the v3 port).

Runs the driver exactly as server_x3 builds it (mono 80, accel 500 / gyro 400,
yolo26n blob if present) for --seconds, then prints PASS/FAIL per check:

  * the driver connected and the link is USB SUPER
  * "speckle filter on" was logged (not "unavailable": unfiltered speckle
    turns into phantom obstacles)
  * depth flows at a sane rate, with the CAM_A-aligned NN grid shape when
    spatial detection is on (MONO 640x400 otherwise)
  * IMU samples arrive
  * NN packets arrive and the raw-FP16 fast read was verified (not DISAGREES)
  * no NN stall, no device error / reconnect

The OAK opens exclusively, so stop anything holding it first
(`sudo systemctl stop x3_server`). This does NOT check geometry: the logged fx
is computed from the assumed crop. Check that against the lidar-camera overlay
(src/tools/calibration/) once the camera mount is final.

    python3 src/tools/hw_tests/oak_v3_smoke.py [--seconds 20] [--model yolo26n] [--no-spatial]
        [--record]      # C1 path: record_rgbd + subpixel, expects RGB-D pairs
        [--cycles 2]    # start/cleanup twice in one process (reconnect path)
"""
import argparse
import logging
import sys
import time
from pathlib import Path

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC))

import oakd_driver  # noqa: E402
from oakd_driver import OakDCamera  # noqa: E402


class _Capture(logging.Handler):
    def __init__(self):
        super().__init__()
        self.lines = []

    def emit(self, record):
        self.lines.append(record.getMessage())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--model", default="yolo26n")
    ap.add_argument("--no-spatial", action="store_true")
    ap.add_argument("--record", action="store_true", help="record_rgbd + subpixel (C1 recording path)")
    ap.add_argument("--cycles", type=int, default=1, help="start/cleanup cycles in one process")
    args = ap.parse_args()

    log = logging.getLogger("x3_server")
    log.setLevel(logging.INFO)
    cap = _Capture()
    log.addHandler(cap)
    log.addHandler(logging.StreamHandler(sys.stdout))

    dai = oakd_driver.dai
    print(f"depthai {getattr(dai, '__version__', None)}  (v3 path: {oakd_driver._DAI_V3})")
    blob_dir = SRC / "blobs" / args.model
    blob, cfg = blob_dir / f"{args.model}.blob", blob_dir / "config.json"
    spatial = (not args.no_spatial) and blob.exists()
    if not args.no_spatial and not blob.exists():
        print(f"note: {blob} missing, running depth/IMU only")

    results = []
    for cycle in range(args.cycles):
        if args.cycles > 1:
            print(f"\n---- cycle {cycle + 1}/{args.cycles}")
        cap.lines.clear()
        results.append(run_once(args, cap, blob, cfg, spatial))
    ok = all(results)
    print("\nRESULT:", "PASS" if ok else "FAIL", f"({sum(results)}/{len(results)} cycles)")
    return 0 if ok else 1


def run_once(args, cap, blob, cfg, spatial):
    cam = OakDCamera(mono_fps=80, spatial_blob=str(blob) if spatial else None,
                     spatial_config=str(cfg) if spatial and cfg.exists() else None,
                     conf_threshold=0.35, accel_hz=500, gyro_hz=400,
                     record_rgbd=args.record, subpixel=args.record)
    cam.start()
    t0 = time.monotonic()
    shapes, imu_n, last_imu_ts, fps = set(), 0, None, []
    det_frames, lat_ms, det_samples, pairs = 0, [], [], 0
    try:
        while time.monotonic() - t0 < args.seconds:
            time.sleep(0.25)
            d = cam.get_raw_depth_frame()
            if d is not None:
                shapes.add(tuple(d.shape))
            imu = cam.get_imu()
            if imu is not None and imu["ts"] != last_imu_ts:
                imu_n, last_imu_ts = imu_n + 1, imu["ts"]
            if getattr(cam, "depth_fps", None):
                fps.append(cam.depth_fps)
            dets, meta = cam.get_detection_observation()
            if meta:
                # capture-time estimate must precede receipt on the host clock
                lat_ms.append((meta["receipt_monotonic_ns"] - meta["host_monotonic_estimate_ns"]) / 1e6)
            if dets:
                det_frames += 1
                if len(det_samples) < 4:
                    det_samples.append([(x.get("label"), round(float(x.get("conf", 0)), 2),
                                         x.get("xyz_base_m")) for x in dets[:3]])
            if args.record:
                pairs += len(cam.pop_rgbd_pairs())
    finally:
        cam.cleanup()

    text = "\n".join(cap.lines)
    exp_shape = (cam.nn_h, cam.nn_w) if spatial else (oakd_driver.MONO_H, oakd_driver.MONO_W)
    settled = fps[len(fps) // 2:] or [0]
    checks = [
        ("connected", "OakDCamera: connected" in text),
        ("USB SUPER", getattr(cam, "usb_speed", None) in ("SUPER", "SUPER_PLUS")),
        ("speckle filter on", "speckle filter on" in text and "speckle filter unavailable" not in text),
        (f"depth shape {exp_shape}", shapes == {exp_shape}),
        ("depth fps >= 10 (settled median)", sorted(settled)[len(settled) // 2] >= 10),
        ("IMU samples", imu_n >= 5),
        ("no device error / NN stall", "device error" not in text and "NN stalled" not in text
         and "spatial pipeline failed" not in text),
        ("capture->receipt latency 0..1000 ms", bool(lat_ms) and 0 < min(lat_ms) and max(lat_ms) < 1000)
        if spatial else ("(no NN, latency not checked)", True),
    ]
    if args.record:
        checks.append(("RGB-D pairs produced", pairs >= 5))
    if spatial:
        checks += [
            ("NN fast read verified", "verified against the list path" in text
             and "DISAGREES" not in text),
            ("spatial active", "host-decoded detections" in text),
        ]
    print("\n==== OAK smoke test")
    print(f"depth shapes seen {sorted(shapes)}, fps samples {settled[-5:]}, IMU updates {imu_n}, "
          f"polls with >=1 detection {det_frames}, RGB-D pairs {pairs}")
    if lat_ms:
        lat_ms.sort()
        print(f"capture->receipt ms: min {lat_ms[0]:.1f} median {lat_ms[len(lat_ms) // 2]:.1f} "
              f"max {lat_ms[-1]:.1f}")
    for sample in det_samples:
        print("  detections (label, conf, xyz_base_m):", sample)
    ok = True
    for name, passed in checks:
        ok &= bool(passed)
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    return ok


if __name__ == "__main__":
    sys.exit(main())
