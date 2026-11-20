#!/usr/bin/env python3
"""Headless checkerboard intrinsics check for an OAK RGB camera (CAM_A).

Opens the camera exactly as calib_capture.py does (same resolution, so the
result applies to tonight's calibration bags), auto-saves a view whenever the
board is found, still, big enough, and in a new spot, then fits a pinhole
model with the rational distortion terms and compares fx/fy with the EEPROM.

    python3 src/tools/calibration/oak_checker_headless.py front --squares 9x6 --square-mm 23.3

x3_server must be stopped (it holds the OAKs). Writes ~/calib/oak_<cam>_intrinsics.json
and the views to ~/calib/oak_<cam>_views/.
"""
import sys as _sys
from pathlib import Path as _Path
_HERE = _Path(__file__).resolve().parent
_sys.path.insert(0, str(_HERE))
_sys.path.insert(0, str(_HERE.parent / "viewers"))

import argparse
import json
import os
import time

import cv2
import numpy as np

from arducam_viewer import find_board
from calib_capture import OakCamera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("camera", choices=["front", "rear"])
    ap.add_argument("--squares", default="9x6")
    ap.add_argument("--square-mm", type=float, required=True)
    ap.add_argument("--views", type=int, default=35)
    ap.add_argument("--min-shift", type=float, default=60.0)
    ap.add_argument("--min-span", type=float, default=450.0,
                    help="board corner grid must span at least this many px")
    a = ap.parse_args()
    sq = [int(v) for v in a.squares.lower().split("x")]
    pattern = (sq[0] - 1, sq[1] - 1)
    out = os.path.expanduser(f"~/calib/oak_{a.camera}_views")
    os.makedirs(out, exist_ok=True)

    cam = OakCamera(a.camera)
    if not cam.start(timeout=30):
        _sys.exit(f"{a.camera} OAK did not start")
    K0 = np.array(cam.K, float).reshape(3, 3)
    print(f"EEPROM fx {K0[0, 0]:.1f} fy {K0[1, 1]:.1f} cx {K0[0, 2]:.1f} cy {K0[1, 2]:.1f}")

    views, cover = [], np.zeros((3, 3), int)
    last_t = last_msg = 0.0
    prev = None
    print("move the board slowly; Ctrl-C to stop and fit")
    try:
        while len(views) < a.views:
            frame = cam.latest()
            if frame is None:
                time.sleep(0.05)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            c = find_board(gray, pattern)
            now = time.time()
            if c is None:
                prev = None
                if now - last_msg > 3:
                    print("  (no board in view)")
                    last_msg = now
                continue
            span = np.ptp(c.reshape(-1, 2), axis=0).max()
            moving = prev is not None and np.mean(np.linalg.norm(c - prev, axis=2)) > 2.0
            prev = c
            if span < a.min_span:
                if now - last_msg > 3:
                    print(f"  board too small ({span:.0f} px, need {a.min_span:.0f}): bring it closer")
                    last_msg = now
                continue
            if moving or now - last_t < 0.7:
                continue
            if views and min(np.mean(np.linalg.norm(c - v, axis=2)) for v in views) < a.min_shift:
                continue
            views.append(c)
            last_t = now
            cv2.imwrite(os.path.join(out, f"view_{len(views):03d}.png"), gray)
            for p in c.reshape(-1, 2):
                cover[min(2, int(3 * p[1] / h)), min(2, int(3 * p[0] / w))] += 1
            empty = [f"{'top mid bot'.split()[r]}-{'left ctr right'.split()[k]}"
                     for r in range(3) for k in range(3) if cover[r, k] == 0]
            print(f"view {len(views):2d} saved; uncovered cells: {', '.join(empty) or 'none'}")
    except KeyboardInterrupt:
        pass
    cam.stop()

    obj = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * a.square_mm / 1000.0
    flags = cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_USE_INTRINSIC_GUESS
    fit = lambda V: cv2.calibrateCamera([obj] * len(V), V, (w, h), K0.copy(), None, flags=flags)
    rms, K, D, rv, tv = fit(views)
    err = [cv2.norm(views[i], cv2.projectPoints(obj, rv[i], tv[i], K, D)[0], cv2.NORM_L2)
           / np.sqrt(len(obj)) for i in range(len(views))]
    keep = [v for v, e in zip(views, err) if e < 1.0]
    if len(keep) < len(views) and len(keep) >= 10:
        print(f"dropping {len(views) - len(keep)} views with per-view RMS > 1 px")
        rms, K, D, rv, tv = fit(keep)
    print(f"RMS {rms:.3f} px from {len(keep)} views")
    print(f"fit    fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}")
    print(f"EEPROM fx {K0[0, 0]:.1f} fy {K0[1, 1]:.1f} cx {K0[0, 2]:.1f} cy {K0[1, 2]:.1f}")
    print(f"fx ratio fit/EEPROM = {K[0, 0] / K0[0, 0]:.4f}")
    res = {"camera": a.camera, "w": w, "h": h, "model": "rational_polynomial",
           "K": K.ravel().tolist(), "D": D.ravel().tolist(), "rms_px": rms,
           "n_views": len(keep), "eeprom_K": K0.ravel().tolist(), "eeprom_D": cam.D,
           "board": f"{a.squares} squares, {a.square_mm} mm", "date": time.strftime("%Y-%m-%d")}
    path = os.path.expanduser(f"~/calib/oak_{a.camera}_intrinsics.json")
    json.dump(res, open(path, "w"), indent=2)
    print("saved ->", path)


if __name__ == "__main__":
    main()
