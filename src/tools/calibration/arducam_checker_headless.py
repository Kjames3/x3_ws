#!/usr/bin/env python3
"""Headless checkerboard view grabber for the Arducam (no display needed over SSH).

Auto-saves a view whenever the board is found and it differs from every saved
view (mean corner shift > --min-shift px). Prints which cells of a 3x3 image
grid the saved views have covered, so you know where to move the board next.
Ctrl-C (or --views reached) stops, then fits the fisheye model with
arducam_viewer's fitter and saves config/arducam_uc844_intrinsics.json.

    python3 src/tools/calibration/arducam_checker_headless.py --squares 9x6 --square-mm 23.3

x3_server must be stopped (it holds the camera).
"""
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[1] / "viewers"))

import argparse
import glob
import os
import time

import cv2
import numpy as np

from arducam_viewer import find_board, fit_fisheye, save_fisheye, INTR_PATH


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--squares", default="9x6")
    ap.add_argument("--square-mm", type=float, required=True)
    ap.add_argument("--views", type=int, default=35)
    ap.add_argument("--min-shift", type=float, default=40.0)
    ap.add_argument("--min-span", type=float, default=300.0,
                    help="board must span at least this many px (width of its corner grid)")
    ap.add_argument("--out", default=os.path.expanduser("~/calib/arducam_views"))
    a = ap.parse_args()
    sq = [int(v) for v in a.squares.lower().split("x")]
    pattern = (sq[0] - 1, sq[1] - 1)
    os.makedirs(a.out, exist_ok=True)

    dev = sorted(glob.glob("/dev/v4l/by-id/*Arducam*-video-index0"))[0]
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
    views, cover = [], np.zeros((3, 3), int)
    last_t, last_msg, prev = 0.0, 0.0, None
    print(f"grabbing from {dev}; move the board slowly, Ctrl-C to stop and fit")
    try:
        while len(views) < a.views:
            ok, frame = cap.read()
            if not ok:
                continue
            gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
            moving = prev is not None and np.mean(np.linalg.norm(c - prev, axis=2)) > 1.5
            prev = c
            if span < a.min_span:
                if now - last_msg > 3:
                    print(f"  board too small ({span:.0f} px wide, need {a.min_span:.0f}): bring it closer")
                    last_msg = now
                continue
            if moving or now - last_t < 0.7:
                continue
            if views and min(np.mean(np.linalg.norm(c - v, axis=2)) for v in views) < a.min_shift:
                continue
            views.append(c)
            last_t = now
            cv2.imwrite(os.path.join(a.out, f"view_{len(views):03d}.png"), gray)
            for p in c.reshape(-1, 2):
                cover[min(2, int(3 * p[1] / h)), min(2, int(3 * p[0] / w))] += 1
            empty = [f"{'top mid bot'.split()[r]}-{'left ctr right'.split()[k]}"
                     for r in range(3) for k in range(3) if cover[r, k] == 0]
            print(f"view {len(views):2d} saved; uncovered cells: {', '.join(empty) or 'none'}")
    except KeyboardInterrupt:
        pass
    cap.release()
    print(f"fitting {len(views)} views ...")
    rms, K, D, n = fit_fisheye(views, pattern, a.square_mm / 1000.0, (w, h))
    hfov = save_fisheye(rms, K, D, n, w, h, a.square_mm / 1000.0, pattern)
    print(f"RMS {rms:.3f} px from {n} views; fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} "
          f"cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}; D {D.ravel()}; centre-HFOV {hfov:.1f}")
    print("saved ->", INTR_PATH)


if __name__ == "__main__":
    main()
