"""
thor_magni_augment.py — Rebuild Thor-Magni training windows with sensor realism.

The shipped model trains on mocap trajectories: no depth noise, and exactly one
sample rate. The robot serves it OAK-D Lite stereo depth at 5.9-7.4 Hz. This
module rebuilds the windows from the processed CSVs with both gaps closed:

  * measured depth noise — sigma(Z) taken from a static-scene capture on the
    robot 2026-08-16 (see SIGMA_TABLE). Noise goes on the POSITIONS, then the
    deltas are recomputed from the noisy positions, because that is what a real
    sensor does and because training's dx[i] == rx[i] - rx[i-1] identity has to
    survive.
  * true multi-rate sampling — the CSVs are natively 10 Hz, so a stride of 2 or
    3 gives a genuine 5 Hz / 3.33 Hz window rather than a simulated one. Each
    strided window then gets the same dt rescale the robot applies.

The split is reproduced exactly from 03_build_training_windows.py (sorted file
list, np.random.seed(42) permutation, 5 test / 5 val) so the test set does not
leak. The label is the velocity one frame (0.1 s) after the window, also
matching that script, at every stride.
"""

import csv
import numpy as np
from collections import defaultdict
from pathlib import Path

T           = 10
CLIP_D      = 0.25
VAL_SEQS    = 5
TEST_SEQS   = 5
FX_EFF      = 307.8    # depth intrinsics as the estimator sees them, 240x320
SIGMA_PX    = 1.5      # centroid jitter in pixels

# Measured on the robot: per-pixel temporal std of depth, edges excluded.
SIGMA_TABLE = np.array([
    [0.6, 0.0009], [1.0, 0.0025], [1.4, 0.0054],
    [2.2, 0.0587], [2.6, 0.0458], [3.0, 0.0984],
    [3.4, 0.0634], [3.8, 0.1026], [4.2, 0.1683],
])


def sigma_depth(Z):
    """Interpolate the measured curve; extrapolate as Z^2 outside its support."""
    Z = np.abs(np.asarray(Z, dtype=np.float64))
    zt, st = SIGMA_TABLE[:, 0], SIGMA_TABLE[:, 1]
    s = np.interp(Z, zt, st)
    lo, hi = Z < zt[0], Z > zt[-1]
    s = np.where(lo, st[0] * (Z / zt[0]) ** 2, s)
    s = np.where(hi, st[-1] * (Z / zt[-1]) ** 2, s)
    return s


def sigma_lateral(Z):
    """Lateral centroid error: a pixel of jitter is Z/fx metres on the ground."""
    return np.abs(np.asarray(Z, dtype=np.float64)) * (SIGMA_PX / FX_EFF)


def load_sequence(path):
    """CSV -> {body: (times, rel_x, rel_y, vx, vy)} sorted by time."""
    by = defaultdict(list)
    with open(path) as fh:
        for r in csv.DictReader(fh):
            try:
                by[r["body"]].append((float(r["time"]), float(r["rel_x"]),
                                      float(r["rel_y"]), float(r["vx"]), float(r["vy"])))
            except (ValueError, TypeError):
                continue        # NaN / blank rows, dropped like the original
    out = {}
    for b, rows in by.items():
        a = np.array(sorted(rows), dtype=np.float64)
        if len(a) > T + 1:
            out[b] = a
    return out


def splits(processed_dir):
    """Reproduce the split from 03_build_training_windows.py exactly."""
    files = sorted(Path(processed_dir).glob("*_features.csv"))
    np.random.seed(42)
    idx = np.random.permutation(len(files))
    return {"test":  [files[i] for i in idx[:TEST_SEQS]],
            "val":   [files[i] for i in idx[TEST_SEQS:TEST_SEQS + VAL_SEQS]],
            "train": [files[i] for i in idx[TEST_SEQS + VAL_SEQS:]]}


def windows_from_track(a, stride, rng, noise, deployed):
    """Yield (features40, label2) for one body's track.

    a: (N,5) [time, rel_x, rel_y, vx, vy] at 10 Hz.
    stride: 1 -> 10 Hz, 2 -> 5 Hz, 3 -> 3.33 Hz.
    """
    n = len(a)
    span = (T - 1) * stride
    dt_scale = 1.0 / stride
    X, Y = [], []
    for start in range(0, n - span - 1):
        idx = np.arange(start, start + span + 1, stride)
        rx = a[idx, 1].copy()
        ry = a[idx, 2].copy()

        if noise:
            Z = np.maximum(0.3, np.abs(rx))
            rx = rx + rng.normal(0.0, sigma_depth(Z))
            ry = ry + rng.normal(0.0, sigma_lateral(Z))

        # Label: 0.1 s after the window's last sample, at every stride.
        lab = a[idx[-1] + 1, 3:5]
        if not np.all(np.isfinite(lab)):
            continue

        rx0, ry0 = rx[0], ry[0]
        f = []
        for i in range(T):
            if deployed:
                rn = (rx[i] - rx0) * dt_scale
                yn = (ry[i] - ry0) * dt_scale
            else:
                rn, yn = rx[i], ry[i]
            if i == 0:
                dx = dy = 0.0
            else:
                dx = np.clip((rx[i] - rx[i - 1]) * dt_scale, -CLIP_D, CLIP_D)
                dy = np.clip((ry[i] - ry[i - 1]) * dt_scale, -CLIP_D, CLIP_D)
            f += [rn, yn, dx, dy]
        X.append(f)
        Y.append(lab)
    return X, Y


def build(processed_dir, split, strides=(1,), noise=False, deployed=True, seed=0):
    """Build one split. Returns (X (N,40) float32, y (N,2) float32)."""
    rng = np.random.default_rng(seed)
    X, Y = [], []
    for path in splits(processed_dir)[split]:
        for a in load_sequence(path).values():
            for s in strides:
                xs, ys = windows_from_track(a, s, rng, noise, deployed)
                X += xs
                Y += ys
    if not X:
        return np.zeros((0, 4 * T), np.float32), np.zeros((0, 2), np.float32)
    return np.array(X, np.float32), np.array(Y, np.float32)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--processed", required=True)
    args = ap.parse_args()
    print("sigma(Z):", {z: round(float(sigma_depth(z)), 4) for z in (0.5, 1, 2, 3, 4)})
    for sp in ("train", "val", "test"):
        X, y = build(args.processed, sp)
        print(f"{sp:6s} clean stride1: X{X.shape} y{y.shape}")
