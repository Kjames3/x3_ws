#!/usr/bin/env python3
"""Score every candidate velocity model against ONE recorded feature capture.

Why this exists
---------------
Comparing models on the robot means one walk per model, and a 1.85-2.0 s pace
spread is +-4% on truth -- larger than the differences being resolved. This
replays a single capture through every model instead, so the model is the only
variable and the pace confound is gone.

What it replays
---------------
The deployed speed is NOT the raw MLP output. It passes through, in order:

  1. per-model input scaler          (each model has its OWN scaler)
  2. forward pass
  3. inverse output scaler
  4. clip to +/-2.5 m/s
  5. multiply by conf = min(1, visible_count / WINDOW_SIZE)
  6. inter-frame acceleration clamp (max 3.0 m/s^2 -> 0.3 m/s per frame)
  7. per-frame max over tracks, with GATED tracks contributing 0.0

A scorer that only ran step 2 would not reproduce any number measured on the
robot. Stages are reported separately so it is visible WHERE speed is lost --
the synthetic check that motivated this found all candidate models OVER-read a
clean walker, which means the observed under-read is in the chain, not the model.

Self-check: replaying the deployed model must reproduce the p95 that
score_ab_logs.py reported for the same session. `--expect-p95` asserts it.

Usage
    python3 src/score_velocity_models.py capture.json --true-speed 1.05
    python3 src/score_velocity_models.py capture.json --static
"""
import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
import torch

MODELS_DIR = Path(__file__).parent / "models"
CLIP_MS = 2.5
MAX_ACCEL_MS2 = 3.0


def load_model(name):
    model = torch.jit.load(str(MODELS_DIR / f"{name}.torchscript"),
                           map_location="cpu").eval()
    with open(MODELS_DIR / f"scaler_{name}.json") as stream:
        scaler = json.load(stream)
    return model, {
        "x_mean": np.array(scaler["scaler_X"]["mean"], dtype=np.float32),
        "x_scale": np.array(scaler["scaler_X"]["scale"], dtype=np.float32),
        "y_mean": np.array(scaler["scaler_y"]["mean"], dtype=np.float32),
        "y_scale": np.array(scaler["scaler_y"]["scale"], dtype=np.float32),
        "convention": scaler.get("convention"),
    }


def replay(capture, name, window_size, infer_hz, gate=None):
    """Run one model over the capture, returning per-stage speeds.

    `gate` re-imposes a range gate in software. The capture is deliberately
    recorded with a WIDE gate (the robot is parked, so the CBF concern that
    keeps the deployed gate at 1.8 m does not apply), which yields far more
    scoreable windows. Passing gate=1.8 reproduces the deployed behaviour from
    that same capture, so one walk serves both the self-check and the
    full-lane comparison.
    """
    model, sc = load_model(name)
    rows = capture["rows"]
    if gate is not None:
        rows = [dict(r, status="gated_range") if (r["z"] > gate and
                                                  r["status"] == "ok") else r
                for r in rows]

    ok = [r for r in rows if r["status"] == "ok" and r["feats"]]
    if not ok:
        raise SystemExit(f"capture has no ungated track-frames")

    X = np.array([r["feats"] for r in ok], dtype=np.float32)
    with torch.no_grad():
        pred = model(torch.from_numpy((X - sc["x_mean"]) / sc["x_scale"])).numpy()
    vel = np.clip(pred * sc["y_scale"] + sc["y_mean"], -CLIP_MS, CLIP_MS)
    raw_speed = np.hypot(vel[:, 0], vel[:, 1])

    # stage 5: confidence multiply, applied to the COMPONENTS (the deployed
    # code scales vx and vy, then recomputes speed).
    conf = np.array([min(1.0, r["visible_count"] / window_size) for r in ok],
                    dtype=np.float32)
    vxy = vel * conf[:, None]
    conf_speed = np.hypot(vxy[:, 0], vxy[:, 1])

    # stage 6+7: the acceleration clamp is applied PER COMPONENT against the
    # previous frame's estimate for that track id, and gated tracks enter that
    # history at vx=vy=0. So a track that leaves the gate and returns must ramp
    # back up at max_delta per frame -- a large rate limiter that a scalar-speed
    # clamp does not reproduce.
    per_frame = {}
    for r, v in zip(ok, vxy):
        per_frame.setdefault(r["frame"], {})[r["tid"]] = [float(v[0]), float(v[1])]
    for r in rows:
        if r["status"] != "ok":
            per_frame.setdefault(r["frame"], {}).setdefault(r["tid"], [0.0, 0.0])

    max_delta = MAX_ACCEL_MS2 / infer_hz
    prev, frame_max = {}, []
    for frame in sorted(per_frame):
        current = {}
        for tid, (vx, vy) in per_frame[frame].items():
            if tid in prev:
                pvx, pvy = prev[tid]
                if abs(vx - pvx) > max_delta:
                    vx = pvx + math.copysign(max_delta, vx - pvx)
                if abs(vy - pvy) > max_delta:
                    vy = pvy + math.copysign(max_delta, vy - pvy)
            current[tid] = (vx, vy)
        prev = current
        frame_max.append(max((math.hypot(*v) for v in current.values()),
                             default=0.0))

    return {
        "raw": raw_speed,
        "conf": conf_speed,
        "frame_max": np.array(frame_max, dtype=np.float32),
        "convention": sc["convention"],
        "n_ok": len(ok),
        "n_frames": len(frame_max),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--models", default="v1,v2,v3,compress25")
    ap.add_argument("--true-speed", type=float, default=None,
                    help="tape-measured walker speed; omit for a static capture")
    ap.add_argument("--static", action="store_true",
                    help="phantom capture: truth is 0 for every frame")
    ap.add_argument("--expect-p95", type=float, default=None,
                    help="assert the FIRST model reproduces this live p95")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--gate", type=float, default=None,
                    help="re-impose a range gate in software (e.g. 1.8 to "
                         "reproduce deployed behaviour from a wide capture)")
    args = ap.parse_args()

    with open(args.capture) as stream:
        capture = json.load(stream)
    window_size = capture.get("window_size", 10)
    infer_hz = capture.get("infer_hz", 10)

    print(f"capture: {os.path.basename(args.capture)}")
    print(f"  deployed model at capture time: {capture.get('model_path')}")
    print(f"  window_size={window_size}  infer_hz={infer_hz}  "
          f"gate={capture.get('max_speed_range_m')} m")
    counts = {}
    for r in capture["rows"]:
        counts[r["status"]] = counts.get(r["status"], 0) + 1
    print(f"  track-frames by status: {counts}")
    if args.gate is not None:
        print(f"  re-imposing range gate at {args.gate} m in software")
    print()

    names = [n.strip() for n in args.models.split(",") if n.strip()]
    header = (f"{'model':12s} {'raw p95':>8s} {'conf p95':>9s} {'frame p95':>10s} "
              f"{'max':>7s} {'>0.30':>7s}")
    if args.true_speed:
        header += f" {'err':>8s}"
    print(header)
    print("-" * len(header))

    first_p95 = None
    for name in names:
        res = replay(capture, name, window_size, infer_hz, gate=args.gate)
        fm = res["frame_max"]
        p95 = float(np.percentile(fm, 95))
        if first_p95 is None:
            first_p95 = p95
        line = (f"{name:12s} {np.percentile(res['raw'],95):8.3f} "
                f"{np.percentile(res['conf'],95):9.3f} {p95:10.3f} "
                f"{fm.max():7.3f} {int((fm>0.30).sum()):7d}")
        if args.true_speed:
            line += f" {100*(p95-args.true_speed)/args.true_speed:+7.0f}%"
        print(line)
        if args.static and fm.max() > 0:
            print(f"{'':12s}  ^ static capture: every non-zero value is a phantom")

    # Exact per-window self-check: replaying the DEPLOYED model must reproduce
    # the numbers it actually produced at capture time. This is sampling
    # independent, unlike comparing p95 against the CSV.
    ref = [r for r in capture["rows"]
           if r["status"] == "ok" and r.get("vx_model") is not None]
    if ref:
        deployed = os.path.basename(str(capture.get("model_path", "")))
        name = {"velocity_mlp.torchscript": "v1"}.get(deployed)
        if name in names:
            model, sc = load_model(name)
            X = np.array([r["feats"] for r in ref], dtype=np.float32)
            with torch.no_grad():
                pred = model(torch.from_numpy(
                    (X - sc["x_mean"]) / sc["x_scale"])).numpy()
            got = np.clip(pred * sc["y_scale"] + sc["y_mean"], -CLIP_MS, CLIP_MS)
            want = np.array([[r["vx_model"], r["vy_model"]] for r in ref],
                            dtype=np.float32)
            err = float(np.abs(got - want).max())
            ok_exact = err < 1e-3
            print(f"\nself-check [{'OK' if ok_exact else 'FAILED'}]: replayed "
                  f"{name} vs the {len(ref)} outputs it produced live -- "
                  f"max component error {err:.2e}")
            if not ok_exact:
                print("  The replay chain does not reproduce the deployed "
                      "model's own numbers; the comparison is not trustworthy.")
                raise SystemExit(1)
    else:
        print("\nself-check [SKIPPED]: capture has no recorded deployed "
              "outputs (recorded before output capture was added)")

    if args.expect_p95 is not None:
        delta = abs(first_p95 - args.expect_p95)
        status = "OK" if delta <= args.tol else "FAILED"
        print(f"\nself-check [{status}]: replayed {names[0]} p95={first_p95:.3f} "
              f"vs live {args.expect_p95:.3f} (delta {delta:.3f}, tol {args.tol})")
        if delta > args.tol:
            print("  The replay does not reproduce the live number, so the "
                  "model comparison below it cannot be trusted.")
            raise SystemExit(1)


if __name__ == "__main__":
    main()
