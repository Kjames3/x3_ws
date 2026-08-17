#!/usr/bin/env python3
"""Overnight sweep #3: the axes the hyperparameter sweeps never touched.

The first two sweeps covered hyperparameters thoroughly (29 models: compression
fraction, delta clamp, noise multiplier, width, depth, T, Huber delta, and the
compression x clamp grid). Time-compression was the entire win and every other
axis landed within noise, so more hyperparameter search is not worth a GPU-night.

What has NEVER been tried is different *data* and a different *objective*:

  1. SIC as TRAINING data. It has only ever been the referee. Thor-Magni is
     clean mocap with synthetic noise bolted on; SIC is real sensor data with
     real depth error, real occlusion, and real sitting people. This was the
     original question that started the whole thread.

  2. Explicit static-jitter negatives. Phantom velocity is currently handled by
     a downstream gate (MIN_COHERENT_FRAMES=4, shipped 2026-08-16). Teaching the
     net that an oscillating centroid means zero attacks it at the source, and
     if it works the gate could drop to k=2 and give a real walker back 0.2 s.

Split discipline: once SIC is in training it cannot referee itself, so the split
is leave-one-ENVIRONMENT-out. Train on Cafeteria + Corridor + Hallway (8,699
windows), test on Courtyard (2,655, 100% pedestrian, 81% moving) -- an unseen
environment, which is a strictly better generalization test than the pooled set.

Three objectives, reported for every model:
  sic_held  unseen-environment SIC (Courtyard). The honest accuracy number.
  fast      time-compressed 1.6-4.0 m/s. The saturation regime.
  jitter    synthetic static jitter, true label 0. The phantom proxy.

Deliberately NOT swept: a longer prediction horizon (target at +2/+3 frames to
compensate the 5.9-7.4 Hz loop). Its entire benefit is latency, which no offline
metric here can see, so it would produce a model that cannot be judged until it
is on the robot. It belongs in a hardware test, not a GPU-night.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

import retrain_velocity_mlp as R
import thor_magni_augment as A
from retrain_velocity_mlp import VelocityMLP, metrics, predict, train

WINDOW_SIZE = 10
CLIP_D = 0.25              # must match the robot's _build_window_features
TEST_ENV = "Courtyard"     # held out entirely from training

# Measured OAK-D depth noise: sigma ~= 0.0025 * Z^2 in the near field.
# See project_oak_depth_characterization.
SIGMA_COEF = 0.0025


# --------------------------------------------------------------------------
# data
# --------------------------------------------------------------------------
def load_sic(npz_path):
    """Split the SIC referee set by environment. Windows are already in the
    deployed convention at CLIP_D 0.25 (sic_to_windows.py --verify)."""
    d = np.load(npz_path, allow_pickle=True)
    X, y, scene = d["X"].astype(np.float32), d["y"].astype(np.float32), d["scene"]
    env = np.array([s.split("_")[1] for s in scene])
    held = env == TEST_ENV
    return (X[~held], y[~held]), (X[held], y[held])


def jitter_windows(n, rng, z_lo=1.5, z_hi=4.0):
    """Synthetic static objects: a fixed point with a noisy centroid, label 0.

    This is the phantom generator, built from the measured depth-noise law
    rather than guessed. The window is emitted in the deployed convention
    (translation-normalized to frame 0, frame-0 deltas zeroed, deltas clamped),
    so it is indistinguishable in format from a real window.
    """
    Z = rng.uniform(z_lo, z_hi, size=n).astype(np.float32)
    sigma = SIGMA_COEF * Z ** 2                      # metres, per axis, per frame
    # Lateral jitter is dominated by the same depth error through the projection,
    # so use the same scale rather than inventing a second constant.
    fwd = Z[:, None] + rng.normal(0, sigma[:, None], (n, WINDOW_SIZE))
    lat = rng.normal(0, sigma[:, None], (n, WINDOW_SIZE))

    X = np.zeros((n, 4 * WINDOW_SIZE), np.float32)
    rx = fwd - fwd[:, :1]                            # translation normalization
    ry = lat - lat[:, :1]
    for i in range(WINDOW_SIZE):
        X[:, 4 * i] = rx[:, i]
        X[:, 4 * i + 1] = ry[:, i]
        if i:
            X[:, 4 * i + 2] = np.clip(rx[:, i] - rx[:, i - 1], -CLIP_D, CLIP_D)
            X[:, 4 * i + 3] = np.clip(ry[:, i] - ry[:, i - 1], -CLIP_D, CLIP_D)
    return X, np.zeros((n, 2), np.float32)


def build_thor(proc_dir, split, seed, cache={}):
    """Thor-Magni at the winning compress25 recipe."""
    key = (split, seed)
    if key not in cache:
        cache[key] = A.build(proc_dir, split, strides=(1, 2, 3), noise=True,
                             compress=1, seed=seed, clip_d=CLIP_D, max_label=4.0)
        Xc, yc = A.build(proc_dir, split, strides=(1,), noise=True, compress=2,
                         seed=seed + 1, clip_d=CLIP_D, max_label=4.0)
        n = int(0.25 * len(cache[key][0]))
        idx = np.random.default_rng(seed).choice(len(Xc), min(n, len(Xc)), replace=False)
        cache[key] = (np.concatenate([cache[key][0], Xc[idx]]),
                      np.concatenate([cache[key][1], yc[idx]]))
    return cache[key]


def fast_set(proc_dir):
    """Held-out test sequences compressed into the 1.6-4.0 m/s band."""
    Xs, ys = [], []
    for c in (2, 3):
        X, y = A.build(proc_dir, "test", strides=(1,), noise=True, compress=c,
                       seed=900 + c, clip_d=CLIP_D, max_label=4.0)
        Xs.append(X)
        ys.append(y)
    X, y = np.concatenate(Xs), np.concatenate(ys)
    k = np.linalg.norm(y, axis=1) >= 1.6
    return X[k], y[k]


# --------------------------------------------------------------------------
# configs
# --------------------------------------------------------------------------
CONFIGS = {
    # control: the shipped recipe, retrained here so every comparison below is
    # against a model built on the same machine, seed and data build.
    "ctrl_compress25": dict(sic=0.0, jitter=0.0, finetune=False),
    # SIC blended in, oversampled so 8.7k windows are not drowned by ~800k.
    "sic_mix":         dict(sic=8.0, jitter=0.0, finetune=False),
    # SIC as a fine-tune on top of the control, low LR, Thor-Magni slice retained
    # so it does not catastrophically forget the fast regime.
    "sic_ft":          dict(sic=1.0, jitter=0.0, finetune=True),
    # phantom negatives only
    "jitter":          dict(sic=0.0, jitter=0.10, finetune=False),
    # both levers
    "jitter_sic_mix":  dict(sic=8.0, jitter=0.10, finetune=False),
}


def assemble(cfg, thor, sic_tr, rng):
    """Build one training set from the Thor-Magni base + optional SIC + jitter."""
    Xt, yt = thor
    parts_X, parts_y = [Xt], [yt]
    if cfg["sic"] > 0:
        reps = max(1, int(round(cfg["sic"])))
        parts_X += [sic_tr[0]] * reps
        parts_y += [sic_tr[1]] * reps
    if cfg["jitter"] > 0:
        n = int(cfg["jitter"] * len(Xt))
        jX, jy = jitter_windows(n, rng)
        parts_X.append(jX)
        parts_y.append(jy)
    return np.concatenate(parts_X), np.concatenate(parts_y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proc-dir", required=True, help="thor_magni_processed/")
    ap.add_argument("--sic-npz", required=True, help="sic_eval_v2.npz")
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--patience", type=int, default=6)
    ap.add_argument("--only", default=None, help="comma-separated config names")
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"device {dev}", flush=True)

    sic_tr, sic_te = load_sic(a.sic_npz)
    print(f"SIC train {sic_tr[0].shape}  held-out {TEST_ENV} {sic_te[0].shape}", flush=True)

    thor_tr = build_thor(a.proc_dir, "train", 100)
    thor_va = build_thor(a.proc_dir, "val", 200)
    print(f"Thor train {thor_tr[0].shape}  val {thor_va[0].shape}", flush=True)

    fX, fy = fast_set(a.proc_dir)
    rng = np.random.default_rng(7)
    jX, jy = jitter_windows(20000, np.random.default_rng(1234))
    print(f"fast {fX.shape}  jitter-eval {jX.shape}", flush=True)

    names = a.only.split(",") if a.only else list(CONFIGS)
    for name in names:
        cfg = CONFIGS[name]
        t0 = time.time()
        print(f"\n===== {name}  {cfg} =====", flush=True)
        try:
            Xtr, ytr = assemble(cfg, thor_tr, sic_tr, rng)
            Xva, yva = assemble(cfg, thor_va, sic_tr, rng)
            print(f"  train {Xtr.shape}", flush=True)

            mX, sX = Xtr.mean(0), Xtr.std(0)
            sX[sX < 1e-8] = 1.0
            mY, sY = ytr.mean(0), ytr.std(0)

            if cfg["finetune"]:
                base = out / "ctrl_compress25.torchscript"
                if not base.is_file():
                    print(f"  SKIP: {base} missing -- run ctrl_compress25 first", flush=True)
                    continue
                # Reuse the control's scaler: a fine-tune must not change the
                # input normalization out from under the weights it starts from.
                bs = json.load(open(out / "scaler_params_ctrl_compress25.json"))
                mX = np.array(bs["scaler_X"]["mean"], np.float32)
                sX = np.array(bs["scaler_X"]["scale"], np.float32)
                mY = np.array(bs["scaler_y"]["mean"], np.float32)
                sY = np.array(bs["scaler_y"]["scale"], np.float32)
                model = VelocityMLP()
                model.load_state_dict(torch.jit.load(str(base)).state_dict())
                _orig = R.VelocityMLP
                R.VelocityMLP = lambda *ar, **kw: model
                _lr = R.LR if hasattr(R, "LR") else None
                m = train(((Xtr - mX) / sX).astype(np.float32),
                          ((ytr - mY) / sY).astype(np.float32),
                          ((Xva - mX) / sX).astype(np.float32),
                          ((yva - mY) / sY).astype(np.float32),
                          dev, max(10, a.epochs // 3), a.patience)
                R.VelocityMLP = _orig
            else:
                m = train(((Xtr - mX) / sX).astype(np.float32),
                          ((ytr - mY) / sY).astype(np.float32),
                          ((Xva - mX) / sX).astype(np.float32),
                          ((yva - mY) / sY).astype(np.float32),
                          dev, a.epochs, a.patience)

            torch.jit.script(m.to("cpu").eval()).save(str(out / f"{name}.torchscript"))
            json.dump({"scaler_X": {"mean": mX.tolist(), "scale": sX.tolist()},
                       "scaler_y": {"mean": mY.tolist(), "scale": sY.tolist()},
                       "cfg": cfg},
                      open(out / f"scaler_params_{name}.json", "w"), indent=2)
            m.to(dev)

            S = (mX, sX, mY, sY)
            row = {"name": name, "cfg": cfg, "secs": round(time.time() - t0),
                   "n_train": int(len(Xtr))}
            row["sic_held"] = metrics(predict(m, sic_te[0], *S, device=dev), sic_te[1])
            row["fast"] = metrics(predict(m, fX, *S, device=dev), fy)
            jp = predict(m, jX, *S, device=dev)
            js = np.linalg.norm(jp, axis=1)
            row["jitter"] = {"mean": float(js.mean()), "p95": float(np.percentile(js, 95)),
                             "max": float(js.max()), "frac_gt_0.3": float((js > 0.3).mean())}
            with open(out / "results.jsonl", "a") as fh:
                fh.write(json.dumps(row) + "\n")
            print(f"  sic_held {row['sic_held']['rmse']:.4f}  "
                  f"fast {row['fast']['rmse']:.4f}  "
                  f"jitter mean {row['jitter']['mean']:.3f} "
                  f"hot {100*row['jitter']['frac_gt_0.3']:.1f}%  "
                  f"[{row['secs']}s]", flush=True)
        except Exception as e:
            import traceback
            traceback.print_exc()
            with open(out / "results.jsonl", "a") as fh:
                fh.write(json.dumps({"name": name, "error": str(e)}) + "\n")

    print("\nsweep complete", flush=True)


if __name__ == "__main__":
    sys.exit(main())
