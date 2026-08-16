"""
sweep_velocity_mlp.py — Overnight hyperparameter/augmentation sweep.

Every config is scored on the same held-out sets, with SIC (real sensor data
with ground truth) as the referee — it is the only test set that is not a model
of reality. Results append to a JSONL as each run finishes, so a partial sweep
is still readable.

    python3 src/sweep_velocity_mlp.py --processed <thor_magni_processed/> \\
        --sic sic_eval.npz --out-dir sweep_out/ [--only 0,3,7] [--epochs 60]

Priority of the axes, from what the measurements say:

  compress   THE saturation fix. The model tops out at 1.83 m/s and the training
             labels' p99 is 1.84 — the tail is missing, not unlearnable. Time
             compression manufactures it from walking data.
  clip_d     the +/-0.25 delta clamp is a 2.5 m/s ceiling on the FEATURES, so
             compression past x2 is wasted unless this moves with it. Any winner
             with clip_d != 0.25 REQUIRES the same change in
             velocity_estimator._build_window_features before deploying.
  noise_mult is the measured sigma(Z) curve right? SIC decides.
  T          window length; 10 frames = 1.0 s of history.
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

import thor_magni_augment as A
from retrain_velocity_mlp import VelocityMLP, metrics, predict, train


def make_configs():
    """(name, dict) pairs. Ordered so the most informative run first."""
    base = dict(compress_frac=0.0, noise_mult=1.0, clip_d=0.25, T=10,
                hidden=(256, 128, 64), dropout=0.2, max_label=4.0, huber=1.0)
    cfgs = [("baseline_v3", dict(base))]

    for frac in (0.10, 0.25, 0.40):                      # axis 1: saturation
        cfgs.append((f"compress{int(frac*100)}", {**base, "compress_frac": frac}))
    for cd in (0.40, 0.60):                              # axis 2: feature ceiling
        cfgs.append((f"clip{cd}", {**base, "compress_frac": 0.25, "clip_d": cd}))
    for nm in (0.5, 2.0):                                # axis 3: is sigma(Z) right
        cfgs.append((f"noise{nm}", {**base, "noise_mult": nm}))
    for T in (15, 20):                                   # axis 4: history length
        cfgs.append((f"T{T}", {**base, "T": T}))
    for h in ((512, 256, 128), (128, 64)):               # axis 5: capacity
        cfgs.append((f"hidden{len(h)}x{h[0]}", {**base, "hidden": h}))
    cfgs.append(("huber0.3", {**base, "huber": 0.3}))
    # best-guess combination of the individually promising axes
    cfgs.append(("combo", {**base, "compress_frac": 0.25, "clip_d": 0.40}))
    return cfgs


def build_split(P, split, cfg, seed, with_compression):
    """Assemble one split under a config: clean + noisy multi-rate (+ compressed)."""
    A.T = cfg["T"]
    kw = dict(noise_mult=cfg["noise_mult"], clip_d=cfg["clip_d"],
              max_label=cfg["max_label"])
    parts = [A.build(P, split, strides=(1,), noise=False, seed=seed, **kw),
             A.build(P, split, strides=(1,), noise=True,  seed=seed + 1, **kw),
             A.build(P, split, strides=(2,), noise=True,  seed=seed + 2, **kw),
             A.build(P, split, strides=(3,), noise=True,  seed=seed + 3, **kw)]
    X = np.concatenate([p[0] for p in parts])
    y = np.concatenate([p[1] for p in parts])

    frac = cfg["compress_frac"]
    if with_compression and frac > 0:
        cX, cy = [], []
        for c in (2, 3):
            a, b = A.build(P, split, strides=(1,), noise=True, compress=c,
                           seed=seed + 10 + c, **kw)
            cX.append(a); cy.append(b)
        cX = np.concatenate(cX); cy = np.concatenate(cy)
        want = int(frac / (1 - frac) * len(X))
        idx = np.random.default_rng(seed).choice(len(cX), min(want, len(cX)), replace=False)
        X = np.concatenate([X, cX[idx]])
        y = np.concatenate([y, cy[idx]])
    return X, y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--processed", required=True)
    ap.add_argument("--sic")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--patience", type=int, default=8)
    ap.add_argument("--only", help="comma-separated config indices")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    results_path = out / "results.jsonl"
    cfgs = make_configs()
    if args.only:
        want = {int(i) for i in args.only.split(",")}
        cfgs = [c for i, c in enumerate(cfgs) if i in want]

    sic = None
    if args.sic:
        d = np.load(args.sic, allow_pickle=True)
        sic = (d["X"].astype(np.float32), d["y"].astype(np.float32))

    print(f"{len(cfgs)} configs -> {results_path}", flush=True)
    for i, (name, cfg) in enumerate(cfgs):
        t0 = time.time()
        print(f"\n[{i+1}/{len(cfgs)}] {name}  {cfg}", flush=True)
        try:
            Xtr, ytr = build_split(args.processed, "train", cfg, 100, True)
            Xva, yva = build_split(args.processed, "val", cfg, 200, True)
            # Test sets never contain compressed data: we score on reality.
            tests = {"clean+noisy": build_split(args.processed, "test", cfg, 300, False)}
            A.T = cfg["T"]

            mX, sX = Xtr.mean(0), Xtr.std(0); sX[sX < 1e-8] = 1.0
            mY, sY = ytr.mean(0), ytr.std(0)
            print(f"    train {Xtr.shape}  val {Xva.shape}", flush=True)

            model = VelocityMLP(input_dim=4 * cfg["T"], hidden_dims=cfg["hidden"],
                                dropout=cfg["dropout"])
            import retrain_velocity_mlp as R
            _orig = R.VelocityMLP
            R.VelocityMLP = lambda *a, **k: model          # train() builds its own otherwise
            R.CONFIG["lr"] = 1e-3
            trained = train(((Xtr - mX) / sX).astype(np.float32),
                            ((ytr - mY) / sY).astype(np.float32),
                            ((Xva - mX) / sX).astype(np.float32),
                            ((yva - mY) / sY).astype(np.float32),
                            args.device, args.epochs, args.patience)
            R.VelocityMLP = _orig

            row = {"idx": i, "name": name,
                   "cfg": {k: (list(v) if isinstance(v, tuple) else v) for k, v in cfg.items()},
                   "secs": round(time.time() - t0)}
            for tname, (X, y) in tests.items():
                row[tname] = metrics(predict(trained, X, mX, sX, mY, sY, args.device), y)
            if sic and cfg["T"] == 10:      # SIC windows are T=10 only
                row["SIC"] = metrics(predict(trained, sic[0], mX, sX, mY, sY, args.device), sic[1])

            torch.jit.script(trained.to("cpu").eval()).save(str(out / f"{name}.torchscript"))
            json.dump({"scaler_X": {"mean": mX.tolist(), "scale": sX.tolist()},
                       "scaler_y": {"mean": mY.tolist(), "scale": sY.tolist()},
                       "cfg": row["cfg"]}, open(out / f"scaler_params_{name}.json", "w"), indent=2)
            trained.to(args.device)
        except Exception as e:                      # one bad config must not kill the night
            row = {"idx": i, "name": name, "error": f"{type(e).__name__}: {e}",
                   "secs": round(time.time() - t0)}
            print(f"    FAILED {row['error']}", flush=True)

        with open(results_path, "a") as fh:
            fh.write(json.dumps(row) + "\n")
        s = row.get("SIC") or row.get("clean+noisy") or {}
        print(f"    done in {row['secs']}s  SIC RMSE {s.get('rmse', float('nan')):.4f}", flush=True)

    print(f"\nsweep complete -> {results_path}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
