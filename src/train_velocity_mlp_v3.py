"""
train_velocity_mlp_v3.py — Velocity MLP trained for the OAK-D Lite serving path.

v2 fixed the train/serve convention skew. v3 additionally closes the two sensor
gaps that mocap training data cannot represent:

  * depth noise, from the curve measured on the robot (thor_magni_augment)
  * off-nominal, variable loop rate (the estimator runs 5.9-7.4 Hz, not 10)

Training mix is clean 10 Hz plus noisy 10 / 5 / 3.33 Hz, so the net sees both the
clean regime and the one it is actually served.

    python3 src/train_velocity_mlp_v3.py \
        --processed <thor_magni_processed/> --sic sic_windows.npz --out-dir src/
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

from retrain_velocity_mlp import VelocityMLP, metrics, predict, show, train
from thor_magni_augment import build


def load_scaler(path):
    sp = json.load(open(path))
    return (np.array(sp["scaler_X"]["mean"], np.float32),
            np.array(sp["scaler_X"]["scale"], np.float32),
            np.array(sp["scaler_y"]["mean"], np.float32),
            np.array(sp["scaler_y"]["scale"], np.float32))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--processed", required=True, help="thor_magni_processed/ CSVs")
    ap.add_argument("--sic")
    ap.add_argument("--out-dir")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--patience", type=int, default=10)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()
    P, dev = args.processed, args.device

    print("=== building windows ===")
    t0 = time.time()
    parts = {
        "clean 10Hz":  dict(strides=(1,), noise=False),
        "noisy 10Hz":  dict(strides=(1,), noise=True),
        "noisy 5Hz":   dict(strides=(2,), noise=True),
        "noisy 3.3Hz": dict(strides=(3,), noise=True),
    }
    Xtr, ytr = [], []
    for name, kw in parts.items():
        X, y = build(P, "train", seed=abs(hash(name)) % 2**31, **kw)
        print(f"  train {name:<12} {X.shape}")
        Xtr.append(X); ytr.append(y)
    Xtr = np.concatenate(Xtr); ytr = np.concatenate(ytr)

    Xva, yva = [], []
    for name, kw in parts.items():
        X, y = build(P, "val", seed=1234, **kw)
        Xva.append(X); yva.append(y)
    Xva = np.concatenate(Xva); yva = np.concatenate(yva)
    print(f"  TOTAL train {Xtr.shape}  val {Xva.shape}   ({time.time()-t0:.0f}s)")

    # Held-out test sets, built once and shared by every model.
    tests = {
        "clean 10Hz":  build(P, "test", strides=(1,), noise=False, seed=7),
        "noisy 10Hz":  build(P, "test", strides=(1,), noise=True,  seed=7),
        "noisy 5Hz":   build(P, "test", strides=(2,), noise=True,  seed=7),
        "noisy 3.3Hz": build(P, "test", strides=(3,), noise=True,  seed=7),
    }
    if args.sic:
        d = np.load(args.sic, allow_pickle=True)
        tests["SIC (real)"] = (d["X"].astype(np.float32), d["y"].astype(np.float32))

    nmX, nsX = Xtr.mean(0), Xtr.std(0)
    nsX[nsX < 1e-8] = 1.0
    nmY, nsY = ytr.mean(0), ytr.std(0)

    print(f"\n=== training v3 on {dev} ===")
    t0 = time.time()
    model = train(((Xtr - nmX) / nsX).astype(np.float32), ((ytr - nmY) / nsY).astype(np.float32),
                  ((Xva - nmX) / nsX).astype(np.float32), ((yva - nmY) / nsY).astype(np.float32),
                  dev, args.epochs, args.patience)
    print(f"  trained in {time.time()-t0:.0f}s")

    here = Path(__file__).parent
    models = [
        ("v1 shipped", torch.jit.load(here / "velocity_mlp.torchscript").to(dev),
         load_scaler(here / "scaler_params.json")),
        ("v2 skew-fix", torch.jit.load(here / "velocity_mlp_v2.torchscript").to(dev),
         load_scaler(here / "scaler_params_v2.json")),
        ("v3 noise+dt", model.to(dev), (nmX, nsX, nmY, nsY)),
    ]

    for tname, (X, y) in tests.items():
        print(f"\n--- {tname}  (n={len(y)}) ---")
        for mname, m, sc in models:
            show(mname, metrics(predict(m, X, *sc, device=dev), y))

    if args.out_dir:
        out = Path(args.out_dir)
        torch.jit.script(model.to("cpu").eval()).save(str(out / "velocity_mlp_v3.torchscript"))
        json.dump({"scaler_X": {"mean": nmX.tolist(), "scale": nsX.tolist()},
                   "scaler_y": {"mean": nmY.tolist(), "scale": nsY.tolist()},
                   "convention": "deployed (translation-normalized x dt_scale, frame-0 deltas "
                                 "zeroed, dx/dy clamped +/-0.25); trained with measured OAK-D "
                                 "Lite depth noise and 10/5/3.3 Hz sampling"},
                  open(out / "scaler_params_v3.json", "w"), indent=4)
        print(f"\nwrote {out/'velocity_mlp_v3.torchscript'} and {out/'scaler_params_v3.json'}")


if __name__ == "__main__":
    raise SystemExit(main())
