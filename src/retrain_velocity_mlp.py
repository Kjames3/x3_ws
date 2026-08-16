"""
retrain_velocity_mlp.py — Fix the train/serve skew in the pedestrian velocity MLP.

BACKGROUND

`velocity_estimator._build_window_features()` applies two transforms that the
training pipeline never applied:

  1. Translation normalization (Idea 48): positions are re-expressed relative to
     the window's first frame, so rx_norm[0] and ry_norm[0] are structurally 0.
     Training used raw absolute rel_x/rel_y (X_train mean 2.31, std 6.08).
  2. First-frame deltas are forced to 0. Training's dx/dy came from
     `group['rel_x'].diff()` over the whole track, so the window's first frame
     carried a real displacement from the frame before it.

`scaler_params.json` is therefore NOT stale — it correctly describes the
training data. The skew is that inference feeds the network something else:
position channels arrive shifted ~0.37 sigma and compressed ~13x.

Two coherent fixes exist. This script implements the forward-looking one:
retrain on the deployed convention and refit the scaler to match, so the robot
code stays as-is. (The alternative — deleting both transforms from
`_build_window_features` — needs no retraining but throws away the translation
invariance, which is worth keeping.)

Both mismatches are pure functions of the stored windows, so no re-preprocessing
of the raw Thor-Magni CSVs is needed.

USAGE
    python3 src/retrain_velocity_mlp.py --data <thor_magni_windows/> --eval-only
    python3 src/retrain_velocity_mlp.py --data <thor_magni_windows/> \
        --sic sic_windows.npz --out-dir src/
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

WINDOW_SIZE = 10
CLIP_D      = 0.25          # matches the Idea 63 dx/dy clamp at inference
PRED_CLIP   = 2.5           # matches the inference-side prediction clamp

CONFIG = dict(batch_size=512, lr=1e-3, weight_decay=1e-4, epochs=100, patience=10)


class VelocityMLP(nn.Module):
    """Same architecture as the shipped model (EE_244 training/model.py)."""

    def __init__(self, input_dim=40, hidden_dims=(256, 128, 64), dropout=0.2):
        super().__init__()
        layers, prev = [], input_dim
        for h in hidden_dims:
            layers += [nn.Linear(prev, h), nn.BatchNorm1d(h), nn.ReLU(), nn.Dropout(dropout)]
            prev = h
        layers.append(nn.Linear(prev, 2))
        self.network = nn.Sequential(*layers)
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, nonlinearity="relu")
                nn.init.zeros_(m.bias)

    def forward(self, x):
        return self.network(x)


# ── Convention transform ──────────────────────────────────────────────────────
def to_deployed_convention(X):
    """Absolute training windows -> exactly what _build_window_features emits.

    Layout is [rx, ry, dx, dy] x 10. Translation-normalize the position channels
    against frame 0, zero the frame-0 deltas, and clamp the rest.
    """
    X = np.asarray(X, dtype=np.float32).copy()
    rx0, ry0 = X[:, 0].copy(), X[:, 1].copy()
    for i in range(WINDOW_SIZE):
        X[:, 4 * i] -= rx0
        X[:, 4 * i + 1] -= ry0
        if i == 0:
            X[:, 2] = 0.0
            X[:, 3] = 0.0
        else:
            np.clip(X[:, 4 * i + 2], -CLIP_D, CLIP_D, out=X[:, 4 * i + 2])
            np.clip(X[:, 4 * i + 3], -CLIP_D, CLIP_D, out=X[:, 4 * i + 3])
    return X


def load_split(data_dir, split):
    d = Path(data_dir)
    return (np.load(d / f"X_{split}.npy").astype(np.float32),
            np.load(d / f"y_{split}.npy").astype(np.float32))


# ── Evaluation ────────────────────────────────────────────────────────────────
def predict(model, X, mX, sX, mY, sY, device="cpu", bs=8192):
    model.eval()
    Xs = ((X - mX) / sX).astype(np.float32)
    out = []
    with torch.no_grad():
        for i in range(0, len(Xs), bs):
            t = torch.from_numpy(Xs[i:i + bs]).to(device)
            out.append(model(t).cpu().numpy())
    return np.clip(np.concatenate(out) * sY + mY, -PRED_CLIP, PRED_CLIP)


def metrics(pred, y):
    err = np.linalg.norm(pred - y, axis=1)
    ps, ts = np.linalg.norm(pred, axis=1), np.linalg.norm(y, axis=1)
    mv = ts > 0.5
    st = ts < 0.1
    return dict(
        n=len(y),
        rmse=float(np.sqrt((err ** 2).mean())),
        mae_vx=float(np.abs(pred[:, 0] - y[:, 0]).mean()),
        mae_vy=float(np.abs(pred[:, 1] - y[:, 1]).mean()),
        moving_rmse=float(np.sqrt((err[mv] ** 2).mean())) if mv.any() else float("nan"),
        moving_ratio=float(np.median(ps[mv]) / max(1e-6, np.median(ts[mv]))) if mv.any() else float("nan"),
        static_pred=float(np.median(ps[st])) if st.any() else float("nan"),
        pred_max=float(ps.max()),
    )


def show(tag, m):
    print(f"  {tag:<34} n={m['n']:>6}  RMSE {m['rmse']:.4f}  "
          f"MAE vx/vy {m['mae_vx']:.4f}/{m['mae_vy']:.4f}  "
          f"moving RMSE {m['moving_rmse']:.4f} (ratio {m['moving_ratio']:.2f})  "
          f"static {m['static_pred']:.3f}  max {m['pred_max']:.2f}")


# ── Training ──────────────────────────────────────────────────────────────────
def train(Xtr, ytr, Xva, yva, device, epochs, patience):
    model = VelocityMLP().to(device)
    crit = nn.HuberLoss(delta=1.0)
    opt = torch.optim.AdamW(model.parameters(), lr=CONFIG["lr"],
                            weight_decay=CONFIG["weight_decay"])
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(opt, mode="min", factor=0.5, patience=5)

    tr = torch.utils.data.TensorDataset(torch.from_numpy(Xtr), torch.from_numpy(ytr))
    va = torch.utils.data.TensorDataset(torch.from_numpy(Xva), torch.from_numpy(yva))
    tl = torch.utils.data.DataLoader(tr, batch_size=CONFIG["batch_size"], shuffle=True,
                                     drop_last=True)
    vl = torch.utils.data.DataLoader(va, batch_size=4096)

    best, best_state, bad = float("inf"), None, 0
    for ep in range(1, epochs + 1):
        model.train()
        tot = 0.0
        for xb, yb in tl:
            xb, yb = xb.to(device), yb.to(device)
            opt.zero_grad()
            loss = crit(model(xb), yb)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            opt.step()
            tot += loss.item() * len(xb)
        tr_loss = tot / max(1, len(tl.dataset))

        model.eval()
        tot = 0.0
        with torch.no_grad():
            for xb, yb in vl:
                xb, yb = xb.to(device), yb.to(device)
                tot += crit(model(xb), yb).item() * len(xb)
        va_loss = tot / len(vl.dataset)
        sched.step(va_loss)

        if va_loss < best - 1e-6:
            best, bad = va_loss, 0
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        else:
            bad += 1
        if ep % 5 == 0 or ep == 1:
            print(f"    epoch {ep:3d}  train {tr_loss:.5f}  val {va_loss:.5f}"
                  f"{'  *' if bad == 0 else ''}")
        if bad >= patience:
            print(f"    early stop at epoch {ep} (best val {best:.5f})")
            break

    model.load_state_dict(best_state)
    return model


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, help="thor_magni_windows/ directory")
    ap.add_argument("--sic", help="sic_windows npz for out-of-domain evaluation")
    ap.add_argument("--old-model", default="src/velocity_mlp.torchscript")
    ap.add_argument("--old-scaler", default="src/scaler_params.json")
    ap.add_argument("--out-dir", help="write refit scaler + retrained model here")
    ap.add_argument("--eval-only", action="store_true", help="quantify the skew, skip training")
    ap.add_argument("--epochs", type=int, default=CONFIG["epochs"])
    ap.add_argument("--patience", type=int, default=CONFIG["patience"])
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    sp = json.load(open(args.old_scaler))
    mX = np.array(sp["scaler_X"]["mean"], dtype=np.float32)
    sX = np.array(sp["scaler_X"]["scale"], dtype=np.float32)
    mY = np.array(sp["scaler_y"]["mean"], dtype=np.float32)
    sY = np.array(sp["scaler_y"]["scale"], dtype=np.float32)

    Xte, yte = load_split(args.data, "test")
    Xte_dep = to_deployed_convention(Xte)
    old = torch.jit.load(args.old_model).to(args.device)

    sic = None
    if args.sic:
        d = np.load(args.sic, allow_pickle=True)
        sic = (d["X"].astype(np.float32), d["y"].astype(np.float32))

    print("\n=== BASELINE: shipped model + shipped scaler ===")
    print("  (Thor-Magni test set, in-domain)")
    show("absolute feats (as TRAINED)", metrics(predict(old, Xte, mX, sX, mY, sY, args.device), yte))
    show("deployed feats (as SERVED)", metrics(predict(old, Xte_dep, mX, sX, mY, sY, args.device), yte))
    if sic:
        print("  (SIC, out-of-domain)")
        show("deployed feats (as SERVED)", metrics(predict(old, sic[0], mX, sX, mY, sY, args.device), sic[1]))

    if args.eval_only:
        return 0

    Xtr, ytr = load_split(args.data, "train")
    Xva, yva = load_split(args.data, "val")
    Xtr_d, Xva_d = to_deployed_convention(Xtr), to_deployed_convention(Xva)

    # Refit the scaler on the convention the robot actually serves.
    nmX, nsX = Xtr_d.mean(0), Xtr_d.std(0)
    nsX[nsX < 1e-8] = 1.0            # frame-0 channels are structurally constant
    nmY, nsY = ytr.mean(0), ytr.std(0)
    print(f"\n=== REFIT SCALER on deployed convention (n={len(Xtr_d):,}) ===")
    print(f"  idx 0-3 mean {np.round(nmX[:4], 4)}  scale {np.round(nsX[:4], 4)}")
    print(f"  idx 36-39 mean {np.round(nmX[36:], 4)}  scale {np.round(nsX[36:], 4)}")

    def sc(X):
        return ((X - nmX) / nsX).astype(np.float32)

    def sy(y):
        return ((y - nmY) / nsY).astype(np.float32)

    print(f"\n=== RETRAIN on {args.device} ===")
    t0 = time.time()
    model = train(sc(Xtr_d), sy(ytr), sc(Xva_d), sy(yva), args.device, args.epochs, args.patience)
    print(f"  trained in {time.time()-t0:.0f}s")

    print("\n=== RETRAINED model + refit scaler ===")
    print("  (Thor-Magni test set, in-domain)")
    show("deployed feats", metrics(predict(model, Xte_dep, nmX, nsX, nmY, nsY, args.device), yte))
    if sic:
        print("  (SIC, out-of-domain)")
        show("deployed feats", metrics(predict(model, sic[0], nmX, nsX, nmY, nsY, args.device), sic[1]))

    if args.out_dir:
        out = Path(args.out_dir)
        model_cpu = model.to("cpu").eval()
        ts = torch.jit.script(model_cpu)
        ts.save(str(out / "velocity_mlp_v2.torchscript"))
        json.dump({"scaler_X": {"mean": nmX.tolist(), "scale": nsX.tolist()},
                   "scaler_y": {"mean": nmY.tolist(), "scale": nsY.tolist()},
                   "convention": "deployed: translation-normalized, frame-0 deltas zeroed, "
                                 "dx/dy clamped to +/-0.25"},
                  open(out / "scaler_params_v2.json", "w"), indent=4)
        print(f"\nwrote {out/'velocity_mlp_v2.torchscript'} and {out/'scaler_params_v2.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
