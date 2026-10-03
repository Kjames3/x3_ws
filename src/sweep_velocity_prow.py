#!/usr/bin/env python3
"""Sweep #4: retrain the velocity MLP for the OAK-D Pro W and try three model changes.

Sweeps 1-3 (see notes/velocity_estimator_history.md) were all trained against the
OAK-D Lite's noise. The robot now carries a Pro W, the estimator's depth band is
0.5-4.0 m, and three ideas from the backlog have never been trained:

  noise   lite | prow   depth-noise law applied to the Thor-Magni positions.
                        lite = the Lite's measured table (thor_magni_augment),
                        prow = 0.011 * Z^2 (Pro W floor sd with subpixel on).
  jitter  0 | 0.1 | 0.2 fraction of synthetic static-jitter negatives (label 0),
                        drawn from the same law as `noise`.
  zdist   native|deploy native keeps Thor-Magni's own ranges (mostly > 4 m, where
                        the noise law is extrapolated). deploy re-places every
                        window at a first-frame range drawn from U(0.5, 4.5) m,
                        the band the estimator actually serves.
  zfeat   0 | 1         append first-frame range as a 41st input. Translation
                        normalization strips range, so a 40-d model cannot tell
                        3 cm of motion at 1 m from 3 cm of noise at 3.5 m.
                        NEEDS a robot change in _build_window_features.
  nll     0 | 1         4 outputs (mean + log-variance per axis), Gaussian NLL.
                        Gives the calibrated uncertainty the STL/MPC plan wants.
                        NEEDS a robot change where the output is unpacked.

Every model is scored on the same fixed sets, whatever it was trained on:
  sic_held    SIC Courtyard (unseen environment). Referee only.
  prow_near/far  Thor-Magni test, Pro W noise, deployed ranges, split at 2 m.
  lite_native Thor-Magni test as sweep 3 built it, for continuity.
  fast        time-compressed 1.6-4.0 m/s, Pro W noise, deployed ranges.
  jit_lite    sweep 3's static-jitter set (0.0025*Z^2, 1.5-4 m), comparable to it.
  jit_prow    static jitter under the Pro W law over 0.5-4 m.

Python 3.8 compatible (REAL-1's graspnet env). Reuses thor_magni_augment.py for
the file split and the Lite noise table, so the test sequences never leak.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

import thor_magni_augment as A
from retrain_velocity_mlp import metrics

T = 10
CLIP_D = 0.25                 # must match the robot's _build_window_features
PRED_CLIP = 2.5               # must match the estimator's output clamp
TEST_ENV = "Courtyard"
MAX_LABEL = 4.0
Z_MIN, Z_MAX = 0.3, 6.0       # clamp on the range feature
DEPLOY_Z = (0.5, 4.5)         # estimator band is 0.5-4.0 m, plus a margin
PROW_COEF = 0.011             # Pro W floor sd per frame with subpixel on
PROW_FX = 338.4               # 676.9 after the estimator's 2x downsample
LITE_JIT_COEF = 0.0025        # what sweep 3's jitter generator used
LOGVAR_RANGE = (-7.0, 5.0)
CHI2_95_2DOF = 5.991
BATCH, LR, WD = 512, 1e-3, 1e-4


# --------------------------------------------------------------------------
# noise laws
# --------------------------------------------------------------------------
def sigma_pair(law, Z):
    """(depth sd, lateral sd) in metres at range Z for one noise law."""
    if law == "lite":
        return A.sigma_depth(Z), A.sigma_lateral(Z)
    if law == "prow":
        Z = np.abs(np.asarray(Z, dtype=np.float64))
        return PROW_COEF * Z ** 2, Z * (A.SIGMA_PX / PROW_FX)
    raise ValueError(law)


# --------------------------------------------------------------------------
# window builder (vectorized; same semantics as thor_magni_augment.build)
# --------------------------------------------------------------------------
def track_windows(a, stride, compress, rng, law, zdist):
    """All windows of one track. Returns (X40, y2, z0) or None."""
    n = len(a)
    step = stride * compress
    span = (T - 1) * step
    n_win = n - span - compress
    if n_win <= 0:
        return None
    idx = np.arange(n_win)[:, None] + np.arange(T)[None, :] * step
    rx = a[idx, 1].copy()
    ry = a[idx, 2].copy()
    lab = a[idx[:, -1] + compress, 3:5] * compress

    if zdist == "deploy":
        z_draw = rng.uniform(DEPLOY_Z[0], DEPLOY_Z[1], size=n_win)
        rx = rx - rx[:, :1] + z_draw[:, None]
        ry = ry - ry[:, :1]

    Z = np.maximum(0.3, np.abs(rx))
    sd, sl = sigma_pair(law, Z)
    rx = rx + rng.normal(0.0, 1.0, rx.shape) * sd
    ry = ry + rng.normal(0.0, 1.0, ry.shape) * sl

    ok = np.isfinite(lab).all(axis=1) & np.isfinite(rx).all(axis=1) & np.isfinite(ry).all(axis=1)
    ok &= np.hypot(lab[:, 0], lab[:, 1]) <= MAX_LABEL
    rx, ry, lab = rx[ok], ry[ok], lab[ok]
    if not len(rx):
        return None

    dt_scale = 1.0 / stride           # compression deliberately does NOT rescale
    X = np.zeros((len(rx), 4 * T), np.float32)
    X[:, 0::4] = (rx - rx[:, :1]) * dt_scale
    X[:, 1::4] = (ry - ry[:, :1]) * dt_scale
    X[:, 6::4] = np.clip(np.diff(rx, axis=1) * dt_scale, -CLIP_D, CLIP_D)
    X[:, 7::4] = np.clip(np.diff(ry, axis=1) * dt_scale, -CLIP_D, CLIP_D)
    z0 = np.clip(np.abs(rx[:, 0]), Z_MIN, Z_MAX).astype(np.float32)
    return X, lab.astype(np.float32), z0


_SEQ_CACHE = {}


def sequences(proc_dir, split, max_files=None):
    key = (split, max_files)
    if key not in _SEQ_CACHE:
        files = A.splits(proc_dir)[split]
        if max_files:
            files = files[:max_files]
        _SEQ_CACHE[key] = [a for p in files for a in A.load_sequence(p).values()]
    return _SEQ_CACHE[key]


def build(proc_dir, split, strides, compress, seed, law, zdist, max_files=None):
    rng = np.random.default_rng(seed)
    Xs, ys, zs = [], [], []
    for a in sequences(proc_dir, split, max_files):
        for s in strides:
            w = track_windows(a, s, compress, rng, law, zdist)
            if w is not None:
                Xs.append(w[0]); ys.append(w[1]); zs.append(w[2])
    return np.concatenate(Xs), np.concatenate(ys), np.concatenate(zs)


_THOR_CACHE = {}


def build_thor(proc_dir, split, seed, law, zdist, max_files=None):
    """The compress25 recipe: strides 1-3 plus 25% time-compressed (x2) windows."""
    key = (split, seed, law, zdist, max_files)
    if key not in _THOR_CACHE:
        X, y, z = build(proc_dir, split, (1, 2, 3), 1, seed, law, zdist, max_files)
        Xc, yc, zc = build(proc_dir, split, (1,), 2, seed + 1, law, zdist, max_files)
        n = min(int(0.25 * len(X)), len(Xc))
        i = np.random.default_rng(seed).choice(len(Xc), n, replace=False)
        _THOR_CACHE[key] = (np.concatenate([X, Xc[i]]), np.concatenate([y, yc[i]]),
                            np.concatenate([z, zc[i]]))
    return _THOR_CACHE[key]


def jitter_windows(n, rng, coef, z_lo, z_hi):
    """Static object with a noisy centroid, label 0, in the deployed convention."""
    Z = rng.uniform(z_lo, z_hi, size=n)
    sigma = (coef * Z ** 2)[:, None]
    fwd = Z[:, None] + rng.normal(0, 1, (n, T)) * sigma
    lat = rng.normal(0, 1, (n, T)) * sigma
    X = np.zeros((n, 4 * T), np.float32)
    X[:, 0::4] = fwd - fwd[:, :1]
    X[:, 1::4] = lat - lat[:, :1]
    X[:, 6::4] = np.clip(np.diff(fwd, axis=1), -CLIP_D, CLIP_D)
    X[:, 7::4] = np.clip(np.diff(lat, axis=1), -CLIP_D, CLIP_D)
    z0 = np.clip(np.abs(fwd[:, 0]), Z_MIN, Z_MAX).astype(np.float32)
    return X, np.zeros((n, 2), np.float32), z0


def load_sic(npz_path):
    d = np.load(npz_path, allow_pickle=True)
    env = np.array([s.split("_")[1] for s in d["scene"]])
    held = env == TEST_ENV
    z = np.clip(d["rng"].astype(np.float32), Z_MIN, Z_MAX)
    return d["X"].astype(np.float32)[held], d["y"].astype(np.float32)[held], z[held]


# --------------------------------------------------------------------------
# model
# --------------------------------------------------------------------------
class VelocityMLP(nn.Module):
    """Shipped architecture; input/output widths are the only change."""

    def __init__(self, input_dim=40, out_dim=2, hidden_dims=(256, 128, 64), dropout=0.2):
        super().__init__()
        layers, prev = [], input_dim
        for h in hidden_dims:
            layers += [nn.Linear(prev, h), nn.BatchNorm1d(h), nn.ReLU(), nn.Dropout(dropout)]
            prev = h
        layers.append(nn.Linear(prev, out_dim))
        self.network = nn.Sequential(*layers)
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight, nonlinearity="relu")
                nn.init.zeros_(m.bias)

    def forward(self, x):
        return self.network(x)


def loss_fn(out, y, nll):
    if not nll:
        return nn.functional.huber_loss(out, y, delta=1.0)
    mu, lv = out[:, :2], out[:, 2:].clamp(LOGVAR_RANGE[0], LOGVAR_RANGE[1])
    return (0.5 * (lv + (y - mu) ** 2 * torch.exp(-lv))).mean()


def train(Xtr, ytr, Xva, yva, nll, dev, epochs, patience, seed):
    """Sweep 3's recipe (AdamW, plateau LR, Huber, batch 512) with the data held
    on the GPU instead of a DataLoader, which is the same maths and much faster."""
    torch.manual_seed(seed)
    model = VelocityMLP(Xtr.shape[1], 4 if nll else 2).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=WD)
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(opt, mode="min", factor=0.5, patience=5)
    Xtr, ytr = torch.from_numpy(Xtr).to(dev), torch.from_numpy(ytr).to(dev)
    Xva, yva = torch.from_numpy(Xva).to(dev), torch.from_numpy(yva).to(dev)
    n_batches = len(Xtr) // BATCH
    best, best_state, bad = float("inf"), None, 0
    for ep in range(1, epochs + 1):
        model.train()
        perm = torch.randperm(len(Xtr), device=dev)
        tot = 0.0
        for b in range(n_batches):
            i = perm[b * BATCH:(b + 1) * BATCH]
            opt.zero_grad()
            loss = loss_fn(model(Xtr[i]), ytr[i], nll)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            opt.step()
            tot += loss.item()
        model.eval()
        with torch.no_grad():
            va = sum(loss_fn(model(Xva[i:i + 8192]), yva[i:i + 8192], nll).item() * len(Xva[i:i + 8192])
                     for i in range(0, len(Xva), 8192)) / len(Xva)
        sched.step(va)
        if va < best - 1e-6:
            best, bad = va, 0
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        else:
            bad += 1
        if ep % 5 == 0 or ep == 1:
            print("    epoch %3d  train %.5f  val %.5f%s" % (ep, tot / max(1, n_batches), va,
                                                           "  *" if bad == 0 else ""), flush=True)
        if bad >= patience:
            print("    early stop at epoch %d (best val %.5f)" % (ep, best), flush=True)
            break
    model.load_state_dict(best_state)
    return model.eval()


# --------------------------------------------------------------------------
# evaluation
# --------------------------------------------------------------------------
def feats(X, z, zfeat):
    return np.concatenate([X, z[:, None]], axis=1) if zfeat else X


def predict(model, X, S, dev):
    """Returns (velocity m/s clipped, per-axis sd m/s or None)."""
    mX, sX, mY, sY = S
    Xs = torch.from_numpy(((X - mX) / sX).astype(np.float32)).to(dev)
    with torch.no_grad():
        out = torch.cat([model(Xs[i:i + 8192]) for i in range(0, len(Xs), 8192)]).cpu().numpy()
    mu = np.clip(out[:, :2] * sY + mY, -PRED_CLIP, PRED_CLIP)
    sd = None
    if out.shape[1] == 4:
        sd = np.exp(0.5 * np.clip(out[:, 2:], LOGVAR_RANGE[0], LOGVAR_RANGE[1])) * sY
    return mu, sd


def score(model, X, y, z, zfeat, S, dev):
    mu, sd = predict(model, feats(X, z, zfeat), S, dev)
    m = metrics(mu, y)
    if sd is not None:
        m["cover95"] = float((((mu - y) / sd) ** 2).sum(axis=1).__le__(CHI2_95_2DOF).mean())
        m["sd_mean"] = float(sd.mean())
    return m


def score_jitter(model, X, z, zfeat, S, dev):
    mu, sd = predict(model, feats(X, z, zfeat), S, dev)
    sp = np.linalg.norm(mu, axis=1)
    m = {"mean": float(sp.mean()), "p95": float(np.percentile(sp, 95)), "max": float(sp.max()),
         "frac_gt_0.3": float((sp > 0.3).mean()), "frac_gt_0.15": float((sp > 0.15).mean())}
    if sd is not None:
        m["sd_mean"] = float(sd.mean())
        # Share of static windows whose 95% region excludes zero velocity.
        m["confident_motion"] = float(((mu / sd) ** 2).sum(axis=1).__gt__(CHI2_95_2DOF).mean())
    return m


# --------------------------------------------------------------------------
# configs
# --------------------------------------------------------------------------
def cfg(noise, jitter, zdist, zfeat=0, nll=0, seed=0):
    return dict(noise=noise, jitter=jitter, zdist=zdist, zfeat=zfeat, nll=nll, seed=seed)


CONFIGS = {}
# A. data axes, stock 40-in / 2-out model: drop-in replacements for v3.
for _n in ("lite", "prow"):
    for _j in (0.0, 0.1):
        for _z in ("native", "deploy"):
            CONFIGS["%s_j%02d_%s" % (_n, int(_j * 100), _z)] = cfg(_n, _j, _z)
CONFIGS["prow_j20_native"] = cfg("prow", 0.2, "native")
CONFIGS["prow_j20_deploy"] = cfg("prow", 0.2, "deploy")
# B. model axes on the Pro W data: these need estimator changes to deploy.
for _z in ("native", "deploy"):
    CONFIGS["prow_j10_%s_zfeat" % _z] = cfg("prow", 0.1, _z, zfeat=1)
    CONFIGS["prow_j10_%s_nll" % _z] = cfg("prow", 0.1, _z, nll=1)
    CONFIGS["prow_j10_%s_zfeat_nll" % _z] = cfg("prow", 0.1, _z, zfeat=1, nll=1)
# C. seed repeats, so a difference can be compared against run-to-run spread.
for _s in (1, 2):
    CONFIGS["lite_j00_native_s%d" % _s] = cfg("lite", 0.0, "native", seed=_s)
    CONFIGS["prow_j10_deploy_s%d" % _s] = cfg("prow", 0.1, "deploy", seed=_s)


def assemble(c, thor, rng):
    X, y, z = thor
    if c["jitter"] > 0:
        coef = PROW_COEF if c["noise"] == "prow" else LITE_JIT_COEF
        jX, jy, jz = jitter_windows(int(c["jitter"] * len(X)), rng, coef, 0.5, 4.0)
        X, y, z = np.concatenate([X, jX]), np.concatenate([y, jy]), np.concatenate([z, jz])
    return feats(X, z, c["zfeat"]), y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proc-dir", required=True, help="thor_magni_processed/")
    ap.add_argument("--sic-npz", required=True, help="sic_eval_v2.npz")
    ap.add_argument("--out", required=True)
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--patience", type=int, default=6)
    ap.add_argument("--only", default=None, help="comma-separated config names")
    ap.add_argument("--threads", type=int, default=4, help="CPU threads (shared machine)")
    ap.add_argument("--smoke", action="store_true", help="2 training files, for a pipeline check")
    a = ap.parse_args()

    torch.set_num_threads(a.threads)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    mf = 2 if a.smoke else None
    print("device %s  smoke %s" % (dev, a.smoke), flush=True)

    sic = load_sic(a.sic_npz)
    te_prow = build_thor(a.proc_dir, "test", 300, "prow", "deploy", mf)
    te_lite = build_thor(a.proc_dir, "test", 300, "lite", "native", mf)
    near = te_prow[2] < 2.0
    fast = [np.concatenate(p) for p in zip(*[build(a.proc_dir, "test", (1,), c, 900 + c,
                                                   "prow", "deploy", mf) for c in (2, 3)])]
    k = np.linalg.norm(fast[1], axis=1) >= 1.6
    fast = [p[k] for p in fast]
    jit_lite = jitter_windows(20000, np.random.default_rng(1234), LITE_JIT_COEF, 1.5, 4.0)
    jit_prow = jitter_windows(20000, np.random.default_rng(1235), PROW_COEF, 0.5, 4.0)
    print("eval sets: sic %d  prow near/far %d/%d  lite %d  fast %d" % (
        len(sic[0]), near.sum(), (~near).sum(), len(te_lite[0]), len(fast[0])), flush=True)

    done = set()
    if (out / "results.jsonl").is_file():
        for line in open(out / "results.jsonl"):
            r = json.loads(line)
            if "error" not in r:
                done.add(r["name"])

    for name in (a.only.split(",") if a.only else list(CONFIGS)):
        c = CONFIGS[name]
        if name in done:
            print("skip %s (already in results.jsonl)" % name, flush=True)
            continue
        t0 = time.time()
        print("\n===== %s  %s =====" % (name, c), flush=True)
        try:
            rng = np.random.default_rng(7 + c["seed"])
            Xtr, ytr = assemble(c, build_thor(a.proc_dir, "train", 100, c["noise"], c["zdist"], mf), rng)
            Xva, yva = assemble(c, build_thor(a.proc_dir, "val", 200, c["noise"], c["zdist"], mf), rng)
            print("  train %s  val %s" % (Xtr.shape, Xva.shape), flush=True)

            mX, sX = Xtr.mean(0), Xtr.std(0)
            sX[sX < 1e-8] = 1.0
            mY, sY = ytr.mean(0), ytr.std(0)
            S = (mX, sX, mY, sY)
            m = train(((Xtr - mX) / sX).astype(np.float32), ((ytr - mY) / sY).astype(np.float32),
                      ((Xva - mX) / sX).astype(np.float32), ((yva - mY) / sY).astype(np.float32),
                      bool(c["nll"]), dev, a.epochs, a.patience, c["seed"])

            torch.jit.script(m.to("cpu")).save(str(out / ("%s.torchscript" % name)))
            json.dump({"scaler_X": {"mean": mX.tolist(), "scale": sX.tolist()},
                       "scaler_y": {"mean": mY.tolist(), "scale": sY.tolist()},
                       "cfg": c, "input_dim": int(Xtr.shape[1]), "out_dim": 4 if c["nll"] else 2,
                       "convention": "deployed 40-d window%s; outputs %s" % (
                           " + first-frame range (m, clipped 0.3-6.0)" if c["zfeat"] else "",
                           "vx, vy, logvar_vx, logvar_vy (scaled)" if c["nll"] else "vx, vy")},
                      open(out / ("scaler_params_%s.json" % name), "w"), indent=2)
            m.to(dev)

            zf = c["zfeat"]
            row = {"name": name, "cfg": c, "secs": round(time.time() - t0), "n_train": int(len(Xtr))}
            row["sic_held"] = score(m, sic[0], sic[1], sic[2], zf, S, dev)
            row["prow_near"] = score(m, te_prow[0][near], te_prow[1][near], te_prow[2][near], zf, S, dev)
            row["prow_far"] = score(m, te_prow[0][~near], te_prow[1][~near], te_prow[2][~near], zf, S, dev)
            row["lite_native"] = score(m, te_lite[0], te_lite[1], te_lite[2], zf, S, dev)
            row["fast"] = score(m, fast[0], fast[1], fast[2], zf, S, dev)
            row["jit_lite"] = score_jitter(m, jit_lite[0], jit_lite[2], zf, S, dev)
            row["jit_prow"] = score_jitter(m, jit_prow[0], jit_prow[2], zf, S, dev)
            with open(out / "results.jsonl", "a") as fh:
                fh.write(json.dumps(row) + "\n")
            print("  sic %.4f  near %.4f  far %.4f  fast %.4f  jit_prow mean %.3f hot %.1f%%  [%ds]" % (
                row["sic_held"]["rmse"], row["prow_near"]["rmse"], row["prow_far"]["rmse"],
                row["fast"]["rmse"], row["jit_prow"]["mean"],
                100 * row["jit_prow"]["frac_gt_0.3"], row["secs"]), flush=True)
        except Exception as e:
            import traceback
            traceback.print_exc()
            with open(out / "results.jsonl", "a") as fh:
                fh.write(json.dumps({"name": name, "error": str(e)}) + "\n")

    print("\nsweep complete", flush=True)


if __name__ == "__main__":
    sys.exit(main())
