#!/usr/bin/env python3
"""Train a GRU pedestrian trajectory predictor on trajectory_baselines --export windows.

Input: windows.npz (hist [N, H, 4: x y vx vy] odom frame, fut [N, F, 2], run [N]).
Every window is made translation-invariant (relative to the last history
position) and augmented with random rotation + mirror, since the odom frame's
orientation means nothing. The GRU predicts a RESIDUAL on top of cv_est (the
estimator velocity extrapolated straight), so with no signal it falls back to
the baseline the harness found best.

Evaluation is grouped by run (K folds, no run in both train and test), and
each fold reports ADE/FDE for cv_est and the GRU on the same test windows.

Progress lines "epoch <fold>/<k> <e>/<E> ..." are written to --log so the
job-progress page can count them (step pattern '^epoch ').

  python3 train_traj_gru.py windows.npz --out out_dir --log train.log
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HORIZONS = (0.5, 1.0, 2.0, 3.0)


class TrajGRU(nn.Module):
    def __init__(self, fut_n, hidden=64, layers=2, dropout=0.1, nll=False):
        super().__init__()
        self.fut_n, self.nll = fut_n, nll
        self.inp = nn.Linear(6, hidden)
        self.gru = nn.GRU(hidden, hidden, layers, batch_first=True, dropout=dropout if layers > 1 else 0)
        self.head = nn.Sequential(nn.Linear(hidden, hidden), nn.ReLU(), nn.Linear(hidden, fut_n * (3 if nll else 2)))

    def forward(self, x):                       # x: [B, H, 6]
        h, _ = self.gru(torch.relu(self.inp(x)))
        o = self.head(h[:, -1]).view(-1, self.fut_n, 3 if self.nll else 2)
        if not self.nll:
            return o, None
        # isotropic per-step sigma, floored at 1 cm
        return o[..., :2], nn.functional.softplus(o[..., 2]) + 0.01


def features(hist, dt):
    """[N, H, 6]: position rel. to last, estimator velocity, finite-difference velocity."""
    rel = hist[:, :, :2] - hist[:, -1:, :2]
    fd = np.diff(hist[:, :, :2], axis=1, prepend=hist[:, :1, :2]) / dt
    return np.concatenate([rel, hist[:, :, 2:4], fd], 2).astype(np.float32)


def cv_est(hist, fut_n, dt):
    steps = (np.arange(1, fut_n + 1) * dt)[None, :, None]
    return (hist[:, -1:, 2:4] * steps).astype(np.float32)          # relative to p0


def cv_damped(hist, fut_n, dt, tau=1.5):
    steps = (np.arange(1, fut_n + 1) * dt)[None, :, None]
    return (hist[:, -1:, 2:4] * tau * (1 - np.exp(-steps / tau))).astype(np.float32)


def nll_loss(pred, sig, y):
    # 2-D isotropic Gaussian negative log-likelihood (constant dropped)
    e2 = ((pred - y) ** 2).sum(-1)
    return (2 * torch.log(sig) + e2 / (2 * sig ** 2)).mean()


def calibration(pred, sig, gt, rate):
    """Fraction of true positions inside k sigma. Ideal for a 2-D Gaussian:
    1 sigma 39.3%, 2 sigma 86.5%, 2.45 sigma 95%."""
    e = np.linalg.norm(pred - gt, axis=2)
    out = {}
    for hz in HORIZONS:
        k = int(round(hz * rate))
        if k <= e.shape[1]:
            r = e[:, k - 1] / sig[:, k - 1]
            out[str(hz)] = dict(in1=float((r < 1).mean()), in2=float((r < 2).mean()),
                                in245=float((r < 2.45).mean()), sigma_med=float(np.median(sig[:, k - 1])))
    return out


def augment(x, y, b):
    """Same random rotation (+ mirror) on every 2-vector channel."""
    B = x.shape[0]
    th = torch.rand(B, device=x.device) * 2 * math.pi
    c, s = torch.cos(th), torch.sin(th)
    R = torch.stack([torch.stack([c, -s], -1), torch.stack([s, c], -1)], -2)  # [B,2,2]
    flip = (torch.rand(B, device=x.device) < 0.5).float() * -2 + 1
    R = R * torch.stack([torch.ones_like(flip), flip], -1)[:, None, :]
    rot = lambda v: torch.einsum('bij,btj->bti', R, v)
    x = torch.cat([rot(x[..., 0:2]), rot(x[..., 2:4]), rot(x[..., 4:6])], -1)
    return x, rot(y), rot(b)


def metrics(pred, gt, rate):
    e = np.linalg.norm(pred - gt, axis=2)
    out = {}
    for hz in HORIZONS:
        k = int(round(hz * rate))
        if k <= e.shape[1]:
            out[str(hz)] = dict(ade=float(e[:, :k].mean()), fde=float(e[:, k - 1].mean()))
    return out


def log(f, msg):
    print(msg, flush=True)
    if f:
        f.write(msg + '\n')
        f.flush()


def fit(train, val, a, dev, lf, tag, t_start):
    """Train one model with early stopping on val; returns (model, scale, best_state)."""
    xtr, ytr, btr = train
    xva, yva, bva = val
    fut_n = ytr.shape[1]
    sx = xtr.reshape(-1, 6).std(0) + 1e-3      # rotation-symmetric enough for a scale
    model = TrajGRU(fut_n, a.hidden, a.layers, nll=a.nll).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    best, best_state = float('inf'), None
    for ep in range(a.epochs):
        model.train()
        perm = torch.randperm(len(xtr), device=dev)
        tot = 0.0
        for i in range(0, len(perm), a.batch):
            j = perm[i:i + a.batch]
            x, y, b = augment(xtr[j], ytr[j], btr[j])
            o, sig = model(x / sx)
            pred = b + o
            loss = nll_loss(pred, sig, y) if a.nll else torch.linalg.norm(pred - y, dim=-1).mean()   # ADE
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            tot += loss.item() * len(j)
        sched.step()
        model.eval()
        with torch.no_grad():
            v = torch.linalg.norm(bva + model(xva / sx)[0] - yva, dim=-1).mean().item() if len(xva) else tot
        if v < best:
            best, best_state = v, {kk: vv.clone() for kk, vv in model.state_dict().items()}
        log(lf, 'epoch %s %d/%d train_ade %.3f val_ade %.3f best %.3f  %.0fs'
            % (tag, ep + 1, a.epochs, tot / max(1, len(xtr)), v, best, time.time() - t_start))
    model.load_state_dict(best_state)
    model.eval()
    return model, sx, best_state


def export_numpy(state, sx, meta, path):
    """Weights as plain arrays, so the robot runs the model without importing torch."""
    arrs = {k: v.detach().cpu().numpy() for k, v in state.items()}
    arrs['scale'] = sx.detach().cpu().numpy()
    arrs['meta'] = np.array(json.dumps(meta))
    np.savez(path, **arrs)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('--out', default='traj_gru_out')
    ap.add_argument('--log')
    ap.add_argument('--folds', type=int, default=5)
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--hidden', type=int, default=64)
    ap.add_argument('--lr', type=float, default=2e-3)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--layers', type=int, default=2)
    ap.add_argument('--nll', action='store_true', help='also predict a per-step sigma (Gaussian NLL loss)')
    ap.add_argument('--deploy', action='store_true',
                    help='skip cross-validation: train ONE model on every run (10%% of runs held out for early '
                         'stopping) and write gru_deploy.pt + gru_deploy.npz (numpy weights for traj_predictor.py)')
    a = ap.parse_args()

    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    d = np.load(a.npz)
    hist, fut, run, rate = d['hist'], d['fut'], d['run'], float(d['rate'])
    runs = list(d['runs'])
    dt, fut_n = 1.0 / rate, fut.shape[1]
    X = features(hist, dt)
    Y = (fut - hist[:, -1:, :2]).astype(np.float32)
    Bse = cv_est(hist, fut_n, dt)
    Bdm = cv_damped(hist, fut_n, dt)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    lf = open(a.log, 'w') if a.log else None
    log(lf, 'device %s  windows %d  runs %d  folds %d  epochs %d' % (dev, len(X), len(runs), a.folds, a.epochs))

    # folds over runs, balanced by window count
    order = rng.permutation(len(runs))
    counts = np.bincount(run, minlength=len(runs))
    fold_of, load = np.zeros(len(runs), int), np.zeros(a.folds)
    for r in sorted(order, key=lambda r: -counts[r]):
        f = int(np.argmin(load))
        fold_of[r], load[f] = f, load[f] + counts[r]

    if a.deploy:
        t_start = time.time()
        live = [r for r in range(len(runs)) if counts[r]]
        val_runs = rng.choice(live, max(1, len(live) // 10), replace=False)
        va = np.isin(run, val_runs)
        T = lambda m: [torch.tensor(v[m], device=dev) for v in (X, Y, Bse)]
        model, sx, state = fit(T(~va), T(va), a, dev, lf, 'deploy', t_start)
        meta = dict(fut_n=fut_n, hidden=a.hidden, layers=a.layers, nll=a.nll, rate=rate,
                    hist_n=hist.shape[1], val_runs=[str(runs[r]) for r in val_runs], npz=str(a.npz))
        torch.save(dict(state=state, scale=sx.cpu(), **meta), out / 'gru_deploy.pt')
        export_numpy(state, sx, meta, out / 'gru_deploy.npz')
        log(lf, 'deploy model -> %s (val runs %s)' % (out, meta['val_runs']))
        return

    results, P_all, B_all, G_all, D_all, S_all = [], [], [], [], [], []
    t_start = time.time()
    for k in range(a.folds):
        te = fold_of[run] == k
        tr = ~te
        # 10% of the training runs held out for early stopping
        tr_runs = [r for r in range(len(runs)) if fold_of[r] != k and counts[r]]
        val_runs = set(rng.choice(tr_runs, max(1, len(tr_runs) // 10), replace=False).tolist())
        va = tr & np.isin(run, list(val_runs))
        if len(tr_runs) < 10 or va.sum() > 0.3 * tr.sum():
            # too few / too lopsided runs for a run-level validation split
            va = tr & (rng.random(len(tr)) < 0.1)
        tr = tr & ~va
        T = lambda m: [torch.tensor(v[m], device=dev) for v in (X, Y, Bse)]
        xte, yte, bte = T(te)
        model, sx, best_state = fit(T(tr), T(va), a, dev, lf, '%d/%d' % (k + 1, a.folds), t_start)
        model.load_state_dict(best_state)
        model.eval()
        with torch.no_grad():
            o, sig = model(xte / sx)
            p = (bte + o).cpu().numpy()
            S_all.append(sig.cpu().numpy() if sig is not None else None)
        g, bb = yte.cpu().numpy(), bte.cpu().numpy()
        r = dict(fold=k, test_runs=[runs[i] for i in range(len(runs)) if fold_of[i] == k],
                 n_test=int(te.sum()), cv_est=metrics(bb, g, rate), gru=metrics(p, g, rate))
        results.append(r)
        P_all.append(p); B_all.append(bb); G_all.append(g); D_all.append(Bdm[te])
        log(lf, 'fold %d  n=%d  FDE@1s cv %.3f gru %.3f   FDE@2s cv %.3f gru %.3f' % (
            k + 1, r['n_test'], r['cv_est']['1.0']['fde'], r['gru']['1.0']['fde'],
            r['cv_est']['2.0']['fde'], r['gru']['2.0']['fde']))
        torch.save(dict(state=best_state, scale=sx.cpu(), fut_n=fut_n, hidden=a.hidden, layers=a.layers,
                        nll=a.nll, rate=rate),
                   out / ('gru_fold%d.pt' % k))

    P, B, G = map(np.concatenate, (P_all, B_all, G_all))
    Dm = np.concatenate(D_all)
    pooled = dict(cv_est=metrics(B, G, rate), cv_damped=metrics(Dm, G, rate), gru=metrics(P, G, rate))
    # moving windows only (mean speed over the horizon > 0.3 m/s), where prediction matters
    pooled_moving = {}
    for hz in pooled['gru']:
        k = int(round(float(hz) * rate))
        mv = np.linalg.norm(G[:, k - 1], axis=1) / float(hz) > 0.3
        pooled_moving[hz] = {n: metrics(v[mv], G[mv], rate)[hz] for n, v in (('cv_est', B), ('cv_damped', Dm), ('gru', P))}
        pooled_moving[hz]['n'] = int(mv.sum())
    pooled['moving'] = pooled_moving
    lines = ['pooled over %d held-out windows (grouped-by-run %d folds)' % (len(G), a.folds),
             '  horizon   cv_est ADE/FDE   damped ADE/FDE   gru ADE/FDE   FDE vs best baseline | moving-only FDE cv/damped/gru (n)']
    for hz in pooled['gru']:
        c, dm, g, mv = pooled['cv_est'][hz], pooled['cv_damped'][hz], pooled['gru'][hz], pooled_moving[hz]
        best = min(c['fde'], dm['fde'])
        lines.append('  %5s s   %5.3f / %5.3f    %5.3f / %5.3f    %5.3f / %5.3f   %+5.1f%%   | %.3f / %.3f / %.3f (%d)' % (
            hz, c['ade'], c['fde'], dm['ade'], dm['fde'], g['ade'], g['fde'], 100 * (g['fde'] / best - 1),
            mv['cv_est']['fde'], mv['cv_damped']['fde'], mv['gru']['fde'], mv['n']))
    if a.nll:
        cal = calibration(P, np.concatenate(S_all), G, rate)
        pooled['calibration'] = cal
        lines.append('  calibration (ideal 39 / 86 / 95 %): ' + '  '.join(
            '%s s: %.0f/%.0f/%.0f%% sigma %.2f m' % (hz, 100 * c['in1'], 100 * c['in2'], 100 * c['in245'], c['sigma_med'])
            for hz, c in cal.items()))
    txt = '\n'.join(lines)
    log(lf, txt)
    (out / 'report.txt').write_text(txt + '\n')
    (out / 'results.json').write_text(json.dumps(dict(args=vars(a), folds=results, pooled=pooled), indent=1))
    log(lf, 'TRAINING DONE')


if __name__ == '__main__':
    main()
